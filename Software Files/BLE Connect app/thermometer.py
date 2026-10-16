"""
Remote thermometer app: log temperatures and configure the thermometer over BLE.

    python thermometer.py scan                                      # list thermometers in range
    python thermometer.py log                                       # print + log to temperature.csv
    python thermometer.py config                                    # show the configuration
    python thermometer.py config --interval 5000 --sensors 1,2,4 --only-connected off

BLE access, --transport:
    tcp-client:127.0.0.1:9000   (default) the emulator's virtual dongle node, via the Bumble host stack
    usb:0, serial:COM5, ...     any other Bumble transport, e.g. an HCI dongle
    pc                          the PC's own Bluetooth adapter, via bleak (for the real thermometer)

The BLE interface is described in thermometer_protocol.py.
"""
from __future__ import annotations

import abc
import argparse
import asyncio
import contextlib
import csv
import logging
import sys
from datetime import datetime
from typing import Callable

from thermometer_protocol import (ALL_UUID, CONFIG_UUID, DEVICE_NAME, INTERVAL_MAX_MS, INTERVAL_MIN_MS,
                                  SENSOR_COUNT, SENSOR_UUIDS, SERVICE_UUID, Config, decode_all,
                                  decode_temperature)

RETRY_DELAY = 3.0  # s between reconnection attempts (log)


class LinkError(Exception):
    """Connection or GATT problem, with a message for the user."""


class ConnectionLost(LinkError):
    pass


# ---------------------------------------------------------------------------------------------- BLE links
class Link(abc.ABC):
    """A connection to one thermometer, independent of the BLE stack underneath."""

    def __init__(self) -> None:
        self.disconnected = asyncio.Event()

    @abc.abstractmethod
    async def open(self) -> None:
        """Prepare the BLE stack (open the HCI transport, ...)."""

    @abc.abstractmethod
    async def close(self) -> None: ...

    @abc.abstractmethod
    async def scan(self, timeout: float) -> list[tuple[str, str, int]]:
        """Thermometers in range: (address, name, RSSI)."""

    @abc.abstractmethod
    async def connect(self, name: str, address: str | None, timeout: float) -> str:
        """Connect and discover the thermometer service. Returns the peer address."""

    @abc.abstractmethod
    async def read(self, uuid: str) -> bytes: ...

    @abc.abstractmethod
    async def write(self, uuid: str, value: bytes) -> None: ...

    @abc.abstractmethod
    async def subscribe(self, uuid: str, callback: Callable[[bytes], None]) -> None: ...

    async def __aenter__(self) -> Link:
        await self.open()
        return self

    async def __aexit__(self, *exc) -> None:
        await self.close()


class BumbleLink(Link):
    """Bumble host stack on an HCI transport: the emulator's dongle node, or a USB/serial HCI dongle."""

    def __init__(self, transport: str, hci_timeout: float) -> None:
        super().__init__()
        self.transport_name = transport
        self.hci_timeout = hci_timeout
        self.transport = None
        self.device = None
        self.connection = None
        self.characteristics: dict[str, object] = {}

    async def open(self) -> None:
        from bumble.device import Device
        from bumble.hci import Address
        from bumble.transport import open_transport

        # Retry while the emulator isn't up yet, or its dongle firmware isn't running (its UART then
        # drops the HCI Reset and the controller never answers).
        while True:
            try:
                self.transport = await open_transport(self.transport_name)
            except OSError as exc:
                print(f'Cannot open {self.transport_name} ({exc.strerror or exc}); is the emulator running? Retrying...')
                await asyncio.sleep(RETRY_DELAY)
                continue
            self.device = Device.with_hci('thermometer.py', Address('F0:F1:F2:F3:F4:F5'),
                                          self.transport.source, self.transport.sink)
            try:
                await asyncio.wait_for(self.device.power_on(), self.hci_timeout)
            except TimeoutError:
                print(f'No answer from the HCI controller within {self.hci_timeout:.0f} s '
                      '(is the emulation running?), retrying...')
                await self.transport.close()
                continue
            self.transport.source.terminated.add_done_callback(lambda _: self.disconnected.set())
            return

    async def close(self) -> None:
        # Disconnect properly, so the thermometer advertises again at once instead of after its
        # supervision timeout
        if self.connection and not self.disconnected.is_set():
            with contextlib.suppress(Exception):
                await asyncio.wait_for(self.connection.disconnect(), 10)
        if self.transport:
            with contextlib.suppress(Exception):
                await self.transport.close()

    async def _scan(self, timeout: float, stop_at_name: str | None = None) -> dict[str, tuple[object, str, int]]:
        from bumble.core import AdvertisingData

        found: dict[str, tuple[object, str, int]] = {}
        done = asyncio.get_running_loop().create_future()

        def on_advertisement(adv):
            name = adv.data.get(AdvertisingData.COMPLETE_LOCAL_NAME)
            if name and name.startswith(DEVICE_NAME) and adv.is_connectable:
                found[str(adv.address)] = (adv.address, name, adv.rssi)
                if stop_at_name and name == stop_at_name and not done.done():
                    done.set_result(adv.address)

        self.device.on('advertisement', on_advertisement)
        await self.device.start_scanning(legacy=True, active=False)
        try:
            with contextlib.suppress(TimeoutError):
                await asyncio.wait_for(done, timeout)
        finally:
            self.device.remove_listener('advertisement', on_advertisement)
            await self.device.stop_scanning(legacy=True)
        return found

    async def scan(self, timeout: float) -> list[tuple[str, str, int]]:
        return [(addr, name, rssi) for addr, (_, name, rssi) in (await self._scan(timeout)).items()]

    async def connect(self, name: str, address: str | None, timeout: float) -> str:
        from bumble.core import UUID
        from bumble.device import Peer

        if address:
            target = address
        else:
            found = await self._scan(timeout, stop_at_name=name)
            matches = [a for a, (_, n, _) in found.items() if n == name]
            if not matches:
                raise LinkError(f'{name} not found within {timeout:.0f} s')
            target = found[matches[0]][0]

        connection = self.connection = await self.device.connect(target)
        connection.on('disconnection', lambda reason: self.disconnected.set())
        peer = Peer(connection)
        await peer.discover_service(UUID(SERVICE_UUID))
        await peer.discover_characteristics()
        for uuid in [*SENSOR_UUIDS, ALL_UUID, CONFIG_UUID]:
            found = peer.get_characteristics_by_uuid(UUID(uuid))
            if found:
                self.characteristics[uuid] = found[0]
        if ALL_UUID not in self.characteristics:
            raise LinkError('the thermometer service was not found (or the firmware is too old)')
        return str(connection.peer_address)

    def _char(self, uuid: str):
        try:
            return self.characteristics[uuid]
        except KeyError:
            raise LinkError(f'characteristic {uuid} not found: thermometer firmware too old?') from None

    async def read(self, uuid: str) -> bytes:
        return bytes(await self._guard(self._char(uuid).read_value()))

    async def write(self, uuid: str, value: bytes) -> None:
        await self._guard(self._char(uuid).write_value(value, with_response=True))

    async def subscribe(self, uuid: str, callback: Callable[[bytes], None]) -> None:
        await self._guard(self._char(uuid).subscribe(lambda value: callback(bytes(value))))

    async def _guard(self, coro):
        from bumble.att import ATT_Error

        try:
            return await coro
        except ATT_Error as exc:
            raise LinkError(f'the thermometer refused: {exc.error_name or hex(exc.error_code)}') from exc
        except asyncio.CancelledError:
            if self.disconnected.is_set():
                raise ConnectionLost('connection lost') from None
            raise


class BleakLink(Link):
    """The PC's own Bluetooth adapter, through bleak (Windows, Linux, macOS)."""

    def __init__(self) -> None:
        super().__init__()
        self.client = None

    async def open(self) -> None:
        pass

    async def close(self) -> None:
        if self.client:
            with contextlib.suppress(Exception):
                await self.client.disconnect()

    async def scan(self, timeout: float) -> list[tuple[str, str, int]]:
        from bleak import BleakScanner

        found = await BleakScanner.discover(timeout=timeout, return_adv=True)
        return [(dev.address, adv.local_name, adv.rssi) for dev, adv in found.values()
                if adv.local_name and adv.local_name.startswith(DEVICE_NAME)]

    async def connect(self, name: str, address: str | None, timeout: float) -> str:
        from bleak import BleakClient, BleakScanner

        if address:
            target = address
        else:
            target = await BleakScanner.find_device_by_name(name, timeout=timeout)
            if target is None:
                raise LinkError(f'{name} not found within {timeout:.0f} s')
        self.client = BleakClient(target, disconnected_callback=lambda _: self.disconnected.set())
        await self.client.connect()
        if self.client.services.get_characteristic(ALL_UUID) is None:
            raise LinkError('the thermometer service was not found (or the firmware is too old)')
        return self.client.address

    async def read(self, uuid: str) -> bytes:
        return bytes(await self._guard(self.client.read_gatt_char(uuid)))

    async def write(self, uuid: str, value: bytes) -> None:
        await self._guard(self.client.write_gatt_char(uuid, value, response=True))

    async def subscribe(self, uuid: str, callback: Callable[[bytes], None]) -> None:
        await self._guard(self.client.start_notify(uuid, lambda _, data: callback(bytes(data))))

    async def _guard(self, coro):
        from bleak.exc import BleakError

        try:
            return await coro
        except BleakError as exc:
            if self.disconnected.is_set():
                raise ConnectionLost('connection lost') from exc
            raise LinkError(str(exc)) from exc


def make_link(args) -> Link:
    if args.transport == 'pc':
        return BleakLink()
    return BumbleLink(args.transport, args.hci_timeout)


# ---------------------------------------------------------------------------------------------- commands
def print_round(temperatures: list[float | None]) -> None:
    now = datetime.now().strftime('%H:%M:%S')
    text = '  '.join(f'T{i + 1} {"-" if t is None else f"{t:8.3f}"}' for i, t in enumerate(temperatures))
    print(f'{now}  {text}')


async def cmd_scan(args) -> int:
    async with make_link(args) as link:
        print(f'Scanning {args.timeout:.0f} s...')
        found = await link.scan(args.timeout)
    for address, name, rssi in found:
        print(f'{address}  {name}  RSSI {rssi}')
    if not found:
        print(f'No {DEVICE_NAME} found')
    return 0 if found else 1


async def log_once(args, csv_file: str) -> None:
    """One connection: log every round the thermometer notifies, until the link drops."""
    async with make_link(args) as link:
        print(f'Looking for {args.name}...')
        address = await link.connect(args.name, args.address, args.timeout)
        print(f'Connected to {address}')

        try:
            print((Config.from_bytes(await link.read(CONFIG_UUID))).describe())
        except LinkError:
            pass  # older firmware without the Config characteristic

        rounds: asyncio.Queue[bytes] = asyncio.Queue()
        await link.subscribe(ALL_UUID, rounds.put_nowait)
        print('Waiting for measurements (one notification per round)...')

        lost = asyncio.ensure_future(link.disconnected.wait())
        try:
            while True:
                got = asyncio.ensure_future(rounds.get())
                await asyncio.wait({got, lost}, return_when=asyncio.FIRST_COMPLETED)
                if not got.done():
                    got.cancel()
                    raise ConnectionLost('disconnected')
                temperatures = decode_all(got.result())
                print_round(temperatures)
                with open(csv_file, 'a', encoding='UTF8', newline='\n') as file:
                    csv.writer(file).writerow([datetime.now().strftime('%H:%M:%S'), *temperatures])
        finally:
            lost.cancel()


async def cmd_log(args) -> int:
    with open(args.csv, 'w', encoding='UTF8', newline='\n') as file:
        csv.writer(file).writerow(['time', *[f'T{i + 1}' for i in range(SENSOR_COUNT)]])

    # Keep logging across disconnections and emulator restarts
    while True:
        try:
            await log_once(args, args.csv)
        except ConnectionLost as exc:
            print(f'{exc.args[0] if exc.args else "Connection lost"}, reconnecting...')
        except LinkError as exc:
            print(f'{exc}, retrying...')
        await asyncio.sleep(RETRY_DELAY)


async def cmd_config(args) -> int:
    changes = args.interval is not None or args.sensors is not None or args.only_connected is not None
    async with make_link(args) as link:
        address = await link.connect(args.name, args.address, args.timeout)
        print(f'Connected to {address}')
        config = Config.from_bytes(await link.read(CONFIG_UUID))

        if changes:
            if args.interval is not None:
                config.interval_ms = args.interval
            if args.sensors is not None:
                config.sensors = args.sensors
            if args.only_connected is not None:
                config.measure_only_connected = args.only_connected
            config.check()
            await link.write(CONFIG_UUID, config.to_bytes())
            config = Config.from_bytes(await link.read(CONFIG_UUID))   # what the thermometer took
            print('New configuration:')
        print(config.describe())

        if args.read:
            print('Current values:')
            for i, uuid in enumerate(SENSOR_UUIDS):
                t = decode_temperature(await link.read(uuid))
                print(f'  T{i + 1} {"-" if t is None else f"{t:.3f}"}')
    return 0


# ---------------------------------------------------------------------------------------------- CLI
def sensor_list(text: str) -> frozenset[int]:
    if text.strip().lower() == 'none':
        return frozenset()
    try:
        sensors = frozenset(int(s) for s in text.split(','))
    except ValueError:
        raise argparse.ArgumentTypeError('use a list like 1,2,4 (or "none")') from None
    if any(not 1 <= s <= SENSOR_COUNT for s in sensors):
        raise argparse.ArgumentTypeError(f'sensors are numbered 1 .. {SENSOR_COUNT}')
    return sensors


def on_off(text: str) -> bool:
    if text.lower() in ('on', 'yes', 'true', '1'):
        return True
    if text.lower() in ('off', 'no', 'false', '0'):
        return False
    raise argparse.ArgumentTypeError('use on or off')


def interval(text: str) -> int:
    value = int(text)
    if not INTERVAL_MIN_MS <= value <= INTERVAL_MAX_MS:
        raise argparse.ArgumentTypeError(f'{INTERVAL_MIN_MS} .. {INTERVAL_MAX_MS} ms')
    return value


def main() -> int:
    parser = argparse.ArgumentParser(description='Remote thermometer over BLE: log temperatures, configure.',
                                     formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    parser.add_argument('--transport', default='tcp-client:127.0.0.1:9000',
                        help='"pc" (PC Bluetooth, bleak) or a Bumble HCI transport (default: the emulator dongle)')
    parser.add_argument('--name', default=DEVICE_NAME, help=f'device name (default {DEVICE_NAME})')
    parser.add_argument('--address', help='connect to this address instead of looking for the name')
    parser.add_argument('--timeout', type=float,
                        help='seconds to scan / to find the thermometer (default: scan 10, others 120)')
    parser.add_argument('--hci-timeout', type=float, default=15.0, help=argparse.SUPPRESS)
    parser.add_argument('--verbose', action='store_true', help='BLE stack debug logging')
    sub = parser.add_subparsers(dest='command', required=True)

    p = sub.add_parser('scan', help='list thermometers in range')
    p.set_defaults(func=cmd_scan)

    p = sub.add_parser('log', help='print every measurement round and append it to a CSV file')
    p.add_argument('--csv', default='temperature.csv', help='output file (default temperature.csv)')
    p.set_defaults(func=cmd_log)

    p = sub.add_parser('config', help='show or change the configuration (stored on the thermometer)')
    p.add_argument('--interval', type=interval, help=f'sampling interval in ms ({INTERVAL_MIN_MS} .. {INTERVAL_MAX_MS})')
    p.add_argument('--sensors', type=sensor_list, help='enabled sensors, e.g. 1,2,4 or none')
    p.add_argument('--only-connected', type=on_off, help='measure only while a client is connected: on / off')
    p.add_argument('--read', action='store_true', help='also read the current temperatures')
    p.set_defaults(func=cmd_config)

    args = parser.parse_args()
    if args.timeout is None:
        args.timeout = 10.0 if args.command == 'scan' else 120.0
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.WARNING)
    try:
        return asyncio.run(args.func(args))
    except KeyboardInterrupt:
        return 130
    except (LinkError, ValueError) as exc:
        print(f'Error: {exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
