"""
Temperature logger for the remote thermometer, using the Bumble BLE host stack.

Same job as BLE_Connect.py (read the four temperatures, print them, append them to temperature.csv),
but instead of the PC's Bluetooth adapter it drives an HCI controller through any Bumble transport.
That lets it talk to the thermometer running in the nrf52_emulator: a virtual dongle node (Zephyr
hci_uart, see Firmware/zephyr_remote_thermometer/emulation/hci_dongle) exposes its HCI UART as a TCP
socket.

It subscribes to the "all sensors" characteristic and logs one row per measurement round, when the
thermometer notifies it. With --poll (or firmware without that characteristic) it reads the four
sensor characteristics every --interval seconds instead, like BLE_Connect.py.

    python BLE_Connect_bumble.py                                  # emulator dongle on tcp 127.0.0.1:9000
    python BLE_Connect_bumble.py --poll --interval 2
    python BLE_Connect_bumble.py --transport usb:0                # a real HCI dongle (e.g. nRF52840 with hci_usb)
"""
import argparse
import asyncio
import contextlib
import csv
import logging
from datetime import datetime

from bumble.core import UUID, AdvertisingData
from bumble.device import Device, Peer
from bumble.hci import Address
from bumble.transport import open_transport

DEVICE_NAME = 'Remote_Thermometer'

SERVICE_UUID = UUID('0000ABCD-5468-616E-6B73-4D6F6D446164')
TEMPERATURE_UUIDS = [
    UUID('00001000-5468-616E-6B73-4D6F6D446164'),
    UUID('00002000-5468-616E-6B73-4D6F6D446164'),
    UUID('00003000-5468-616E-6B73-4D6F6D446164'),
    UUID('00004000-5468-616E-6B73-4D6F6D446164'),
]
# All four temperatures in one value (4 x int32), notified once per measurement round
ALL_UUID = UUID('00005000-5468-616E-6B73-4D6F6D446164')

# Value of a characteristic before its sensor's first measurement (or of a disabled sensor)
NO_MEASUREMENT = bytes([0xDE, 0xAD, 0xBE, 0xEF])

header = ['time', 'T1', 'T2', 'T3', 'T4']


def decode(value: bytes):
    """int32 little endian, milli-degrees Celsius -> degrees, or None if there is no measurement."""
    if bytes(value) == NO_MEASUREMENT:
        return None
    return int.from_bytes(value, 'little', signed=True) / 1000


def log_row(csv_file: str, temperatures: list) -> None:
    """Print one measurement round and append it to the CSV file."""
    for i, temperature in enumerate(temperatures):
        if temperature is None:
            print(f'Temperature {i + 1}: -')
        else:
            print(f'Temperature {i + 1}: {temperature:.3f}')
    print('')

    with open(csv_file, 'a', encoding='UTF8', newline='\n') as file:
        csv.writer(file).writerow([datetime.now().strftime("%H:%M:%S"), *temperatures])


async def find_thermometer(device: Device, timeout: float) -> Address:
    found = asyncio.get_running_loop().create_future()

    def on_advertisement(advertisement):
        name = advertisement.data.get(AdvertisingData.COMPLETE_LOCAL_NAME)
        if name == DEVICE_NAME and advertisement.is_connectable and not found.done():
            print(f'Found {DEVICE_NAME} at {advertisement.address} (RSSI {advertisement.rssi})')
            found.set_result(advertisement.address)

    device.on('advertisement', on_advertisement)
    await device.start_scanning(legacy=True, active=False)
    try:
        return await asyncio.wait_for(found, timeout)
    finally:
        device.remove_listener('advertisement', on_advertisement)
        await device.stop_scanning(legacy=True)


class ConnectionLost(Exception):
    """The HCI transport went away, e.g. the emulator was stopped or its project reloaded."""


RETRY_DELAY = 3.0  # seconds between reconnection attempts


async def open_controller(transport: str, timeout: float):
    """
    Open the HCI transport and initialize the controller. Retries while the transport can't be opened
    (emulator not started yet) or the controller doesn't answer (emulation paused, or the dongle firmware
    hadn't started yet and its UART dropped the HCI Reset).
    """
    while True:
        print(f'Opening HCI transport {transport}')
        try:
            hci_transport = await open_transport(transport)
        except OSError as exc:
            print(f'Cannot open {transport} ({exc.strerror or exc}); is the emulator running? Retrying...')
            await asyncio.sleep(RETRY_DELAY)
            continue
        device = Device.with_hci('BLE_Connect', Address('F0:F1:F2:F3:F4:F5'),
                                 hci_transport.source, hci_transport.sink)
        try:
            await asyncio.wait_for(device.power_on(), timeout)
            return hci_transport, device
        except TimeoutError:
            print(f'No answer from the HCI controller within {timeout:.0f} s '
                  '(is the emulation running?), retrying...')
            await hci_transport.close()


async def log_connected(device: Device, interval: float, csv_file: str, scan_timeout: float, poll: bool):
    """Find the thermometer, connect and log until the BLE connection drops."""
    print(f'Scanning for {DEVICE_NAME}...')
    address = await find_thermometer(device, scan_timeout)

    connection = await device.connect(address)
    print("SUCCESFULLY CONNECTED")
    disconnected = asyncio.Event()
    connection.on('disconnection', lambda reason: disconnected.set())

    peer = Peer(connection)
    await peer.discover_service(SERVICE_UUID)
    await peer.discover_characteristics()
    all_sensors = peer.get_characteristics_by_uuid(ALL_UUID)

    if all_sensors and not poll:
        # One notification per measurement round, sent by the thermometer when all sensors are read
        rounds: asyncio.Queue = asyncio.Queue()
        await peer.subscribe(all_sensors[0], lambda value: rounds.put_nowait(bytes(value)))
        print('Waiting for measurements (notifications)')
        lost = asyncio.ensure_future(disconnected.wait())
        try:
            while True:
                got = asyncio.ensure_future(rounds.get())
                await asyncio.wait({got, lost}, return_when=asyncio.FIRST_COMPLETED)
                if not got.done():
                    got.cancel()
                    break
                value = got.result()
                log_row(csv_file, [decode(value[i:i + 4]) for i in range(0, 16, 4)])
        finally:
            lost.cancel()
        print('Disconnected')
        return

    if not all_sensors:
        print('Thermometer firmware without the "all sensors" characteristic: polling the sensors')
    characteristics = []
    for uuid in TEMPERATURE_UUIDS:
        found = peer.get_characteristics_by_uuid(uuid)
        if not found:
            raise RuntimeError(f'Characteristic {uuid} not found')
        characteristics.append(found[0])

    while not disconnected.is_set():
        log_row(csv_file, [decode(await c.read_value()) for c in characteristics])
        try:
            await asyncio.wait_for(disconnected.wait(), interval)
        except TimeoutError:
            pass

    print('Disconnected')


async def log_once(transport: str, interval: float, csv_file: str, scan_timeout: float, hci_timeout: float,
                   poll: bool):
    """One HCI session: returns when the BLE connection drops, raises ConnectionLost if the transport does."""
    hci_transport, device = await open_controller(transport, hci_timeout)
    async with hci_transport:
        session = asyncio.create_task(log_connected(device, interval, csv_file, scan_timeout, poll))
        lost = hci_transport.source.terminated
        try:
            await asyncio.wait({session, lost}, return_when=asyncio.FIRST_COMPLETED)
        finally:
            if not session.done():
                session.cancel()
                with contextlib.suppress(asyncio.CancelledError, Exception):
                    await session
        if session.done() and not session.cancelled():
            session.result()  # re-raise errors from the logging session
            return
        raise ConnectionLost()


async def run(transport: str, interval: float, csv_file: str, scan_timeout: float, hci_timeout: float,
              poll: bool):
    with open(csv_file, 'w', encoding='UTF8', newline='\n') as file:
        csv.writer(file).writerow(header)

    # Keep logging across disconnections and emulator restarts (e.g. "Reload project")
    while True:
        try:
            await log_once(transport, interval, csv_file, scan_timeout, hci_timeout, poll)
        except ConnectionLost:
            print('Connection to the HCI controller lost (emulator stopped or reloaded?), reconnecting...')
        except TimeoutError:
            print(f'{DEVICE_NAME} not found within {scan_timeout:.0f} s, retrying...')
        await asyncio.sleep(RETRY_DELAY)


def main():
    parser = argparse.ArgumentParser(description='Log temperatures from the remote thermometer over BLE (Bumble).')
    parser.add_argument('--transport', default='tcp-client:127.0.0.1:9000',
                        help='Bumble HCI transport (default: emulator dongle on tcp-client:127.0.0.1:9000)')
    parser.add_argument('--poll', action='store_true',
                        help='read the four sensors every --interval s instead of waiting for notifications')
    parser.add_argument('--interval', type=float, default=1.0, help='with --poll: seconds between readings (default 1)')
    parser.add_argument('--csv', default='temperature.csv', help='output file (default temperature.csv)')
    parser.add_argument('--scan-timeout', type=float, default=120.0,
                        help='seconds to look for the thermometer (default 120, the emulator is slow)')
    parser.add_argument('--hci-timeout', type=float, default=15.0,
                        help='seconds to wait for the HCI controller before reconnecting (default 15)')
    parser.add_argument('--verbose', action='store_true', help='Bumble debug logging')
    args = parser.parse_args()

    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.WARNING)
    try:
        asyncio.run(run(args.transport, args.interval, args.csv, args.scan_timeout, args.hci_timeout, args.poll))
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
