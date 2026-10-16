"""
BLE interface of the remote thermometer (firmware: Software Files/Firmware/zephyr_remote_thermometer,
src/temp_service.h): UUIDs and the byte formats of its characteristics. No BLE code here.
"""
from __future__ import annotations

from dataclasses import dataclass

DEVICE_NAME = 'Remote_Thermometer'

_BASE = '-5468-616E-6B73-4D6F6D446164'
SERVICE_UUID = '0000ABCD' + _BASE
SENSOR_UUIDS = ['00001000' + _BASE, '00002000' + _BASE, '00003000' + _BASE, '00004000' + _BASE]
ALL_UUID = '00005000' + _BASE        # 4 x int32, read + notify once per measurement round
CONFIG_UUID = '00006000' + _BASE     # 8 bytes, read + write

SENSOR_COUNT = 4
NO_MEASUREMENT = bytes([0xDE, 0xAD, 0xBE, 0xEF])   # before the first round, or sensor disabled

INTERVAL_MIN_MS = 100
INTERVAL_MAX_MS = 3_600_000
CONFIG_VERSION = 1
FLAG_MEASURE_ONLY_CONNECTED = 0x01


def decode_temperature(value: bytes) -> float | None:
    """int32 little endian in milli-degrees Celsius -> degrees, None if not measured."""
    value = bytes(value)
    if value == NO_MEASUREMENT:
        return None
    return int.from_bytes(value, 'little', signed=True) / 1000


def decode_all(value: bytes) -> list[float | None]:
    """The "all sensors" value (16 bytes) -> four temperatures."""
    value = bytes(value)
    if len(value) != 4 * SENSOR_COUNT:
        raise ValueError(f'"all sensors" value must be {4 * SENSOR_COUNT} bytes, got {len(value)}')
    return [decode_temperature(value[i:i + 4]) for i in range(0, len(value), 4)]


@dataclass
class Config:
    """Runtime configuration of the thermometer (Config characteristic)."""
    interval_ms: int
    sensors: frozenset[int]              # enabled sensors, 1..4
    measure_only_connected: bool

    def check(self) -> None:
        if not INTERVAL_MIN_MS <= self.interval_ms <= INTERVAL_MAX_MS:
            raise ValueError(f'interval must be {INTERVAL_MIN_MS} .. {INTERVAL_MAX_MS} ms, got {self.interval_ms}')
        bad = [s for s in self.sensors if not 1 <= s <= SENSOR_COUNT]
        if bad:
            raise ValueError(f'sensors are numbered 1 .. {SENSOR_COUNT}, got {bad}')

    def to_bytes(self) -> bytes:
        self.check()
        mask = sum(1 << (s - 1) for s in self.sensors)
        flags = FLAG_MEASURE_ONLY_CONNECTED if self.measure_only_connected else 0
        return bytes([CONFIG_VERSION, mask, flags, 0]) + self.interval_ms.to_bytes(4, 'little')

    @classmethod
    def from_bytes(cls, value: bytes) -> Config:
        value = bytes(value)
        if len(value) != 8:
            raise ValueError(f'config value must be 8 bytes, got {len(value)}')
        if value[0] != CONFIG_VERSION:
            raise ValueError(f'unknown config version {value[0]} (this app knows {CONFIG_VERSION})')
        return cls(interval_ms=int.from_bytes(value[4:8], 'little'),
                   sensors=frozenset(s for s in range(1, SENSOR_COUNT + 1) if value[1] & (1 << (s - 1))),
                   measure_only_connected=bool(value[2] & FLAG_MEASURE_ONLY_CONNECTED))

    def describe(self) -> str:
        sensors = ', '.join(str(s) for s in sorted(self.sensors)) or 'none'
        return (f'Sampling interval:          {self.interval_ms} ms\n'
                f'Enabled sensors:            {sensors}\n'
                f'Measure only when connected: {"yes" if self.measure_only_connected else "no"}')
