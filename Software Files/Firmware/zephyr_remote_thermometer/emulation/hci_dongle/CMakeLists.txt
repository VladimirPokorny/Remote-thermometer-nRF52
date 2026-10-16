# SPDX-License-Identifier: Apache-2.0
#
# Virtual BLE dongle for the emulator: Zephyr's hci_uart sample (BLE controller with H4 HCI on uart0).
# A PC-side host stack (Bumble) drives it through the uart0 TCP socket.

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(hci_dongle)

target_sources(app PRIVATE ${ZEPHYR_BASE}/samples/bluetooth/hci_uart/src/main.c)
