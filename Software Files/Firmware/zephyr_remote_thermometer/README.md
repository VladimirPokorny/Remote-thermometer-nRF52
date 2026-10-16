# Remote thermometer: Zephyr / nRF Connect SDK port

Port of the nRF5 SDK 17.1.0 + S140 firmware in [`../ble_remote_thermometer`](../ble_remote_thermometer) to
nRF Connect SDK v3.4 (Zephyr 4.4). It has the same function and keeps the BLE interface, so existing clients
such as [`BLE_Connect.py`](../../BLE%20Connect%20app/BLE_Connect.py) keep working. It is tuned for battery
life, see [Power consumption](#power-consumption).

It also runs without hardware in the nrf52_emulator workbench (Renode), with a model of the thermometer PCB
(four MAX31865 chips and the channel LEDs).

## Build

Requires NCS v3.4.0 in `C:\ncs` (the toolchain is found automatically). There is one board for everything,
`nrf52833dk/nrf52833` with the PCB pins: the hardware and the emulator build share the sources, `prj.conf`
and the board overlay, and differ only by two small files in `emulation\`.

```powershell
.\build.ps1                     # emulator build
.\build.ps1 -Target hardware    # thermometer PCB
.\build.ps1 -Target client      # BLE test client for a second emulator node
.\build.ps1 -Target dongle      # virtual BLE dongle for PC apps using Bumble (see below)
.\build.ps1 -Config Debug       # debug configuration (combine with -Target emulator / hardware)
.\build.ps1 -Pristine           # clean rebuild (combine with -Target)
```

Output goes to `<repo>\build\<target>\`, because both the Zephyr build and Renode break on the space in
`Software Files`. The script builds the sources through a temporary `subst` drive for the same reason.

| File | Hardware | Emulator | Content |
| --- | --- | --- | --- |
| [`prj.conf`](prj.conf) | ✓ | ✓ | the thermometer: settings, BLE, flash storage, power management, no logging |
| [`boards/nrf52833dk_nrf52833.overlay`](boards/nrf52833dk_nrf52833.overlay) | ✓ | ✓ | PCB pins, sensors (R0, reference resistor), LDO regulator |
| [`emulation/emulator.conf`](emulation/emulator.conf) | | ✓ | Kconfig differences, see [Hardware vs. emulator build](#hardware-vs-emulator-build) |
| [`emulation/emulator.overlay`](emulation/emulator.overlay) | | ✓ | devicetree differences |
| [`config/debug.conf`](config/debug.conf) | Debug | Debug | logging, `-Og`, asserts, thread info |
| [`config/debug_hardware.conf`](config/debug_hardware.conf) | Debug | | log over SEGGER RTT |
| [`config/debug_emulator.conf`](config/debug_emulator.conf) | | Debug | log over UART0 (the emulator's Terminal tab) |

The hardware build is what a plain `west build -b nrf52833dk/nrf52833` produces too; the emulator build adds
`-DEXTRA_CONF_FILE=emulation/emulator.conf -DEXTRA_DTC_OVERLAY_FILE=emulation/emulator.overlay`.

| Target | Board | Output |
| --- | --- | --- |
| `hardware` | `nrf52833dk/nrf52833` | `build\hardware\zephyr_remote_thermometer\zephyr\zephyr.hex` |
| `emulator` | `nrf52833dk/nrf52833` | `build\emulator\zephyr_remote_thermometer\zephyr\zephyr.elf` |
| `client` | `nrf52833dk/nrf52833` | `build\client\thermometer_client\zephyr\zephyr.elf` |
| `dongle` | `nrf52833dk/nrf52833` | `build\dongle\hci_dongle\zephyr\zephyr.elf` |

### Release and Debug

| | Release (default) | Debug (`-Config Debug`) |
| --- | --- | --- |
| Logging | none, no UART, no boot banner | emulator: UART (Terminal tab); hardware: SEGGER RTT |
| Code | optimized for size | `-Og`, asserts, thread info for the debugger / Tasks view |
| Use | the battery-powered thermometer; power measurements in the emulator's Power tab | stepping, reading logs |

Both configurations build into the same `build\<target>` folder, so the emulator projects load whichever was
built last (**↻ Reload project** after a rebuild). Build messages end with `Configuration: emulator, Release`
(or `hardware`, `Debug`). In Release the emulator's Terminal stays empty; watch the values with the logger or
the test client instead.

### Hardware vs. emulator build

Renode only models the nRF52840. The emulator runs the nRF52833 firmware on that model: the peripherals sit at
the same addresses, and the nRF52833's 512 KB flash and 128 KB RAM (what the linker uses) fit inside the
nRF52840's. The emulator build therefore keeps the board, the pins and the application code of the hardware
build, and changes only what Renode can't run:

| # | What | Hardware build | Emulator build | Why | Where |
| --- | --- | --- | --- | --- | --- |
| 1 | BLE link layer | Nordic SoftDevice Controller (NCS default) | Zephyr's open-source link layer (`CONFIG_BT_LL_SW_SPLIT`, `zephyr,bt-hci = &bt_hci_controller`) | the SoftDevice Controller / MPSL doesn't run in Renode | `emulator.conf`, `emulator.overlay` |
| 2 | SPI for the MAX31865 | SPIM2 (EasyDMA, `nordic,nrf-spim`) | SPI2 legacy mode (`nordic,nrf-spi`), same pins and 4 MHz | Renode models SPI2 only without EasyDMA. The legacy driver moves every byte by interrupt, so the emulator's CPU time per round is a bit higher than the hardware's | `emulator.overlay` |
| 3 | Bonds and runtime configuration | stored in flash (settings + NVS), survive a reset | RAM only: every restart begins with the Kconfig defaults and no bonds | Renode has no NVMC (flash controller) model; flash writes would hang | `emulator.conf` |
| 4 | Reset pin | P0.18 is nRESET (`gpio-as-nreset`, set in UICR at the first boot) | `gpio-as-nreset` removed | Renode reads UICR.PSELRESET as P0.21; the startup code would rewrite it to P0.18 and reset forever | `emulator.overlay` |
| 5 | Debug log (Debug only) | SEGGER RTT | UART0, TX only on P0.06 (RX would be P0.08 = EN_3) | Renode has no RTT. Release has no UART in either build | `config\debug_*.conf`, `emulator.overlay` |
| 6 | Sensors enabled by default | 1, 2 | 1, 2, 3, 4 | test convenience only: the model has four chips. Delete the two lines to measure exactly what the hardware does | `emulator.conf` |

Everything else is the same in both builds: board, PCB pins, sensor parameters, LDO regulator, runtime power
management (SPI suspended between transfers), entropy (the RNG peripheral; the nRF52833 has no CryptoCell),
connection and advertising parameters, BLE interface, and the code.

What the emulator itself does differently from the PCB (not a build setting):

- The `thermo` node is the bare `nrf52840` Renode machine, without the DK's buttons and LEDs, which would sit on
  the PCB pins (buttons on P0.12 = EN_1 and P0.24 = CS_2). The PCB is added by
  [`max31865.repl`](emulation/max31865.repl): the four MAX31865 on SPI2 and LED1..LED4 on EN_1..EN_4. On the
  PCB the LEDs hang on the switched channel supply, so in the emulator they light while a sensor is measured.
- No encryption (CCM/AAR aren't modelled): pairing and encrypted links can't be tested.
- No current measurement: the Power tab estimates from the CPU sleep state, the radio and the load-switch pins
  (5.8 mA per powered channel, nRF52833 LDO currents).
- The temperatures come from the model (set from the Monitor tab), not from PT100 sensors.

## Run in the emulator

The emulator project files in [`emulation/`](emulation) load everything in one step: nodes, ELFs, the sensor
model and the dongle's TCP port. Build the firmware first (`.\build.ps1`, plus `-Target dongle` or
`-Target client`), then pick a project:

| Project file | Nodes |
| --- | --- |
| [`thermometer.nrfemu.json`](emulation/thermometer.nrfemu.json) | `thermo` + `dongle` (HCI on TCP 9000) for the PC logger below |
| [`thermometer_client.nrfemu.json`](emulation/thermometer_client.nrfemu.json) | `thermo` + `client` (Zephyr test client, prints notifications) |

```powershell
C:\Programming\nrf52_emulator\start.ps1 -Project "C:\Programming\Remote-thermometer-nRF52\Software Files\Firmware\zephyr_remote_thermometer\emulation\thermometer.nrfemu.json"
```

Or open the file with **Open project** on the emulator's welcome screen. The nodes start running right away.
Once a client is connected, the thermometer measures every second. With a Release build (the default) it is
silent: watch the values with the PC logger or in the test client's Terminal, and the current in the
thermometer's **Power** tab. With a Debug build (`.\build.ps1 -Config Debug`) its own Terminal shows
`Sensor 1: 21.507 C` .. `Sensor 4: -20.001 C`. In both, the Board tab shows the PCB's LED1..LED4 lighting one
after the other while each sensor is measured. After rebuilding the firmware, click **↻ Reload project** in the header.

Change the simulated temperatures from the Monitor tab at any time:

```
mach set "thermo"
sysbus.spi2.max31865 Temperature1 36.6
sysbus.spi2.max31865 SetTemperature 2 -10
sysbus.spi2.max31865 SetFault 1 0x84      # inject fault status bits, 0 clears
```

The model starts at 21.5, 85, 150 and -20 °C. Readings differ from the set value by up to about 0.03 °C,
the MAX31865's ADC resolution.

To set things up by hand instead: add a node with the ELF and the bare `nrf52840` board (the DK's buttons
would sit on the PCB pins), then in the Monitor tab
`mach set "thermo"` and `include @C:/Programming/Remote-thermometer-nRF52/Software\ Files/Firmware/zephyr_remote_thermometer/emulation/max31865.resc`
(Renode needs the space escaped as `\ `).

## Log and configure from the PC (thermometer.py)

[`thermometer.py`](../../BLE%20Connect%20app/thermometer.py) in `Software Files\BLE Connect app` logs the
temperatures and reads or changes the configuration (see its [README](../../BLE%20Connect%20app/README.md)).

The PC's Bluetooth adapter can't reach the emulated radio. Instead, a virtual dongle node gives a PC-side
BLE host stack ([Bumble](https://github.com/google/bumble)) its own emulated controller:

```
thermometer.py ──HCI over TCP :9000──▶ "dongle" node (hci_uart) ──Renode BLE medium──▶ "thermo" node
```

1. Build once: `.\build.ps1` and `.\build.ps1 -Target dongle`.
2. Set up the Python app once (in `Software Files\BLE Connect app`):
   ```powershell
   python -m venv .venv
   .\.venv\Scripts\pip install -r requirements.txt
   ```
3. Load [`emulation/thermometer.nrfemu.json`](emulation/thermometer.nrfemu.json) in the emulator (see above).
4. Use the app:
   ```powershell
   .\.venv\Scripts\python thermometer.py log                                   # one line per round + CSV
   .\.venv\Scripts\python thermometer.py config --interval 2000 --sensors 1,3  # change the configuration
   ```

For the real thermometer add `--transport pc` (the PC's Bluetooth, through bleak).

The logger can be started before or after the emulator and keeps running across **Reload project**: whenever
the emulator isn't there, isn't running yet or restarts, it says so and reconnects.

Notes:

- With two nodes on the BLE medium the emulation runs slower than real time. In the test, readings arrived
  about every 3.5 s of wall-clock time, and the CSV timestamps are wall-clock time.
- `--transport usb:0` drives a real HCI dongle through Bumble instead (for example an nRF52840 dongle with
  Zephyr's `hci_usb`). Not tested; on Windows the dongle needs the WinUSB driver (e.g. installed with Zadig).
- Only one emulator instance can run at a time with the BLE medium on: `emulation LogBLETraffic` holds a
  named pipe, and a second Renode crashes with "All pipe instances are busy".

## Configuration

The sampling interval, the enabled sensors and "measure only when connected" are a **runtime
configuration**: a BLE client changes them through the Config characteristic (`thermometer.py config`), they
take effect at once, and the hardware build keeps them in flash (settings). The Kconfig options below are
their factory defaults, used until a client changes them (the emulator build starts from them at every
restart, because Renode has no flash controller model).

The thermometer settings are Kconfig options (defined in [`Kconfig`](Kconfig)). Set them in the
"Thermometer settings" block at the top of [`prj.conf`](prj.conf) for both builds, or in
[`emulation/emulator.conf`](emulation/emulator.conf) for the emulator only, then rebuild:

| Option | Default | Emulator build | Meaning |
| --- | --- | --- | --- |
| `CONFIG_APP_SAMPLING_INTERVAL_MS` | 1000 | 1000 | Time between two measurement rounds, 100 to 3600000 ms |
| `CONFIG_APP_MEASURE_ONLY_CONNECTED` | on | on | Sensors stay off while no client is connected; a connecting client gets a round at once |
| `CONFIG_APP_SENSOR_1` .. `_4` | 1, 2 on | all on | Sensors that are powered, read and published |
| `CONFIG_APP_ADV_FAST_TIMEOUT_S` | 30 | 30 | Advertise every 100 ms this long after boot/disconnect, 0 = never |
| `CONFIG_APP_ADV_SLOW_INTERVAL_MS` | 1000 | 1000 | Advertising interval after that |

One sensor takes about 77 ms to read (load switch, bias settling, conversion), so a round of four takes about
310 ms. A shorter interval makes the rounds run back to back. `west build -d <build dir> -t menuconfig` shows
the options under "Remote thermometer". The startup log prints the active values:
`Measuring temperature started: every 1000 ms, sensors 1 2 3 4.`

| Other settings | Where |
| --- | --- |
| R0 (PT100/PT1000), reference resistor, 3-wire, SPI pins, CS and EN pins | [`boards/nrf52833dk_nrf52833.overlay`](boards/nrf52833dk_nrf52833.overlay) |
| BLE name, preferred connection parameters (`CONFIG_BT_PERIPHERAL_PREF_*`), DIS manufacturer | [`prj.conf`](prj.conf) |
| Main regulator (LDO / DC-DC) | `boards/nrf52833dk_nrf52833.overlay`, see [Power consumption](#power-consumption) |

Pin mapping, the same in both builds (the emulator's sensor model is wired to these pins too):

| Signal | PCB (nRF52833) |
| --- | --- |
| SCK / MOSI / MISO | P0.26 / P0.29 / P0.30 |
| CS_1..CS_4 | P0.23, P0.24, P0.17, P0.31 |
| EN_1..EN_4 (load switches) | P0.12, P0.03, P0.08, P0.14 |

The overlay disables the nRF52833 DK peripherals on the PCB pins (UART0, I2C0, SPI1, Arduino SPI). Neither
build logs in Release; the Debug configuration logs over RTT on hardware, like the original firmware.

## BLE interface

| | UUID | Properties |
| --- | --- | --- |
| Service | `0000ABCD-5468-616E-6B73-4D6F6D446164` | |
| Sensor 1..4 | `00001000-…`, `00002000-…`, `00003000-…`, `00004000-…` (same base) | read |
| All sensors | `00005000-5468-616E-6B73-4D6F6D446164` | read, notify |
| Config | `00006000-5468-616E-6B73-4D6F6D446164` | read, write |

Sensor value: int32, little endian, milli-degrees Celsius. "All sensors" is the four values in a row (16 bytes)
and is notified **once per measurement round**, after all enabled sensors were read. Before the first
measurement, and for disabled sensors, a value is `DE AD BE EF`. Advertising includes the name
`Remote_Thermometer` and the Device Information Service.

Config, 8 bytes, little endian, always written as a whole:

| Byte | Content |
| --- | --- |
| 0 | Format version, 1 |
| 1 | Sensor mask: bit n = sensor n+1 enabled |
| 2 | Flags: bit 0 = measure only while a client is connected |
| 3 | Reserved, 0 |
| 4..7 | Sampling interval in ms, 100 .. 3600000 |

A valid write starts a measurement round with the new settings at once. Out-of-range values are rejected with
ATT error 0x13 (value not allowed), a wrong length with 0x0D. The characteristic needs no pairing, so anyone
in range can change it; requiring an encrypted link is a one-line change (`BT_GATT_PERM_WRITE_ENCRYPT`), but
encryption can't be tested in Renode.

Compared with the nRF5 SDK firmware the UUIDs and value format are unchanged, but the per-sensor
characteristics no longer notify. Clients that read them (like `BLE_Connect.py`) work as before. Clients that
subscribed to them should subscribe to "All sensors" instead.

## Power consumption

The sensor channels dominate: while a channel is powered (about 77 ms per sensor and round), its MAX31865,
bus buffer, RTD bias current and status LED draw several mA, against tens of µA for the nRF52833 between
radio events. In order of impact:

1. **Sampling interval** (`CONFIG_APP_SAMPLING_INTERVAL_MS`). The channel energy scales with the number of
   rounds: 10 s instead of 1 s cuts it tenfold.
2. **Status LEDs.** LED1..LED4 on the PCB hang on the switched channel supply (330 Ω, about 4 mA each while the
   channel is on), more than the MAX31865 and the RTD together. Firmware can't switch them off: they light
   whenever their channel is powered. Open solder bridges SB1..SB4 on battery-powered units. (The emulator
   models them the same way, on EN_1..EN_4.)
3. **Enabled sensors** (`CONFIG_APP_SENSOR_n`): disabled channels are never powered.
4. **No client, no measurement** (`CONFIG_APP_MEASURE_ONLY_CONNECTED`): while nobody is connected the
   thermometer only advertises.

What the firmware does:

- **The CPU sleeps** between events in System ON idle, the deepest sleep that keeps the RTC running for the
  sampling timer and keeps the BLE connection (a few µA). The measurement thread sleeps on a kernel timer, and
  every delay is a sleep, not a busy-wait. System OFF would drop the connection and could only be woken by a
  pin or reset, so it doesn't fit periodic measurement.
- **One BLE packet per round**: the four temperatures go out in a single notification after the last sensor is
  read, instead of four.
- **Low-duty connection**: the thermometer asks for a 400–500 ms interval with peripheral latency 1, so an idle
  link wakes the radio about once a second (the nRF5 SDK firmware used 100–200 ms, latency 0). A notification
  leaves at the next interval (≤ 500 ms). A polling client waits up to about 1 s per read.
- **Advertising** every 100 ms only for 30 s after boot or a disconnect, then every 1 s.
- **Channels** are powered only while measured. Each MAX31865 is configured and converted with 3 SPI transfers
  right after power-up: write configuration with bias on, start a one-shot conversion, read the result. The
  load switch then cuts the bias together with the chip.
- **No logging and no UART** in the Release configuration (both builds), and SPI is suspended between
  transfers (runtime PM).

Measured in Renode (thermometer + test client, 4 sensors, 1 s interval, 20 s window; Renode can't measure
current, so radio packets and executed CPU instructions stand in for radio and CPU on-time):

| | Radio packets sent by the thermometer | CPU instructions |
| --- | --- | --- |
| Before (4 notifications/round, 100–200 ms interval) | 140 | 3.86 M |
| After | 40 (−71 %) | 2.41 M (−37 %) |

These were measured with an earlier emulator build that still logged to its UART.

**DC/DC regulator.** The hardware build uses the LDO, like the nRF5 SDK firmware: the nRF52833 DK board file
would switch on the DC/DC converter, which needs an inductor at the DCC pin, and the PCB has none. If the
BL653 module integrates it (check its datasheet), set `regulator-initial-mode = <NRF5X_REG_MODE_DCDC>` in
`boards/nrf52833dk_nrf52833.overlay`: about a third less current while the CPU or radio runs.

## Differences from the nRF5 SDK version

- **Measurement thread.** A Zephyr thread replaces the `app_timer` flag polled in the main loop. Delays
  (`k_msleep`) sleep instead of busy-waiting.
- **Values are always current.** A read returns the latest measurement even when nobody is subscribed. Before,
  the stored value was only updated through notifications while connected.
- **Sensor 4 enable fixed.** The original main loop checked `SENSOR_3_ENABLE` for sensor 4.
- **`temperature_new()`** no longer uses an uninitialised loop counter.
- **No System OFF after 60 s of advertising.** The device advertises until a client connects and resumes
  advertising after a disconnect. There are no BSP buttons either: no wake-up button, bond erase or whitelist.
- **LEDs.** The original's BSP indications (LED1 advertising/connected, LED4 toggling every round) are gone.
  Only the PCB's channel LEDs remain, lit by the load switches.
- **Chips are configured after every power-up.** The load switch powers the whole channel, so a MAX31865
  loses its configuration between measurements. The original configured all four chips once at boot, while
  they were unpowered. That only worked because 4-wire with the 60 Hz filter is the chip's reset state; a
  3-wire sensor would have been measured wrong.
- **One notification per round** on the new "All sensors" characteristic, instead of one per sensor.
- **Power-saving connection parameters, advertising and no logging** in the hardware build (see above).
- **Bonds.** The hardware build stores bonds in flash (settings/NVS). The emulator build keeps them in RAM
  only, because Renode doesn't model NVMC.

## Emulator limitations

See [Hardware vs. emulator build](#hardware-vs-emulator-build) for everything the emulator build changes.

- Renode doesn't model the CCM/AAR encryption peripherals, so pairing and encrypted links can't be tested
  there. The characteristics don't require encryption, and the test client doesn't pair.
- AAR warnings and `Unhandled short DisabledRSSIStop` in the Renode log are expected.
