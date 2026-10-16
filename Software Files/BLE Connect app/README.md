# BLE Connect app

PC tools for the remote thermometer.

| Script | What it does |
| --- | --- |
| `thermometer.py` | Log temperatures and configure the thermometer (Zephyr firmware). Works with the emulator and with the real device |
| `BLE_Connect.py` | Original logger (bleak): reads the four sensors in a loop. Works with both firmware versions |
| `ScanDiscover.py` | Lists BLE devices around the PC |
| `BLE_Connect_bumble.py` | Earlier logger through the emulator; replaced by `thermometer.py log` |

## Setup

```powershell
python -m venv .venv
.\.venv\Scripts\pip install -r requirements.txt
```

## thermometer.py

```powershell
.\.venv\Scripts\python thermometer.py scan                     # thermometers in range
.\.venv\Scripts\python thermometer.py log                      # one line per measurement round, + temperature.csv
.\.venv\Scripts\python thermometer.py config                   # show the configuration
.\.venv\Scripts\python thermometer.py config --interval 5000 --sensors 1,2,4 --only-connected off --read
```

| `config` option | Meaning |
| --- | --- |
| `--interval <ms>` | Sampling interval, 100 .. 3600000 ms |
| `--sensors 1,2,4` | Enabled sensors (`none` for all off); disabled ones read as `-` |
| `--only-connected on/off` | Measure only while a client is connected |
| `--read` | Also print the current temperatures |

A change takes effect at once: the thermometer starts a measurement round with the new settings. The real
device keeps the configuration in flash. The emulator build can't (Renode has no flash controller model), so
there it returns to the defaults from `prj.conf` after a restart.

`log` subscribes to the "all sensors" notification, so it gets exactly one value set per measurement round. It
keeps running across disconnects and emulator restarts and reconnects by itself.

### Which Bluetooth (`--transport`)

| `--transport` | Use |
| --- | --- |
| `tcp-client:127.0.0.1:9000` (default) | The emulator: the `dongle` node of `emulation/thermometer.nrfemu.json` |
| `pc` | The real thermometer through the PC's Bluetooth (bleak) |
| `usb:0`, `serial:COM5,1000000` | A Bumble HCI dongle, e.g. an nRF52840 dongle with Zephyr's `hci_usb` / `hci_uart` |

Other options: `--name` (default `Remote_Thermometer`), `--address` (skip the scan), `--timeout`, `--verbose`.

The BLE interface (UUIDs, byte formats) is in `thermometer_protocol.py` and in the firmware's
`src/temp_service.h`.
