# SDM26 ESP32-S3 Data Logger Firmware

Firmware for the Sun Devil Motorsports SDM26 vehicle data logger. The logger runs on an ESP32-S3 based custom PCB and records analog sensors, vehicle CAN data, GNSS data, and onboard power monitor data to a removable microSD card.

This README is intended for new firmware and electrical team members who need to build, flash, wire, validate, and debug the logger.

## Project Overview

- Team: Sun Devil Motorsports FSAE
- Board/project: SDM26 Datalogger
- Hardware reference: `SDM26LoggerV4.0.kicad_sch`, revision 4.0, dated 2025-06-09
- MCU: ESP32-S3-WROOM-1-N8R8
- Firmware framework: ESP-IDF
- ESP-IDF target: `esp32s3`
- Current lock file IDF version: 5.5.1, from `dependencies.lock`

The firmware initializes the SD card, GNSS UART, debug UART, DTC tracking, I2C power monitor, ADC, and CAN receiver. A 10 ms logging task builds a fixed-size binary record and writes it through a ring buffer to the current `.benji2` log file on the SD card.

## Features

- 100 Hz log buffer generation (`REFRESH_MS = 10` in `main/main.c`)
- microSD logging at mount point `/sdcard`
- Binary `.benji2` log files with a channel-name header
- UART command menu over the programming/debug serial port
- CAN receive path for wheel boards, IMU, ECU streams, strain gauges, and logger commands
- GNSS NMEA parsing for position, speed, date/time, and fix quality
- INA260 bus voltage/current readings over I2C
- Diagnostic trouble code (DTC) status tracking for CAN devices
- Temporary Wi-Fi SoftAP and HTTP file browser/download server when no CAN messages are seen

## Hardware Requirements

### Logger Board

| Item | Value |
| --- | --- |
| Board | SDM26 Datalogger |
| Custom PCB revision | 4.0 |
| MCU | ESP32-S3-WROOM-1-N8R8 |
| USB-to-UART programmer | CP2102N-A02-GQFN28 on `ESP_PROG` USB |
| GNSS module | u-blox NEO-F9P |
| ADC | ADC128S102, 8 channels |
| CAN transceiver | SN65HVD230 |
| microSD connector | Molex 1040310811 |
| Front/rear connectors | 23-pin Ampseal-style, value `1-776087-4` |

### Power Warnings

- Vehicle input enters through Front Ampseal J6 pin 1 (`+BATT`) and pin 2 (`GND`).
- Schematic note lists VCC as approximately 11-14 V, typical about 12.8 V.
- Do not connect raw battery voltage to analog inputs.
- Analog inputs are intended for 0-5 V sensors referenced to logger ground.
- Analog sensors must share logger ground.
- The 5 V SMPS output is noted as 5 V up to about 3 A, with system output current target about 2.5 A.
- Onboard 3.3 V max current is noted as about 0.5 A.
- Do not power external sensors beyond the regulator/current limits.

## Pinout / Wiring

### ESP32-S3 Pin Map

| ESP32-S3 pin | Schematic net | Firmware use |
| --- | --- | --- |
| GPIO4 | `ADC_CS` | ADC chip select, `main/adc.c` |
| GPIO5 | `ADC_CLK` | ADC SPI clock, `main/adc.c` |
| GPIO6 | `ADC_DOUT` | ADC MISO, `main/adc.c` |
| GPIO7 | `ADC_DIN` | ADC MOSI, `main/adc.c` |
| GPIO8 | `CAN_RTX` | CAN TX in firmware, see TODO below |
| GPIO17 | `CAN_TERM` | Switchable CAN termination control, TODO: not implemented in firmware |
| GPIO18 | `CAN_CTX` | CAN RX in firmware, see TODO below |
| GPIO19 | `UART2_TX` | GNSS UART TX from ESP, firmware uses UART1 |
| GPIO20 | `UART2_RX` | GNSS UART RX to ESP, firmware uses UART1 |
| GPIO35 | `USER_LED` | TODO: firmware command says LED toggle, but no GPIO implementation found |
| GPIO47 | `I2C1_SCL` | INA260 I2C SCL, `main/ina260.h` |
| GPIO48 | `I2C1_SDA` | INA260 I2C SDA, `main/ina260.h` |
| U0RXD/U0TXD | CP2102N UART | Flashing, monitor, UART command menu |
| GPIO9 | `SD_DAT1` | Schematic 4-bit SD pin, not used by firmware SDMMC 1-bit mode |
| GPIO10 | `SD_DAT0` | SDMMC D0, `main/sdcard.h` |
| GPIO11 | `SD_CLK` | SDMMC CLK, `main/sdcard.h` |
| GPIO12 | `SD_CMD` | SDMMC CMD, `main/sdcard.h` |
| GPIO13 | `SD_DAT3` | Schematic 4-bit SD pin, not used by firmware SDMMC 1-bit mode |
| GPIO14 | `SD_DAT2` | Schematic 4-bit SD pin, not used by firmware SDMMC 1-bit mode |

TODO: `main/can.c` comments say "Switched these, spotted a possible issue with schematic naming." The schematic lists GPIO18 as `CAN_CTX` and GPIO8 as `CAN_RTX`; firmware defines `TX GPIO_NUM_8` and `RX GPIO_NUM_18`. Verify against the PCB/transceiver before relying on CAN.

### Front Ampseal J6

| Pin | Net | Notes |
| --- | --- | --- |
| 1 | `+BATT` | Vehicle input power, about 11-14 V |
| 2 | `GND` | Vehicle/logger ground |
| 3 | `+5V` | Sensor/output power, observe current limits |
| 4 | `+5V` | Sensor/output power |
| 5 | NC / not labeled | TODO |
| 6 | NC / not labeled | TODO |
| 7 | `AIN0_EXT` | Analog input |
| 8 | `AIN1_EXT` | Analog input |
| 9 | NC / not labeled | TODO |
| 10 | `+5V` | Sensor/output power |
| 11 | `+5V` | Sensor/output power |
| 12 | `+5V` | Sensor/output power |
| 13 | NC / not labeled | TODO |
| 14 | NC / not labeled | TODO |
| 15 | NC / not labeled | TODO |
| 16 | `GND` | Ground |
| 17 | `GND` | Ground |
| 18 | `GND` | Ground |
| 19 | `GND` | Ground |
| 20 | `GND` | Ground |
| 21 | `GND` | Ground |
| 22 | `AIN2_EXT` | Analog input |
| 23 | `AIN3_EXT` | Analog input |

### Rear Ampseal J5

| Pin | Net | Notes |
| --- | --- | --- |
| 1 | `AIN6_EXT` | Analog input |
| 2 | `AIN7_EXT` | Analog input |
| 3 | `GND` | Ground |
| 4 | `GND` | Ground |
| 5 | `+5V` | Sensor/output power |
| 6 | `+5V` | Sensor/output power |
| 7 | `CAN-` | CAN bus low |
| 8 | `CAN+` | CAN bus high |
| 9 | NC / not labeled | TODO |
| 10 | NC / not labeled | TODO |
| 11 | NC / not labeled | TODO |
| 12 | `+5V` | Sensor/output power |
| 13 | `+5V` | Sensor/output power |
| 14 | `+5V` | Sensor/output power |
| 15 | NC / not labeled | TODO |
| 16 | `AIN4_EXT` | Analog input |
| 17 | `AIN5_EXT` | Analog input |
| 18 | `GND` | Ground |
| 19 | `GND` | Ground |
| 20 | NC / not labeled | TODO |
| 21 | NC / not labeled | TODO |
| 22 | `GND` | Ground |
| 23 | NC / not labeled | TODO |

## Data Sources

### Analog ADC Channels

The ADC is an ADC128S102 with 8 single-ended analog inputs. Inputs have 100 ohm series resistance, 33 nF to ground, and Schottky clamp protection to `+5VA`.

Firmware mapping is in `logBuffer_task()` in `main/main.c`:

| ADC channel | Logged field | Notes |
| --- | --- | --- |
| ADC0 / `AIN0` | `FRSHOCK` | Firmware-defined |
| ADC1 / `AIN1` | `RRSHOCK` | Firmware-defined |
| ADC2 / `AIN2` | `R_BRAKEPRESSURE` | Code comment: "Might be swapped" |
| ADC3 / `AIN3` | `RLSHOCK` | Firmware-defined |
| ADC4 / `AIN4` | `F_BRAKEPRESSURE` | Code comment: "Might be swapped" |
| ADC5 / `AIN5` | unused | TODO: assign or document spare channel |
| ADC6 / `AIN6` | `FLSHOCK` | Firmware-defined |
| ADC7 / `AIN7` | `STEERING` | Firmware-defined |

TODO: Confirm physical sensor-to-connector mapping and calibration constants. Steering display calibration exists only in UART debug command `S` in `main/uart.c`.

### CAN Data

CAN is configured in `main/can.c`:

- Bitrate: 1,000,000 bit/s
- Mode: listen-only enabled
- RX queue size: 256
- Transmit queue depth: 1

Main decoded CAN IDs are in `process_can_message()` in `main/main.c`:

| CAN ID | Source / meaning |
| --- | --- |
| `0x35F` | DRS byte |
| `0x360` | IMU gyro |
| `0x361` | IMU accel and IMU DTC response |
| `0x370` | Front-left wheel board RPM/object temp/ambient temp |
| `0x371`, `0x372` | Front-left tire temperature payloads, TODO: verify copy direction in code |
| `0x380` | Front-right wheel board RPM/object temp/ambient temp |
| `0x381`, `0x382` | Front-right tire temperature payloads, TODO: verify copy direction in code |
| `0x390` | Rear-right wheel board RPM/object temp/ambient temp |
| `0x391`, `0x392` | Rear-right tire temperature payloads, TODO: verify copy direction in code |
| `0x3A0` | Rear-left wheel board RPM/object temp/ambient temp |
| `0x3A1`, `0x3A2` | Rear-left tire temperature payloads, TODO: verify copy direction in code |
| `1000` | ECU engine stream 2 |
| `1002` | ECU engine stream 6 |
| `1003` | ECU engine stream 7 |
| `1004` | ECU engine stream 8 |
| `0x4E2` | Front-left strain gauge |
| `0x4E3` | Front-right strain gauge |
| `0x4E4` | Rear-right strain gauge |
| `0x4E5` | Rear-left strain gauge |
| `0x69E` | Telemetry/log file change command |

TODO: CAN termination appears switchable with ESP32 GPIO17 controlling a TLP175A relay and 120 ohm resistor. No firmware code currently configures GPIO17 or controls termination.

### GNSS

GNSS firmware is in `main/gnss.c`:

- Module: u-blox NEO-F9P
- UART port in firmware: `UART_NUM_1`
- ESP TX: GPIO19
- ESP RX: GPIO20
- Baud rate: 38400
- Data format: 8N1, no flow control
- Parser: NMEA sentence parsing for `$GNGGA`, `$GNRMC`, GSV, GSA, VTG, and GLL recognition

Logged GNSS fields:

- `GPS_LAT`: signed integer latitude in degrees times 10,000,000
- `GPS_LON`: signed integer longitude in degrees times 10,000,000
- `GPS_SPD`: ground speed converted from knots to mph in `parse_gnrmc()`
- `GPS_FIX`: GGA quality/fix field

TODO: Firmware does not configure the NEO-F9P over UBX. Confirm module default output rate and enabled NMEA messages.

### Power Monitor

INA260 I2C support is in `main/ina260.h` and `main/ina260.c`:

- I2C SCL: GPIO47
- I2C SDA: GPIO48
- I2C port: `I2C_NUM_0`
- I2C speed: 100 kHz
- INA260 address used by reads: `0x40`
- Logged fields: `CURRENT`, `BATTERY`

TODO: Confirm units. The current code logs raw register values from INA260 current and bus voltage registers.

## Firmware Architecture

Startup in `app_main()` (`main/main.c`):

1. Create log ring buffer.
2. Mount SD card and open the first log file.
3. Initialize GNSS UART.
4. Initialize UART0 command interface.
5. Initialize DTC devices.
6. Initialize I2C for INA260.
7. Initialize ADC SPI interface.
8. Initialize CAN receive path.
9. Start RTOS tasks.
10. Start the Wi-Fi/http log server only if no CAN messages have been received.

Main tasks are created in `main/tasks.c`:

| Task | Purpose |
| --- | --- |
| `can_rx` | Receive queued CAN frames and call decoder |
| `log buffer` | Build one `CH_COUNT` log record every 10 ms |
| `log flush` | Flush ring-buffer records to the SD file |
| `gnss_uart_task` | Process NMEA from the GNSS module |
| `uart_input` | Read UART0 user input |
| `uart_output` | Process UART0 command menu |
| `dtc_check` | Update DTC error states |

TODO: `can_init()` also creates `can_receive_task()`, and `tasks_start_all()` creates it again. Verify whether this duplicate task is intentional.

## Repository Structure

| Path | Purpose |
| --- | --- |
| `CMakeLists.txt` | Top-level ESP-IDF project definition, project name `logger` |
| `sdkconfig` | Active ESP-IDF configuration |
| `dependencies.lock` | ESP-IDF component manager lock file, target and IDF version |
| `main/` | Active firmware source |
| `main/main.c` | Startup, CAN message decode, log record assembly |
| `main/adc.c`, `main/adc.h` | ADC128S102 SPI-like readout |
| `main/can.c`, `main/can.h` | ESP-IDF TWAI/CAN receive setup |
| `main/sdcard.c`, `main/sdcard.h` | SDMMC mount, NVS log name/test number, log file writes |
| `main/gnss.c`, `main/gnss.h` | NEO-F9P UART and NMEA parser |
| `main/ina260.c`, `main/ina260.h` | I2C and INA260 register reads |
| `main/logger.c`, `main/logger.h` | Binary emplace helpers and ring buffer |
| `main/log_chnl.h` | Log channel byte layout and header generation |
| `main/uart.c`, `main/uart.h` | UART0 command menu/debug interface |
| `main/wifi.c`, `main/server.c` | SoftAP and HTTP SD-card file browser |
| `main/dtc.c`, `main/dtc.h` | CAN/DTC response-time monitoring |
| `components/ModuleCore/` | C++ reusable module core component; present but not used by current `main` component |
| `logger/main/main.c` | TODO: appears to be an old or separate source tree; not built by current root CMake |
| `component_test/` | TODO: inspect before relying on it |

No source `Kconfig` files or source `partitions.csv` were found. `sdkconfig` uses ESP-IDF's built-in single-app partition table, with `CONFIG_PARTITION_TABLE_FILENAME="partitions_singleapp.csv"`.

## Prerequisites

Install ESP-IDF for Windows and open an ESP-IDF terminal or run the ESP-IDF export script before building.

Recommended:

- ESP-IDF 5.5.1, matching `dependencies.lock`
- Python environment installed by ESP-IDF
- USB driver for CP210x/CP2102N
- A microSD card formatted FAT32/exFAT-compatible for ESP-IDF FAT VFS use. FAT32 is the safest default.

## Build Instructions

From the repository root:

```powershell
idf.py set-target esp32s3
idf.py build
```

If `idf.py` is not found, open an ESP-IDF command prompt or run the ESP-IDF export script for your install.

Useful cleanup command:

```powershell
idf.py fullclean
idf.py build
```

## Flash And Serial Monitor

Connect the `ESP_PROG` USB connector. The CP2102N connects to ESP32-S3 UART0 and the auto-reset/boot circuit.

Find the COM port in Device Manager, then flash and monitor:

```powershell
idf.py -p COMx flash monitor
```

Replace `COMx` with the actual port, for example `COM7`.

Serial monitor settings:

- Baud: 115200
- UART: ESP32-S3 UART0 through CP2102N
- Exit monitor: `Ctrl+]`

Common one-step command:

```powershell
idf.py -p COMx build flash monitor
```

## Configuration Options

Important constants are currently compile-time values:

| Setting | Value | File |
| --- | --- | --- |
| ESP-IDF target | `esp32s3` | `sdkconfig`, `dependencies.lock` |
| Flash size | 8 MB | `sdkconfig` |
| Flash mode | DIO | `sdkconfig` |
| Partition table | ESP-IDF single app | `sdkconfig` |
| Log period | 10 ms | `main/main.c` |
| Log extension | `.benji2` | `main/sdcard.h` |
| SD mount point | `/sdcard` | `main/sdcard.h` |
| SD bus mode | SDMMC 1-bit | `main/sdcard.c` |
| CAN bitrate | 1 Mbit/s | `main/can.c` |
| CAN mode | listen-only | `main/can.c` |
| GNSS baud | 38400 | `main/gnss.c` |
| UART command baud | 115200 | `main/uart.c` |
| I2C speed | 100 kHz | `main/ina260.h` |
| Wi-Fi SSID | `sdm26_logger` | `main/wifi.c` |
| Wi-Fi password | `sdmfsae26` | `main/wifi.c` |

Runtime log filename controls:

- Default base name is read from NVS key `log_name`.
- If missing, firmware writes `data_log_` to NVS.
- UART command `F` changes the base log filename and starts a new file.
- UART command `I` increments to a new numbered file.
- CAN ID `0x69E` can also set the base filename from the CAN payload and start a new file.

Filename restrictions are enforced in `main/sdcard.c`: no FAT32-invalid characters, no periods, no leading/trailing spaces, and no reserved DOS device names.

## Data Output Format

Log files are created under `/sdcard` with this pattern:

```text
/sdcard/{base_name}{optional_gps_date}{test_number}.benji2
```

If GNSS fix type equals `3`, the filename includes:

```text
{month}_{day}_{hour}_{sec}_
```

Each log file contains:

1. 4-byte little-endian unsigned integer: header length in bytes.
2. Header bytes: comma-separated channel names generated from `main/log_chnl.h`.
3. Repeating fixed-size binary records, one record per logger sample.

Record size is `CH_COUNT` bytes. Multi-byte values are little-endian, written by helpers in `main/logger.c`.

The channel layout is defined by the `enum LogChannel` in `main/log_chnl.h`. For multi-byte fields, the first channel name is the low byte and following names ending in `1`, `2`, etc. are continuation bytes. Example: `TS` through `TS7` are one 64-bit timestamp in microseconds from `esp_timer_get_time()`.

TODO: Add a host-side parser script for `.benji2` files and document the exact conversion to CSV.

## How To Retrieve Logs

### Method 1: Remove The microSD Card

1. Stop the vehicle/logger or otherwise make sure logging is not actively writing.
2. Wait a few seconds after the last logging activity.
3. Remove the microSD card.
4. Copy `.benji2` files from the card on a PC.

Warning: The firmware calls `fflush()` periodically and `fsync()` during flushes, but unexpected power loss or card removal during active logging can still corrupt the active file.

### Method 2: Wi-Fi HTTP File Browser

If no CAN messages have been received after boot, the firmware starts a SoftAP and HTTP server.

1. Power the logger with an SD card installed.
2. Keep CAN disconnected or quiet.
3. Connect a laptop/phone to Wi-Fi:
   - SSID: `sdm26_logger`
   - Password: `sdmfsae26`
4. Open:

```text
http://192.168.4.1/api/view
```

The file browser lists `/sdcard` contents. Click a file to download it. The server auto-stops after 5 minutes of inactivity.

TODO: `start_server()` logs `data-logger` in one message, but firmware Wi-Fi SSID is `sdm26_logger`.

## UART Debug Commands

Open the serial monitor and press `H` for the menu.

Useful commands:

| Key | Function |
| --- | --- |
| `1` | System status, free heap, uptime |
| `4` | Memory info |
| `5` | CPU usage |
| `A` | Analog sensor raw values |
| `C` | CAN message count |
| `D` | Toggle DTC display |
| `E` | Engine diagnostics |
| `F` | Change log file name |
| `G` | GPS info |
| `I` | Increment/start new log file |
| `S` | Steering angle debug calculation |
| `T` | IMU data |
| `W` | Wheel board info |
| `X` | Strain gauge info |
| `R` | Restart |
| `ESC` | Clear screen |

## Bench Test / Validation Checklist

Use this sequence before installing the logger in the car:

1. Inspect power wiring. Confirm `+BATT` and `GND` on Front J6 pins 1 and 2.
2. Confirm bench supply current limit is set conservatively before first power-up.
3. Insert a known-good FAT32 microSD card.
4. Connect `ESP_PROG` USB and run:

```powershell
idf.py -p COMx flash monitor
```

5. Confirm boot log shows SD card mounted and a log file opened.
6. Press `A` in the serial monitor and confirm ADC values change when applying safe 0-5 V test signals.
7. Press `G` and confirm GNSS fields update after the antenna has sky view.
8. Connect CAN to a known-good 1 Mbit/s source and press `C`; confirm CAN message count increments.
9. Press `W`, `T`, and `E` as relevant while CAN devices are active.
10. Press `I` to start a new log file.
11. Let the logger run for at least 30 seconds.
12. Retrieve the `.benji2` file by SD card or Wi-Fi.
13. Confirm the file has a nonzero size and begins with a 4-byte header length followed by channel names.
14. Confirm record count increases roughly 100 records per second after the header.

## Troubleshooting

| Symptom | Checks |
| --- | --- |
| `idf.py` not found | Use an ESP-IDF terminal or run the ESP-IDF export script. |
| Flash fails | Check `ESP_PROG` cable, CP210x driver, correct COM port, and boot/reset circuitry. |
| Monitor shows repeated SD mount failure/restarts | Check card insertion, format card FAT32, try a different card, inspect SD wiring. Firmware does not enable card-detect. |
| No log file appears | Check SD mount logs, NVS filename validity, and whether `sdcard_create_numbered_log_file()` succeeded. |
| Active log file corrupt after removal | Stop power/logging before removing card. Avoid removing during active writes. |
| No CAN messages | Verify 1 Mbit/s bus, wiring to Rear J5 pins 7/8, bus ground reference, and the firmware TX/RX TODO in `main/can.c`. |
| CAN bus disturbed by logger | Firmware is listen-only, but verify transceiver wiring and termination. TODO: termination GPIO17 is not controlled by firmware. |
| GNSS never gets fix | Confirm active antenna, sky view, NEO-F9P power, GNSS UART at 38400 baud, and NMEA output enabled. |
| Analog readings wrong | Confirm sensor ground, 0-5 V range, ADC channel mapping, and brake pressure "might be swapped" comments in `main/main.c`. |
| INA260 readings zero | Check I2C pullups, GPIO47/GPIO48, address `0x40`, and INA260 power. |
| Wi-Fi file browser not starting | It only starts when `can_msg_count == 0` and the server has not already started. Disconnect/quiet CAN and reboot. |
| UART CPU usage command reports no timing data | Runtime stats are enabled in `sdkconfig`; rebuild if config changed. |

## Known Issues / TODOs

- Fill in final board/module ordering, PCB assembly notes, and harness documentation.
- Verify CAN TX/RX pin naming mismatch between schematic and `main/can.c`.
- Implement or document CAN termination control on GPIO17.
- Confirm ADC physical sensor-to-channel mapping and brake pressure channel order.
- Add calibration files or constants for analog channels.
- Add a host-side `.benji2` parser and CSV export workflow.
- Confirm tire temperature CAN copy direction for `0x371`, `0x372`, `0x381`, `0x382`, `0x391`, `0x392`, `0x3A1`, and `0x3A2`.
- Decide whether `can_receive_task()` should be created in both `can_init()` and `tasks_start_all()`.
- Implement `USER_LED` behavior or remove the UART "Toggle LED" placeholder.
- Confirm GNSS NEO-F9P configuration expectations: NMEA rate, enabled sentences, RTK status logging, and PPS behavior.
- Add source-controlled hardware docs or link to schematic/layout files.
- Add explicit license file if the team intends to share this outside Sun Devil Motorsports.

## Contributing Guidelines

- Keep hardware pin definitions centralized and cite the schematic revision when changing them.
- Do not change log channel order in `main/log_chnl.h` without updating parsers and documenting the format change.
- Add new logged fields at the end of the channel list when possible to reduce parser breakage.
- Document new CAN IDs in this README and near `process_can_message()` in `main/main.c`.
- Keep UART debug commands short, documented, and safe to use while logging.
- Test SD logging for at least a few minutes after changes to timing, buffering, or file I/O.
- For vehicle-facing changes, bench test with current-limited power and simulated sensors before connecting to the car.
- Do not commit generated `build/` output.

## License / Ownership

This firmware is owned and maintained by Sun Devil Motorsports for the SDM26 FSAE vehicle data logger.

TODO: Add the final repository license or internal-use policy. Until then, do not assume the code is open-source licensed.
