# Project-of-Electronic-Circuits

24 Aug. - 11 Sept. 2026

An **ESP-IDF / ESP32-S3** three-wheel omnidirectional car project for an electronic circuits course, progressing through three tasks: from line following & obstacle avoidance, to autonomous ball transport, to phone remote control & first-person view.

## Tasks

| Directory | Title | Description |
| --- | --- | --- |
| [task 1](task%201/) | Line Following, Obstacle Avoidance & Display | 4-way IR line following, 3-wheel omnidirectional PID, ultrasonic avoidance, TFT data display |
| [task 2](task%202/) | Autonomous Line Following + Ball Transport | Line following → avoidance → cross the finish line, then use a UVC camera to detect and transport white/orange balls to black target zones |
| [task 3](task%203/) | Phone Remote Control + FPV + Expressions | Firmware `RemoteControl/` (BLE/Wi-Fi control, FPV, "naiwa" expressions) with a companion WeChat Mini Program `miniprogram/` |

## Requirements

- ESP-IDF 5.4.x
- Target chip: ESP32-S3
- WeChat DevTools (task 3 only)

## Quick Start

Each task is an independent ESP-IDF project. Enter its directory and run:

```powershell
idf.py build
idf.py -p COMx flash monitor
```

Replace `COMx` with the actual serial port. See each task's README for wiring, parameters, and usage.
