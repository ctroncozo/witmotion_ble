# WT901BLECL5.0 BLE Python Client

A Python library and command-line tool for scanning, connecting, and streaming data from WT901BLECL5.0 Bluetooth Low Energy (BLE) Inertial Measurement Units (IMUs) and compatible sensors. This project enables real-time access to orientation, acceleration, gyroscope, and other sensor data for applications in robotics, sports analytics, IoT, and research.

## Features
- Scan for nearby WT901BLECL5.0 and compatible BLE IMU devices
- Connect to devices and manage BLE connections robustly
- Stream and decode sensor data (acceleration, angular velocity, orientation, quaternions, temperature, timestamp, etc.)
- Calibrate sensors (accelerometer, magnetometer)
- Synchronize device time with host
- Easily extensible and modular Python codebase

## TODO
- Improve exception handling.
- Support all update rates. Currently only supports 20, 50, and 100 Hz.
- Only static accelerometer and spherical magnetometer calibration are supported.
- Currently only supports handling default packets (acceleration, angular velocity, orientation) and timestamp packets. It needs support for magnetometer, quaternions, temperature.

## Installation

1. **Clone this repository:**
   ```bash
   git clone git@github.com:ctroncozo/witmotion_ble.git
   cd witmotion_ble
   ```

2. **Install dependencies:**
   ```bash
   pip install -r requirements.txt
   ```
   Additional system dependencies (for `pynput` and BLE):
   ```bash
   sudo apt-get install python3-dev python3-pip python3-venv build-essential libevdev-dev
   ```

## Usage

### Scan for Devices
```bash
python scan.py
```

### Connect and Stream Data
```bash
python stream.py
```

- By default, the main application connects to a hardcoded MAC address (edit `app.py` to change).
- Press `q` to gracefully disconnect and stop streaming.

### Calibrate Sensors
Calibration routines are available in the `BleakClientWrapper` class. See `app.py` for usage examples.

## Code Structure
- `app.py` - Main entry point for connecting and streaming from a device
- `scan.py` - BLE device scanning utilities
- `client.py` - BLE client abstraction
- `wit/` - Protocol, message, and register definitions for WT901 sensors
- `viz.py` - (Optional) Visualization tools
- `calibrate.py` - (Optional) Calibration routines

## Device Addresses
- Example WT901BLECL5.0 MAC: `FC:7F:CD:6E:97:25`
- Example MetaMotionS MAC: `EE:33:49:03:91:45`

## References
- [WT901BLECL5.0 Data Sheet](documentation/wit_standard_comm_protocol.pdf)
- [WIT Standard Communication Protocol](documentation/WT901BLECL_data_sheet.pdf)

## License
MIT License

---
**Author:** Cristian Troncoso

For questions or contributions, please open an issue or contact the maintainer.