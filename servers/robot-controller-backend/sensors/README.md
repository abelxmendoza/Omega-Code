# Sensors Directory

This directory contains sensor drivers, WebSocket servers, and diagnostic tools for various sensors used in the robot controller.

## 📁 File Overview

### 🎯 Ultrasonic Sensor (HC-SR04)

#### Production Server
- **`main_ultrasonic.go`** ⭐ **RECOMMENDED FOR PRODUCTION**
  - Optimized Go WebSocket server
  - Port: 8080 (configurable via `PORT_ULTRASONIC`)
  - Features: Error handling, adaptive polling, performance optimizations
  - Run: `go run main_ultrasonic.go`
  - **This is the main production server - use this for deployment**

#### Python Alternatives
- **`ultrasonic_ws_server.py`** 🔄 **PYTHON ALTERNATIVE**
  - Python WebSocket server (alternative to Go version)
  - Same functionality, different language
  - Use if you prefer Python or need Python-specific features
  - Run: `python3 ultrasonic_ws_server.py`
  - **Note**: Go version (`main_ultrasonic.go`) is recommended for production

- **`ultrasonic_sensor.py`** 🔧 **UTILITY CLASS**
  - Python class for HC-SR04 sensor control
  - Used by other Python scripts
  - Can be imported: `from ultrasonic_sensor import Ultrasonic`
  - Not meant to be run directly (use other scripts)

- **`ultrasonic_sensor_runner.py`** 🚀 **CLI TOOL WITH OPTIMIZATIONS**
  - Command-line tool with caching and performance monitoring
  - Features: Async processing, caching, performance metrics
  - Run: `python3 ultrasonic_sensor_runner.py`
  - **Use for**: Testing, development, performance analysis

#### Diagnostic Tools
- **`test_ultrasonic_hardware.go`** 🔍 **GO DIAGNOSTIC**
  - Comprehensive hardware test suite (Go)
  - Tests: Pin configuration, echo response, pulse measurement, continuous readings
  - Run: `go run test_ultrasonic_hardware.go`
  - **Use for**: Hardware troubleshooting, wiring verification

- **`debug_ultrasonic_wiring.py`** 🔍 **PYTHON DIAGNOSTIC**
  - Simple wiring debugger (Python)
  - Tests: Echo pin responsiveness, state changes
  - Run: `python3 debug_ultrasonic_wiring.py`
  - **Use for**: Quick wiring checks, Python-based debugging

### 📊 ADC Sensors

- **`adc.py`** 📡 **PCF8591/ADS7830 ADC**
  - ADC utility for PCF8591 and ADS7830 chips
  - Auto-detects chip type
  - Reads analog voltages from channels 0-2
  - Run: `python3 adc.py`

- **`read_voltage.py`** 📡 **ADS1115 ADC**
  - ADC utility for ADS1115 chip
  - Reads voltages from all 4 channels (AIN0-AIN3)
  - Rich table display
  - Run: `python3 read_voltage.py`

### 🛤️ Line Tracker

- **`line_tracking_ws_server.py`** 🛤️ **LINE TRACKER SERVER**
  - WebSocket server for line tracking sensors
  - Reads IR sensor values
  - Different sensor type (not ultrasonic)

## 🎯 Which File Should I Use?

### For Production Deployment
👉 **Use `main_ultrasonic.go`** - Optimized, production-ready Go server

### For Python Development
👉 **Use `ultrasonic_ws_server.py`** - Python WebSocket server

### For Testing/Development
👉 **Use `ultrasonic_sensor_runner.py`** - CLI tool with optimizations

### For Hardware Troubleshooting
👉 **Use `test_ultrasonic_hardware.go`** (comprehensive) or `debug_ultrasonic_wiring.py` (quick check)

### For ADC Readings
👉 **Use `adc.py`** (PCF8591/ADS7830) or `read_voltage.py` (ADS1115)

## 🔌 Hardware Configuration

### Ultrasonic Sensor (HC-SR04)
- **Trigger**: GPIO27 (Physical Pin 13)
- **Echo**: GPIO22 (Physical Pin 15)
- **Power**: 5V (VCC), GND

### ADC Sensors
- **I2C Bus**: Bus 1 (default)
- **Addresses**: 0x48 (PCF8591/ADS7830), 0x48/0x40 (ADS1115)

## 📦 Dependencies

### Go Files
- `periph.io/x/host/v3` - GPIO control
- `github.com/gorilla/websocket` - WebSocket support

### Python Files
- `lgpio` - GPIO control (Raspberry Pi)
- `websockets` - WebSocket support (for ws_server.py)
- `smbus2` - I2C communication (for ADC files)
- `rich` - Terminal formatting (for read_voltage.py)

## 🚀 Quick Start

### Run Production Ultrasonic Server
```bash
cd sensors
go run main_ultrasonic.go
```

### Run Python Alternative
```bash
cd sensors
python3 ultrasonic_ws_server.py
```

### Test Hardware
```bash
cd sensors
go run test_ultrasonic_hardware.go
```

### Read ADC Values
```bash
cd sensors
python3 adc.py          # PCF8591/ADS7830
python3 read_voltage.py  # ADS1115
```

## 📝 Notes

- **`main_ultrasonic.go`** is the **recommended production server**
- Python versions are alternatives for Python-based workflows
- Diagnostic tools are useful for troubleshooting hardware issues
- All files are kept separate to maintain flexibility and language diversity

## 🔗 Related Documentation

- `ULTRASONIC_ERROR_HANDLING.md` - Error handling guide
- `ULTRASONIC_OPTIMIZATION.md` - Performance optimizations
- `ULTRASONIC_TROUBLESHOOTING.md` - Hardware troubleshooting
- `CONSOLIDATION_ANALYSIS.md` - Detailed file analysis

