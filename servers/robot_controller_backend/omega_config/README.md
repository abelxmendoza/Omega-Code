# OmegaOS Configuration Layer

Unified configuration management system for Omega-1 robot. Provides centralized configuration storage, validation, and API access.

## 📁 Structure

```
omega_config/
├── config.yaml              # Main system configuration
├── robot_profile.yaml       # Robot profile definitions
├── hardware_map.yaml        # Hardware device mapping
├── persistent_state.json    # Runtime state (auto-generated)
├── config_manager.py        # Configuration manager
├── backups/                 # Automatic config backups
└── __init__.py
```

## 🎯 Features

- ✅ **Unified Configuration** - Single source of truth for robot settings
- ✅ **YAML-based** - Human-readable configuration files
- ✅ **Validation** - Automatic config validation
- ✅ **Backup System** - Automatic backups before changes
- ✅ **Version Control** - Config versioning support
- ✅ **REST API** - Full API access to configuration
- ✅ **Import/Export** - JSON export/import for backups
- ✅ **Persistent State** - Runtime state tracking

## 📋 Configuration Files

### `config.yaml` - Main Configuration

Contains all robot-wide settings:

- **robot** - Name, profile, version
- **network** - AP/client mode, IPs, Wi-Fi settings
- **services** - Autostart services, restart policies
- **camera** - Backend, resolution, FPS
- **movement** - Default profile, speed limits
- **lighting** - Default pattern, brightness
- **logging** - Log level, rotation settings
- **security** - API auth, allowed origins
- **telemetry** - Update intervals, Prometheus export

### `robot_profile.yaml` - Robot Profiles

Defines hardware profiles:

- **pi4b** - Raspberry Pi 4B configuration
- **jetson** - Jetson Orin Nano configuration
- **dev** - Development mode (no hardware)

Each profile includes:
- Hardware specifications
- Capabilities (ML, SLAM, etc.)
- Recommended settings

### `hardware_map.yaml` - Hardware Mapping

Maps logical devices to physical hardware:

- **GPIO pins** - Motor, sensor, LED pins
- **I2C devices** - PCA9685, sensors addresses
- **Camera devices** - CSI/USB camera paths
- **Network interfaces** - Wi-Fi, Bluetooth

### `persistent_state.json` - Runtime State

Auto-generated file tracking:

- Boot count
- Last boot time
- Service states
- Network state
- Movement/lighting last settings

## 🚀 Usage

### Python API

```python
from omega_config.config_manager import get_config_manager

# Get config manager
manager = get_config_manager()

# Get full config
config = manager.get_config()

# Get specific section
network_config = manager.get_section("network")

# Update section
manager.update_section("camera", {"width": 1280, "height": 720})
manager.save_config()

# Get profile
profile = manager.get_profile("pi4b")

# Get hardware map
hardware = manager.get_hardware_map()

# Update state
manager.update_state("last_profile", "aggressive")

# Export/Import
export_data = manager.export_config()
success, errors = manager.import_config(import_data)

# Validate
valid, errors = manager.validate_config()
```

### REST API

```bash
# Get full config
GET /api/config

# Get section
GET /api/config/{section}

# Update section
POST /api/config/{section}
{
  "width": 1280,
  "height": 720
}

# Get profile
GET /api/config/profile/pi4b

# Get hardware map
GET /api/config/hardware/map

# Get state
GET /api/config/state

# Update state
POST /api/config/state/{key}
"value"

# Export config
GET /api/config/export

# Import config
POST /api/config/import
{
  "config": {...},
  "profile": {...},
  "hardware": {...}
}

# Validate config
GET /api/config/validate
```

## 🔧 Configuration Sections

### Robot Section
```yaml
robot:
  name: "Omega-1"
  profile: "pi4b"
  version: "1.0"
  serial_number: ""
```

### Network Section
```yaml
network:
  default_mode: "ap"  # ap | client
  ap:
    ssid: "Omega1-AP"
    password: "omegawifi123"
    ip: "192.168.4.1"
```

### Services Section
```yaml
services:
  autostart:
    - movement_ws_server
    - video_server
  restart_policies:
    movement_ws_server: "always"
    video_server: "on-failure"
```

### Camera Section
```yaml
camera:
  backend: "picamera2"
  device: "/dev/video0"
  width: 640
  height: 480
  fps: 30
```

### Movement Section
```yaml
movement:
  default_profile: "smooth"
  max_speed: 4095
  min_speed: 0
```

## 🔒 Security

- Config files are readable by owner only
- Backups stored in `backups/` directory
- Validation prevents invalid configurations
- Import requires validation before applying

## 📝 Best Practices

1. **Always backup before changes:**
   ```python
   manager._create_backup()  # Automatic on save
   ```

2. **Validate after updates:**
   ```python
   valid, errors = manager.validate_config()
   ```

3. **Use sections for updates:**
   ```python
   manager.update_section("camera", {"width": 1280})
   ```

4. **Export for backups:**
   ```python
   export_data = manager.export_config()
   # Save to external location
   ```

## 🔄 Integration

The configuration layer integrates with:

- **Service Orchestrator** - Loads config at boot
- **Network Wizard** - Uses network config
- **Movement V2** - Uses movement config
- **Video Server** - Uses camera config
- **Settings UI** - Provides config editing interface

## 📚 API Reference

See `api/config_routes.py` for complete API documentation.

## 🐛 Troubleshooting

### Config Not Loading

```bash
# Check file permissions
ls -la omega_config/config.yaml

# Check YAML syntax
python3 -c "import yaml; yaml.safe_load(open('config.yaml'))"
```

### Validation Errors

```bash
# Validate config
curl http://localhost:8000/api/config/validate
```

### Backup Recovery

```bash
# List backups
ls -la omega_config/backups/

# Restore backup
cp omega_config/backups/config_20250101_120000.yaml omega_config/config.yaml
```

## ✅ Status

**Current Implementation:** ✅ Complete
- Config files ✅
- Config manager ✅
- REST API ✅
- Validation ✅
- Backup system ✅

**Ready for Integration:** ✅ Yes

