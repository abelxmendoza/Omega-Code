# Omega-1 Network Management System

## 🎯 Overview

Unified network management system for Omega-1 robot. Provides clean, production-ready networking with three core modes:

1. **AP Mode** (Field Mode) - Creates Wi-Fi hotspot
2. **Client Mode** (Home/Lab Mode) - Connects to existing Wi-Fi
3. **Tailscale VPN** (Remote Control) - Secure remote access

## 📁 Structure

```
network/
├── wizard/              # AP/Client Mode System (core)
│   ├── network_wizard.py
│   ├── config/          # Jinja2 templates
│   │   ├── hostapd.conf.j2
│   │   ├── dnsmasq.conf.j2
│   │   └── dhcpcd.conf.j2
│   └── __init__.py
│
├── wifi/                # Wi-Fi client management
│   ├── connect.py       # nmcli connect logic
│   ├── scan.py          # Network scanning
│   └── __init__.py
│
├── vpn/                 # Tailscale VPN
│   ├── tailscale_status.py
│   └── __init__.py
│
├── diagnostics/         # Network diagnostics
│   ├── net_summary.py   # Unified network summary
│   └── __init__.py
│
├── api/                 # REST API
│   ├── network_routes.py
│   └── __init__.py
│
├── cli/                 # Command line interface
│   ├── omega_network.py
│   └── __init__.py
│
├── install.sh           # Installation script
└── README.md
```

## 🚀 Quick Start

### Installation

```bash
cd servers/robot_controller_backend/network
sudo bash install.sh
```

### CLI Usage

```bash
# Enable AP mode (field mode)
sudo omega-network ap

# Enable client mode (home mode)
sudo omega-network client

# Show network status
sudo omega-network status

# Validate configuration
sudo omega-network validate

# View logs
sudo omega-network logs
```

## 🔌 API Endpoints

### Unified Network Endpoint

**GET `/api/network`**

Returns comprehensive network summary:

```json
{
  "ok": true,
  "mode": "ap",
  "interface": "wlan0",
  "ssid": "Omega1-AP",
  "ip": "192.168.4.1",
  "gateway": null,
  "rssi": -45,
  "vpn_status": {
    "enabled": true,
    "ip": "100.93.225.61",
    "status": "connected"
  },
  "pan_status": {
    "enabled": false,
    "status": "disconnected"
  },
  "services_running": {
    "hostapd": {"status": "active", "enabled": true},
    "dnsmasq": {"status": "active", "enabled": true},
    "dhcpcd": {"status": "active", "enabled": true},
    "wpa_supplicant": {"status": "inactive", "enabled": false}
  },
  "ap_config": {
    "ssid": "Omega1-AP",
    "ip": "192.168.4.1"
  },
  "errors": [],
  "warnings": []
}
```

### Mode Switching

**POST `/api/network/mode`**

```json
{
  "mode": "ap"  // or "client"
}
```

### Validation

**POST `/api/network/validate`**

Returns validation results for current configuration.

### Wi-Fi Management

**GET `/api/network/wifi/scan`** - Scan for networks

**POST `/api/network/wifi/connect`** - Connect to network
```json
{
  "ssid": "MyNetwork",
  "password": "mypassword"
}
```

**DELETE `/api/network/wifi/forget?ssid=MyNetwork`** - Forget network

## 🔧 Configuration

### AP Mode Configuration

- **SSID**: `Omega1-AP`
- **Password**: `omegawifi123`
- **Static IP**: `192.168.4.1`
- **DHCP Range**: `192.168.4.2` - `192.168.4.20`

### State File

Network state is saved to `/etc/omega-network/state.json`:

```json
{
  "mode": "ap",
  "ap_ssid": "Omega1-AP",
  "ap_ip": "192.168.4.1",
  "last_updated": "2025-01-XX..."
}
```

## 📝 Features

### Core Features

- ✅ AP/Client mode switching
- ✅ Unified network summary endpoint
- ✅ Wi-Fi network scanning and connection
- ✅ Tailscale VPN status
- ✅ Service status monitoring
- ✅ Configuration validation
- ✅ Jinja2 template-based config generation

### Optional Features

- Bluetooth PAN support (if module installed)
- Network diagnostics and error reporting

## 🛠️ Development

### Adding New Features

1. **Wi-Fi Features**: Add to `wifi/` module
2. **VPN Features**: Add to `vpn/` module
3. **Diagnostics**: Add to `diagnostics/` module
4. **API Routes**: Add to `api/network_routes.py`
5. **CLI Commands**: Add to `cli/omega_network.py`

### Testing

```bash
# Test AP mode
sudo python3 wizard/network_wizard.py ap

# Test client mode
sudo python3 wizard/network_wizard.py client

# Test status
sudo python3 wizard/network_wizard.py status

# Test validation
sudo python3 wizard/network_wizard.py validate
```

## 📋 Requirements

- Python 3.7+
- Jinja2 (`pip install jinja2`)
- System packages: `hostapd`, `dnsmasq`, `dhcpcd5`
- Root/sudo access for mode switching

## 🔒 Security Notes

- AP mode password is hardcoded (change in `wizard/network_wizard.py` for production)
- State file is readable by root only
- Logs are written to `/var/log/omega-network.log`

## 📚 Documentation

- See `wizard/network_wizard.py` for core implementation
- See `diagnostics/net_summary.py` for unified summary logic
- See `api/network_routes.py` for REST API endpoints
