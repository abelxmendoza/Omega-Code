# Omega-1 Network Architecture

## 🏗️ Architecture Overview

This document describes the clean, production-ready networking architecture for Omega-1.

## 📦 Module Structure

### Core Modules

#### `wizard/` - AP/Client Mode System
- **Purpose**: Core networking functionality for AP/Client mode switching
- **Key Files**:
  - `network_wizard.py`: Main wizard implementation
  - `config/*.j2`: Jinja2 templates for config generation
- **Functions**:
  - `enable_ap_mode()`: Enable Access Point mode
  - `enable_client_mode()`: Enable Wi-Fi Client mode
  - `get_network_status()`: Get current network status
  - `validate_network_config()`: Validate configuration

#### `diagnostics/` - Network Diagnostics
- **Purpose**: Unified network summary and diagnostics
- **Key Files**:
  - `net_summary.py`: Unified network summary generator
- **Functions**:
  - `get_network_summary()`: Single comprehensive network status

#### `wifi/` - Wi-Fi Client Management
- **Purpose**: Wi-Fi network scanning and connection
- **Key Files**:
  - `scan.py`: Network scanning using nmcli
  - `connect.py`: Wi-Fi connection management
- **Functions**:
  - `scan_wifi_networks()`: Scan for available networks
  - `connect_to_wifi()`: Connect to a network
  - `forget_wifi()`: Forget a network

#### `vpn/` - VPN Status
- **Purpose**: Tailscale VPN status checking
- **Key Files**:
  - `tailscale_status.py`: Tailscale status checker
- **Functions**:
  - `get_tailscale_status()`: Get Tailscale VPN status

#### `api/` - REST API
- **Purpose**: Unified REST API endpoints
- **Key Files**:
  - `network_routes.py`: FastAPI router with all endpoints
- **Endpoints**:
  - `GET /api/network`: Unified network summary
  - `POST /api/network/mode`: Switch network mode
  - `POST /api/network/validate`: Validate configuration
  - `GET /api/network/wifi/scan`: Scan Wi-Fi networks
  - `POST /api/network/wifi/connect`: Connect to Wi-Fi
  - `DELETE /api/network/wifi/forget`: Forget Wi-Fi network

#### `cli/` - Command Line Interface
- **Purpose**: CLI tool for network management
- **Key Files**:
  - `omega_network.py`: CLI implementation
- **Commands**:
  - `sudo omega-network ap`: Enable AP mode
  - `sudo omega-network client`: Enable client mode
  - `sudo omega-network status`: Show status
  - `sudo omega-network validate`: Validate config
  - `sudo omega-network logs`: View logs

## 🔄 Data Flow

### Network Summary Flow

```
Frontend Request
    ↓
GET /api/network
    ↓
network/api/network_routes.py
    ↓
diagnostics/net_summary.py
    ↓
├─ wizard/network_wizard.py (state)
├─ wifi/scan.py (interface info)
├─ vpn/tailscale_status.py (VPN status)
└─ systemctl (service status)
    ↓
Unified JSON Response
```

### Mode Switching Flow

```
Frontend Request
    ↓
POST /api/network/mode
    ↓
network/api/network_routes.py
    ↓
sudo python3 wizard/network_wizard.py [ap|client]
    ↓
wizard/network_wizard.py
    ├─ Generate configs (Jinja2 templates)
    ├─ Update system files
    ├─ Restart services
    └─ Save state
    ↓
Success Response
```

## 🎯 Design Principles

### 1. Single Source of Truth
- **Unified Endpoint**: `GET /api/network` returns all network information
- **State File**: `/etc/omega-network/state.json` stores current mode
- **No Duplication**: One endpoint, one summary function

### 2. Modular Design
- **Separation of Concerns**: Each module has a single responsibility
- **Clear Interfaces**: Well-defined function signatures
- **Easy Extension**: Add new modules without breaking existing code

### 3. Production Ready
- **Error Handling**: Comprehensive error checking
- **Logging**: All operations logged to `/var/log/omega-network.log`
- **Validation**: Configuration validation before applying changes
- **Backup**: Config files backed up before modification

### 4. Template-Based Configuration
- **Jinja2 Templates**: All configs generated from templates
- **Easy Customization**: Change templates, not code
- **Version Control**: Templates tracked in git

## 🔌 API Design

### Unified Endpoint Pattern

All network information comes from a single endpoint:

```python
GET /api/network
```

Returns:
- Current mode (ap/client)
- Interface details (wlan0/bnep0)
- SSID, IP, Gateway, RSSI
- VPN status (Tailscale)
- PAN status (Bluetooth)
- Service statuses
- Errors/warnings

### Mode Switching Pattern

```python
POST /api/network/mode
{
  "mode": "ap" | "client"
}
```

### Validation Pattern

```python
POST /api/network/validate
```

Returns validation results for current configuration.

## 🛠️ Extension Points

### Adding New Network Types

1. Create new module in `network/` directory
2. Add status function (e.g., `get_xxx_status()`)
3. Integrate into `diagnostics/net_summary.py`
4. Add API endpoint if needed

### Adding New CLI Commands

1. Add function to `cli/omega_network.py`
2. Add command handler in `main()`
3. Update `install.sh` help text

### Adding New API Endpoints

1. Add route to `api/network_routes.py`
2. Implement handler function
3. Update frontend to use new endpoint

## 📊 State Management

### State File Location
`/etc/omega-network/state.json`

### State Structure
```json
{
  "mode": "ap",
  "ap_ssid": "Omega1-AP",
  "ap_ip": "192.168.4.1",
  "last_updated": "2025-01-XX..."
}
```

### State Updates
- Updated on mode switch
- Read by diagnostics module
- Used by API endpoints

## 🔒 Security Considerations

1. **Root Access**: Mode switching requires sudo
2. **State File**: Readable by root only
3. **Logs**: Written to `/var/log/` (system logs)
4. **AP Password**: Hardcoded (change for production)

## 🧪 Testing Strategy

### Unit Tests
- Test each module independently
- Mock system calls (subprocess, file I/O)

### Integration Tests
- Test API endpoints
- Test CLI commands
- Test mode switching flow

### Manual Testing
```bash
# Test AP mode
sudo omega-network ap
sudo omega-network status

# Test client mode
sudo omega-network client
sudo omega-network status

# Test validation
sudo omega-network validate
```

## 📝 Migration Notes

### From Old Structure

Old files (`network_wizard.py`, `network_cli.py`) at root are kept for backward compatibility but deprecated. New code should use:

- `wizard/network_wizard.py` instead of `network_wizard.py`
- `cli/omega_network.py` instead of `network_cli.py`

### API Migration

Old endpoints (`/api/network/status`, `/api/network/info`) are replaced by unified endpoint (`GET /api/network`).

Frontend updated to use unified endpoint.

