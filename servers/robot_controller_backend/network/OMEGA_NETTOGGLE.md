# Omega-NetToggle: Clean Network Recovery + AP Mode Script

## Overview

**Omega-NetToggle** is a clean, robust script for managing network modes on Raspberry Pi OS Bookworm (NetworkManager default). It provides reliable WiFi recovery and AP mode switching with safety backups and comprehensive logging.

**Now includes self-healing capabilities:**
- ✅ **Network Watchdog** - Auto-restores WiFi if connection is lost
- ✅ **Boot Safety** - Ensures WiFi is restored on boot, falls back to AP mode after 3 failures
- ✅ **Systemd Integration** - Automatic monitoring and recovery

## Features

✅ **Restore Normal WiFi Mode** - Complete recovery from breakage  
✅ **Enable AP Mode** - Clean Access Point setup  
✅ **Network Diagnostics** - Comprehensive status reporting  
✅ **Safety Backups** - Automatic config backups before changes  
✅ **Comprehensive Logging** - All operations logged to `/var/log/omega-nettoggle.log`  
✅ **NetworkManager Compatible** - Works with Raspberry Pi OS Bookworm default  

## Quick Start

### Installation

```bash
cd servers/robot_controller_backend/network
sudo ./install.sh
```

This will:
- Make `omega-nettoggle.sh` executable
- Create symlink: `/usr/local/bin/omega-nettoggle`
- Install omega-network CLI tool

### Usage

```bash
# Restore normal WiFi mode (recover from breakage)
sudo omega-nettoggle restore

# Enable AP mode
sudo omega-nettoggle ap

# Show network diagnostics
sudo omega-nettoggle status

# Or use direct path
sudo ./omega-nettoggle.sh restore
sudo ./omega-nettoggle.sh ap
sudo ./omega-nettoggle.sh status
```

## Modes

### `restore` - Restore Normal WiFi Mode

**What it does:**
- Unblocks WiFi radio
- Stops AP services (hostapd, dnsmasq)
- Restores dhcpcd.conf from backup
- Reloads WiFi driver (brcmfmac)
- Enables and restarts NetworkManager
- Brings wlan0 back online

**When to use:**
- WiFi is broken/stuck
- Need to switch back from AP mode
- NetworkManager not working
- WiFi driver needs reload

**Example:**
```bash
sudo omega-nettoggle restore
```

### `ap` - Enable Access Point Mode

**What it does:**
- Creates safety backups of all configs
- Stops NetworkManager
- Configures static IP (192.168.4.1)
- Sets up hostapd (SSID: Omega1-AP)
- Configures dnsmasq (DHCP range: 192.168.4.10-200)
- Reloads WiFi driver
- Starts AP services

**Configuration:**
- **SSID:** `Omega1-AP`
- **Password:** `omegawifi123`
- **Pi IP:** `192.168.4.1`
- **DHCP Range:** `192.168.4.10` - `192.168.4.200`

**Example:**
```bash
sudo omega-nettoggle ap
```

After enabling, connect to `Omega1-AP` and SSH:
```bash
ssh omega1@192.168.4.1
```

### `status` - Network Diagnostics

**What it shows:**
- WiFi radio status (rfkill)
- Network interfaces (ip addr)
- NetworkManager status
- WiFi scan results
- Service status (NetworkManager, hostapd, dnsmasq, dhcpcd)
- Current mode detection (AP vs Client)
- Backup files
- Recent log entries

**Example:**
```bash
sudo omega-nettoggle status
```

## Safety Features

### Automatic Backups

Before making changes, the script automatically backs up:
- `/etc/dhcpcd.conf` → `/etc/omega-network/backups/dhcpcd.conf.{timestamp}`
- `/etc/hostapd/hostapd.conf` → `/etc/omega-network/backups/hostapd.conf.{timestamp}`
- `/etc/dnsmasq.conf` → `/etc/omega-network/backups/dnsmasq.conf.{timestamp}`

### Restore Backup

The script creates `/etc/dhcpcd.conf.backup-ap` when enabling AP mode. This is automatically restored when switching back to normal mode.

### Comprehensive Logging

All operations are logged to `/var/log/omega-nettoggle.log` with timestamps:
```bash
# View logs
sudo tail -f /var/log/omega-nettoggle.log

# View recent entries
sudo tail -20 /var/log/omega-nettoggle.log
```

## Integration

### API Integration

The script is integrated with the network API:

```bash
# Via API
curl -X POST http://localhost:8000/api/network/mode \
  -H "Content-Type: application/json" \
  -d '{"mode": "ap"}'

curl -X POST http://localhost:8000/api/network/mode \
  -H "Content-Type: application/json" \
  -d '{"mode": "client"}'

# Get status
curl http://localhost:8000/api/network/toggle/status
```

### CLI Integration

The `omega-network` CLI tool can use omega-nettoggle internally:

```bash
sudo omega-network ap      # Uses omega-nettoggle.sh
sudo omega-network client  # Uses omega-nettoggle.sh restore
sudo omega-network status  # Uses omega-nettoggle.sh status
```

## Troubleshooting

### WiFi Won't Come Back Online

```bash
# Check status
sudo omega-nettoggle status

# Force restore
sudo omega-nettoggle restore

# Check logs
sudo tail -50 /var/log/omega-nettoggle.log

# Manual driver reload
sudo modprobe -r brcmfmac brcmutil cfg80211
sudo modprobe brcmfmac
```

### AP Mode Not Working

```bash
# Check service status
sudo systemctl status hostapd
sudo systemctl status dnsmasq

# Check logs
sudo journalctl -u hostapd -n 50
sudo journalctl -u dnsmasq -n 50

# Verify configs
cat /etc/hostapd/hostapd.conf
cat /etc/dnsmasq.conf

# Restart services
sudo systemctl restart hostapd
sudo systemctl restart dnsmasq
```

### NetworkManager Conflicts

If NetworkManager is interfering:

```bash
# Stop NetworkManager
sudo systemctl stop NetworkManager

# Run omega-nettoggle
sudo omega-nettoggle ap

# Or restore
sudo omega-nettoggle restore
```

## File Locations

- **Script:** `servers/robot_controller_backend/network/omega-nettoggle.sh`
- **Symlink:** `/usr/local/bin/omega-nettoggle`
- **Log:** `/var/log/omega-nettoggle.log`
- **Backups:** `/etc/omega-network/backups/`
- **AP Backup:** `/etc/dhcpcd.conf.backup-ap`

## Technical Details

### WiFi Driver Reload

The script reloads the Broadcom WiFi driver stack:
```bash
sudo modprobe -r brcmfmac brcmutil cfg80211
sudo modprobe brcmfmac
```

This ensures a clean driver state.

### NetworkManager Handling

- **AP Mode:** NetworkManager is stopped to prevent conflicts
- **Normal Mode:** NetworkManager is enabled and restarted

### Service Management

The script uses systemd for service management:
- `systemctl enable --now` for starting and enabling
- `systemctl disable` for stopping and disabling
- Proper service dependencies handled

## Comparison with Network Wizard

| Feature | Network Wizard | Omega-NetToggle |
|---------|---------------|-----------------|
| AP Mode | ✅ | ✅ |
| Client Mode | ✅ | ✅ |
| Recovery Mode | ❌ | ✅ |
| Driver Reload | ❌ | ✅ |
| NetworkManager Handling | Partial | ✅ Full |
| Safety Backups | ✅ | ✅ |
| Comprehensive Logging | ✅ | ✅ |
| Status Diagnostics | ✅ | ✅ Enhanced |

**Omega-NetToggle** is designed for **recovery and reliability**, while the Network Wizard provides more comprehensive configuration options.

## Best Practices

1. **Always check status first:**
   ```bash
   sudo omega-nettoggle status
   ```

2. **Use restore when WiFi is broken:**
   ```bash
   sudo omega-nettoggle restore
   ```

3. **Check logs after operations:**
   ```bash
   sudo tail -20 /var/log/omega-nettoggle.log
   ```

4. **Backup before major changes:**
   ```bash
   sudo cp /etc/dhcpcd.conf ~/dhcpcd.conf.backup
   ```

5. **Test AP mode in safe environment:**
   - Ensure you have physical access to Pi
   - Have alternative network connection ready
   - Test restore mode before enabling AP

## Examples

### Complete Recovery Workflow

```bash
# 1. Check current status
sudo omega-nettoggle status

# 2. Restore normal WiFi
sudo omega-nettoggle restore

# 3. Wait for NetworkManager to connect
sleep 10

# 4. Verify connection
nmcli device status

# 5. Check logs if issues
sudo tail -50 /var/log/omega-nettoggle.log
```

### AP Mode Setup Workflow

```bash
# 1. Check current status
sudo omega-nettoggle status

# 2. Enable AP mode
sudo omega-nettoggle ap

# 3. Wait for services to start
sleep 5

# 4. Verify AP is running
sudo omega-nettoggle status

# 5. Connect from laptop/phone
# SSID: Omega1-AP
# Password: omegawifi123

# 6. SSH into Pi
ssh omega1@192.168.4.1
```

## Security Notes

- **AP Password:** Default is `omegawifi123` (change in script for production)
- **SSID:** Default is `Omega1-AP` (changeable in script)
- **Backups:** Stored in `/etc/omega-network/backups/` (protected directory)
- **Logs:** May contain network info (review before sharing)

## Future Enhancements

- [ ] Custom SSID/password via command-line args
- [ ] Interactive mode for guided setup
- [ ] Web UI integration
- [ ] Scheduled mode switching
- [ ] Network health monitoring
- [ ] Automatic recovery on failure

---

**Built for Omega-1 Robot Platform**  
**Compatible with Raspberry Pi OS Bookworm**

