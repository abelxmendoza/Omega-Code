# Omega-NetToggle v2: NetworkManager Native Network Management

## Overview

**Omega-NetToggle v2** is a complete rewrite using NetworkManager's native AP mode capabilities, designed specifically for Raspberry Pi OS Bookworm. It eliminates the need for hostapd/dnsmasq and provides a cleaner, more reliable network management system.

**Key Features:**
- ✅ **NetworkManager Native** - Uses built-in AP mode (no hostapd/dnsmasq)
- ✅ **Simple & Reliable** - Clean `nmcli` commands for all operations
- ✅ **Self-Healing** - Network Watchdog auto-restores WiFi or falls back to AP mode
- ✅ **Boot Safety** - Ensures WiFi is restored on boot, falls back to AP mode if needed
- ✅ **Systemd Integration** - Automatic monitoring and recovery

## Features

✅ **Restore WiFi Client Mode** - Switch back to normal WiFi client  
✅ **Enable AP Mode** - NetworkManager native Access Point (no hostapd needed)  
✅ **Network Diagnostics** - Comprehensive status reporting  
✅ **Comprehensive Logging** - All operations logged to `/var/log/omega-nettoggle.log`  
✅ **NetworkManager Native** - Uses Bookworm's default NetworkManager  
✅ **Always Works** - AP mode works even when WiFi is broken (radio-level control)  

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

# Enable AP mode (uses existing profile)
sudo omega-nettoggle ap

# One-command AP fix (rebuilds AP profile from scratch)
sudo omega-ap-fix

# Show network diagnostics
sudo omega-nettoggle status

# Or use direct path
sudo ./omega-nettoggle.sh restore
sudo ./omega-nettoggle.sh ap
sudo ./omega-ap-fix.sh
sudo ./omega-nettoggle.sh status
```

**Quick Reference:**
| Mode | Command |
|------|---------|
| Restore WiFi | `sudo omega-nettoggle restore` |
| Enable AP | `sudo omega-ap-fix` |
| Status | `sudo omega-nettoggle status` |

## Modes

### `restore` - Restore WiFi Client Mode

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

## Self-Healing System

### Network Watchdog

The **Omega Network Watchdog** continuously monitors WiFi connectivity and automatically restores it if connection is lost.

**Features:**
- Checks WiFi every 30 seconds
- Verifies interface is UP
- Verifies IP address is assigned
- Verifies default gateway is reachable
- Auto-restores WiFi if any check fails

**Installation:**
```bash
# Already installed via install.sh
sudo systemctl start omega-network-watchdog
sudo systemctl status omega-network-watchdog
```

**Service:**
- **File:** `/etc/systemd/system/omega-network-watchdog.service`
- **Script:** `/usr/local/bin/omega-network-watchdog.sh`
- **Logs:** `/var/log/omega-nettoggle.log`

### Boot Safety

The **Omega Network Boot Safety** ensures WiFi is restored on boot, and falls back to AP mode after 3 consecutive failures.

**Features:**
- Runs automatically on boot
- Attempts WiFi restore first
- Waits for NetworkManager to be ready
- Enables AP mode immediately if WiFi fails (no retry counter needed)
- Ensures robot is always accessible

**Installation:**
```bash
# Already installed via install.sh
# Runs automatically on boot
sudo systemctl status omega-netboot
```

**Service:**
- **File:** `/etc/systemd/system/omega-netboot.service`
- **Script:** `/usr/local/bin/omega-netboot.sh`
- **State:** `/etc/omega-net/bootcount`

### Manual Control

You can still manually control network modes:

```bash
# Restore WiFi (overrides watchdog temporarily)
sudo omega-nettoggle restore

# Enable AP mode
sudo omega-nettoggle ap

# Check status
sudo omega-nettoggle status

# Stop watchdog (if needed)
sudo systemctl stop omega-network-watchdog

# Start watchdog
sudo systemctl start omega-network-watchdog
```

## Future Enhancements

- [x] Network health monitoring
- [x] Automatic recovery on failure
- [x] Boot safety fallback
- [ ] Custom SSID/password via command-line args
- [ ] Interactive mode for guided setup
- [ ] Scheduled mode switching
- [ ] Web UI integration for watchdog control

---

**Built for Omega-1 Robot Platform**  
**Compatible with Raspberry Pi OS Bookworm**  
**Self-Healing Network System v1**

