# Omega-1 Network Setup Wizard

A comprehensive network management tool for Raspberry Pi that allows seamless switching between Access Point (AP) mode and Wi-Fi Client mode, enabling headless SSH access without a router or monitor.

## 🎯 Features

- **Access Point Mode**: Create your own Wi-Fi network (SSID: `Omega1-AP`)
- **Client Mode**: Connect to existing Wi-Fi networks
- **Headless Operation**: Full SSH access without router or monitor
- **Automatic Configuration**: Generates all required config files
- **Validation**: Comprehensive network status checks
- **Interactive Menu**: User-friendly CLI interface

## 📋 Requirements

- Raspberry Pi with Wi-Fi adapter (wlan0)
- Root/sudo access
- Python 3.7+
- Required packages (auto-installed): `hostapd`, `dnsmasq`, `dhcpcd5`

## 🚀 Quick Start

### Installation

```bash
cd servers/robot_controller_backend/network
sudo bash install.sh
```

### Basic Usage

```bash
# Enable AP mode (field mode)
sudo omega-network ap

# Enable client mode (home mode)
sudo omega-network client

# Show network status
sudo omega-network status

# Run validation checks
sudo omega-network validate

# Interactive menu
sudo omega-network
```

## 🔧 Configuration

### Access Point Mode Settings

- **SSID**: `Omega1-AP`
- **Password**: `omegawifi123`
- **Pi IP**: `192.168.4.1`
- **DHCP Range**: `192.168.4.2` - `192.168.4.20`
- **Subnet**: `192.168.4.0/24`

### Configuration Files

The wizard creates/updates:

- `/etc/hostapd/hostapd.conf` - Access Point configuration
- `/etc/dnsmasq.conf` - DHCP server configuration
- `/etc/dhcpcd.conf` - Network interface configuration
- `/etc/default/hostapd` - Hostapd service defaults
- `/etc/omega-network/state.json` - Current network state

## 📖 Detailed Usage

### Access Point Mode (Field Mode)

Perfect for field operations where no Wi-Fi network is available:

```bash
sudo omega-network ap
```

**What happens:**
1. Installs required packages (if missing)
2. Creates AP configuration files
3. Sets static IP `192.168.4.1` on wlan0
4. Starts hostapd and dnsmasq services
5. Enables services to start on boot

**After setup:**
- Connect to `Omega1-AP` Wi-Fi network
- Password: `omegawifi123`
- SSH: `ssh omega1@192.168.4.1`

### Client Mode (Home Mode)

Connect to your home Wi-Fi network:

```bash
sudo omega-network client
```

**What happens:**
1. Stops AP services (hostapd, dnsmasq)
2. Removes static IP configuration
3. Enables wpa_supplicant for client mode
4. Restarts dhcpcd to get DHCP IP

**Note**: You'll need to configure Wi-Fi credentials separately using `raspi-config` or `/etc/wpa_supplicant/wpa_supplicant.conf`.

### Network Status

Check current network configuration:

```bash
sudo omega-network status
```

**Shows:**
- Current mode (AP/Client)
- Service status (hostapd, dnsmasq, dhcpcd, wpa_supplicant)
- wlan0 interface IP address
- Network state information

### Validation

Run comprehensive validation checks:

```bash
sudo omega-network validate
```

**Checks:**
- ✓ AP configuration files exist
- ✓ hostapd is running with correct config
- ✓ wlan0 has correct static IP (AP mode)
- ✓ DHCP service is working

## 🔄 Switching Between Modes

### AP → Client Mode

```bash
sudo omega-network client
# Wait for services to stop
# Configure Wi-Fi credentials if needed
sudo systemctl restart dhcpcd
```

### Client → AP Mode

```bash
sudo omega-network ap
# Wait ~10 seconds for services to start
# Connect to Omega1-AP network
```

## 🛠️ Troubleshooting

### AP Mode Not Working

1. **Check service status:**
   ```bash
   sudo systemctl status hostapd
   sudo systemctl status dnsmasq
   ```

2. **Check logs:**
   ```bash
   sudo journalctl -u hostapd -n 50
   sudo journalctl -u dnsmasq -n 50
   tail -f /var/log/omega-network.log
   ```

3. **Validate configuration:**
   ```bash
   sudo omega-network validate
   ```

4. **Check wlan0 interface:**
   ```bash
   ip addr show wlan0
   ```

5. **Restart services:**
   ```bash
   sudo systemctl restart hostapd
   sudo systemctl restart dnsmasq
   sudo systemctl restart dhcpcd
   ```

### Can't Connect to AP

- Verify SSID: `Omega1-AP`
- Verify password: `omegawifi123`
- Check Pi IP: `192.168.4.1`
- Ensure hostapd is running: `sudo systemctl status hostapd`
- Check firewall rules (if any)

### Client Mode Not Connecting

- Verify Wi-Fi credentials in `/etc/wpa_supplicant/wpa_supplicant.conf`
- Check wpa_supplicant status: `sudo systemctl status wpa_supplicant`
- Restart dhcpcd: `sudo systemctl restart dhcpcd`
- Check Wi-Fi signal strength

## 📝 First Boot Setup

For automatic AP mode on first boot:

1. **Add to startup script:**
   ```bash
   # Add to /etc/rc.local or create systemd service
   /usr/local/bin/omega-network ap
   ```

2. **Or create systemd service:**
   ```bash
   sudo nano /etc/systemd/system/omega-network.service
   ```
   
   ```ini
   [Unit]
   Description=Omega Network Wizard
   After=network.target
   
   [Service]
   Type=oneshot
   ExecStart=/usr/local/bin/omega-network ap
   RemainAfterExit=yes
   
   [Install]
   WantedBy=multi-user.target
   ```
   
   ```bash
   sudo systemctl enable omega-network.service
   ```

## 🔒 Security Notes

- **Change default password**: Edit `AP_PASSWORD` in `network_wizard.py`
- **Firewall**: Consider enabling `ufw` or `iptables` rules
- **SSH**: Ensure SSH is enabled and secured
- **Updates**: Keep system packages updated

## 📊 Logs

Network wizard logs are written to:
- `/var/log/omega-network.log` - Wizard operations
- `/var/log/syslog` - System service logs
- `journalctl -u hostapd` - Hostapd service logs
- `journalctl -u dnsmasq` - Dnsmasq service logs

## 🧪 Testing

### Manual Testing

1. **Enable AP mode:**
   ```bash
   sudo omega-network ap
   ```

2. **Connect from another device:**
   - Find `Omega1-AP` in Wi-Fi networks
   - Connect with password `omegawifi123`
   - Verify IP assignment (should be 192.168.4.x)

3. **SSH test:**
   ```bash
   ssh omega1@192.168.4.1
   ```

4. **Validate:**
   ```bash
   sudo omega-network validate
   ```

## 🔄 Advanced Usage

### Custom SSID/Password

Edit `network_wizard.py`:
```python
AP_SSID = "YourCustomSSID"
AP_PASSWORD = "YourSecurePassword"
```

### Custom IP Range

Edit `network_wizard.py`:
```python
AP_IP = "192.168.5.1"
AP_DHCP_START = "192.168.5.2"
AP_DHCP_END = "192.168.5.20"
```

### Backup Configurations

Configurations are automatically backed up with `.backup` extension before modification.

## 📚 Additional Resources

- [hostapd Documentation](https://wireless.wiki.kernel.org/en/users/Documentation/hostapd)
- [dnsmasq Documentation](http://www.thekelleys.org.uk/dnsmasq/doc.html)
- [Raspberry Pi Networking](https://www.raspberrypi.org/documentation/configuration/wireless/)

## 🐛 Known Issues

- **wpa_supplicant conflict**: Automatically disabled in AP mode
- **Interface naming**: Assumes `wlan0` (adjust if different)
- **Channel selection**: Uses channel 7 (adjust if interference)

## 📄 License

Part of the Omega-1 Robotics Platform.

## 🤝 Contributing

Report issues and submit pull requests via GitHub.

---

**Made for Omega-1 Robotics Platform** 🤖

