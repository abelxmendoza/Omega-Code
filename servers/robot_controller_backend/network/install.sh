#!/bin/bash
# Omega Network Wizard Installation Script v2
# Installs NetworkManager-based network management system

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INSTALL_DIR="/usr/local/bin"
SYSTEMD_DIR="/etc/systemd/system"
NM_CONN_DIR="/etc/NetworkManager/system-connections"

echo "🔧 Installing Omega Network System v2 (NetworkManager Native)..."
echo ""

# Check if running as root
if [ "$EUID" -ne 0 ]; then 
    echo "❌ Please run as root (use sudo)"
    exit 1
fi

# Check if NetworkManager is installed
if ! command -v nmcli &> /dev/null; then
    echo "❌ NetworkManager (nmcli) not found. Please install it first:"
    echo "   sudo apt-get update && sudo apt-get install network-manager"
    exit 1
fi

# Install Python dependencies
echo "📦 Installing Python dependencies..."
pip3 install jinja2 --quiet 2>/dev/null || pip install jinja2 --quiet 2>/dev/null || true

# Make scripts executable
chmod +x "$SCRIPT_DIR/wizard/network_wizard.py" 2>/dev/null || true
chmod +x "$SCRIPT_DIR/cli/omega_network.py" 2>/dev/null || true

# Generate UUIDs for NetworkManager connections
echo "🔑 Generating connection UUIDs..."
UUID_AP=$(uuidgen 2>/dev/null || python3 -c "import uuid; print(uuid.uuid4())")
UUID_CLIENT=$(uuidgen 2>/dev/null || python3 -c "import uuid; print(uuid.uuid4())")

# Get client WiFi credentials
echo ""
echo "📡 WiFi Client Configuration"
echo "Enter your WiFi network credentials for client mode:"
read -p "  SSID: " CLIENT_SSID
read -sp "  Password: " CLIENT_PASS
echo ""
echo ""

# Ensure NetworkManager connections directory exists
mkdir -p "$NM_CONN_DIR"

# Build client profile
echo "📝 Creating NetworkManager client profile..."
if [ -f "$SCRIPT_DIR/nm-client-profile.nmconnection" ]; then
    sed "s/REPLACE_ME_CLIENT_UUID/$UUID_CLIENT/;
         s/REPLACE_ME_CLIENT_SSID/$CLIENT_SSID/;
         s/REPLACE_ME_CLIENT_PSK/$CLIENT_PASS/" \
         "$SCRIPT_DIR/nm-client-profile.nmconnection" > "$NM_CONN_DIR/Omega1-Client.nmconnection"
    echo "✅ Created Omega1-Client connection profile"
else
    echo "⚠️  Warning: nm-client-profile.nmconnection template not found"
fi

# Build AP profile
echo "📝 Creating NetworkManager AP profile..."
if [ -f "$SCRIPT_DIR/nm-ap-profile.nmconnection" ]; then
    sed "s/REPLACE_ME_AP_UUID/$UUID_AP/" \
        "$SCRIPT_DIR/nm-ap-profile.nmconnection" > "$NM_CONN_DIR/Omega1-AP.nmconnection"
    echo "✅ Created Omega1-AP connection profile"
else
    echo "⚠️  Warning: nm-ap-profile.nmconnection template not found"
fi

# Set proper permissions for NetworkManager connections
chmod 600 "$NM_CONN_DIR"/*.nmconnection 2>/dev/null || true
chown root:root "$NM_CONN_DIR"/*.nmconnection 2>/dev/null || true

# Reload NetworkManager connections
echo "🔄 Reloading NetworkManager connections..."
nmcli connection reload

# Install omega-nettoggle script
NETTOGGLE_SCRIPT="$SCRIPT_DIR/omega-nettoggle.sh"
if [ -f "$NETTOGGLE_SCRIPT" ]; then
    chmod +x "$NETTOGGLE_SCRIPT"
    ln -sf "$NETTOGGLE_SCRIPT" "$INSTALL_DIR/omega-nettoggle"
    chmod +x "$INSTALL_DIR/omega-nettoggle"
    echo "✅ Installed omega-nettoggle.sh"
else
    echo "⚠️  Warning: omega-nettoggle.sh not found at $NETTOGGLE_SCRIPT"
fi

# Install watchdog script
WATCHDOG_SCRIPT="$SCRIPT_DIR/omega-network-watchdog.sh"
if [ -f "$WATCHDOG_SCRIPT" ]; then
    chmod +x "$WATCHDOG_SCRIPT"
    ln -sf "$WATCHDOG_SCRIPT" "$INSTALL_DIR/omega-network-watchdog.sh"
    chmod +x "$INSTALL_DIR/omega-network-watchdog.sh"
    echo "✅ Installed omega-network-watchdog.sh"
else
    echo "⚠️  Warning: omega-network-watchdog.sh not found"
fi

# Install boot safety script
BOOT_SCRIPT="$SCRIPT_DIR/omega-netboot.sh"
if [ -f "$BOOT_SCRIPT" ]; then
    chmod +x "$BOOT_SCRIPT"
    ln -sf "$BOOT_SCRIPT" "$INSTALL_DIR/omega-netboot.sh"
    chmod +x "$INSTALL_DIR/omega-netboot.sh"
    echo "✅ Installed omega-netboot.sh"
else
    echo "⚠️  Warning: omega-netboot.sh not found"
fi

# Install systemd services
WATCHDOG_SERVICE="$SCRIPT_DIR/omega-network-watchdog.service"
BOOT_SERVICE="$SCRIPT_DIR/omega-netboot.service"

if [ -f "$WATCHDOG_SERVICE" ]; then
    cp "$WATCHDOG_SERVICE" "$SYSTEMD_DIR/omega-network-watchdog.service"
    chmod 644 "$SYSTEMD_DIR/omega-network-watchdog.service"
    echo "✅ Installed omega-network-watchdog.service"
else
    echo "⚠️  Warning: omega-network-watchdog.service not found"
fi

if [ -f "$BOOT_SERVICE" ]; then
    cp "$BOOT_SERVICE" "$SYSTEMD_DIR/omega-netboot.service"
    chmod 644 "$SYSTEMD_DIR/omega-netboot.service"
    echo "✅ Installed omega-netboot.service"
else
    echo "⚠️  Warning: omega-netboot.service not found"
fi

# Reload systemd and enable services
if [ -f "$SYSTEMD_DIR/omega-network-watchdog.service" ] || [ -f "$SYSTEMD_DIR/omega-netboot.service" ]; then
    systemctl daemon-reload
    echo "✅ Reloaded systemd daemon"
    
    if [ -f "$SYSTEMD_DIR/omega-network-watchdog.service" ]; then
        systemctl enable omega-network-watchdog.service
        echo "✅ Enabled omega-network-watchdog.service (start with: sudo systemctl start omega-network-watchdog)"
    fi
    
    if [ -f "$SYSTEMD_DIR/omega-netboot.service" ]; then
        systemctl enable omega-netboot.service
        echo "✅ Enabled omega-netboot.service (runs on boot)"
    fi
fi

# Create state directory
mkdir -p /etc/omega-net
echo "✅ Created /etc/omega-net directory"

# Create log directory
mkdir -p /var/log
touch /var/log/omega-nettoggle.log
chmod 666 /var/log/omega-nettoggle.log
echo "✅ Created log file: /var/log/omega-nettoggle.log"

echo ""
echo "✅ Installation complete!"
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  Omega Network System v2 (NetworkManager Native)"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "Network Management Commands:"
echo ""
echo "  Omega-NetToggle (Quick Mode Switching):"
echo "    sudo omega-nettoggle restore  # Restore WiFi client mode"
echo "    sudo omega-nettoggle ap       # Enable AP mode"
echo "    sudo omega-nettoggle status   # Show network diagnostics"
echo ""
echo "  Network Wizard (Full Configuration):"
if [ -f "$INSTALL_DIR/omega-network" ]; then
    echo "    sudo omega-network ap          # Enable AP mode"
    echo "    sudo omega-network client      # Enable client mode"
    echo "    sudo omega-network status      # Show network status"
    echo "    sudo omega-network validate    # Validate configuration"
    echo "    sudo omega-network logs        # View logs"
fi
echo ""
echo "  System Services:"
echo "    sudo systemctl start omega-network-watchdog   # Start watchdog"
echo "    sudo systemctl status omega-network-watchdog  # Check watchdog status"
echo "    sudo systemctl status omega-netboot           # Check boot service"
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "📝 Next Steps:"
echo "  1. Test client mode: sudo omega-nettoggle restore"
echo "  2. Test AP mode: sudo omega-nettoggle ap"
echo "  3. Start watchdog: sudo systemctl start omega-network-watchdog"
echo ""
