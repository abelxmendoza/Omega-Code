#!/bin/bash
# Omega Network Wizard Installation Script
# Installs the network wizard and creates system symlinks

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INSTALL_DIR="/usr/local/bin"
SYSTEMD_DIR="/etc/systemd/system"

echo "🔧 Installing Omega Network Wizard..."

# Check if running as root
if [ "$EUID" -ne 0 ]; then 
    echo "❌ Please run as root (use sudo)"
    exit 1
fi

# Install Python dependencies
echo "📦 Installing Python dependencies..."
pip3 install jinja2 --quiet || pip install jinja2 --quiet

# Make scripts executable
chmod +x "$SCRIPT_DIR/wizard/network_wizard.py"
chmod +x "$SCRIPT_DIR/cli/omega_network.py"

# Make omega-nettoggle.sh executable and create symlink
NETTOGGLE_SCRIPT="$SCRIPT_DIR/omega-nettoggle.sh"
if [ -f "$NETTOGGLE_SCRIPT" ]; then
    chmod +x "$NETTOGGLE_SCRIPT"
    echo "✅ Made omega-nettoggle.sh executable"
    
    # Create symlink for omega-nettoggle command
    ln -sf "$NETTOGGLE_SCRIPT" "$INSTALL_DIR/omega-nettoggle"
    chmod +x "$INSTALL_DIR/omega-nettoggle"
    echo "✅ Created symlink: /usr/local/bin/omega-nettoggle"
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

# Create symlink for omega-network command
ln -sf "$SCRIPT_DIR/cli/omega_network.py" "$INSTALL_DIR/omega-network"
chmod +x "$INSTALL_DIR/omega-network"

# Create state directory
mkdir -p /etc/omega-network

# Create log directory
mkdir -p /var/log
touch /var/log/omega-network.log
chmod 666 /var/log/omega-network.log

echo "✅ Installation complete!"
echo ""
echo "Network Management Tools:"
echo ""
echo "Omega-NetToggle (Recovery + AP Mode):"
echo "  sudo omega-nettoggle restore  # Restore normal WiFi"
echo "  sudo omega-nettoggle ap      # Enable AP mode"
echo "  sudo omega-nettoggle status  # Show diagnostics"
echo ""
echo "Network Wizard (Full Configuration):"
echo "  sudo omega-network ap          # Enable AP mode"
echo "  sudo omega-network client      # Enable client mode"
echo "  sudo omega-network status      # Show network status"
echo "  sudo omega-network validate    # Validate configuration"
echo "  sudo omega-network logs        # View logs"
echo ""

