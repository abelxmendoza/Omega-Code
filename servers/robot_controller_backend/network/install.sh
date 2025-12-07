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

# Make scripts executable
chmod +x "$SCRIPT_DIR/network_wizard.py"
chmod +x "$SCRIPT_DIR/network_cli.py"

# Create symlink for omega-network command
ln -sf "$SCRIPT_DIR/network_cli.py" "$INSTALL_DIR/omega-network"
chmod +x "$INSTALL_DIR/omega-network"

# Create state directory
mkdir -p /etc/omega-network

# Create log directory
touch /var/log/omega-network.log
chmod 666 /var/log/omega-network.log

echo "✅ Installation complete!"
echo ""
echo "Usage:"
echo "  sudo omega-network ap       # Enable AP mode"
echo "  sudo omega-network client    # Enable client mode"
echo "  sudo omega-network status    # Show network status"
echo "  sudo omega-network           # Interactive menu"
echo ""

