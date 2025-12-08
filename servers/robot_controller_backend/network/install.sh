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
echo "Usage:"
echo "  sudo omega-network ap          # Enable AP mode"
echo "  sudo omega-network client      # Enable client mode"
echo "  sudo omega-network status      # Show network status"
echo "  sudo omega-network validate    # Validate configuration"
echo "  sudo omega-network logs        # View logs"
echo ""

