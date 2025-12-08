#!/bin/bash
# OmegaOS Service Orchestrator Installation Script

set -e

echo "=========================================="
echo "OmegaOS Service Orchestrator Installation"
echo "=========================================="

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BACKEND_DIR="$(dirname "$SCRIPT_DIR")"
PROJECT_ROOT="$(dirname "$(dirname "$SCRIPT_DIR")")"

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Check if running as root for systemd installation
if [ "$EUID" -ne 0 ]; then 
    echo -e "${YELLOW}Warning: Not running as root. Systemd service installation will be skipped.${NC}"
    INSTALL_SYSTEMD=false
else
    INSTALL_SYSTEMD=true
fi

# 1. Check Python dependencies
echo ""
echo "Checking Python dependencies..."
if ! python3 -c "import psutil" 2>/dev/null; then
    echo -e "${YELLOW}psutil not found. Installing...${NC}"
    pip3 install psutil
else
    echo -e "${GREEN}✓ psutil is installed${NC}"
fi

# 2. Create logs directory
echo ""
echo "Creating logs directory..."
mkdir -p "$SCRIPT_DIR/logs"
chmod 755 "$SCRIPT_DIR/logs"
echo -e "${GREEN}✓ Logs directory created${NC}"

# 3. Verify service registry
echo ""
echo "Verifying service registry..."
if [ -f "$SCRIPT_DIR/service_registry.json" ]; then
    if python3 -c "import json; json.load(open('$SCRIPT_DIR/service_registry.json'))" 2>/dev/null; then
        echo -e "${GREEN}✓ Service registry is valid JSON${NC}"
    else
        echo -e "${RED}✗ Service registry JSON is invalid!${NC}"
        exit 1
    fi
else
    echo -e "${RED}✗ Service registry not found!${NC}"
    exit 1
fi

# 4. Install systemd service (if root)
if [ "$INSTALL_SYSTEMD" = true ]; then
    echo ""
    echo "Installing systemd service..."
    
    # Update service file with correct paths
    SERVICE_FILE="$SCRIPT_DIR/omega-orchestrator.service"
    SYSTEMD_FILE="/etc/systemd/system/omega-orchestrator.service"
    
    # Get actual paths
    USER_HOME=$(eval echo ~$(logname))
    PYTHON_PATH=$(which python3)
    
    # Create systemd service file
    cat > "$SYSTEMD_FILE" << EOF
[Unit]
Description=OmegaOS Service Orchestrator
Documentation=https://github.com/your-repo/Omega-Code
After=network.target
Wants=network-online.target

[Service]
Type=simple
User=omega1
Group=omega1
WorkingDirectory=$BACKEND_DIR
ExecStart=$PYTHON_PATH -m omega_services.service_manager
Restart=always
RestartSec=10
StandardOutput=journal
StandardError=journal
SyslogIdentifier=omega-orchestrator

# Environment
Environment="PYTHONUNBUFFERED=1"
Environment="PATH=/usr/local/bin:/usr/bin:/bin"

# Security
NoNewPrivileges=true
PrivateTmp=true

[Install]
WantedBy=multi-user.target
EOF
    
    # Reload systemd
    systemctl daemon-reload
    echo -e "${GREEN}✓ Systemd service installed${NC}"
    
    # Ask if user wants to enable/start
    echo ""
    read -p "Enable orchestrator to start on boot? (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        systemctl enable omega-orchestrator
        echo -e "${GREEN}✓ Orchestrator enabled on boot${NC}"
    fi
    
    echo ""
    read -p "Start orchestrator now? (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        systemctl start omega-orchestrator
        echo -e "${GREEN}✓ Orchestrator started${NC}"
        echo ""
        echo "Check status with: sudo systemctl status omega-orchestrator"
        echo "View logs with: sudo journalctl -u omega-orchestrator -f"
    fi
else
    echo ""
    echo -e "${YELLOW}Skipping systemd installation (not running as root)${NC}"
    echo "To install systemd service manually:"
    echo "  sudo cp $SCRIPT_DIR/omega-orchestrator.service /etc/systemd/system/"
    echo "  sudo systemctl daemon-reload"
    echo "  sudo systemctl enable omega-orchestrator"
    echo "  sudo systemctl start omega-orchestrator"
fi

# 5. Test import
echo ""
echo "Testing Python module import..."
cd "$BACKEND_DIR"
if python3 -c "from omega_services import ServiceManager" 2>/dev/null; then
    echo -e "${GREEN}✓ Module imports successfully${NC}"
else
    echo -e "${RED}✗ Module import failed!${NC}"
    echo "Make sure you're in the correct directory and Python path is set."
    exit 1
fi

# 6. Summary
echo ""
echo "=========================================="
echo -e "${GREEN}Installation Complete!${NC}"
echo "=========================================="
echo ""
echo "Service Orchestrator is ready to use."
echo ""
echo "Manual start:"
echo "  cd $BACKEND_DIR"
echo "  python3 -m omega_services.service_manager"
echo ""
echo "API endpoints:"
echo "  GET  /api/services/list"
echo "  GET  /api/services/status/{name}"
echo "  POST /api/services/start/{name}"
echo "  POST /api/services/stop/{name}"
echo "  POST /api/services/restart/{name}"
echo "  GET  /api/services/logs/{name}"
echo ""
echo "Web UI:"
echo "  Navigate to http://omega1.local:3000/services"
echo ""
echo "Documentation:"
echo "  See $SCRIPT_DIR/README.md"
echo ""

