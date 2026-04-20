#!/usr/bin/env bash
# =============================================================================
# setup_fresh_pi.sh -- Omega-1 full bootstrap for a new Pi or SD card
# =============================================================================
# Run this once on a fresh Ubuntu 22.04 aarch64 Pi image to bring up the
# complete Omega-1 stack: system packages, Python venv, Go, Node.js, crontab,
# and all services.
#
# Usage (run as the robot user, NOT root):
#   git clone <repo> ~/Desktop/Omega-Code
#   cd ~/Desktop/Omega-Code
#   chmod +x scripts/setup_fresh_pi.sh
#   ./scripts/setup_fresh_pi.sh
#
# What this does:
#   1. System apt packages (Python, Go, Node.js, I2C, GPIO, camera, BT, GPIO)
#   2. Python venv + pip install -r requirements.txt
#   3. Go dependencies (sensors/ultrasonic WebSocket server)
#   4. Node.js + npm install for UI (optional, UI usually runs on dev machine)
#   5. Pi hardware config (I2C, SPI, camera enable via raspi-config / config.txt)
#   6. udev rule for Xbox controller
#   7. @reboot crontab entries for all Omega-1 services
#   8. Servo neutral position on boot (via Freenove PCA9685)
#
# After this script:
#   - Reboot the Pi
#   - The main FastAPI (port 8000), movement WS, line-tracking WS, and
#     ultrasonic (Go) WS all start automatically via crontab @reboot
#   - Point the UI's NEXT_PUBLIC_ROBOT_HOST env var to this Pi's IP
#
# =============================================================================

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BACKEND="$REPO_ROOT/servers/robot_controller_backend"
UI="$REPO_ROOT/ui/robot-controller-ui"
USER_HOME="$HOME"
PI_USER="$(whoami)"

echo "=== Omega-1 Fresh Pi Setup (steps 1–9) ==="
echo "Repo:    $REPO_ROOT"
echo "Backend: $BACKEND"
echo "User:    $PI_USER ($USER_HOME)"
echo ""

# ─────────────────────────────────────────────────────────────────────────────
# 1. System packages
# ─────────────────────────────────────────────────────────────────────────────
echo "[1/8] Installing system packages..."
sudo apt update
sudo apt install -y \
    python3 python3-pip python3-venv python3-dev \
    python3-smbus i2c-tools \
    golang-go \
    git curl wget \
    libgpiod2 libgpiod-dev \
    bluez bluez-tools rfkill \
    libjpeg-dev libopenjp2-7 libatlas-base-dev \
    libcamera-dev libcamera-apps \
    v4l-utils \
    build-essential cmake pkg-config \
    netcat-openbsd dnsutils iproute2

# picamera2 from apt (pip version doesn't work on Pi)
sudo apt install -y python3-picamera2 --no-install-recommends || true

# OpenCV with GStreamer (system build — pip opencv has no GStreamer on aarch64)
sudo apt install -y python3-opencv || true

echo "  System packages installed."

# ─────────────────────────────────────────────────────────────────────────────
# 2. Python venv
# ─────────────────────────────────────────────────────────────────────────────
echo "[2/8] Setting up Python venv..."
cd "$BACKEND"
if [ ! -d venv ]; then
    python3 -m venv venv --system-site-packages
    echo "  venv created."
else
    echo "  venv already exists."
fi

# Fix requirements.txt if malformed (legacy issue)
venv/bin/pip install --upgrade pip

# Install all Python dependencies
venv/bin/pip install -r requirements.txt

echo "  Python deps installed."

# ─────────────────────────────────────────────────────────────────────────────
# 3. Go dependencies (ultrasonic WebSocket server)
# ─────────────────────────────────────────────────────────────────────────────
echo "[3/8] Fetching Go dependencies..."
cd "$BACKEND"
if [ -f go.mod ]; then
    go mod download
    echo "  Go deps fetched."
else
    echo "  No go.mod found — skipping."
fi

# ─────────────────────────────────────────────────────────────────────────────
# 4. Node.js / UI (optional — UI usually runs on dev machine)
# ─────────────────────────────────────────────────────────────────────────────
echo "[4/8] UI dependencies (optional — skip if running UI on dev machine)..."
if command -v node &>/dev/null; then
    echo "  Node.js $(node --version) found."
    if [ -f "$UI/package.json" ]; then
        cd "$UI"
        npm ci --prefer-offline 2>/dev/null || npm install
        echo "  npm install done."
    fi
else
    echo "  Node.js not found — installing via NodeSource..."
    curl -fsSL https://deb.nodesource.com/setup_20.x | sudo -E bash -
    sudo apt install -y nodejs
    cd "$UI"
    npm ci --prefer-offline 2>/dev/null || npm install
    echo "  npm install done."
fi

# ─────────────────────────────────────────────────────────────────────────────
# 5. Pi hardware config (I2C, SPI, camera)
# ─────────────────────────────────────────────────────────────────────────────
echo "[5/8] Configuring Pi hardware interfaces..."

# Enable I2C (needed for PCA9685 servo/motor driver)
if ! grep -q "^dtparam=i2c_arm=on" /boot/firmware/config.txt 2>/dev/null; then
    echo "dtparam=i2c_arm=on" | sudo tee -a /boot/firmware/config.txt
    echo "  I2C enabled in config.txt."
else
    echo "  I2C already enabled."
fi

# Enable SPI (needed for some sensors)
if ! grep -q "^dtparam=spi=on" /boot/firmware/config.txt 2>/dev/null; then
    echo "dtparam=spi=on" | sudo tee -a /boot/firmware/config.txt
    echo "  SPI enabled in config.txt."
fi

# Enable camera (ov5647 CSI)
if ! grep -q "^start_x=1\|^camera_auto_detect=1" /boot/firmware/config.txt 2>/dev/null; then
    echo "camera_auto_detect=1" | sudo tee -a /boot/firmware/config.txt
    echo "  Camera auto-detect enabled."
fi

# Add user to required groups
sudo usermod -aG i2c,spi,gpio,video,dialout "$PI_USER" 2>/dev/null || true
echo "  User groups updated."

# ─────────────────────────────────────────────────────────────────────────────
# 6. udev rule for Xbox controller
# ─────────────────────────────────────────────────────────────────────────────
echo "[6/8] Installing Xbox controller udev rule..."
UDEV_SRC="$BACKEND/movement/99-xbox-controller.rules"
if [ -f "$UDEV_SRC" ]; then
    sudo cp "$UDEV_SRC" /etc/udev/rules.d/
    sudo udevadm control --reload-rules
    echo "  Xbox udev rule installed."
else
    echo "  No udev rule found at $UDEV_SRC — skipping."
fi

# ─────────────────────────────────────────────────────────────────────────────
# 7. Crontab (@reboot services)
# ─────────────────────────────────────────────────────────────────────────────
echo "[7/8] Installing @reboot crontab..."

CRONTAB_BLOCK="

# --- Omega-1 servo neutral on boot (via Freenove PCA9685) ---
@reboot sleep 8 && cd /home/omega1/Desktop/Freenove_4WD_Smart_Car_Kit_for_Raspberry_Pi/Code/Server && python3 -c \"from pca9685 import PCA9685; pwm = PCA9685(0x40); pwm.set_pwm_freq(50); pwm.set_servo_pulse(8, 1500); pwm.set_servo_pulse(9, 1500)\" 2>/dev/null || true

# --- Omega-1 WebSocket servers ---
@reboot sleep 12 && cd $BACKEND && venv/bin/python3 movement/movement_ws_server.py >> /tmp/movement_ws.log 2>&1 &
@reboot sleep 12 && cd $BACKEND && venv/bin/python3 sensors/line_tracking_ws_server.py >> /tmp/line_tracker.log 2>&1 &
@reboot sleep 12 && cd $BACKEND/sensors && go run main_ultrasonic.go >> /tmp/ultrasonic.log 2>&1 &
@reboot sleep 15 && cd $BACKEND && venv/bin/python3 ./venv/bin/uvicorn main_api:app --host 0.0.0.0 --port 8000 >> /tmp/main_api.log 2>&1 &
"

# Only add if not already present
if ! crontab -l 2>/dev/null | grep -q "Omega-1 WebSocket servers"; then
    (crontab -l 2>/dev/null; echo "$CRONTAB_BLOCK") | crontab -
    echo "  Crontab entries added."
else
    echo "  Crontab already configured — skipping."
fi

# ─────────────────────────────────────────────────────────────────────────────
# 8. Camera stack (libcamera + picamera2 shims)
# ─────────────────────────────────────────────────────────────────────────────
echo "[8/8] Camera stack setup..."
CAMERA_SCRIPT="$REPO_ROOT/scripts/setup_pi_camera.sh"
if [ -f "$CAMERA_SCRIPT" ]; then
    bash "$CAMERA_SCRIPT"
else
    echo "  Camera setup script not found at $CAMERA_SCRIPT — skipping."
    echo "  Run scripts/setup_pi_camera.sh separately after clone."
fi

# ─────────────────────────────────────────────────────────────────────────────
# 9. AP mode (field hotspot + auto-fallback)
# ─────────────────────────────────────────────────────────────────────────────
echo "[9/9] Configuring AP mode..."
AP_SCRIPT="$REPO_ROOT/scripts/setup_ap_mode.sh"
if [ -f "$AP_SCRIPT" ]; then
    bash "$AP_SCRIPT"
else
    echo "  AP setup script not found — skipping. Run scripts/setup_ap_mode.sh manually."
fi

# ─────────────────────────────────────────────────────────────────────────────
# Done
# ─────────────────────────────────────────────────────────────────────────────
echo ""
echo "============================================================"
echo "  Omega-1 setup complete!"
echo "============================================================"
echo ""
echo "  Next steps:"
echo "    1. Reboot the Pi:  sudo reboot"
echo "    2. After reboot, check service logs:"
echo "       tail -f /tmp/main_api.log"
echo "       tail -f /tmp/movement_ws.log"
echo "       tail -f /tmp/line_tracker.log"
echo "       tail -f /tmp/ultrasonic.log"
echo "    3. Point your dev machine UI at this Pi's IP:"
echo "       NEXT_PUBLIC_ROBOT_HOST=<pi-ip> npm run dev"
echo ""
echo "  To verify hardware:"
echo "       i2cdetect -y 1   # should show 0x40 (PCA9685)"
echo "       vcgencmd get_camera  # should show supported=1"
echo ""
