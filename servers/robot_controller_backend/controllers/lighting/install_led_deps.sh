#!/usr/bin/env bash
# Install LED dependencies (rpi-ws281x) in the virtual environment
# This script ensures the lighting server can control NeoPixel LED strips

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BACKEND_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"

echo "🔍 Checking for virtual environment..."

# Find virtual environment Python
VENV_PYTHON=""
VENV_PIP=""
if [ -f "$BACKEND_DIR/venv/bin/python3" ]; then
    VENV_PYTHON="$BACKEND_DIR/venv/bin/python3"
    VENV_PIP="$BACKEND_DIR/venv/bin/pip"
elif [ -f "$BACKEND_DIR/venv/bin/python" ]; then
    VENV_PYTHON="$BACKEND_DIR/venv/bin/python"
    VENV_PIP="$BACKEND_DIR/venv/bin/pip"
fi

if [ -z "$VENV_PYTHON" ]; then
    echo "❌ Virtual environment not found at $BACKEND_DIR/venv"
    echo "💡 Creating virtual environment..."
    cd "$BACKEND_DIR"
    python3 -m venv venv
    VENV_PYTHON="$BACKEND_DIR/venv/bin/python3"
    VENV_PIP="$BACKEND_DIR/venv/bin/pip"
    echo "✅ Created virtual environment"
fi

echo "✅ Using virtual environment Python: $VENV_PYTHON"
echo ""

# Check if rpi-ws281x is already installed
echo "🔍 Checking for rpi-ws281x..."
if "$VENV_PYTHON" -c "import rpi_ws281x" 2>/dev/null; then
    echo "✅ rpi-ws281x is already installed"
    "$VENV_PIP" show rpi-ws281x | grep -E "Name|Version|Location"
else
    echo "❌ rpi-ws281x not found"
    echo "📦 Installing rpi-ws281x..."
    
    # Upgrade pip first (required for some packages)
    "$VENV_PIP" install --upgrade pip --quiet
    
    # Install rpi-ws281x
    if "$VENV_PIP" install rpi-ws281x --quiet; then
        echo "✅ Successfully installed rpi-ws281x"
        "$VENV_PIP" show rpi-ws281x | grep -E "Name|Version|Location"
    else
        echo "❌ Failed to install rpi-ws281x"
        echo "💡 Try manually: $VENV_PIP install rpi-ws281x"
        exit 1
    fi
fi

echo ""
echo "✅ LED dependencies installed successfully!"
echo "💡 The lighting server should now be able to control LED strips."

