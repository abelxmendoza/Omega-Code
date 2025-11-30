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

# Check and install dependencies
echo "🔍 Checking dependencies..."

# Function to check and install a package
check_and_install_pkg() {
    local package=$1
    local import_name=${2:-$package}
    
    if "$VENV_PYTHON" -c "import $import_name" 2>/dev/null; then
        echo "✅ $package is already installed"
        "$VENV_PIP" show "$package" | grep -E "Name|Version|Location" || true
        return 0
    else
        echo "❌ $package not found"
        echo "📦 Installing $package..."
        if "$VENV_PIP" install "$package" --quiet; then
            echo "✅ Successfully installed $package"
            return 0
        else
            echo "❌ Failed to install $package"
            return 1
        fi
    fi
}

# Upgrade pip first (required for some packages)
echo "📦 Upgrading pip..."
"$VENV_PIP" install --upgrade pip --quiet >/dev/null 2>&1 || true

# Install rpi-ws281x (required for LED control)
check_and_install_pkg "rpi-ws281x" "rpi_ws281x" || exit 1

# Install numpy (required for patterns.py)
check_and_install_pkg "numpy" "numpy" || exit 1

echo ""
echo "✅ LED dependencies installed successfully!"
echo "💡 The lighting server should now be able to control LED strips."

