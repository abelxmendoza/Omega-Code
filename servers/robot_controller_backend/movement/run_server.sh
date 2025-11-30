#!/bin/bash
# Run the movement WebSocket server with proper virtual environment activation
# Automatically installs missing dependencies

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BACKEND_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
MOVEMENT_DIR="$SCRIPT_DIR"

# Find and activate virtual environment
VENV_PYTHON=""
VENV_PIP=""
if [ -d "$BACKEND_DIR/venv" ]; then
    VENV_PYTHON="$BACKEND_DIR/venv/bin/python3"
    VENV_PIP="$BACKEND_DIR/venv/bin/pip"
elif [ -d "$BACKEND_DIR/../venv" ]; then
    VENV_PYTHON="$BACKEND_DIR/../venv/bin/python3"
    VENV_PIP="$BACKEND_DIR/../venv/bin/pip"
fi

if [ -n "$VENV_PYTHON" ] && [ -f "$VENV_PYTHON" ]; then
    echo "✅ Using virtual environment Python: $VENV_PYTHON"
    PYTHON_CMD="$VENV_PYTHON"
    PIP_CMD="$VENV_PIP"
else
    echo "⚠️  No virtual environment found. Using system Python."
    echo "⚠️  Warning: Hardware drivers (smbus2) may not be available!"
    PYTHON_CMD="python3"
    PIP_CMD="pip3"
    
    # Try to create venv if it doesn't exist
    echo "💡 Creating virtual environment..."
    if [ -d "$BACKEND_DIR" ]; then
        cd "$BACKEND_DIR"
        $PYTHON_CMD -m venv venv 2>/dev/null && {
            VENV_PYTHON="$BACKEND_DIR/venv/bin/python3"
            VENV_PIP="$BACKEND_DIR/venv/bin/pip"
            PYTHON_CMD="$VENV_PYTHON"
            PIP_CMD="$VENV_PIP"
            echo "✅ Created virtual environment: $VENV_PYTHON"
        } || echo "⚠️  Could not create venv, continuing with system Python"
    fi
fi

# Function to check and install a package
check_and_install() {
    local package=$1
    local import_name=${2:-$package}
    
    echo -n "🔍 Checking $package... "
    if $PYTHON_CMD -c "import $import_name" 2>/dev/null; then
        echo "✅"
        return 0
    else
        echo "❌ Missing"
        echo "   Installing $package..."
        if $PIP_CMD install "$package" --quiet; then
            echo "   ✅ Installed $package"
            return 0
        else
            echo "   ❌ Failed to install $package"
            return 1
        fi
    fi
}

# Check and install required dependencies
echo ""
echo "📦 Checking dependencies..."
echo ""

MISSING_DEPS=0

# Core dependencies for movement server
check_and_install "smbus2" "smbus2" || MISSING_DEPS=1
check_and_install "websockets" "websockets" || MISSING_DEPS=1

# Optional: Check for minimal requirements.txt dependencies
# Only install core dependencies needed for movement server
if [ -f "$BACKEND_DIR/requirements.txt" ]; then
    # Check if we need to install any core dependencies
    # We'll only install minimal ones, not all ML/CV libraries
    CORE_DEPS_INSTALLED=true
    for dep in "websockets" "smbus2"; do
        if ! $PYTHON_CMD -c "import $dep" 2>/dev/null; then
            CORE_DEPS_INSTALLED=false
            break
        fi
    done
    
    if [ "$CORE_DEPS_INSTALLED" = false ]; then
        echo ""
        echo "💡 Note: Full requirements.txt available at $BACKEND_DIR/requirements.txt"
        echo "   (Only installing minimal dependencies needed for movement server)"
    fi
fi

if [ $MISSING_DEPS -eq 1 ]; then
    echo ""
    echo "❌ Some critical dependencies failed to install."
    echo "   Try manually: $PIP_CMD install smbus2 websockets"
    exit 1
fi

# Final verification
echo ""
echo "✅ All dependencies verified"
echo ""

# Load environment variables if direnv is available
if command -v direnv &> /dev/null; then
    cd "$BACKEND_DIR"
    eval "$(direnv export bash)" 2>/dev/null || true
fi

# Change to movement directory
cd "$MOVEMENT_DIR"

# Run the server
echo "🚀 Starting movement WebSocket server..."
echo "   Working directory: $(pwd)"
echo "   Python: $PYTHON_CMD"
echo ""
exec "$PYTHON_CMD" movement_ws_server.py

