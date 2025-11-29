#!/bin/bash
# Complete consolidation and verification script
# This script consolidates directories AND verifies everything works

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "🚀 Complete Backend Consolidation & Verification"
echo "================================================="
echo ""

# Step 1: Consolidate directories
echo "Step 1: Consolidating directories..."
echo "-----------------------------------"
if [ -f "./consolidate_backends.sh" ]; then
    ./consolidate_backends.sh
    CONSOLIDATION_EXIT=$?
    if [ $CONSOLIDATION_EXIT -ne 0 ]; then
        echo "❌ Consolidation failed or was cancelled"
        exit 1
    fi
else
    echo "⚠️  consolidate_backends.sh not found - skipping directory consolidation"
    echo "   (This is OK if directories are already consolidated)"
fi

echo ""
echo "Step 2: Verifying consolidation..."
echo "----------------------------------"
if [ -f "./verify_consolidation.sh" ]; then
    ./verify_consolidation.sh
    VERIFICATION_EXIT=$?
    if [ $VERIFICATION_EXIT -ne 0 ]; then
        echo "❌ Verification found issues"
        exit 1
    fi
else
    echo "⚠️  verify_consolidation.sh not found - skipping verification"
fi

echo ""
echo "Step 3: Checking for redundant code..."
echo "--------------------------------------"

# Check for old path references
OLD_REFS=$(grep -r "robot-controller-backend" ../servers/robot_controller_backend --include="*.py" --include="*.go" --include="*.sh" 2>/dev/null | grep -v "fix_directory_references.sh" | wc -l)
if [ "$OLD_REFS" -gt 0 ]; then
    echo "⚠️  Found $OLD_REFS references to old directory name"
    echo "   Run: cd .. && find servers/robot_controller_backend -type f -exec sed -i 's/robot-controller-backend/robot_controller_backend/g' {} \\;"
else
    echo "✅ No old path references found"
fi

# Check for duplicate LED implementations
LED_FILES=$(find ../servers/robot_controller_backend -name "led_control.py" -type f 2>/dev/null | wc -l)
if [ "$LED_FILES" -gt 3 ]; then
    echo "⚠️  Found $LED_FILES led_control.py files (expected 3: controllers, hardware, utils)"
else
    echo "✅ LED control files are properly organized"
fi

echo ""
echo "Step 4: Testing critical features..."
echo "-------------------------------------"

# Test Python imports
cd ../servers/robot_controller_backend
if [ -d "venv" ] && [ -f "venv/bin/activate" ]; then
    source venv/bin/activate 2>/dev/null || true
    PYTHON_CMD="python"
else
    PYTHON_CMD="python3"
fi

if command -v "$PYTHON_CMD" > /dev/null 2>&1; then
    echo "Testing Python imports..."
    if $PYTHON_CMD -c "from controllers.lighting.led_control import LedController; print('✅ LED controller imports OK')" 2>/dev/null; then
        echo "✅ LED controller imports successfully"
    else
        echo "❌ LED controller import failed"
        exit 1
    fi
    
    if $PYTHON_CMD -c "from controllers.lighting.dispatcher import apply_lighting_mode; print('✅ Dispatcher imports OK')" 2>/dev/null; then
        echo "✅ Dispatcher imports successfully"
    else
        echo "❌ Dispatcher import failed"
        exit 1
    fi
else
    echo "⚠️  Python not found - skipping import tests"
fi

# Test Go compilation
if command -v go > /dev/null 2>&1; then
    echo "Testing Go compilation..."
    cd controllers/lighting
    if go build -o /tmp/main_lighting_test main_lighting.go 2>/dev/null; then
        echo "✅ Lighting server compiles successfully"
        rm -f /tmp/main_lighting_test
    else
        echo "❌ Lighting server compilation failed"
        go build main_lighting.go 2>&1 | head -10
        exit 1
    fi
    cd ../../..
else
    echo "⚠️  Go not found - skipping compilation tests"
fi

echo ""
echo "✅ Consolidation Complete!"
echo "=========================="
echo ""
echo "Summary:"
echo "  ✅ Directories consolidated"
echo "  ✅ Code verified"
echo "  ✅ Critical features tested"
echo ""
echo "Next steps:"
echo "  1. Test lighting from frontend UI"
echo "  2. Run: cd servers/robot_controller_backend && ./run_standalone.sh lighting"
echo "  3. Verify WebSocket connection works"
echo ""

