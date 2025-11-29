#!/bin/bash
# Comprehensive verification script to ensure consolidation worked and all features work

set -e

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BACKEND_DIR="$PROJECT_ROOT/servers/robot_controller_backend"

echo "🔍 Backend Consolidation Verification"
echo "======================================"
echo ""
echo "Project root: $PROJECT_ROOT"
echo "Backend dir: $BACKEND_DIR"
echo ""

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

ERRORS=0
WARNINGS=0

# Function to check and report
check() {
    local name="$1"
    local command="$2"
    echo -n "Checking $name... "
    if eval "$command" > /dev/null 2>&1; then
        echo -e "${GREEN}✅${NC}"
        return 0
    else
        echo -e "${RED}❌${NC}"
        ERRORS=$((ERRORS + 1))
        return 1
    fi
}

warn() {
    local name="$1"
    local message="$2"
    echo -e "${YELLOW}⚠️  $name: $message${NC}"
    WARNINGS=$((WARNINGS + 1))
}

# 1. Check directory structure
echo "📁 Directory Structure Checks"
echo "----------------------------"
check "Backend directory exists" "[ -d '$BACKEND_DIR' ]"
check "No duplicate directory" "[ ! -d '$PROJECT_ROOT/servers/robot-controller-backend' ]"

if [ -d "$PROJECT_ROOT/servers/robot-controller-backend" ]; then
    warn "Duplicate directory" "robot-controller-backend still exists - run consolidate_backends.sh"
fi

# 2. Check for duplicate/redundant code
echo ""
echo "🔍 Code Duplication Checks"
echo "---------------------------"

# Check for old API routes that might be redundant
if [ -f "$BACKEND_DIR/api/lighting_routes.py" ]; then
    if grep -q "subprocess.run" "$BACKEND_DIR/api/lighting_routes.py" 2>/dev/null; then
        warn "Redundant API" "api/lighting_routes.py uses subprocess (less efficient than controllers/lighting/lighting_routes.py)"
    fi
fi

# Check for multiple LED control implementations
LED_CONTROL_FILES=$(find "$BACKEND_DIR" -name "led_control.py" -type f 2>/dev/null | wc -l)
if [ "$LED_CONTROL_FILES" -gt 1 ]; then
    echo "Found $LED_CONTROL_FILES led_control.py files:"
    find "$BACKEND_DIR" -name "led_control.py" -type f 2>/dev/null | sed 's|^|   - |'
    echo ""
    echo "   These serve different purposes:"
    echo "   - controllers/lighting/led_control.py (production)"
    echo "   - hardware/led_control.py (async hardware abstraction)"
    echo "   - utils/led_control.py (backward compatibility wrapper)"
fi

# 3. Check for old path references
echo ""
echo "🔗 Path Reference Checks"
echo "-------------------------"
OLD_REF_COUNT=$(grep -r "robot-controller-backend" "$BACKEND_DIR" --include="*.py" --include="*.go" --include="*.sh" 2>/dev/null | grep -v "fix_directory_references.sh" | wc -l)
if [ "$OLD_REF_COUNT" -gt 0 ]; then
    warn "Old path references" "Found $OLD_REF_COUNT references to robot-controller-backend"
    echo "   Files with old references:"
    grep -r "robot-controller-backend" "$BACKEND_DIR" --include="*.py" --include="*.go" --include="*.sh" 2>/dev/null | grep -v "fix_directory_references.sh" | cut -d: -f1 | sort -u | head -5 | sed 's|^|   - |'
else
    check "No old path references" "true"
fi

# 4. Check critical files exist
echo ""
echo "📄 Critical Files Check"
echo "-----------------------"
check "LED controller exists" "[ -f '$BACKEND_DIR/controllers/lighting/led_control.py' ]"
check "Lighting WebSocket server exists" "[ -f '$BACKEND_DIR/controllers/lighting/main_lighting.go' ]"
check "LED run script exists" "[ -f '$BACKEND_DIR/controllers/lighting/run_led.sh' ]"
check "Dispatcher exists" "[ -f '$BACKEND_DIR/controllers/lighting/dispatcher.py' ]"
check "Patterns exist" "[ -f '$BACKEND_DIR/controllers/lighting/patterns.py' ]"

# 5. Check script permissions
echo ""
echo "🔐 Script Permissions"
echo "---------------------"
if [ -f "$BACKEND_DIR/controllers/lighting/run_led.sh" ]; then
    if [ -x "$BACKEND_DIR/controllers/lighting/run_led.sh" ]; then
        check "run_led.sh is executable" "true"
    else
        warn "run_led.sh" "Not executable - run: chmod +x $BACKEND_DIR/controllers/lighting/run_led.sh"
    fi
fi

# 6. Check Python imports work
echo ""
echo "🐍 Python Import Checks"
echo "----------------------"
cd "$BACKEND_DIR"
if [ -d "venv" ]; then
    if [ -f "venv/bin/activate" ]; then
        source venv/bin/activate 2>/dev/null || true
        PYTHON_CMD="python"
    else
        PYTHON_CMD="python3"
    fi
else
    PYTHON_CMD="python3"
fi

if command -v "$PYTHON_CMD" > /dev/null 2>&1; then
    check "Python available" "command -v $PYTHON_CMD"
    
    # Try importing LED controller
    if $PYTHON_CMD -c "from controllers.lighting.led_control import LedController" 2>/dev/null; then
        check "LED controller imports" "true"
    else
        warn "LED controller import" "Failed to import LedController"
        echo "   Try: cd $BACKEND_DIR && $PYTHON_CMD -c 'from controllers.lighting.led_control import LedController'"
    fi
    
    # Try importing dispatcher
    if $PYTHON_CMD -c "from controllers.lighting.dispatcher import apply_lighting_mode" 2>/dev/null; then
        check "Dispatcher imports" "true"
    else
        warn "Dispatcher import" "Failed to import dispatcher"
    fi
else
    warn "Python" "Python not found - skipping import checks"
fi

# 7. Check Go compilation
echo ""
echo "🔨 Go Compilation Checks"
echo "------------------------"
if command -v go > /dev/null 2>&1; then
    check "Go available" "command -v go"
    
    cd "$BACKEND_DIR/controllers/lighting"
    if go build -o /dev/null main_lighting.go 2>/dev/null; then
        check "Lighting server compiles" "true"
    else
        warn "Lighting server" "Failed to compile - check Go dependencies"
        go build main_lighting.go 2>&1 | head -5
    fi
    cd "$PROJECT_ROOT"
else
    warn "Go" "Go not found - skipping compilation checks"
fi

# 8. Check WebSocket endpoints
echo ""
echo "🌐 WebSocket Endpoint Checks"
echo "----------------------------"
if [ -f "$BACKEND_DIR/controllers/lighting/main_lighting.go" ]; then
    if grep -q 'PORT_LIGHTING' "$BACKEND_DIR/controllers/lighting/main_lighting.go"; then
        check "Port configurable" "true"
    fi
    
    if grep -q '/lighting' "$BACKEND_DIR/controllers/lighting/main_lighting.go"; then
        check "WebSocket path configured" "true"
    fi
fi

# 9. Check for duplicate route definitions
echo ""
echo "🛣️  Route Definition Checks"
echo "---------------------------"
LIGHTING_ROUTE_FILES=$(find "$BACKEND_DIR" -name "*lighting*route*.py" -type f 2>/dev/null | wc -l)
if [ "$LIGHTING_ROUTE_FILES" -gt 1 ]; then
    echo "Found $LIGHTING_ROUTE_FILES lighting route files:"
    find "$BACKEND_DIR" -name "*lighting*route*.py" -type f 2>/dev/null | sed 's|^|   - |'
    echo ""
    echo "   Review these to ensure no duplicate route definitions"
fi

# 10. Summary
echo ""
echo "📊 Summary"
echo "=========="
echo -e "Errors: ${RED}$ERRORS${NC}"
echo -e "Warnings: ${YELLOW}$WARNINGS${NC}"
echo ""

if [ $ERRORS -eq 0 ] && [ $WARNINGS -eq 0 ]; then
    echo -e "${GREEN}✅ All checks passed! Consolidation successful.${NC}"
    exit 0
elif [ $ERRORS -eq 0 ]; then
    echo -e "${YELLOW}⚠️  Consolidation complete with warnings. Review above.${NC}"
    exit 0
else
    echo -e "${RED}❌ Consolidation has errors. Please fix above issues.${NC}"
    exit 1
fi

