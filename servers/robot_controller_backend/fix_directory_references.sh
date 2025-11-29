#!/bin/bash
# Script to fix all references from robot-controller-backend to robot_controller_backend
# This unifies the directory naming convention

set -e

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BACKEND_DIR="$PROJECT_ROOT/servers/robot_controller_backend"

echo "🔍 Finding and fixing references to robot-controller-backend..."
echo "   Project root: $PROJECT_ROOT"
echo "   Backend dir: $BACKEND_DIR"
echo ""

# Find all files containing robot-controller-backend
FILES=$(grep -r "robot-controller-backend" "$PROJECT_ROOT" --include="*.py" --include="*.go" --include="*.sh" --include="*.md" --include="Makefile" --include="*.yml" --include="*.yaml" --include="*.json" 2>/dev/null | cut -d: -f1 | sort -u)

if [ -z "$FILES" ]; then
    echo "✅ No files found with robot-controller-backend references"
    exit 0
fi

echo "📝 Found $(echo "$FILES" | wc -l) files to update:"
echo "$FILES" | sed 's/^/   - /'
echo ""

# Ask for confirmation
read -p "Continue with replacement? (y/N) " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "❌ Cancelled"
    exit 1
fi

# Replace robot-controller-backend with robot_controller_backend in all files
for file in $FILES; do
    if [ -f "$file" ]; then
        echo "   Updating: $file"
        # Use sed to replace, handling both cases
        sed -i.bak 's/robot-controller-backend/robot_controller_backend/g' "$file"
        # Remove backup files
        rm -f "${file}.bak"
    fi
done

echo ""
echo "✅ All references updated!"
echo ""
echo "📋 Summary:"
echo "   - Replaced 'robot-controller-backend' with 'robot_controller_backend'"
echo "   - Updated $(echo "$FILES" | wc -l) files"
echo ""
echo "⚠️  Next steps:"
echo "   1. Review the changes: git diff"
echo "   2. Test that everything still works"
echo "   3. If robot-controller-backend directory exists on server, check if it's different"
echo "   4. If different, merge contents, then remove duplicate directory"

