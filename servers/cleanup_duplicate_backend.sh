#!/bin/bash
# Quick script to remove the duplicate robot-controller-backend directory
# This directory only contains __pycache__ and logs, so it's safe to remove

set -e

SERVERS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SERVERS_DIR"

OLD_DIR="robot-controller-backend"
NEW_DIR="robot_controller_backend"

echo "🧹 Cleaning up duplicate backend directory"
echo "=========================================="
echo ""

if [ ! -d "$OLD_DIR" ]; then
    echo "✅ $OLD_DIR doesn't exist - nothing to clean up!"
    exit 0
fi

if [ ! -d "$NEW_DIR" ]; then
    echo "❌ ERROR: $NEW_DIR doesn't exist!"
    echo "   Cannot remove $OLD_DIR without primary directory"
    exit 1
fi

echo "Checking contents of $OLD_DIR..."
echo ""

# Show what's in the old directory
echo "Contents:"
find "$OLD_DIR" -type f -o -type d | head -20 | sed 's/^/   /'
TOTAL_FILES=$(find "$OLD_DIR" -type f | wc -l)
TOTAL_DIRS=$(find "$OLD_DIR" -type d | wc -l)
echo ""
echo "   Total files: $TOTAL_FILES"
echo "   Total directories: $TOTAL_DIRS"
echo ""

# Check if it's mostly just pycache
PYCACHE_COUNT=$(find "$OLD_DIR" -type d -name "__pycache__" | wc -l)
if [ "$PYCACHE_COUNT" -gt 0 ]; then
    echo "   Found $PYCACHE_COUNT __pycache__ directories"
fi

echo ""
echo "⚠️  This will remove: $OLD_DIR"
echo "   Primary directory ($NEW_DIR) will remain untouched"
echo ""

read -p "Remove duplicate directory? (y/N) " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "❌ Cancelled"
    exit 1
fi

echo ""
echo "🗑️  Removing $OLD_DIR..."
rm -rf "$OLD_DIR"
echo "✅ Removed $OLD_DIR"
echo ""
echo "✅ Cleanup complete!"
echo "   Only $NEW_DIR remains (the primary directory)"

