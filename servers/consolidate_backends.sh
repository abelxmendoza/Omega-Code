#!/bin/bash
# Script to consolidate robot-controller-backend and robot_controller_backend directories
# This script safely merges any differences and removes the duplicate directory

set -e

SERVERS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SERVERS_DIR"

OLD_DIR="robot-controller-backend"
NEW_DIR="robot_controller_backend"

echo "🔍 Backend Directory Consolidation Script"
echo "=========================================="
echo ""
echo "Checking directories in: $SERVERS_DIR"
echo ""

# Check if both directories exist
if [ ! -d "$OLD_DIR" ] && [ ! -d "$NEW_DIR" ]; then
    echo "❌ Neither directory exists!"
    exit 1
fi

if [ ! -d "$OLD_DIR" ]; then
    echo "✅ Only $NEW_DIR exists - no consolidation needed!"
    exit 0
fi

if [ ! -d "$NEW_DIR" ]; then
    echo "⚠️  Only $OLD_DIR exists - renaming to $NEW_DIR..."
    mv "$OLD_DIR" "$NEW_DIR"
    echo "✅ Renamed $OLD_DIR to $NEW_DIR"
    exit 0
fi

echo "📋 Both directories exist. Analyzing differences..."
echo ""

# Create temporary directory for comparison
TMP_DIR=$(mktemp -d)
trap "rm -rf $TMP_DIR" EXIT

# Function to get file list with checksums
get_file_list() {
    local dir=$1
    find "$dir" -type f -exec md5sum {} \; | sort
}

# Compare directories
echo "🔍 Comparing directory structures..."
DIFF_OUTPUT=$(diff -rq "$OLD_DIR" "$NEW_DIR" 2>&1 || true)

if [ -z "$DIFF_OUTPUT" ]; then
    echo "✅ Directories are identical!"
    echo ""
    echo "🗑️  Removing duplicate directory: $OLD_DIR"
    rm -rf "$OLD_DIR"
    echo "✅ Consolidation complete!"
    exit 0
fi

echo "⚠️  Directories differ. Analyzing differences..."
echo ""

# Get file lists
get_file_list "$OLD_DIR" > "$TMP_DIR/old_files.txt"
get_file_list "$NEW_DIR" > "$TMP_DIR/new_files.txt"

# Find files only in old directory
ONLY_IN_OLD=$(comm -23 <(cut -d' ' -f3- "$TMP_DIR/old_files.txt" | sort) \
                       <(cut -d' ' -f3- "$TMP_DIR/new_files.txt" | sort))

# Find files only in new directory
ONLY_IN_NEW=$(comm -13 <(cut -d' ' -f3- "$TMP_DIR/old_files.txt" | sort) \
                       <(cut -d' ' -f3- "$TMP_DIR/new_files.txt" | sort))

# Find files that differ
DIFFERENT_FILES=$(comm -12 <(cut -d' ' -f3- "$TMP_DIR/old_files.txt" | sort) \
                           <(cut -d' ' -f3- "$TMP_DIR/new_files.txt" | sort) | while read file; do
    old_hash=$(grep "  $file$" "$TMP_DIR/old_files.txt" | cut -d' ' -f1)
    new_hash=$(grep "  $file$" "$TMP_DIR/new_files.txt" | cut -d' ' -f1)
    if [ "$old_hash" != "$new_hash" ]; then
        echo "$file"
    fi
done)

echo "📊 Analysis Results:"
echo "   Files only in $OLD_DIR: $(echo "$ONLY_IN_OLD" | grep -c . || echo 0)"
echo "   Files only in $NEW_DIR: $(echo "$ONLY_IN_NEW" | grep -c . || echo 0)"
echo "   Files that differ: $(echo "$DIFFERENT_FILES" | grep -c . || echo 0)"
echo ""

# Show files only in old directory
if [ ! -z "$ONLY_IN_OLD" ]; then
    echo "📁 Files only in $OLD_DIR (will be copied to $NEW_DIR):"
    echo "$ONLY_IN_OLD" | sed 's/^/   - /'
    echo ""
fi

# Show files only in new directory
if [ ! -z "$ONLY_IN_NEW" ]; then
    echo "📁 Files only in $NEW_DIR (already present):"
    echo "$ONLY_IN_NEW" | head -10 | sed 's/^/   - /'
    if [ $(echo "$ONLY_IN_NEW" | wc -l) -gt 10 ]; then
        echo "   ... and $(($(echo "$ONLY_IN_NEW" | wc -l) - 10)) more"
    fi
    echo ""
fi

# Show differing files
if [ ! -z "$DIFFERENT_FILES" ]; then
    echo "⚠️  Files that differ between directories:"
    echo "$DIFFERENT_FILES" | head -10 | sed 's/^/   - /'
    if [ $(echo "$DIFFERENT_FILES" | wc -l) -gt 10 ]; then
        echo "   ... and $(($(echo "$DIFFERENT_FILES" | wc -l) - 10)) more"
    fi
    echo ""
    echo "   These files will be backed up before overwriting."
    echo ""
fi

# Ask for confirmation
echo "⚠️  This will:"
echo "   1. Copy unique files from $OLD_DIR to $NEW_DIR"
echo "   2. Backup differing files in $NEW_DIR before overwriting"
echo "   3. Remove $OLD_DIR after successful merge"
echo ""
read -p "Continue with merge? (y/N) " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "❌ Cancelled"
    exit 1
fi

# Create backup directory
BACKUP_DIR="${NEW_DIR}.backup.$(date +%Y%m%d_%H%M%S)"
echo ""
echo "💾 Creating backup: $BACKUP_DIR"
cp -r "$NEW_DIR" "$BACKUP_DIR"
echo "✅ Backup created"
echo ""

# Copy files only in old directory
if [ ! -z "$ONLY_IN_OLD" ]; then
    echo "📋 Copying unique files from $OLD_DIR to $NEW_DIR..."
    echo "$ONLY_IN_OLD" | while read -r file; do
        src="$OLD_DIR/$file"
        dst="$NEW_DIR/$file"
        dst_dir=$(dirname "$dst")
        mkdir -p "$dst_dir"
        cp -v "$src" "$dst"
    done
    echo "✅ Unique files copied"
    echo ""
fi

# Handle differing files - backup new version, then copy old version
if [ ! -z "$DIFFERENT_FILES" ]; then
    echo "⚠️  Handling differing files..."
    echo "$DIFFERENT_FILES" | while read -r file; do
        src="$OLD_DIR/$file"
        dst="$NEW_DIR/$file"
        backup="$BACKUP_DIR/$file"
        
        # Backup the new version if it exists
        if [ -f "$dst" ]; then
            echo "   Backing up: $file"
            mkdir -p "$(dirname "$backup")"
            cp "$dst" "$backup"
        fi
        
        # Copy old version
        echo "   Copying: $file"
        dst_dir=$(dirname "$dst")
        mkdir -p "$dst_dir"
        cp "$src" "$dst"
    done
    echo "✅ Differing files handled"
    echo ""
fi

# Remove old directory
echo "🗑️  Removing duplicate directory: $OLD_DIR"
rm -rf "$OLD_DIR"
echo "✅ Removed $OLD_DIR"
echo ""

echo "✅ Consolidation complete!"
echo ""
echo "📋 Summary:"
echo "   - Backup created: $BACKUP_DIR"
echo "   - Unique files copied: $(echo "$ONLY_IN_OLD" | grep -c . || echo 0)"
echo "   - Differing files merged: $(echo "$DIFFERENT_FILES" | grep -c . || echo 0)"
echo "   - Old directory removed: $OLD_DIR"
echo ""
echo "⚠️  Review the backup if needed: $BACKUP_DIR"
echo "   You can remove it once you've verified everything works."

