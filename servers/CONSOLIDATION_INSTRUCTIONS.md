# Backend Directory Consolidation Instructions

## Overview
This script consolidates the two backend directories (`robot-controller-backend` and `robot_controller_backend`) into a single unified directory.

## Prerequisites
- Run this script from the `servers/` directory
- Ensure you have write permissions
- Make sure no services are running that depend on these directories

## Usage

### Step 1: Navigate to servers directory
```bash
cd ~/Omega-Code/servers
```

### Step 2: Review what will be merged
```bash
# See what's different
diff -r robot-controller-backend robot_controller_backend | head -50

# Or get a summary
./consolidate_backends.sh
# (It will show you what will be merged before doing it)
```

### Step 3: Run the consolidation script
```bash
./consolidate_backends.sh
```

The script will:
1. ✅ Analyze both directories
2. ✅ Show you what's different
3. ✅ Ask for confirmation
4. ✅ Create a backup of `robot_controller_backend`
5. ✅ Copy unique files from `robot-controller-backend` to `robot_controller_backend`
6. ✅ Handle conflicting files (backup new, copy old)
7. ✅ Remove the duplicate `robot-controller-backend` directory

### Step 4: Verify everything works
```bash
cd ~/Omega-Code/servers/robot_controller_backend

# Test that scripts work
./run_standalone.sh lighting

# Check that paths are correct
grep -r "robot-controller-backend" . --include="*.py" --include="*.go" --include="*.sh" | head -5
# Should show no results (or only in fix_directory_references.sh)
```

### Step 5: Clean up backup (optional)
Once you've verified everything works:
```bash
cd ~/Omega-Code/servers
rm -rf robot_controller_backend.backup.*
```

## What the Script Does

### Safety Features
- ✅ Creates a timestamped backup before making changes
- ✅ Shows you exactly what will change before doing it
- ✅ Asks for confirmation before proceeding
- ✅ Preserves all files (backups differing files)

### Merge Strategy
1. **Files only in old directory**: Copied to new directory
2. **Files only in new directory**: Left as-is
3. **Files that differ**: 
   - New version backed up
   - Old version copied (preserves any unique content)
4. **Identical files**: Left as-is

## Troubleshooting

### If directories are identical
The script will detect this and simply remove the duplicate directory.

### If you need to restore from backup
```bash
cd ~/Omega-Code/servers
# Find the backup
ls -la robot_controller_backend.backup.*

# Restore if needed
rm -rf robot_controller_backend
mv robot_controller_backend.backup.YYYYMMDD_HHMMSS robot_controller_backend
```

### If merge conflicts occur
The script backs up differing files. Check the backup directory:
```bash
cd ~/Omega-Code/servers
ls -la robot_controller_backend.backup.*/
```

## Manual Alternative

If you prefer to do it manually:

```bash
cd ~/Omega-Code/servers

# 1. Compare directories
diff -r robot-controller-backend robot_controller_backend > diff.txt

# 2. Review differences
less diff.txt

# 3. Copy unique files (example)
cp -r robot-controller-backend/some_unique_file robot_controller_backend/

# 4. Remove duplicate
rm -rf robot-controller-backend
```

