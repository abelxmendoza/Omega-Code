#!/usr/bin/env python3
"""
Script to update the movement server with correct center positions
Run this on omega1 device to update the servo center positions
"""

import os
import re

def update_movement_server():
    file_path = "movement_ws_server.py"
    
    if not os.path.exists(file_path):
        print(f"Error: {file_path} not found!")
        print("Make sure you're in the movement directory on omega1")
        return False
    
    # Read the file
    with open(file_path, 'r') as f:
        content = f.read()
    
    # Update initial servo positions (around line 200-201)
    content = re.sub(
        r'current_horizontal_angle\s*=\s*90',
        'current_horizontal_angle  = 75   # Updated to match current position',
        content
    )
    content = re.sub(
        r'current_vertical_angle\s*=\s*90',
        'current_vertical_angle    = 20   # Updated to match current position',
        content
    )
    
    # Update reset-servo command (around line 668-677)
    reset_pattern = r'(elif cmd == "reset-servo":\s*\n\s*)(current_horizontal_angle = 90\s*\n\s*current_vertical_angle = 90)'
    reset_replacement = r'\1# Use current positions as the new "center" positions\n    # Horizontal: 75° (was 90°), Vertical: 20° (was 90°)\n    current_horizontal_angle = 75\n    current_vertical_angle = 20'
    
    content = re.sub(reset_pattern, reset_replacement, content, flags=re.MULTILINE)
    
    # Write the updated file
    with open(file_path, 'w') as f:
        f.write(content)
    
    print("✅ Successfully updated movement_ws_server.py")
    print("   - Horizontal center: 75° (was 90°)")
    print("   - Vertical center: 20° (was 90°)")
    print("   - Reset command updated")
    print("\n🔄 Now restart the movement server:")
    print("   python3 movement_ws_server.py")
    
    return True

if __name__ == "__main__":
    update_movement_server()

