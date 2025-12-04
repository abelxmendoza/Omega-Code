#!/usr/bin/env python3
"""
Omega-1 Camera Hardware Diagnostic Tool

Runs comprehensive hardware checks to diagnose camera issues.
Use this when camera returns no frames or initialization fails.
"""

import subprocess
import os
import sys


def run(cmd):
    """Run shell command and return output."""
    try:
        out = subprocess.check_output(cmd, shell=True, stderr=subprocess.STDOUT, timeout=10)
        return out.decode('utf-8', errors='replace')
    except subprocess.CalledProcessError as e:
        return e.output.decode('utf-8', errors='replace') if e.output else str(e)
    except subprocess.TimeoutExpired:
        return "Command timed out after 10 seconds"
    except Exception as e:
        return f"Error: {str(e)}"


def check_file_exists(path):
    """Check if file exists and return size."""
    if os.path.exists(path):
        size = os.path.getsize(path)
        return f"✓ Exists ({size} bytes)"
    return "✗ Not found"


def main():
    print("=" * 60)
    print("🧪 Omega-1 Camera Hardware Diagnostic")
    print("=" * 60)

    # 1. Check vcgencmd camera status
    print("\n[1] Raspberry Pi Camera Module Status:")
    print("-" * 60)
    result = run("vcgencmd get_camera")
    print(result.strip())
    if "supported=1" in result and "detected=1" in result:
        print("✓ Camera hardware detected and supported")
    elif "detected=0" in result:
        print("⚠ Camera hardware NOT detected - check ribbon cable connection")
    else:
        print("⚠ Unable to determine camera status")

    # 2. List video devices
    print("\n[2] Video Device List:")
    print("-" * 60)
    result = run("ls -l /dev/video* 2>&1")
    if result.strip() and "No such file" not in result:
        print(result.strip())
        # Count devices
        lines = [l for l in result.strip().split('\n') if '/dev/video' in l]
        print(f"\nFound {len(lines)} video device(s)")
    else:
        print("✗ No /dev/video* devices found")

    # 3. Check for libcamera tools
    print("\n[3] libcamera Tools Availability:")
    print("-" * 60)
    libcamera_tools = ['libcamera-still', 'libcamera-hello', 'libcamera-vid']
    for tool in libcamera_tools:
        result = run(f"which {tool}")
        if result.strip() and "not found" not in result.lower():
            print(f"✓ {tool}: {result.strip()}")
        else:
            print(f"✗ {tool}: Not found")

    # 4. Test raw capture with libcamera-still
    print("\n[4] Raw Capture Test (libcamera-still):")
    print("-" * 60)
    test_file = "/tmp/omega_camera_test.jpg"
    if os.path.exists(test_file):
        os.remove(test_file)
    
    result = run(f"libcamera-still -o {test_file} --timeout 2000 2>&1")
    print(result.strip())
    
    if os.path.exists(test_file):
        size = os.path.getsize(test_file)
        if size > 0:
            print(f"\n✓ Test capture successful: {test_file} ({size} bytes)")
            print("  Camera hardware is working correctly!")
        else:
            print(f"\n⚠ Test file created but empty ({size} bytes)")
            print("  Camera may be connected but not capturing properly")
    else:
        print("\n✗ Test capture failed - no file created")
        print("  Camera hardware may not be functioning")

    # 5. Check camera interface status
    print("\n[5] Camera Interface Status:")
    print("-" * 60)
    result = run("vcgencmd get_camera")
    print(result.strip())
    
    # Check for camera interface in dmesg
    result = run("dmesg | grep -i camera | tail -5")
    if result.strip():
        print("\nRecent camera-related kernel messages:")
        print(result.strip())

    # 6. Check permissions
    print("\n[6] User Permissions:")
    print("-" * 60)
    user = os.environ.get('USER', 'unknown')
    groups_result = run(f"groups {user}")
    print(f"User: {user}")
    print(f"Groups: {groups_result.strip()}")
    if 'video' in groups_result:
        print("✓ User is in 'video' group")
    else:
        print("⚠ User NOT in 'video' group - may need: sudo usermod -a -G video $USER")

    # Summary
    print("\n" + "=" * 60)
    print("📋 Diagnostic Summary:")
    print("=" * 60)
    
    camera_detected = False
    vcgencmd_result = run("vcgencmd get_camera")
    if "detected=1" in vcgencmd_result:
        camera_detected = True
    
    test_file_exists = os.path.exists(test_file) and os.path.getsize(test_file) > 0
    
    if camera_detected and test_file_exists:
        print("✓ Camera hardware appears to be working correctly")
        print("  If video_server still fails, check software configuration")
    elif camera_detected and not test_file_exists:
        print("⚠ Camera detected but capture test failed")
        print("  → Check ribbon cable connection")
        print("  → Try reseating the CSI ribbon connector")
        print("  → Power cycle the Pi")
    elif not camera_detected:
        print("✗ Camera hardware NOT detected")
        print("  → Check ribbon cable connection")
        print("  → Verify camera module is properly seated")
        print("  → Check for physical damage to ribbon cable")
        print("  → Try reseating connector (silver contacts face HDMI side)")
    
    print("\n" + "=" * 60)
    print("Done. Review output above for issues.")
    print("=" * 60)


if __name__ == "__main__":
    main()

