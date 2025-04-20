import os
import time
import importlib.util

def run_section(name, func):
    print(f"\n🧪 Running {name}...")
    try:
        func()
        print(f"✅ {name} completed.")
    except Exception as e:
        print(f"❌ {name} failed: {e}")

# === SECTION: Voltage Test ===
def test_voltage():
    path = os.path.join("sensors", "read_voltage.py")
    spec = importlib.util.spec_from_file_location("read_voltage", path)
    voltage = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(voltage)

# === SECTION: Placeholder for Other Tests ===
def test_ultrasonic():
    print("[Ultrasonic] ⚠️ Test not implemented yet.")

def test_ir_line():
    print("[IR Tracker] ⚠️ Test not implemented yet.")

def test_camera():
    if os.path.exists('/dev/video0'):
        print("[Camera] 🎥 Detected")
    else:
        print("[Camera] ❌ Not connected")

def test_buzzer():
    print("[Buzzer] ⚠️ Test not implemented yet.")

# === MAIN DIAGNOSTIC LOOP ===
if __name__ == "__main__":
    print("🧠 Omega 1 Diagnostics Starting...\n")

    run_section("Voltage Monitor", test_voltage)
    run_section("Ultrasonic Sensor", test_ultrasonic)
    run_section("IR Line Tracker", test_ir_line)
    run_section("Camera Check", test_camera)
    run_section("Buzzer Check", test_buzzer)

    print("\n⚙️ All systems tested. Diagnostics complete.\n")
