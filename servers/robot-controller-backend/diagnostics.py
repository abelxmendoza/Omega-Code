#!/usr/bin/env python3
# File Location: ~/Omega-Code/servers/robot-controller-backend/diagnostics.py

"""
Omega 1 Diagnostic Utility
--------------------------
This script performs a full diagnostics check of Omega 1's hardware and system environment.
It tests sensors, actuators (e.g. buzzer, LEDs), and logs system info like CPU load and memory.

🧪 Features:
- Rich terminal output (with color, emoji, and layout)
- Flags: --silent (no console), --log (save to diagnostics_report.txt)
- GPIO hardware testing (ultrasonic, IR sensors, buzzer, LEDs) via pigpio
- System info summary (RAM, CPU, uptime, etc.)

🔧 Requires:
- pigpio daemon running (launch with `sudo pigpiod`)
"""

import os
import sys
import time
import importlib.util
from datetime import datetime
from rich.console import Console
from rich.table import Table
from rich.panel import Panel
from rich.text import Text
from rich import box
import pigpio
from io import StringIO

# === Config ===
console = Console()
LOG_FILE = "diagnostics_report.txt"
SILENT = "--silent" in sys.argv
LOG = "--log" in sys.argv
pi = pigpio.pi()

if not pi.connected:
    raise RuntimeError("❌ Failed to connect to pigpiod. Run `sudo pigpiod` first.")

# === Output Handler ===
def log_or_print(msg):
    if not SILENT:
        console.print(msg)
    if LOG:
        with open(LOG_FILE, "a") as f:
            if isinstance(msg, str):
                f.write(f"{msg}\n")
            else:
                temp_buf = StringIO()
                temp_console = Console(file=temp_buf, force_terminal=False, color_system=None, width=80)
                temp_console.print(msg)
                plain_text = temp_buf.getvalue()
                f.write(plain_text + "\n")

def run_section(name, func):
    log_or_print(f"\n[bold cyan]🧪 Running {name}...[/bold cyan]")
    try:
        func()
        log_or_print(f"[green]✅ {name} completed.[/green]")
    except Exception as e:
        log_or_print(f"[red]❌ {name} failed: {e}[/red]")

# === Pin Mapping (Adjust for your bot) ===
ULTRASONIC_TRIG = 23
ULTRASONIC_ECHO = 24
IR_LINE_PINS = [17, 27, 22]
BUZZER_PIN = 18
LED_PINS = [5, 6, 13]

# === Voltage Sensor Test (calls external script) ===
def test_voltage():
    path = os.path.join("sensors", "read_voltage.py")
    if not os.path.exists(path):
        log_or_print("[Voltage] ⚠️ Script not found.")
        return
    spec = importlib.util.spec_from_file_location("read_voltage", path)
    voltage = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(voltage)

# === Ultrasonic Distance Test ===
def test_ultrasonic():
    pi.set_mode(ULTRASONIC_TRIG, pigpio.OUTPUT)
    pi.set_mode(ULTRASONIC_ECHO, pigpio.INPUT)

    pi.write(ULTRASONIC_TRIG, 0)
    time.sleep(0.5)
    pi.write(ULTRASONIC_TRIG, 1)
    time.sleep(0.00001)
    pi.write(ULTRASONIC_TRIG, 0)

    start = time.time()
    timeout = start + 1
    pulse_start = pulse_end = time.time()

    while pi.read(ULTRASONIC_ECHO) == 0 and time.time() < timeout:
        pulse_start = time.time()
    while pi.read(ULTRASONIC_ECHO) == 1 and time.time() < timeout:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    distance = round(pulse_duration * 17150, 2)
    log_or_print(f"[Ultrasonic] 📡 Distance: {distance} cm")

# === IR Line Tracker Test ===
def test_ir_line():
    states = []
    for pin in IR_LINE_PINS:
        pi.set_mode(pin, pigpio.INPUT)
        state = "🔘" if pi.read(pin) else "⚫"
        states.append(f"{pin}:{state}")
    log_or_print(f"[IR Tracker] 🔦 Sensor states: {' | '.join(states)}")

# === Camera Detection ===
def test_camera():
    if os.path.exists('/dev/video0'):
        log_or_print("[Camera] 🎥 Detected")
    else:
        log_or_print("[Camera] ❌ Not connected")

# === Buzzer Output Test ===
def test_buzzer():
    pi.set_mode(BUZZER_PIN, pigpio.OUTPUT)
    pi.write(BUZZER_PIN, 1)
    time.sleep(0.3)
    pi.write(BUZZER_PIN, 0)
    log_or_print("[Buzzer] 🔊 Buzzed")

# === LED Blinker Test ===
def test_leds():
    for pin in LED_PINS:
        pi.set_mode(pin, pigpio.OUTPUT)
        pi.write(pin, 1)
    time.sleep(0.5)
    for pin in LED_PINS:
        pi.write(pin, 0)
    log_or_print("[LEDs] 💡 Flashed")

# === System Info Summary ===
def system_info():
    from subprocess import getoutput

    table = Table(title="🧠 Omega 1 System Info", box=box.ROUNDED, style="bold white")
    table.add_column("Metric", style="cyan", no_wrap=True)
    table.add_column("Value", style="magenta")

    table.add_row("Device Model", getoutput("cat /proc/cpuinfo | grep 'Model' | head -1").split(":")[-1].strip())
    table.add_row("OS", getoutput("grep PRETTY_NAME /etc/os-release").split("=")[1].strip().replace('"', ''))
    table.add_row("Uptime", getoutput("uptime -p").replace("up ", ""))
    table.add_row("CPU Load", getoutput("top -bn1 | grep 'Cpu(s)'").strip())
    table.add_row("RAM", getoutput("free -h | grep Mem").strip())
    table.add_row("Disk", getoutput("df -h / | tail -1").strip())

    log_or_print(table)

# === Main Diagnostics Loop ===
if __name__ == "__main__":
    if LOG:
        with open(LOG_FILE, "w") as f:
            f.write(f"Omega 1 Diagnostic Report – {datetime.now()}\n{'='*50}\n")

    log_or_print(Panel.fit(Text("Omega 1 Diagnostics Starting...", justify="center"), border_style="bold blue"))

    try:
        run_section("Voltage Monitor", test_voltage)
        run_section("Ultrasonic Sensor", test_ultrasonic)
        run_section("IR Line Tracker", test_ir_line)
        run_section("Camera Check", test_camera)
        run_section("Buzzer Check", test_buzzer)
        run_section("LED Check", test_leds)
        run_section("System Info Summary", system_info)
    finally:
        pi.stop()
        log_or_print(Panel.fit(Text("⚙️ All systems tested. Diagnostics complete.\n", justify="center"), border_style="green"))
        if LOG:
            log_or_print(f"[yellow]📁 Report saved to:[/] [bold white]{os.path.abspath(LOG_FILE)}[/bold white]")
