
# diagnostics.py

import os
import sys
import time
import importlib.util
import platform
from datetime import datetime
from rich.console import Console
from rich.table import Table
from rich.panel import Panel
from rich.text import Text
from rich import box
import RPi.GPIO as GPIO

console = Console()
LOG_FILE = "diagnostics_report.txt"
SILENT = "--silent" in sys.argv
LOG = "--log" in sys.argv

def log_or_print(msg):
    if not SILENT:
        console.print(msg)
    if LOG:
        with open(LOG_FILE, "a") as f:
            f.write(f"{msg if isinstance(msg, str) else msg.plain}\n")

def run_section(name, func):
    log_or_print(f"\n[bold cyan]🧪 Running {name}...[/bold cyan]")
    try:
        func()
        log_or_print(f"[green]✅ {name} completed.[/green]")
    except Exception as e:
        log_or_print(f"[red]❌ {name} failed: {e}[/red]")

# === GPIO Setup ===
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Example pin mappings (adjust to match your bot's hardware)
ULTRASONIC_TRIG = 23
ULTRASONIC_ECHO = 24
IR_LINE_PINS = [17, 27, 22]
BUZZER_PIN = 18
LED_PINS = [5, 6, 13]

# === SECTION: Voltage Test ===
def test_voltage():
    path = os.path.join("sensors", "read_voltage.py")
    if not os.path.exists(path):
        log_or_print("[Voltage] ⚠️ Script not found.")
        return
    spec = importlib.util.spec_from_file_location("read_voltage", path)
    voltage = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(voltage)

# === SECTION: Hardware Tests ===
def test_ultrasonic():
    GPIO.setup(ULTRASONIC_TRIG, GPIO.OUT)
    GPIO.setup(ULTRASONIC_ECHO, GPIO.IN)

    GPIO.output(ULTRASONIC_TRIG, False)
    time.sleep(0.5)

    GPIO.output(ULTRASONIC_TRIG, True)
    time.sleep(0.00001)
    GPIO.output(ULTRASONIC_TRIG, False)

    pulse_start, pulse_end = time.time(), time.time()
    timeout = time.time() + 1

    while GPIO.input(ULTRASONIC_ECHO) == 0 and time.time() < timeout:
        pulse_start = time.time()
    while GPIO.input(ULTRASONIC_ECHO) == 1 and time.time() < timeout:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150
    distance = round(distance, 2)

    log_or_print(f"[Ultrasonic] 📡 Distance: {distance} cm")

def test_ir_line():
    for pin in IR_LINE_PINS:
        GPIO.setup(pin, GPIO.IN)
    states = [f"{pin}:{'🔘' if GPIO.input(pin) else '⚫'}" for pin in IR_LINE_PINS]
    log_or_print(f"[IR Tracker] 🔦 Sensor states: {' | '.join(states)}")

def test_camera():
    if os.path.exists('/dev/video0'):
        log_or_print("[Camera] 🎥 Detected")
    else:
        log_or_print("[Camera] ❌ Not connected")

def test_buzzer():
    GPIO.setup(BUZZER_PIN, GPIO.OUT)
    GPIO.output(BUZZER_PIN, True)
    time.sleep(0.3)
    GPIO.output(BUZZER_PIN, False)
    log_or_print("[Buzzer] 🔊 Buzzed")

def test_leds():
    for pin in LED_PINS:
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, True)
    time.sleep(0.5)
    for pin in LED_PINS:
        GPIO.output(pin, False)
    log_or_print("[LEDs] 💡 Flashed")

# === SECTION: System Info ===
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

# === MAIN DIAGNOSTIC LOOP ===
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
        GPIO.cleanup()
        log_or_print(Panel.fit(Text("⚙️ All systems tested. Diagnostics complete.\n", justify="center"), border_style="green"))
        if LOG:
            log_or_print(f"[yellow]📁 Report saved to:[/] [bold white]{os.path.abspath(LOG_FILE)}[/bold white]")
