#!/usr/bin/env python3
# File Location: ~/Omega-Code/servers/robot-controller-backend/diagnostics.py

"""
Omega 1 Diagnostic Utility (Pi 5-Compatible – using lgpio)
-----------------------------------------------------------
This script performs a full diagnostics check of Omega 1's onboard systems including GPIO-controlled
sensors, actuators, and general device health using `lgpio`, a Pi 5-compatible GPIO library.

✅ Tested on:
- Raspberry Pi 5
- Ubuntu 24.04 LTS (64-bit)
- Hardware: HC-SR04, IR Sensors, Buzzer, LEDs, Camera, ADC (via I2C)

🧪 Features:
- Rich terminal output with emojis and formatting (via Rich)
- Supports flags: 
  • --silent : suppress console output 
  • --log    : saves to `diagnostics_report.txt`
- Tests:
  • Voltage sensor (via separate script)
  • Ultrasonic rangefinder (GPIO)
  • IR line-tracker array (GPIO)
  • Camera device presence (/dev/video0)
  • Buzzer signal test
  • RGB LEDs or status LEDs
  • System summary (RAM, CPU, uptime, etc.)

🔧 Requirements:
- Python 3.9+ and `lgpio` (`pip install lgpio`)
- Proper `/dev/gpiochip0` permissions (`sudo chown root:gpio` + `chmod 660`)
- User must be in the `gpio` group or run via `sudo` or `newgrp gpio`
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
from io import StringIO
import lgpio

# ========== Configuration ==========
console = Console()
LOG_FILE = "diagnostics_report.txt"
SILENT = "--silent" in sys.argv
LOG = "--log" in sys.argv
h = lgpio.gpiochip_open(0)

# GPIO Pin Mapping (BCM Mode)
TRIG = 27
ECHO = 22
IR_LINE_PINS = [17, 27, 22]
BUZZER = 18
LEDS = [5, 6, 13]

# ========== Logging ==========

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
                f.write(temp_buf.getvalue() + "\n")

def run_section(name, func):
    log_or_print(f"\n[bold cyan]🧪 Running {name}...[/bold cyan]")
    try:
        func()
        log_or_print(f"[green]✅ {name} completed.[/green]")
    except Exception as e:
        log_or_print(f"[red]❌ {name} failed: {e}[/red]")

# ========== Test Sections ==========

def test_voltage():
    """Run the external voltage sensor script (ADS1115/PCF8591)."""
    path = os.path.join("sensors", "read_voltage.py")
    if not os.path.exists(path):
        log_or_print("[Voltage] ⚠️ Script not found.")
        return
    spec = importlib.util.spec_from_file_location("read_voltage", path)
    voltage = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(voltage)

def test_ultrasonic():
    """Trigger the ultrasonic rangefinder and read distance."""
    lgpio.gpio_claim_output(h, TRIG, 0)
    lgpio.gpio_claim_input(h, ECHO)

    lgpio.gpio_write(h, TRIG, 0)
    time.sleep(0.000002)
    lgpio.gpio_write(h, TRIG, 1)
    time.sleep(0.00001)
    lgpio.gpio_write(h, TRIG, 0)

    start = lgpio.tick()
    timeout = start + 1_000_000  # 1 second timeout in microseconds

    while lgpio.gpio_read(h, ECHO) == 0:
        if lgpio.tick() - start > 1_000_000:
            raise TimeoutError("Timeout waiting for ECHO to go HIGH")
    t_start = lgpio.tick()

    while lgpio.gpio_read(h, ECHO) == 1:
        if lgpio.tick() - t_start > 1_000_000:
            raise TimeoutError("Timeout waiting for ECHO to go LOW")
    t_end = lgpio.tick()

    pulse_us = lgpio.tick_diff(t_start, t_end)
    distance_cm = round(pulse_us / 58.0, 2)
    log_or_print(f"[Ultrasonic] 📡 Distance: {distance_cm} cm")

def test_ir_line():
    """Read IR sensors and show their logic state."""
    result = []
    for pin in IR_LINE_PINS:
        lgpio.gpio_claim_input(h, pin)
        state = "🔘" if lgpio.gpio_read(h, pin) else "⚫"
        result.append(f"{pin}:{state}")
    log_or_print(f"[IR Tracker] 🔦 Sensor states: {' | '.join(result)}")

def test_camera():
    """Check if /dev/video0 exists."""
    if os.path.exists("/dev/video0"):
        log_or_print("[Camera] 🎥 Detected")
    else:
        log_or_print("[Camera] ❌ Not connected")

def test_buzzer():
    """Buzz the onboard buzzer for 0.3 seconds."""
    lgpio.gpio_claim_output(h, BUZZER, 0)
    lgpio.gpio_write(h, BUZZER, 1)
    time.sleep(0.3)
    lgpio.gpio_write(h, BUZZER, 0)
    log_or_print("[Buzzer] 🔊 Buzzed")

def test_leds():
    """Blink all LED pins on for half a second."""
    for pin in LEDS:
        lgpio.gpio_claim_output(h, pin, 1)
    time.sleep(0.5)
    for pin in LEDS:
        lgpio.gpio_write(h, pin, 0)
    log_or_print("[LEDs] 💡 Flashed")

def system_info():
    """Gather system metrics and print as a Rich table."""
    from subprocess import getoutput
    table = Table(title="🧠 Omega 1 System Info", box=box.ROUNDED, style="bold white")
    table.add_column("Metric", style="cyan", no_wrap=True)
    table.add_column("Value", style="magenta")

    table.add_row("Device", getoutput("cat /proc/cpuinfo | grep 'Model' | head -1").split(":")[-1].strip())
    table.add_row("OS", getoutput("grep PRETTY_NAME /etc/os-release").split("=")[1].strip().replace('"', ''))
    table.add_row("Uptime", getoutput("uptime -p").replace("up ", ""))
    table.add_row("CPU", getoutput("top -bn1 | grep 'Cpu(s)'"))
    table.add_row("RAM", getoutput("free -h | grep Mem"))
    table.add_row("Disk", getoutput("df -h / | tail -1"))
    log_or_print(table)

# ========== Main Entry ==========

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
        lgpio.gpiochip_close(h)
        log_or_print(Panel.fit(Text("⚙️ All systems tested. Diagnostics complete.\n", justify="center"), border_style="green"))
        if LOG:
            log_or_print(f"[yellow]📁 Report saved to:[/] [bold white]{os.path.abspath(LOG_FILE)}[/bold white]")
