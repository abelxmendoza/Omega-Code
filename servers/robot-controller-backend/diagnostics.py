#!/usr/bin/env python3
# File Location: ~/Omega-Code/servers/robot-controller-backend/diagnostics.py

"""
Omega 1 Diagnostic Utility (Pi 5 Compatible – using lgpio)
-----------------------------------------------------------
This script performs a full diagnostics check of Omega 1's hardware and system environment.
It tests sensors, actuators (e.g. buzzer, LEDs), and logs system info like CPU load and memory.

🧪 Features:
- Rich terminal output (with emoji and layout)
- Flags: --silent (no console), --log (save to diagnostics_report.txt)
- GPIO hardware testing (ultrasonic, IR sensors, buzzer, LEDs) via lgpio
- System info summary (RAM, CPU, uptime, etc.)

🔧 Requires:
- `lgpio` installed via pip
- Proper `/dev/gpiochip0` permissions (`root:gpio` + `chmod 660`)
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

console = Console()
LOG_FILE = "diagnostics_report.txt"
SILENT = "--silent" in sys.argv
LOG = "--log" in sys.argv
h = lgpio.gpiochip_open(0)

# Pin mapping
TRIG = 27
ECHO = 22
IR_LINE_PINS = [17, 27, 22]
BUZZER = 18
LEDS = [5, 6, 13]

# Output function
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

# Sensor functions
def test_voltage():
    path = os.path.join("sensors", "read_voltage.py")
    if not os.path.exists(path):
        log_or_print("[Voltage] ⚠️ Script not found.")
        return
    spec = importlib.util.spec_from_file_location("read_voltage", path)
    voltage = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(voltage)

def test_ultrasonic():
    lgpio.gpio_claim_output(h, TRIG, 0)
    lgpio.gpio_claim_input(h, ECHO)

    lgpio.gpio_write(h, TRIG, 1)
    time.sleep(10e-6)
    lgpio.gpio_write(h, TRIG, 0)

    start = lgpio.tick()
    timeout = start + 1_000_000

    while lgpio.gpio_read(h, ECHO) == 0:
        if lgpio.tick() - start > 1_000_000:
            raise TimeoutError("Timeout waiting for ECHO to go HIGH")
    t_start = lgpio.tick()

    while lgpio.gpio_read(h, ECHO) == 1:
        if lgpio.tick() - t_start > 1_000_000:
            raise TimeoutError("Timeout waiting for ECHO to go LOW")
    t_end = lgpio.tick()

    pulse = lgpio.tick_diff(t_start, t_end)
    dist = round(pulse / 58.0, 2)
    log_or_print(f"[Ultrasonic] 📡 Distance: {dist} cm")

def test_ir_line():
    result = []
    for pin in IR_LINE_PINS:
        lgpio.gpio_claim_input(h, pin)
        state = "🔘" if lgpio.gpio_read(h, pin) else "⚫"
        result.append(f"{pin}:{state}")
    log_or_print(f"[IR Tracker] 🔦 Sensor states: {' | '.join(result)}")

def test_camera():
    if os.path.exists("/dev/video0"):
        log_or_print("[Camera] 🎥 Detected")
    else:
        log_or_print("[Camera] ❌ Not connected")

def test_buzzer():
    lgpio.gpio_claim_output(h, BUZZER, 0)
    lgpio.gpio_write(h, BUZZER, 1)
    time.sleep(0.3)
    lgpio.gpio_write(h, BUZZER, 0)
    log_or_print("[Buzzer] 🔊 Buzzed")

def test_leds():
    for pin in LEDS:
        lgpio.gpio_claim_output(h, pin, 1)
    time.sleep(0.5)
    for pin in LEDS:
        lgpio.gpio_write(h, pin, 0)
    log_or_print("[LEDs] 💡 Flashed")

def system_info():
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

# Main logic
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
