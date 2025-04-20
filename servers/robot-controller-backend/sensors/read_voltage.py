# File: sensors/read_voltage.py
# Location: ~/Omega-Code/servers/robot-controller-backend/sensors/
# Summary: 
#   This script scans all 4 analog input channels (AIN0–AIN3) on an ADS1115 ADC
#   connected via I2C and logs voltage readings. It uses auto-detection for the
#   ADC I2C address and displays results in a clean, emoji-enhanced Rich table.
#
#   Hardware Requirements:
#     • ADS1115 Analog-to-Digital Converter (I2C)
#     • Connected to Raspberry Pi I2C bus (typically bus 1)
#     • smbus2 and rich Python libraries installed
#
#   Example Usage:
#     $ python3 sensors/read_voltage.py

from smbus2 import SMBus
import time
from rich.console import Console
from rich.table import Table
from rich.panel import Panel
from rich.text import Text

console = Console()

# Known possible addresses for ADS1115
POSSIBLE_ADDRESSES = [0x48, 0x40]
GAIN = 2 / 3
CHANNELS = [0, 1, 2, 3]

MUX = {
    0: 0x4000,
    1: 0x5000,
    2: 0x6000,
    3: 0x7000
}

CONFIG_REG = 0x01
CONVERSION_REG = 0x00
PGA = 0x0000
MODE = 0x0100
DR = 0x0080
COMP = 0x0003


def find_valid_address():
    for addr in POSSIBLE_ADDRESSES:
        try:
            with SMBus(1) as bus:
                bus.read_byte(addr)
            return addr
        except:
            continue
    raise IOError("No valid ADS1115 I2C device found.")


def read_channel(ch, addr):
    config = 0x8000 | MUX[ch] | PGA | MODE | DR | COMP
    with SMBus(1) as bus:
        bus.write_i2c_block_data(addr, CONFIG_REG, [(config >> 8) & 0xFF, config & 0xFF])
        time.sleep(0.1)
        data = bus.read_i2c_block_data(addr, CONVERSION_REG, 2)
        raw = (data[0] << 8) | data[1]
        if raw > 32767:
            raw -= 65536
        voltage = raw * 6.144 / 32768.0
        return round(voltage, 3)


if __name__ == "__main__":
    try:
        address = find_valid_address()
        console.print(Panel(Text(f"🔌 Using ADC at I2C address: 0x{address:X}", style="bold green"), title="Voltage Monitor"))

        table = Table(title="🔋 Voltage Readings from ADS1115", show_lines=True, header_style="bold cyan")
        table.add_column("Channel", justify="center")
        table.add_column("Voltage (V)", justify="center")

        for ch in CHANNELS:
            try:
                volts = read_channel(ch, address)
                table.add_row(f"[white]{ch}[/white]", f"[green]{volts}[/green]")
            except Exception as e:
                table.add_row(f"[white]{ch}[/white]", f"[red]Error: {str(e)}[/red]")

        console.print(table)
    except Exception as e:
        console.print(Panel(f"❌ [bold red]Voltage scan failed:[/] {e}", border_style="red"))

