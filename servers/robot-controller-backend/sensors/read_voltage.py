from smbus2 import SMBus
import time

ADDRESS = 0x48  # ADS1115 I2C address
GAIN = 2/3      # +/-6.144V range
CHANNELS = [0, 1, 2, 3]

MUX = {
    0: 0x4000,  # AIN0
    1: 0x5000,  # AIN1
    2: 0x6000,  # AIN2
    3: 0x7000   # AIN3
}

CONFIG_REG = 0x01
CONVERSION_REG = 0x00
PGA = 0x0000        # Gain = 2/3
MODE = 0x0100       # Single-shot mode
DR = 0x0080         # Data rate = 1600 SPS
COMP = 0x0003       # Disable comparator

def read_channel(ch):
    config = 0x8000 | MUX[ch] | PGA | MODE | DR | COMP
    with SMBus(1) as bus:
        bus.write_i2c_block_data(ADDRESS, CONFIG_REG, [(config >> 8) & 0xFF, config & 0xFF])
        time.sleep(0.1)  # Give time for conversion
        data = bus.read_i2c_block_data(ADDRESS, CONVERSION_REG, 2)
        raw = (data[0] << 8) | data[1]
        if raw > 32767:
            raw -= 65536
        voltage = raw * 6.144 / 32768.0
        return round(voltage, 3)

print("🔋 Scanning ADC channels for voltage:")
for ch in CHANNELS:
    try:
        volts = read_channel(ch)
        print(f"Channel {ch}: {volts} V")
    except Exception as e:
        print(f"Channel {ch}: Error ({e})")

