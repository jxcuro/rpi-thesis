import spidev
import time

# Initialize SPI
spi = spidev.SpiDev()
spi.open(0, 0)  # Bus 0, Device 0 (SPI0 CE0)
spi.max_speed_hz = 500000
spi.mode = 1  # SPI mode 1 (CPOL=0, CPHA=1)

def read_register(addr):
    """Read one byte from the LDC1101 register."""
    tx = [addr & 0x7F, 0x00]
    rx = spi.xfer2(tx)
    return rx[1]

def write_register(addr, value):
    """Write one byte to the LDC1101 register."""
    tx = [addr | 0x80, value]
    spi.xfer2(tx)

# Start testing
print("Testing LDC1101 connection...")

chip_id = read_register(0x3F)
print(f"CHIP_ID: 0x{chip_id:02X}")

if chip_id != 0xD4:
    print("Error: Incorrect CHIP_ID. Check power, SPI wiring, and chip orientation.")
else:
    print("LDC1101 communication successful.")

# Set to active mode
print("Writing FUNC_CONFIG = 0x01 to enter Active mode...")
write_register(0x01, 0x01)
time.sleep(0.1)
func_config = read_register(0x01)
print(f"FUNC_CONFIG register: 0x{func_config:02X}")

if func_config == 0x01:
    print("LDC1101 is in Active mode.")
else:
    print("Failed to enter Active mode. Check SPI writes and device readiness.")

# Read status register
status = read_register(0x00)
print(f"STATUS register: 0x{status:02X}")

spi.close()
