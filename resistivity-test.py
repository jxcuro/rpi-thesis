import spidev
import time

spi = spidev.SpiDev()
spi.open(0, 0)  # Bus 0, CE0
spi.max_speed_hz = 500000
spi.mode = 0b00

# Read status register to check communication
STATUS_REG = 0x00  # Status register address
def read_register(reg):
    return spi.xfer2([reg | 0x80, 0x00])[1]  # MSB=1 for read

# Read and print status
status = read_register(STATUS_REG)
print(f"Status Register: {status}")
