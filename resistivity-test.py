import spidev
import time

spi = spidev.SpiDev()
spi.open(0, 0)  # bus 0, device 0 (CE0)
spi.max_speed_hz = 500000  # Adjust the speed if necessary
spi.mode = 0b00  # SPI mode 0

def read_register(addr):
    tx = [0x80 | addr, 0x00]  # Set MSB for read
    rx = spi.xfer2(tx)
    print(f"Sent: {tx} => Received: {rx}")
    return rx[1]

# Try reading CHIP_ID
chip_id = read_register(0x3F)
print(f"LDC1101 Chip ID Register (0x3F): 0x{chip_id:02X}")
