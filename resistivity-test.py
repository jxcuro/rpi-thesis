import spidev
import time

# Chip ID register
REGISTER_CHIP_ID = 0x3E

# Read function (LDC1101 uses 0x80 | reg to read)
def read_register(register):
    spi = spidev.SpiDev()
    spi.open(0, 0)  # bus 0, device 0
    spi.max_speed_hz = 500000
    spi.mode = 1  # SPI mode 1: CPOL=0, CPHA=1

    read_command = 0x80 | (register & 0x7F)
    resp = spi.xfer2([read_command, 0x00])  # Send read command and dummy byte
    spi.close()
    return resp[1]

chip_id = read_register(REGISTER_CHIP_ID)
print(f"LDC1101 Chip ID: {hex(chip_id)}")
