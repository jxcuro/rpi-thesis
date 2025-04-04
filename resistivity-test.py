import spidev
import time

def read_register(addr):
    spi = spidev.SpiDev()
    spi.open(0, 0)  # CE0
    spi.max_speed_hz = 1000000
    spi.mode = 1  # SPI mode 1 for LDC1101

    read_cmd = [0x80 | addr, 0x00]  # MSB = 1 for read
    response = spi.xfer2(read_cmd)
    spi.close()
    return response[1]

chip_id = read_register(0x3F)
print(f"CHIP_ID: 0x{chip_id:02X}")
