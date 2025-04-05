import spidev
import time

# SPI setup
SPI_BUS = 0
SPI_DEVICE = 0
SPI_SPEED = 10000  # 10 kHz
SPI_MODE = 0
SPI_BITS = 8

# Initialize SPI
spi = spidev.SpiDev()
spi.open(SPI_BUS, SPI_DEVICE)
spi.max_speed_hz = SPI_SPEED
spi.mode = SPI_MODE
spi.bits_per_word = SPI_BITS

# Function to read a register
def read_register(register):
    response = spi.xfer2([register | 0x80, 0x00])  # MSB=1 for read
    print(f"Read from 0x{register:02X}: 0x{response[1]:02X}")
    return response[1]

# Delay for power-up
time.sleep(0.001)

# Read DEVICE_ID (0x3F) — should return 0xB4
device_id = read_register(0x3F)
print(f"DEVICE_ID (0x3F) Value: 0x{device_id:02X}")

# Close SPI connection
spi.close()
