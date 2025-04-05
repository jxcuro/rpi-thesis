import spidev
import time

# Define SPI parameters
SPI_BUS = 0
SPI_DEVICE = 0
SPI_SPEED = 10000
SPI_MODE = 0
SPI_BITS = 8

# Initialize SPI
spi = spidev.SpiDev()
spi.open(SPI_BUS, SPI_DEVICE)
spi.max_speed_hz = SPI_SPEED
spi.mode = SPI_MODE
spi.bits_per_word = SPI_BITS

# Function to read data from LDC1101 register
def read_register(register):
    response = spi.xfer2([register | 0x80, 0x00])  # 0x80 enables read
    return response[1]

# List of registers to read (from datasheet)
register_addresses = [
    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 
    0x0B, 0x0C, 0x16, 0x17, 0x18, 0x19, 0x20, 0x21, 0x22, 0x23, 
    0x24, 0x30, 0x31, 0x32, 0x33, 0x34, 0x38, 0x39, 0x3A, 0x3B, 
    0x3E, 0x3F
]

# Read and print all registers
for register in register_addresses:
    value = read_register(register)
    print(f"Register 0x{register:02X} Value: 0x{value:02X}")

# Close SPI connection
spi.close()
