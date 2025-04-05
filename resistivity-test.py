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

# Read the Status Register (0x01)
status_value = read_register(0x01)

# Check for error flags or status bits
error_flags = status_value & 0x03  # Example: Check the lower 2 bits for error flags (adjust if needed)

print(f"Status Register (0x01) Value: 0x{status_value:02X}")
print(f"Error Flags (lower 2 bits): 0x{error_flags:02X}")

# Close SPI connection
spi.close()
