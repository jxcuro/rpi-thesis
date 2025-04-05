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

# Function to write data to LDC1101 register
def write_register(register, value):
    spi.xfer2([register & 0x7F, value])  # 0x7F disables write operation

# Trigger measurement by setting the appropriate power state in START_CONFIG (0x0B)
write_register(0x0B, 0x01)  # Set to start the measurement (active mode)

# Wait a little for the measurement to complete
time.sleep(1)

# Read the STATUS register (0x20) again to check for changes
status_value = read_register(0x20)

# Check the lower 2 bits of the status register for error flags
error_flags = status_value & 0x03  # Mask the lower 2 bits

print(f"STATUS Register Value after triggering measurement: 0x{status_value:02X}")
print(f"Error Flags after triggering: 0x{error_flags:02X}")

# Close SPI connection
spi.close()
