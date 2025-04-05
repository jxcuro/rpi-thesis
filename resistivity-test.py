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

# Force the LDC1101 into active mode by setting START_CONFIG (0x0B)
write_register(0x0B, 0x01)  # Set to Active Mode (0x01)

# Wait a bit for the sensor to switch modes
time.sleep(1)

# Read and verify the START_CONFIG register value
start_config_value = read_register(0x0B)
print(f"START_CONFIG (0x0B) Register Value: 0x{start_config_value:02X}")

# Check the DIG_CONFIG register to ensure it's not still at 0x00
dig_config_value = read_register(0x04)
print(f"DIG_CONFIG (0x04) Register Value after forcing active mode: 0x{dig_config_value:02X}")

# Close SPI connection
spi.close()
