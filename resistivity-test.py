import spidev

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

# Modify the DIG_CONFIG register (0x04) to set the measurement interval if necessary
write_register(0x04, 0x03)  # Example: Set RP+L conversion interval (you may adjust as needed)

# Read the DIG_CONFIG register to confirm the change
dig_config_value = read_register(0x04)

print(f"DIG_CONFIG (0x04) Register Value: 0x{dig_config_value:02X}")

# Close SPI connection
spi.close()
