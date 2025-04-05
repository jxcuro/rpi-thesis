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

# Step 1: Set START_CONFIG (0x0B) to 0x01 to force the sensor into active mode
write_register(0x0B, 0x01)  # Active Mode (0x01)
time.sleep(1)  # Wait for the sensor to switch to active mode

# Step 2: Set DIG_CONFIG (0x04) to 0x03 (example configuration)
write_register(0x04, 0x03)  # RP+L conversion interval setting
time.sleep(1)  # Give time for the configuration to take effect

# Step 3: Set RP_SET (0x01) to an appropriate value, like 0x07 (for dynamic range)
write_register(0x01, 0x07)

# Verify the changes
start_config_value = read_register(0x0B)
dig_config_value = read_register(0x04)
rp_set_value = read_register(0x01)

# Print the results
print(f"START_CONFIG (0x0B) Value: 0x{start_config_value:02X}")
print(f"DIG_CONFIG (0x04) Value: 0x{dig_config_value:02X}")
print(f"RP_SET (0x01) Value: 0x{rp_set_value:02X}")

# Close SPI connection
spi.close()
