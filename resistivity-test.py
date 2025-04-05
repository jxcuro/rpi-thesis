import spidev
import time

# Define constants for the LDC1101 register addresses
LDC1101_REG_CFG_ADDITIONAL_DEVICE = 0x30  # Register address for Additional device configuration
LDC1101_REG_AMPLITUDE_CONTROL_REQUIREMENT = 0x34  # Register address for Amplitude control
LDC1101_REG_L_DATA_MSB = 0x10  # Register address for MSB of L data
LDC1101_REG_L_DATA_LSB = 0x11  # Register address for LSB of L data
LDC1101_SPI_READ = 0x80  # Read operation mask for SPI

# LDC1101 command masks for LHR mode
LDC1101_ALT_CFG_L_OPTIMAL_ENABLE = 0x10  # Enable LHR mode
LDC1101_CONTINUES_CONVERT = 0x01  # Continuous conversion mode

# Setup SPI
spi = spidev.SpiDev()
spi.open(0, 0)  # Open SPI bus 0, device 0 (adjust according to your setup)
spi.max_speed_hz = 100000  # Set SPI speed to 100 kHz (can be adjusted)
spi.mode = 0b00  # SPI mode 0 (CPOL=0, CPHA=0)

# Function to read from the LDC1101 sensor
def ldc1101_generic_read(register):
    # Send the register address with the read bit (0x80)
    response = spi.xfer2([register | LDC1101_SPI_READ, 0x00])
    return response[1]  # Return the received byte

# Function to write to the LDC1101 sensor
def ldc1101_generic_write(register, data):
    spi.xfer2([register, data])

# Function to set the LDC1101 to LHR mode
def ldc1101_go_to_l_mode():
    # Write to the Additional Device Configuration register to enable LHR mode
    ldc1101_generic_write(LDC1101_REG_CFG_ADDITIONAL_DEVICE, LDC1101_ALT_CFG_L_OPTIMAL_ENABLE)
    # Set the Amplitude Control register to continue conversions
    ldc1101_generic_write(LDC1101_REG_AMPLITUDE_CONTROL_REQUIREMENT, LDC1101_CONTINUES_CONVERT)

# Function to get L data from the LDC1101 sensor
def ldc1101_get_l_data():
    msb = ldc1101_generic_read(LDC1101_REG_L_DATA_MSB)
    lsb = ldc1101_generic_read(LDC1101_REG_L_DATA_LSB)

    # Combine the MSB and LSB to form a 16-bit value
    l_data = (msb << 8) | lsb
    return l_data

# Set the sensor to LHR mode before reading data
ldc1101_go_to_l_mode()

# Read L data from the sensor
l_data = ldc1101_get_l_data()
print(f"L data: {l_data}")

# Close the SPI connection
spi.close()
