import spidev
import time

# Define SPI parameters
SPI_BUS = 0
SPI_DEVICE = 0
SPI_SPEED = 10000  # 10 kHz for safe timing
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
    # Add 0x80 to register to indicate a read operation (MSB = 1)
    response = spi.xfer2([register | 0x80, 0x00])  # 0x80 enables read
    print(f"Read from 0x{register:02X}: 0x{response[1]:02X}")
    return response[1]

# Function to write data to LDC1101 register
def write_register(register, value):
    # Remove 0x80 to indicate write operation (MSB = 0)
    response = spi.xfer2([register & 0x7F, value])  # 0x7F disables read operation
    print(f"Writing to 0x{register:02X}: 0x{value:02X}, Response: 0x{response[1]:02X}")

# Step 1: Delay after power-up to allow initialization (0.8 ms)
time.sleep(0.001)  # Wait for 1 ms to ensure proper initialization

# Step 2: Write to START_CONFIG (0x0B) to set it to active mode (0x01)
write_register(0x0B, 0x01)  # Active Mode (0x01)
time.sleep(0.01)  # Ensure the sensor is properly awake

# Step 3: Write to DIG_CONFIG (0x04) to configure RP+L conversion interval
write_register(0x04, 0xE7)  # RP+L conversion interval setting
time.sleep(0.01)  # Give time for the configuration to take effect

# Step 4: Write to RP_SET (0x01) to configure measurement dynamic range
write_register(0x01, 0x07)  # Example setting for RP_SET
time.sleep(0.01)  # Ensure proper setting time

# Step 5: Request data (LHR) from the sensor using the correct command
write_register(0xB8, 0x00)  # Read LHR data command (0xB8)

# Step 6: Return to conversion mode by writing back to START_CONFIG
write_register(0x0B, 0x00)  # Active mode (0x00)

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
