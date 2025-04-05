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
    # Set the 7th bit to 1 for reading (addresses are 8 bits, the 7th bit must be 1 for reading)
    response = spi.xfer2([register | 0x80, 0x00])  # 0x80 enables read (set bit 7)
    print(f"Read from 0x{register:02X}: 0x{response[1]:02X}")
    return response[1]

# Function to write data to LDC1101 register
def write_register(register, value):
    # Set the 7th bit to 0 for writing (addresses are 8 bits, the 7th bit must be 0 for writing)
    response = spi.xfer2([register & 0x7F, value])  # 0x7F disables read (clear bit 7)
    print(f"Writing to 0x{register:02X}: 0x{value:02X}, Response: 0x{response[1]:02X}")

# Step 1: Delay after power-up to allow initialization (0.8 ms)
time.sleep(0.001)  # Wait for 1 ms to ensure proper initialization

# Step 2: Write to START_CONFIG (0x0B) to set it to active mode (0x01)
write_register(0x0B, 0x01)  # Active Mode (0x01)
time.sleep(0.1)  # Increased delay to ensure proper mode transition

# Step 3: Write to DIG_CONFIG (0x04) to configure RP+L conversion interval
write_register(0x04, 0xE7)  # RP+L conversion interval setting
time.sleep(0.01)  # Give time for the configuration to take effect

# Step 4: Write to RP_SET (0x01) to configure measurement dynamic range
write_register(0x01, 0x07)  # Example setting for RP_SET
time.sleep(0.01)  # Ensure proper setting time

# Step 5: Initiate LHR Read by sending the appropriate command
write_register(0xB8, 0x00)  # Command to request LHR data

# Step 6: Read the LHR data from the appropriate register (0x38)
lhr_data = read_register(0x38)  # Read the LHR data from register 0x38

# Step 7: Return to conversion mode (ensure sensor is not in shutdown)
write_register(0x0B, 0x00)  # Return to conversion mode (0x00)

# Verify the changes
start_config_value = read_register(0x0B)
dig_config_value = read_register(0x04)
rp_set_value = read_register(0x01)

# Print the results
print(f"START_CONFIG (0x0B) Value: 0x{start_config_value:02X}")
print(f"DIG_CONFIG (0x04) Value: 0x{dig_config_value:02X}")
print(f"RP_SET (0x01) Value: 0x{rp_set_value:02X}")
print(f"LHR Data: 0x{lhr_data:02X}")

# Close SPI connection
spi.close()
