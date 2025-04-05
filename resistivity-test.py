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

# Function to write to a register
def write_register(register, value):
    response = spi.xfer2([register & 0x7F, value])  # MSB=0 for write
    print(f"Writing to 0x{register:02X}: 0x{value:02X}, Response: 0x{response[1]:02X}")

# Delay for power-up
time.sleep(0.001)

# Read DEVICE_ID (0x3F) — should return 0xD4 for the LDC1101
device_id = read_register(0x3F)
print(f"DEVICE_ID (0x3F) Value: 0x{device_id:02X}")

# Step 1: Write to START_CONFIG (0x0B) to enable active mode (value: 0x00 for Active Mode)
write_register(0x0B, 0x00)  # Set to Active Conversion Mode (b00)
time.sleep(0.01)  # Give time for the change to take effect

# Step 2: Write to DIG_CONFIG (0x04) with MIN_FREQ = 0x00 (default 500 kHz) and RESP_TIME = 0x03 (384 sensor periods)
write_register(0x04, 0x03)  # Setting RESP_TIME = 384 (b011)
time.sleep(0.01)  # Give time for the change to take effect

# Step 3: Read back values to verify the changes
start_config_value = read_register(0x0B)
dig_config_value = read_register(0x04)

# Print the results
print(f"START_CONFIG (0x0B) Value: 0x{start_config_value:02X}")
print(f"DIG_CONFIG (0x04) Value: 0x{dig_config_value:02X}")

# Close SPI connection
spi.close()
