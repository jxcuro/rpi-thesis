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
    response = spi.xfer2([register | 0x80, 0x00])  # 0x80 enables read
    print(f"Read from 0x{register:02X}: 0x{response[1]:02X}")
    return response[1]

# Function to write data to LDC1101 register
def write_register(register, value):
    response = spi.xfer2([register & 0x7F, value])  # 0x7F disables read operation
    print(f"Writing to 0x{register:02X}: 0x{value:02X}, Response: 0x{response[1]:02X}")

# Step 1: Delay after power-up to allow initialization (0.8 ms)
time.sleep(0.001)  # Wait for 1 ms to ensure proper initialization

# Step 2: Write to START_CONFIG (0x0B) to set it to active mode (0x01)
write_register(0x0B, 0x01)  # Active Mode (0x01)
time.sleep(0.01)  # Ensure the sensor is properly awake

# Step 3: Write to DIG_CONFIG (0x04) to configure RP+L conversion interval
write_register(0x04, 0x03)  # RP+L conversion interval setting
time.sleep(0.01)  # Give time for the configuration to take effect

# Step 4: Write to RP_SET (0x01) to configure measurement dynamic range
write_register(0x01, 0x07)  # Example setting for RP_SET
time.sleep(0.01)  # Ensure proper setting time

# Step 5: Read Inductance (two-byte value)
inductance_msb = read_register(0x12)  # Most Significant Byte of Inductance
inductance_lsb = read_register(0x13)  # Least Significant Byte of Inductance
inductance_value = (inductance_msb << 8) | inductance_lsb  # Combine MSB and LSB
print(f"Inductance Value: {inductance_value}")

# Step 6: Read Rp (two-byte value)
rp_msb = read_register(0x14)  # Most Significant Byte of RP
rp_lsb = read_register(0x15)  # Least Significant Byte of RP
rp_value = (rp_msb << 8) | rp_lsb  # Combine MSB and LSB
print(f"Rp Value: {rp_value}")

# Close SPI connection
spi.close()
