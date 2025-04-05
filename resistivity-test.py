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
    return response[1]

# Function to write data to LDC1101 register
def write_register(register, value):
    spi.xfer2([register & 0x7F, value])  # 0x7F disables read operation

# Step 1: Delay after power-up to allow initialization (0.8 ms)
time.sleep(0.001)  # Wait for 1 ms to ensure proper initialization

# Step 2: Write to START_CONFIG (0x0B) to set it to active mode (0x01)
write_register(0x0B, 0x01)
time.sleep(0.01)

# Step 3: Write to DIG_CONFIG (0x04) to configure RP+L conversion interval
write_register(0x04, 0x03)  # Setting RP+L conversion interval (default is fine here)
time.sleep(0.01)

# Step 4: Write to RP_SET (0x01) to configure measurement dynamic range
write_register(0x01, 0x07)  # Setting dynamic range for RP measurement
time.sleep(0.01)

# Step 5: Reset LHR_CONFIG to 0x01 to ensure High-Resolution L mode is enabled
write_register(0x34, 0x01)  # Set LHR_CONFIG register to 0x01
time.sleep(0.01)

# Step 6: Check and ensure the LHR_STATUS has changed
print("Waiting for LHR measurement to complete...")
timeout = time.time() + 5  # Timeout after 5 seconds to prevent an infinite loop

while time.time() < timeout:
    lhr_status = read_register(0x3B)  # Read LHR_STATUS register
    print(f"LHR_STATUS: 0x{lhr_status:02X}")
    if lhr_status == 0x00:  # Measurement completed
        print("Measurement completed!")
        break
    time.sleep(0.1)  # Check the status every 100 ms

# Step 7: If measurement completed, read and print LHR data
if lhr_status == 0x00:
    lhr_data_lsb = read_register(0x38)
    lhr_data_msb = read_register(0x39)
    lhr_data_high = read_register(0x3A)
    print(f"LHR_DATA: 0x{lhr_data_high:02X}{lhr_data_msb:02X}{lhr_data_lsb:02X}")

# Step 8: Close SPI connection
spi.close()
