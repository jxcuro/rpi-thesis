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
write_register(0x04, 0x03)
time.sleep(0.01)

# Step 4: Write to RP_SET (0x01) to configure measurement dynamic range
write_register(0x01, 0x07)
time.sleep(0.01)

# Step 5: Configure High Resolution L (LHR) Mode
write_register(0x34, 0x01)  # Set LHR_CONFIG to 0x01 to enable High-Resolution L mode
time.sleep(0.01)

# Step 6: Set LHR reference count and offset to default values for testing
write_register(0x30, 0x00)  # LHR_RCOUNT_LSB = 0
write_register(0x31, 0x00)  # LHR_RCOUNT_MSB = 0
write_register(0x32, 0x00)  # LHR_OFFSET_LSB = 0
write_register(0x33, 0x00)  # LHR_OFFSET_MSB = 0
time.sleep(0.01)

# Step 7: Ensure LHR measurement is ready
timeout = 10  # Set a timeout limit (in seconds)
start_time = time.time()

while True:
    lhr_status = read_register(0x3B)
    print(f"LHR_STATUS: 0x{lhr_status:02X}")  # Print LHR_STATUS to debug
    if lhr_status == 0x00:  # Measurement complete
        break
    elif time.time() - start_time > timeout:
        print("Timeout reached, measurement not complete.")
        break
    time.sleep(0.1)  # Wait before checking status again

# Step 8: Read LHR conversion data
lhr_data_lsb = read_register(0x38)
lhr_data_mid = read_register(0x39)
lhr_data_msb = read_register(0x3A)

# Combine the data bytes into a 24-bit value (LHR result)
lhr_data = (lhr_data_msb << 16) | (lhr_data_mid << 8) | lhr_data_lsb
print(f"High-Resolution L Inductance Data: 0x{lhr_data:06X}")

# Step 9: Close SPI connection
spi.close()
