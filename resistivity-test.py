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

# Function to write data to LDC1101 register
def write_register(register, value):
    spi.xfer2([register, value])

# Function to read data from LDC1101 register
def read_register(register):
    response = spi.xfer2([register | 0x80, 0x00])  # 0x80 enables read
    return response[1]

# Step 1: Set D_CONF to a higher value (e.g., 0x01) to increase amplitude sensitivity
write_register(0x0C, 0x01)  # Adjust as necessary (try 0x02 for higher sensitivity)

# Step 2: Wait for 100 ms to allow the system to perform measurement
time.sleep(0.1)  # Increase delay as needed

# Step 3: Read and print LHR-related registers again to see if data changes
registers_to_check = [0x30, 0x31, 0x32, 0x33, 0x38, 0x39, 0x3A, 0x3B]
print("Reading LHR-related registers again after adjustment...")
for reg in registers_to_check:
    value = read_register(reg)
    print(f"Register 0x{reg:02X}: 0x{value:02X}")

# Step 4: Close SPI connection
spi.close()
