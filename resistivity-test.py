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

# Step 1: Delay after power-up to allow initialization (0.8 ms)
time.sleep(0.001)  # Wait for 1 ms to ensure proper initialization

# Step 2: Write to START_CONFIG (0x0B) to set it to active mode (0x01)
spi.xfer2([0x0B & 0x7F, 0x01])
time.sleep(0.01)

# Step 3: Write to LHR_CONFIG (0x34) to enable LHR mode (set to 0x01)
spi.xfer2([0x34 & 0x7F, 0x01])  # Enable LHR mode
time.sleep(0.01)

# Step 4: Read and print the values of the relevant registers
print("Reading LHR-related registers...")

lhr_config = read_register(0x34)
lhr_rcount_lsb = read_register(0x30)
lhr_rcount_msb = read_register(0x31)
lhr_offset_lsb = read_register(0x32)
lhr_offset_msb = read_register(0x33)
lhr_status = read_register(0x3B)
lhr_data_lsb = read_register(0x38)
lhr_data_mid = read_register(0x39)
lhr_data_msb = read_register(0x3A)

# Step 5: Print the values of the registers to analyze the status
print(f"LHR_CONFIG (0x34): 0x{lhr_config:02X}")
print(f"LHR_RCOUNT_LSB (0x30): 0x{lhr_rcount_lsb:02X}")
print(f"LHR_RCOUNT_MSB (0x31): 0x{lhr_rcount_msb:02X}")
print(f"LHR_OFFSET_LSB (0x32): 0x{lhr_offset_lsb:02X}")
print(f"LHR_OFFSET_MSB (0x33): 0x{lhr_offset_msb:02X}")
print(f"LHR_STATUS (0x3B): 0x{lhr_status:02X}")
print(f"LHR_DATA_LSB (0x38): 0x{lhr_data_lsb:02X}")
print(f"LHR_DATA_MID (0x39): 0x{lhr_data_mid:02X}")
print(f"LHR_DATA_MSB (0x3A): 0x{lhr_data_msb:02X}")

# Step 6: Close SPI connection
spi.close()
