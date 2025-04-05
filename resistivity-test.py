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

# Step 2: Read and print the values of all registers
all_registers = [
    0x01, 0x02, 0x03, 0x04, 0x05, 0x0B, 0x0C, 
    0x34, 0x30, 0x31, 0x32, 0x33, 0x3B, 0x38, 
    0x39, 0x3A, 0x0A, 0x16, 0x17, 0x18, 0x19, 
    0x20, 0x21, 0x22, 0x23, 0x24, 0x05, 0x0A, 0x0F, 
    0x10, 0x11, 0x12, 0x13, 0x14, 0x15
]

print("Reading all registers...")

for reg in all_registers:
    value = read_register(reg)
    print(f"Register 0x{reg:02X}: 0x{value:02X}")

# Step 3: Close SPI connection
spi.close()
