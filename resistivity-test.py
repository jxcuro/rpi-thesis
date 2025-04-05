import spidev
import time

# Initialize SPI
spi = spidev.SpiDev()
spi.open(0, 0)  # Bus 0, Device 0
spi.max_speed_hz = 50000  # Lower SPI speed if needed
spi.mode = 0b00

# Function to write data to the LDC1101
def write_register(register, value):
    # Send register address with write command (register | 0x80) and value
    spi.xfer2([register | 0x80, value])

# Function to read data from a register
def read_register(register):
    # Send register address (no write command) and read the data
    response = spi.xfer2([register, 0x00])
    return response[1]

# Function to initialize LDC1101
def init_ldc1101():
    # Example: Write configuration to Control Register (0x00)
    # You can adjust these values as needed
    write_register(0x00, 0x00)  # Reset to default state (example)
    time.sleep(0.1)

# Example to read ID register
def read_chip_id():
    # Read the chip ID register (0x3F)
    return read_register(0x3F)

# Initialize LDC1101
init_ldc1101()

# Read and print the chip ID
chip_id = read_chip_id()
print(f"LDC1101 Chip ID: {hex(chip_id)}")

# Read other registers if needed
# Example: Read the status register (0x08)
status = read_register(0x08)
print(f"Status Register: {hex(status)}")

# Optionally, you can print other registers to verify communication
