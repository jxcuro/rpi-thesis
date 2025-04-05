import spidev
import time

# Initialize SPI
spi = spidev.SpiDev()
spi.open(0, 0)  # Bus 0, Device 0
spi.max_speed_hz = 10000  # Lower SPI speed (10 kHz)
spi.mode = 0b01  # SPI Mode 1 (CPOL=0, CPHA=1)

# Function to write data to the LDC1101
def write_register(register, value):
    # Send register address with write command (register | 0x80) and value
    response = spi.xfer2([register | 0x80, value])
    print(f"Write: {hex(register)} = {hex(value)}")

# Function to read data from a register
def read_register(register):
    # Send register address (no write command) and read the data
    response = spi.xfer2([register, 0x00])
    print(f"Read: {hex(register)} -> {hex(response[1])}")
    return response[1]

# Function to initialize LDC1101
def init_ldc1101():
    print("Initializing LDC1101...")
    write_register(0x01, 0x00)  # Soft reset command to register 0x01
    time.sleep(0.1)  # Wait for the device to reset
    write_register(0x00, 0x00)  # Default control register setup (example)
    time.sleep(0.1)

# Example to read ID register
def read_chip_id():
    return read_register(0x3F)  # Chip ID register

# Initialize LDC1101
init_ldc1101()

# Read and print the chip ID
chip_id = read_chip_id()
print(f"LDC1101 Chip ID: {hex(chip_id)}")

# Read other registers for further diagnostics
status = read_register(0x08)  # Status register for debugging
print(f"Status Register: {hex(status)}")
