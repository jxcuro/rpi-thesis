import spidev
import time

# Define SPI parameters
SPI_BUS = 0  # SPI Bus 0
SPI_DEVICE = 0  # SPI CE0 pin (CE0 corresponds to GPIO8, which is often used for SPI)
SPI_SPEED = 10000  # Lower SPI speed to 10,000 Hz
SPI_MODE = 0  # SPI Mode 0 (CPOL = 0, CPHA = 0)
SPI_BITS = 8  # 8-bit data frames

# Initialize SPI
spi = spidev.SpiDev()
spi.open(SPI_BUS, SPI_DEVICE)  # Open SPI bus and device
spi.max_speed_hz = SPI_SPEED
spi.mode = SPI_MODE
spi.bits_per_word = SPI_BITS

# Function to read data from LDC1101 register
def read_register(register):
    # Send register address with read bit (MSB set to 1)
    # Format: [register address, 0x00] to receive data
    response = spi.xfer2([register | 0x80, 0x00])  # 0x80 enables read (MSB set to 1)
    return response[1]  # Return the data byte received

# Debug: Read all registers (assuming LDC1101 has 12-bit registers)
# Replace with the actual register addresses of your LDC1101
registers = [0x00, 0x01, 0x02, 0x03]  # List of registers to check (just an example)

print("LDC1101 Register Debugging with Lower SPI Speed:")
for reg in registers:
    print(f"Reading register 0x{reg:02X}...")
    reg_value = read_register(reg)
    print(f"Register 0x{reg:02X} Value: 0x{reg_value:02X}")

# Close SPI connection
spi.close()
