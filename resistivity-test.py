import spidev
import time

# Define SPI parameters
SPI_BUS = 0
SPI_DEVICE = 0
SPI_SPEED = 10000
SPI_MODE = 0
SPI_BITS = 8

# Initialize SPI
spi = spidev.SpiDev()
spi.open(SPI_BUS, SPI_DEVICE)
spi.max_speed_hz = SPI_SPEED
spi.mode = SPI_MODE
spi.bits_per_word = SPI_BITS

# Function to write to a register
def write_register(register, value):
    spi.xfer2([register & 0x7F, value])  # Write to register (MSB 0 for write)
    time.sleep(0.1)  # Add a small delay to allow the LDC1101 to process the write

# Function to read data from LDC1101 register
def read_register(register):
    response = spi.xfer2([register | 0x80, 0x00])  # 0x80 enables read
    return response[1]

# Initialize LDC1101 (write values to necessary registers)
write_register(0x02, 0x01)  # Example: Write to register 0x02 (LDC configuration)
write_register(0x03, 0x02)  # Example: Write to register 0x03 (Channel data)

# Debug: Read all registers (only valid ones based on datasheet)
registers = [0x01, 0x02, 0x03]  # Replace with actual valid registers for LDC1101
print("LDC1101 Register Debugging with Correct Register Addresses:")
for reg in registers:
    reg_value = read_register(reg)
    print(f"Register 0x{reg:02X} Value: 0x{reg_value:02X}")

# Close SPI connection
spi.close()
