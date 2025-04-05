import time
import spidev

# Initialize SPI
spi = spidev.SpiDev()
spi.open(0, 0)  # Use SPI bus 0, device 0
spi.max_speed_hz = 50000  # Set speed (adjust if needed)

# Function to write to a register
def write_register(register, value):
    spi.xfer2([register, value])
    time.sleep(0.1)  # Wait for the register to update

# Write the configuration values to the registers
write_register(0x01, 0x75)  # RP_SET (0x01)
write_register(0x04, 0xE7)  # DIG_CONFIG (0x04)
write_register(0x0B, 0x00)  # START_CONFIG (0x0B)
write_register(0x30, 0x4A)  # LHR_RCOUNT_LSB (0x30)
write_register(0x31, 0x01)  # LHR_RCOUNT_MSB (0x31)

# Function to read from a register
def read_register(register):
    return spi.xfer2([register, 0x00])[1]

# Verify the register values
print(f"START_CONFIG (0x0B): {hex(read_register(0x0B))}")
print(f"DIG_CONFIG (0x04): {hex(read_register(0x04))}")
print(f"RP_SET (0x01): {hex(read_register(0x01))}")
