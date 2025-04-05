import spidev
import time

# Create an SPI object
spi = spidev.SpiDev()

# Open SPI bus 0, device 0 (you can also try device 1 if this doesn't work)
spi.open(0, 0)

# Set SPI speed and mode
spi.max_speed_hz = 500000  # 500kHz is a safe speed for testing
spi.mode = 0b00  # SPI Mode 0

# Try reading and writing a byte (e.g., 0x01)
write_data = [0x01]
read_data = spi.xfer2(write_data)

# Output the results
print(f"Sent: {write_data}")
print(f"Received: {read_data}")

# Close the SPI connection
spi.close()
