import spidev
import time

# Open SPI (SPI bus 0, device 0)
spi = spidev.SpiDev()
spi.open(0, 0)  # SPI bus 0, device 0 (CE0)

# Set SPI parameters
spi.max_speed_hz = 1000000    # Set speed to 500kHz
spi.mode = 0  # Try mode 0 (CPOL = 0, CPHA = 0)

# Send a byte (e.g., 0x01) and receive the response
response = spi.xfer([0x01])
print("Loopback Response:", response)  # You should see the same byte sent (0x01)

# Send and receive multiple bytes (you can try different bytes here)
response = spi.xfer([0x01, 0x02])  # Send two bytes
print("Loopback Response (multiple bytes):", response)

# Close SPI after use
spi.close()
