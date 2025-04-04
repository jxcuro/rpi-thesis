import spidev
import time

# Open SPI (SPI bus 0, device 0)
spi = spidev.SpiDev()
spi.open(0, 0)  # SPI bus 0, device 0 (CE0)

# Set SPI parameters
spi.max_speed_hz = 500000  # Set speed to 500kHz (or try 1MHz)
spi.mode = 0  # Mode 0 (CPOL=0, CPHA=0)

# Send a byte and receive the same
print("Sending 0x01, expecting loopback.")
response = spi.xfer([0x01])
print("Response:", response)  # You should see something like [0x01]

# Close SPI after use
spi.close()
