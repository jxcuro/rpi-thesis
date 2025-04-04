import spidev

# Initialize SPI (SPI bus 0, device 0)
spi = spidev.SpiDev()
spi.open(0, 0)  # SPI bus 0, device 0 (CE0)

# Set SPI speed and mode
spi.max_speed_hz = 50000
spi.mode = 0

# Send a test byte
response = spi.xfer([0x01])
print("Response:", response)

spi.close()
