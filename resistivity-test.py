import spidev
import time

# Open SPI (SPI bus 0, device 0)
spi = spidev.SpiDev()
spi.open(0, 0)  # SPI bus 0, device 0 (CE0)

# Set SPI parameters (slower speed for testing)
spi.max_speed_hz = 100000  # Set to 100kHz to avoid timing issues
spi.mode = 0  # Mode 0 (CPOL=0, CPHA=0)

# Send a byte and receive the same (loopback)
print("Sending byte [0x01], expecting loopback.")
response = spi.xfer([0x01])  # Transmit byte and get response
print("Response:", response)  # Should see [0x01]

# Send multiple bytes and receive data
print("Sending bytes [0x01, 0x02, 0x03], expecting a response.")
response = spi.xfer([0x01, 0x02, 0x03])
print("Response:", response)  # Expect [0x01, 0x02, 0x03]

# Close SPI after use
spi.close()
