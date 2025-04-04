import spidev
import time

# Initialize SPI
spi = spidev.SpiDev()
spi.open(0, 0)  # SPI bus 0, device 0 (CE0)
spi.max_speed_hz = 50000  # 50 kHz is slow, but enough for testing
spi.mode = 0  # Mode 0 (CPOL=0, CPHA=0)

# Test loopback: send data and get it back
print("Sending byte [0x01], expecting loopback.")
response = spi.xfer([0x01])
print("Response:", response)  # Should return [0x01]

# Send multiple bytes and expect loopback
print("Sending bytes [0x01, 0x02, 0x03], expecting loopback.")
response = spi.xfer([0x01, 0x02, 0x03])
print("Response:", response)  # Should return [0x01, 0x02, 0x03]

# Close SPI after use
spi.close()
