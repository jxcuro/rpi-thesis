import spidev
import time

# Open SPI bus
spi = spidev.SpiDev()
spi.open(0, 0)  # Open SPI bus 0, device 0 (usually the default)
spi.max_speed_hz = 50000  # Set SPI speed
spi.mode = 0b00  # SPI mode 0 (CPOL=0, CPHA=0)

# Send a test value and receive the same value
test_value = [0xAA]  # Sending 0xAA as an example
response = spi.xfer2(test_value)
print(f"Loopback SPI response: {response}")
