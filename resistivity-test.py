import spidev
import time

# Open SPI bus
spi = spidev.SpiDev()
spi.open(0, 0)  # Open SPI bus 0, device 0
spi.max_speed_hz = 50000
spi.mode = 0b00

# Send a test value and receive the same value
test_value = [0xAA]  # Sending 0xAA as an example
response = spi.xfer2(test_value)
print(f"Loopback SPI response: {response}")
