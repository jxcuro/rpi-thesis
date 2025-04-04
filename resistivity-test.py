import spidev
import time

# Open SPI bus
spi = spidev.SpiDev()
spi.open(0, 0)  # Open SPI bus 0, device 0
spi.max_speed_hz = 50000
spi.mode = 0b00

# Test command to send over SPI
command = [0x01, 0x00]  # You can send any test command; this is just an example

# Send and receive data
response = spi.xfer2(command)
print(f"SPI response: {response}")
