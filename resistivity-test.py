import spidev
import time

# Open SPI (SPI bus 0, device 0)
spi = spidev.SpiDev()
spi.open(0, 0)  # SPI bus 0, device 0 (CE0)

# Set SPI parameters
spi.max_speed_hz = 100000  # Use a lower speed to avoid timing issues
spi.mode = 0  # Mode 0 (CPOL=0, CPHA=0), this is the default for many devices
spi.bits_per_word = 8  # 8 bits per word is common

# Send a single byte and receive the same byte (loopback)
print("Sending byte [0x01], expecting loopback.")
response = spi.xfer([0x01])  # Send a byte and get response
print("Response:", response)  # Expect [0x01] if the loopback is working

# Send multiple bytes and receive the same bytes (loopback)
print("Sending bytes [0x01, 0x02, 0x03], expecting a loopback response.")
response = spi.xfer([0x01, 0x02, 0x03])
print("Response:", response)  # Expect [0x01, 0x02, 0x03] if the loopback is working

# Close SPI after use
spi.close()
