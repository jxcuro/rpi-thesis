import spidev

# Open SPI (SPI bus 0, device 0)
spi = spidev.SpiDev()
spi.open(0, 0)  # SPI bus 0, device 0 (CE0)

# Test with different speeds and modes
spi.max_speed_hz = 500000  # Try a lower speed
spi.mode = 0  # Change the mode if needed, try mode 1 or mode 3 if mode 0 doesn't work

# Send data and receive response
response = spi.xfer([0x01])  # Replace with your data
print("Response:", response)

# Close SPI
spi.close()
