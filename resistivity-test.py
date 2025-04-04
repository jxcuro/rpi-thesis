import spidev

spi = spidev.SpiDev()
spi.open(0, 0)  # Open SPI bus 0, device 0 (CE0)
spi.max_speed_hz = 50000

# Read 1 byte from MCP3008 (or another SPI device)
response = spi.xfer2([0x01, 0x80, 0x00])  # Example for MCP3008
print("SPI response:", response)

spi.close()
