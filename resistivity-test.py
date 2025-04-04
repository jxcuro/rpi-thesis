import spidev

spi = spidev.SpiDev()
spi.open(0, 0)  # Open SPI bus 0, device 0 (CE0)
spi.max_speed_hz = 50000

# Send a test byte
response = spi.xfer2([0x00])
print("SPI response:", response)

spi.close()
