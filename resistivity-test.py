import spidev

spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 1000000
spi.mode = 1

# Try reading register 0x0B (STATUS)
response = spi.xfer2([0x8B, 0x00])
print(f"Raw SPI response: {response}")
