# spi_loopback_test.py
import spidev

spi = spidev.SpiDev()
spi.open(0, 0)  # SPI bus 0, device 0
spi.max_speed_hz = 100000
spi.mode = 1

print("Sending test byte 0xAA...")
response = spi.xfer2([0xAA])
print(f"Received byte: 0x{response[0]:02X}")

spi.close()
