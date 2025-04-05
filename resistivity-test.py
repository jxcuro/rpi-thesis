import spidev
import time

spi = spidev.SpiDev()
spi.open(0, 0)  # bus 0, device 0 (CE0)
spi.max_speed_hz = 500000  # Adjust the speed if necessary
spi.mode = 0b00  # SPI mode 0

def read_registers():
    # Example: Read multiple registers starting from address 0x00
    for addr in range(0x00, 0x10):  # Adjust the range as needed
        val = read_register(addr)
        print(f"Register 0x{addr:02X}: 0x{val:02X}")

read_registers()
