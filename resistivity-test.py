import spidev

spi = spidev.SpiDev()
spi.open(0, 0)  # bus 0, device 0
spi.max_speed_hz = 500000

def read_register(addr):
    tx = [0x80 | addr, 0x00]  # 0x80 to set MSB for read
    rx = spi.xfer2(tx)
    return rx[1]
