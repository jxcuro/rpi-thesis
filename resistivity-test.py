import spidev
import time

spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 500000
spi.mode = 1

def write_register(addr, value):
    spi.xfer2([addr & 0x7F, value])

def read_register(addr):
    spi.xfer2([addr | 0x80, 0x00])
    return spi.xfer2([addr | 0x80, 0x00])[1]

def init_ldc1101():
    write_register(0x0F, 0xFF)  # RP_SET (max drive)
    write_register(0x10, 0x1D)  # LDC_CONFIG: enable LDC, 13MHz, high-res
    write_register(0x19, 0x30)  # DCONFIG: LHR mode
    write_register(0x1A, 0xFF)  # RCOUNT: full range

def read_lhr():
    spi.xfer2([0x22 | 0x80, 0x00])
    msb = spi.xfer2([0x22 | 0x80, 0x00])[1]
    spi.xfer2([0x23 | 0x80, 0x00])
    mid = spi.xfer2([0x23 | 0x80, 0x00])[1]
    spi.xfer2([0x24 | 0x80, 0x00])
    lsb = spi.xfer2([0x24 | 0x80, 0x00])[1]
    return (msb << 16) | (mid << 8) | lsb

def read_status():
    return read_register(0x20)

# Main loop
try:
    print("Initializing LDC1101...")
    init_ldc1101()
    time.sleep(0.1)

    while True:
        status = read_status()
        print(f"STATUS Register (0x20): {status:08b}")

        if status & 0x04:
            print("Oscillator Error — check coil wiring or increase RP_SET")
        elif status & 0x01:
            lhr = read_lhr()
            print(f"LHR: {lhr}")
        else:
            print("Waiting for data...")

        time.sleep(0.2)

except KeyboardInterrupt:
    spi.close()
    print("SPI closed")
