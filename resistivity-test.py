import spidev
import time

spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 500000
spi.mode = 1

def write_register(addr, value):
    spi.xfer2([addr & 0x7F, value])
    time.sleep(0.001)  # Small delay between write and read
    readback = read_register(addr)
    if readback != value:
        print(f"Write failed: Reg 0x{addr:02X}, wrote 0x{value:02X}, read back 0x{readback:02X}")

def read_register(addr):
    spi.xfer2([addr | 0x80, 0x00])  # Dummy
    return spi.xfer2([addr | 0x80, 0x00])[1]

def init_ldc1101():
    print("Waiting for LDC1101 to stabilize...")
    time.sleep(0.5)  # Let the device fully power on

    print("Writing registers...")
    write_register(0x0F, 0xFF)  # RP_SET: max drive strength
    write_register(0x10, 0x1D)  # LDC_CONFIG: enable, high-res, active mode
    write_register(0x19, 0x30)  # DCONFIG: enable LHR mode
    write_register(0x1A, 0xFF)  # RCOUNT: full scale

def read_lhr():
    msb = read_register(0x22)
    mid = read_register(0x23)
    lsb = read_register(0x24)
    return (msb << 16) | (mid << 8) | lsb

def read_status():
    return read_register(0x20)

# Main loop
try:
    init_ldc1101()
    time.sleep(0.2)

    while True:
        status = read_status()
        print(f"STATUS Register (0x20): {status:08b}")

        if status & 0x04:
            print("Oscillator Error — check coil")
        elif status & 0x01:
            lhr = read_lhr()
            print(f"LHR: {lhr}")
        else:
            print("Waiting for data...")

        time.sleep(0.5)

except KeyboardInterrupt:
    spi.close()
    print("SPI closed")
