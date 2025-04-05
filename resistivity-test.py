import spidev
import time

# SPI setup
spi = spidev.SpiDev()
spi.open(0, 0)  # Bus 0, CE0
spi.max_speed_hz = 500000
spi.mode = 0b00

# LDC1101 register addresses
REG_START_CONFIG     = 0x01
REG_RCOUNT_LSB       = 0x08
REG_RCOUNT_MSB       = 0x09
REG_SETTLECOUNT_LSB  = 0x10
REG_SETTLECOUNT_MSB  = 0x11
REG_CLOCK_DIVIDERS   = 0x1E
REG_MUX_CONFIG       = 0x1B
REG_DRIVE_CURRENT    = 0x1C
REG_CONFIG           = 0x1A
REG_RP_MSB           = 0x20
REG_RP_LSB           = 0x21

def write_register(reg, value):
    spi.xfer2([reg & 0x7F, value])  # MSB=0 for write

def read_register(reg):
    return spi.xfer2([reg | 0x80, 0x00])[1]  # MSB=1 for read

def init_ldc1101():
    # Put in standby first (optional safety)
    write_register(REG_START_CONFIG, 0x01)
    time.sleep(0.1)

    # RCOUNT = 0x0858 (longer conversion time = more stable results)
    write_register(REG_RCOUNT_MSB, 0x08)
    write_register(REG_RCOUNT_LSB, 0x58)

    # SETTLECOUNT = 0x000A (let LC circuit stabilize)
    write_register(REG_SETTLECOUNT_MSB, 0x00)
    write_register(REG_SETTLECOUNT_LSB, 0x0A)

    # CLOCK_DIVIDERS = 0x00 (default)
    write_register(REG_CLOCK_DIVIDERS, 0x00)

    # MUX_CONFIG = 0x00 → RP+L mode
    write_register(REG_MUX_CONFIG, 0x00)

    # DRIVE_CURRENT = 0xC0 (moderate drive current)
    write_register(REG_DRIVE_CURRENT, 0xC0)

    # CONFIG = 0x00 (default)
    write_register(REG_CONFIG, 0x00)

    # START_CONFIG = 0x00 → Active mode, default response time, internal clock
    write_register(REG_START_CONFIG, 0x00)
    time.sleep(0.1)

def read_rp():
    msb = read_register(REG_RP_MSB)
    lsb = read_register(REG_RP_LSB)
    return (msb << 8) | lsb

# Initialize and loop
init_ldc1101()
print("LDC1101 initialized in Active RP+L mode. Reading RP values...")

while True:
    rp = read_rp()
    print(f"RP Measurement: {rp}")
    time.sleep(0.5)
