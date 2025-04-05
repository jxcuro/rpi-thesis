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

# SPI read/write functions
def write_register(reg, value):
    spi.xfer2([reg & 0x7F, value])  # MSB=0 for write

def read_register(reg):
    return spi.xfer2([reg | 0x80, 0x00])[1]  # MSB=1 for read

# LDC1101 initialization function
def init_ldc1101():
    # Put in Sleep mode, then Standby mode to reset the LDC1101 cleanly
    write_register(REG_START_CONFIG, 0x03)  # Sleep mode
    time.sleep(0.05)
    write_register(REG_START_CONFIG, 0x01)  # Standby mode
    time.sleep(0.05)

    # Set RCOUNT (measurement resolution), longer conversion time
    write_register(REG_RCOUNT_MSB, 0x08)  # RCOUNT = 0x0858
    write_register(REG_RCOUNT_LSB, 0x58)

    # Set SETTLECOUNT (stabilization time for the LC circuit)
    write_register(REG_SETTLECOUNT_MSB, 0x00)
    write_register(REG_SETTLECOUNT_LSB, 0x0A)

    # Set CLOCK_DIVIDERS (default, internal clock)
    write_register(REG_CLOCK_DIVIDERS, 0x00)

    # Set MUX_CONFIG to RP+L mode
    write_register(REG_MUX_CONFIG, 0x00)  # 0x00 for RP+L mode

    # Set DRIVE_CURRENT to a moderate value
    write_register(REG_DRIVE_CURRENT, 0xC0)  # Moderate drive current

    # Set CONFIG to default (0x00)
    write_register(REG_CONFIG, 0x00)

    # Set START_CONFIG to Active mode (FUNC_MODE = b00)
    write_register(REG_START_CONFIG, 0x00)  # Active Conversion Mode
    time.sleep(0.1)

# Read RP values from LDC1101
def read_rp():
    msb = read_register(REG_RP_MSB)
    lsb = read_register(REG_RP_LSB)
    return (msb << 8) | lsb

# Main function to initialize and read RP values
init_ldc1101()
print("LDC1101 initialized in Active RP+L mode. Reading RP values...")

# Continuously read RP values
while True:
    rp = read_rp()
    print(f"RP Measurement: {rp}")
    time.sleep(0.5)
