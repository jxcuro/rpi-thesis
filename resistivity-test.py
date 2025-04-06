import spidev
import time

# Register addresses (as seen in the MikroSDK driver)
REG_RP_SETTLE_LSB       = 0x10
REG_RP_SETTLE_MSB       = 0x11
REG_SETTLE_COUNT_LSB    = 0x12
REG_SETTLE_COUNT_MSB    = 0x13
REG_CLOCK_DIVIDERS      = 0x14
REG_STATUS              = 0x18
REG_CONFIG              = 0x1A
REG_MUX_CONFIG          = 0x1B
REG_DEMUX_CONFIG        = 0x1C
REG_RP_CONFIG           = 0x1D
REG_LHR_RCOUNT_LSB      = 0x1E
REG_LHR_RCOUNT_MID      = 0x1F
REG_LHR_RCOUNT_MSB      = 0x20
REG_LHR_OFFSET_LSB      = 0x21
REG_LHR_OFFSET_MID      = 0x22
REG_LHR_OFFSET_MSB      = 0x23
REG_LHR_CONFIG          = 0x24
REG_POWER_CONFIG        = 0x25

# SPI setup
spi = spidev.SpiDev()
spi.open(0, 0)  # Bus 0, CE0
spi.max_speed_hz = 10000000  # Up to 10MHz
spi.mode = 1  # CPOL = 0, CPHA = 1

def write_register(addr, data):
    # MSB=0 for write, so we send the register address with MSB cleared
    spi.xfer2([addr & 0x7F, data])

def read_register(addr):
    # MSB=1 for read
    return spi.xfer2([addr | 0x80, 0x00])[1]

def configure_lhr_mode():
    # Based on MikroSDK and LDC1101 datasheet default LHR mode setup

    # 1. Power down before config
    write_register(REG_POWER_CONFIG, 0x01)  # Power down
    time.sleep(0.01)

    # 2. Set the LHR Mode
    write_register(REG_CONFIG, 0x02)  # MODE = 0b10 -> LHR mode

    # 3. Recommended values (adjust based on your sensor setup)
    write_register(REG_SETTLE_COUNT_LSB, 0xFF)
    write_register(REG_SETTLE_COUNT_MSB, 0xFF)
    write_register(REG_CLOCK_DIVIDERS, 0x03)  # FREF_DIV = 1, FIN_DIV = 1
    write_register(REG_LHR_RCOUNT_LSB, 0xFF)
    write_register(REG_LHR_RCOUNT_MID, 0xFF)
    write_register(REG_LHR_RCOUNT_MSB, 0x0F)
    write_register(REG_LHR_CONFIG, 0x20)  # Enable data averaging (optional)

    # 4. Power up
    write_register(REG_POWER_CONFIG, 0x00)

    time.sleep(0.01)

    print("LHR Mode configured.")

def read_lhr_data():
    # Read LHR data registers
    msb = read_register(0x30)
    mid = read_register(0x31)
    lsb = read_register(0x32)
    result = ((msb << 16) | (mid << 8) | lsb)
    return result

# Example usage
configure_lhr_mode()

while True:
    data = read_lhr_data()
    print("LHR Data:", data)
    time.sleep(0.5)
