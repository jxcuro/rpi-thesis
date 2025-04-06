import spidev
import time

spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 8000000
spi.mode = 1  # CPOL=0, CPHA=1

def write_register(addr, value):
    spi.xfer2([addr & 0x7F, value])
    time.sleep(0.001)

def read_register(addr):
    return spi.xfer2([addr | 0x80, 0x00])[1]

def configure_lhr_mode():
    write_register(0x25, 0x01)  # Power down
    time.sleep(0.01)

    write_register(0x1A, 0x02)  # LHR mode
    write_register(0x12, 0xFF)  # SETTLE_COUNT_LSB
    write_register(0x13, 0xFF)  # SETTLE_COUNT_MSB
    write_register(0x14, 0x03)  # CLOCK_DIVIDERS: FIN_DIV=1, FREF_DIV=1
    write_register(0x1E, 0xFF)  # LHR_RCOUNT_LSB
    write_register(0x1F, 0xFF)  # LHR_RCOUNT_MID
    write_register(0x20, 0x0F)  # LHR_RCOUNT_MSB
    write_register(0x24, 0x20)  # LHR_CONFIG with averaging

    write_register(0x25, 0x00)  # Power up
    time.sleep(0.01)

def dump_registers():
    print("Register Dump (0x00 to 0x3F):")
    for addr in range(0x00, 0x40):
        val = read_register(addr)
        print(f"0x{addr:02X}: 0x{val:02X}")

configure_lhr_mode()
dump_registers()
