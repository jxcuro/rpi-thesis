import spidev
import time

# Initialize SPI
spi = spidev.SpiDev()
spi.open(0, 1)  # Bus 0, CE1 (Chip Select 1)
spi.max_speed_hz = 500000  # 500kHz is safe
spi.mode = 0b00

# LDC1101 Register Addresses
REG_MODE_CONFIG = 0x01
REG_RP_MSB = 0x20
REG_RP_LSB = 0x21

def write_register(register, value):
    spi.xfer2([register & 0x7F, value])  # MSB = 0 for write

def read_register(register):
    result = spi.xfer2([0x80 | register, 0x00])  # MSB = 1 for read
    return result[1]

# Initialize LDC1101 to Active Mode
def init_ldc1101():
    # Set to Standby mode
    write_register(REG_MODE_CONFIG, 0x01)
    time.sleep(0.1)

    # Set Active Mode
    write_register(REG_MODE_CONFIG, 0x02)
    time.sleep(0.1)

def read_rp():
    msb = read_register(REG_RP_MSB)
    lsb = read_register(REG_RP_LSB)
    return (msb << 8) | lsb

# --- Main Execution ---
init_ldc1101()

# Simple check of registers after initialization
print(f"Mode Config: {read_register(REG_MODE_CONFIG)}")

while True:
    rp = read_rp()
    print(f"RP Measurement: {rp}")
    time.sleep(0.5)
