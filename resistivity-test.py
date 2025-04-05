import spidev
import time

# SPI setup
spi = spidev.SpiDev()
spi.open(0, 0)  # SPI bus 0, CS0 (use 0,1 if on CE1)
spi.max_speed_hz = 1000000
spi.mode = 1  # SPI mode 1 (CPOL=0, CPHA=1)

# Register addresses
RP_MSB_REG      = 0x20
RP_LSB_REG      = 0x21
L_MSB_REG       = 0x22
L_LSB_REG       = 0x23
STATUS_REG      = 0x0B
MODE_CONFIG_REG = 0x1D
POWER_CONFIG    = 0x1C

# SPI register read/write
def write_register(reg, value):
    spi.xfer2([reg & 0x7F, value])  # MSB = 0 for write

def read_register(reg):
    return spi.xfer2([reg | 0x80, 0x00])[1]  # MSB = 1 for read

# Sensor initialization (mimics ldc1101_default_cfg)
def ldc1101_init():
    write_register(MODE_CONFIG_REG, 0x0C)  # Set to RP + L mode
    write_register(POWER_CONFIG, 0x01)     # Set to Active Conversion mode
    time.sleep(0.1)  # Allow sensor to stabilize

# Read RP data (16-bit)
def read_rp_data():
    msb = read_register(RP_MSB_REG)
    lsb = read_register(RP_LSB_REG)
    return (msb << 8) | lsb

# Read L data (16-bit)
def read_l_data():
    msb = read_register(L_MSB_REG)
    lsb = read_register(L_LSB_REG)
    return (msb << 8) | lsb

# Wait until data is ready
def wait_for_data_ready():
    while not (read_register(STATUS_REG) & 0x01):  # Check DATA_READY bit
        time.sleep(0.005)

# Main loop
try:
    ldc1101_init()
    print("Reading RP and L values from LDC1101...")
    while True:
        wait_for_data_ready()
        rp = read_rp_data()
        l = read_l_data()
        print(f"RP: {rp:5d}    L: {l:5d}")
        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nExiting...")
    spi.close()
