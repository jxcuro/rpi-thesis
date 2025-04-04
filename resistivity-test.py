import spidev
import time

spi = spidev.SpiDev()
spi.open(0, 0)  # Bus 0, CE0
spi.max_speed_hz = 500000  # LDC1101 supports up to 4 MHz

def write_register(register, value):
    spi.xfer2([register & 0x7F, value])  # MSB 0 = write

def read_register(register):
    spi.xfer2([register | 0x80])  # MSB 1 = read
    return spi.readbytes(1)[0]

# --- LDC1101 Register Addresses ---
POWER_CONFIG     = 0x0F
LHR_RCOUNT       = 0x1D
LHR_OFFSET       = 0x1E
MUX_CONFIG       = 0x1B
LHR_CONFIG       = 0x1C
STATUS           = 0x20
LHR_DATA_MSB     = 0x22
LHR_DATA_LSB     = 0x23
LHR_DATA_MLSB    = 0x24

# --- Setup Sequence for Active Mode ---
def initialize_ldc1101():
    write_register(POWER_CONFIG, 0x01)       # Enable active mode
    write_register(MUX_CONFIG, 0x01)         # Use sensor IN0
    write_register(LHR_CONFIG, 0x1D)         # Recommended config
    write_register(LHR_RCOUNT, 0xFF)         # Max resolution
    write_register(LHR_OFFSET, 0x00)         # No offset

def read_lhr_data():
    msb = read_register(LHR_DATA_MSB)
    mlsb = read_register(LHR_DATA_MLSB)
    lsb = read_register(LHR_DATA_LSB)
    lhr = (msb << 16) | (mlsb << 8) | lsb
    return lhr

initialize_ldc1101()

while True:
    status = read_register(STATUS)
    if status & 0x01:  # Check if LHR_DATA_READY
        value = read_lhr_data()
        print("LHR Value:", value)
    time.sleep(0.1)
