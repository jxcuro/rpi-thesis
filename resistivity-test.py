import spidev
import time

# Constants
LHR_RCOUNT_LSB = 0x30
LHR_RCOUNT_MSB = 0x31
DIG_CONFIG = 0x04
START_CONFIG = 0x0B
LHR_STATUS = 0x3B
LHR_DATA_LSB = 0x38
LHR_DATA_MID = 0x39
LHR_DATA_MSB = 0x3A

# Initialize SPI
spi = spidev.SpiDev()
spi.open(0, 0)  # SPI0, CS0
spi.max_speed_hz = 10000  # 10 kHz
spi.mode = 1  # SPI mode 1: CPOL=0, CPHA=1

def write_register(register, value):
    spi.xfer2([register & 0x7F, value])  # MSB=0 for write

def read_register(register):
    return spi.xfer2([0x80 | register, 0x00])[1]  # MSB=1 for read

def read_multi_registers(start_register, length):
    response = spi.xfer2([0x80 | start_register] + [0x00]*length)
    return response[1:]  # skip the address byte

def configure_ldc1101_lhr():
    # Set RCOUNT = 330 (0x014A)
    write_register(LHR_RCOUNT_LSB, 0x4A)
    write_register(LHR_RCOUNT_MSB, 0x01)

    # Set DIG_CONFIG = 0xE7 (MIN_FREQ = 4MHz, RPMIN = 1.5kΩ, RPMAX = RP_SET)
    write_register(DIG_CONFIG, 0xE7)

    # Set FUNC_MODE = active (start conversions)
    write_register(START_CONFIG, 0x00)

def wait_for_data_ready():
    while True:
        status = read_register(LHR_STATUS)
        if status & 0x01 == 0:
            break
        time.sleep(0.01)

def read_lhr_data():
    data = read_multi_registers(LHR_DATA_LSB, 3)
    raw_value = (data[2] << 16) | (data[1] << 8) | data[0]
    return raw_value

# Main
configure_ldc1101_lhr()
print("LDC1101 configured in LHR mode.")

try:
    while True:
        wait_for_data_ready()
        lhr_val = read_lhr_data()
        print(f"LHR Inductance Data: {lhr_val}")
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Measurement stopped.")
    spi.close()
