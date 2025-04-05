import spidev
import time

# Constants
LDC1101_SPI_READ = 0x80
DUMMY = 0x00

# Register addresses
REG_DEVICE_ID = 0x00
REG_PWRCONFIG = 0x1F
REG_RP_L_MEASUREMENT_CONFIG = 0x1E
REG_LHR_CONFIG = 0x1D
REG_LHR_DATA_MSB = 0x23
REG_LHR_DATA_MID = 0x24
REG_LHR_DATA_LSB = 0x25

# Config values
MODE_ACTIVE_CONVERSION = 0x01
ALT_CONFIG_LHR = 0x30
LHR_CONTINUOUS = 0x20

# Clock
F_CLKIN = 16000000  # 16 MHz

class LDC1101:
    def __init__(self, bus=0, device=0):
        self.spi = spidev.SpiDev()
        self.spi.open(bus, device)
        self.spi.max_speed_hz = 100000  # Slower SPI for reliability
        self.spi.mode = 0b00

    def write_register(self, reg, value):
        self.spi.xfer2([reg, value])

    def read_register(self, reg):
        response = self.spi.xfer2([reg | LDC1101_SPI_READ, DUMMY])
        print(f"Read Reg 0x{reg:02X}: sent {[hex(b) for b in response]}")
        return response[1]

    def read_device_id(self):
        return self.read_register(REG_DEVICE_ID)

    def configure_lhr(self):
        self.write_register(REG_RP_L_MEASUREMENT_CONFIG, ALT_CONFIG_LHR)
        self.write_register(REG_LHR_CONFIG, LHR_CONTINUOUS)
        self.write_register(REG_PWRCONFIG, MODE_ACTIVE_CONVERSION)
        time.sleep(0.1)

    def read_lhr_data(self):
        msb = self.read_register(REG_LHR_DATA_MSB)
        mid = self.read_register(REG_LHR_DATA_MID)
        lsb = self.read_register(REG_LHR_DATA_LSB)
        return (msb << 16) | (mid << 8) | lsb

    def calculate_frequency(self, lhr_data):
        return (lhr_data * F_CLKIN) / (2**24)

ldc = LDC1101()

device_id = ldc.read_device_id()
print(f"Device ID: 0x{device_id:02X}")

if device_id == 0x84:  # Typical LDC1101 ID
    ldc.configure_lhr()
    while True:
        lhr = ldc.read_lhr_data()
        freq = ldc.calculate_frequency(lhr)
        print(f"LHR raw: {lhr}, Sensor Frequency: {freq:.2f} Hz")
        time.sleep(0.5)
else:
    print("LDC1101 not detected. Check wiring, SPI config, and power.")
