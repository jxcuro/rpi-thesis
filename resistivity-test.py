import spidev
import time

# Constants
LDC1101_DUMMY = 0x00
LDC1101_SPI_READ = 0x80

# Register addresses
REG_PWRCONFIG = 0x1F
REG_RP_L_MEASUREMENT_CONFIG = 0x1E
REG_LHR_CONFIG = 0x1D
REG_STATUS = 0x20
REG_LHR_DATA_MSB = 0x23
REG_LHR_DATA_MID = 0x24
REG_LHR_DATA_LSB = 0x25

# Config values
MODE_ACTIVE_CONVERSION = 0x01
ALT_CONFIG_LHR = 0x30       # Enable LHR mode
LHR_CONV_MODE_CONTINUOUS = 0x20

# Sensor constants (adjust based on your hardware setup)
F_CLKIN = 16000000  # External clock frequency (16 MHz typical)
L_SENSOR = 10e-6    # Inductance of your sensor coil in Henries (example: 10uH)

class LDC1101:
    def __init__(self, spi_bus=0, spi_device=0):
        self.spi = spidev.SpiDev()
        self.spi.open(spi_bus, spi_device)
        self.spi.max_speed_hz = 100000
        self.spi.mode = 0b00

    def write_register(self, reg, val):
        self.spi.xfer2([reg, val])

    def read_register(self, reg):
        return self.spi.xfer2([reg | LDC1101_SPI_READ, LDC1101_DUMMY])[1]

    def read_lhr_data(self):
        msb = self.read_register(REG_LHR_DATA_MSB)
        mid = self.read_register(REG_LHR_DATA_MID)
        lsb = self.read_register(REG_LHR_DATA_LSB)
        raw = (msb << 16) | (mid << 8) | lsb
        return raw & 0xFFFFFF

    def configure_lhr_mode(self):
        self.write_register(REG_RP_L_MEASUREMENT_CONFIG, ALT_CONFIG_LHR)
        self.write_register(REG_LHR_CONFIG, LHR_CONV_MODE_CONTINUOUS)
        self.write_register(REG_PWRCONFIG, MODE_ACTIVE_CONVERSION)
        time.sleep(0.1)

    def calculate_inductance(self, lhr_data):
        """
        Uses formula from TI's LDC1101 datasheet for LHR mode:
        f_sensor = (LHR_data * f_CLKIN) / 2^24
        L = 1 / (4 * π² * f_sensor² * C)
        Since C is constant and unknown, usually this is used for relative changes.
        """
        f_sensor = (lhr_data * F_CLKIN) / (2**24)
        # For absolute inductance, need capacitor value.
        # If you're measuring change, just return f_sensor or lhr_data.
        return f_sensor

# Usage
ldc = LDC1101(spi_bus=0, spi_device=0)
ldc.configure_lhr_mode()

while True:
    lhr_raw = ldc.read_lhr_data()
    f_sensor = ldc.calculate_inductance(lhr_raw)
    print(f"LHR raw: {lhr_raw}, Sensor Frequency: {f_sensor:.2f} Hz")
    time.sleep(0.5)
