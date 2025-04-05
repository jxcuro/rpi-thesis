import spidev
import time

class LDC1101:
    REG_DEVICE_ID = 0x3F
    REG_POWER_CONFIG = 0x0B
    REG_LHR_DATA_MSB = 0x22
    REG_LHR_DATA_MID = 0x23
    REG_LHR_DATA_LSB = 0x24
    REG_LHR_MODE = 0x0D

    def __init__(self, bus=0, device=0, spi_speed=10000):  # SPI speed set to 10kHz
        self.spi = spidev.SpiDev()
        self.spi.open(bus, device)
        self.spi.max_speed_hz = spi_speed
        self.spi.mode = 0b00

    def write_register(self, reg, value):
        self.spi.xfer2([reg & 0x7F, value])

    def read_register(self, reg):
        return self.spi.xfer2([reg | 0x80, 0x00])[1]

    def init_device(self):
        device_id = self.read_register(self.REG_DEVICE_ID)
        print(f"Device ID: 0x{device_id:02X}")
        return device_id

    def set_lhr_mode(self):
        self.write_register(self.REG_POWER_CONFIG, 0x01)  # Active Conversion Mode
        self.write_register(self.REG_LHR_MODE, 0x01)      # Enable LHR mode
        time.sleep(0.1)

    def read_lhr_raw(self):
        msb = self.read_register(self.REG_LHR_DATA_MSB)
        mid = self.read_register(self.REG_LHR_DATA_MID)
        lsb = self.read_register(self.REG_LHR_DATA_LSB)
        raw = (msb << 16) | (mid << 8) | lsb
        return raw

    def calculate_frequency(self, lhr_raw, ref_clk=16000000):  # 16 MHz internal clock
        return (lhr_raw * ref_clk) / (2**24)

    def close(self):
        self.spi.close()

# --- Example usage ---
sensor = LDC1101()
if sensor.init_device() == 0x84:
    sensor.set_lhr_mode()
    lhr_raw = sensor.read_lhr_raw()
    freq = sensor.calculate_frequency(lhr_raw)
    print(f"LHR raw: {lhr_raw}, Sensor Frequency: {freq:.2f} Hz")
else:
    print("LDC1101 not detected. Check wiring and SPI setup.")
sensor.close()
