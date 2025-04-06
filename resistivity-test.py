import spidev
import RPi.GPIO as GPIO
from time import sleep

# GPIO & SPI Setup
SPI_BUS = 0
SPI_DEVICE = 0
SPI_MAX_SPEED_HZ = 10000000  # 10 MHz max for LDC1101
CS_PIN = 8  # CE0 (BCM pin 8)

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(CS_PIN, GPIO.OUT)
GPIO.output(CS_PIN, GPIO.HIGH)

# Register addresses
REG_FUNC_CONFIG = 0x0B
REG_MUX_CONFIG = 0x0C
REG_DIGITAL_CONFIG = 0x04
REG_LHR_DATA_MSB = 0x24

# Functional modes
FUNC_MODE_ACTIVE = 0x00
FUNC_MODE_SLEEP = 0x01

class LDC1101:
    def __init__(self, bus=SPI_BUS, device=SPI_DEVICE, cs_pin=CS_PIN):
        self.spi = spidev.SpiDev()
        self.spi.open(bus, device)
        self.spi.max_speed_hz = SPI_MAX_SPEED_HZ
        self.spi.mode = 1  # CPOL=0, CPHA=1 (Mode 1)
        self.cs_pin = cs_pin

    def write_register(self, reg_addr, value):
        GPIO.output(self.cs_pin, GPIO.LOW)
        self.spi.xfer2([reg_addr & 0x7F, value])
        GPIO.output(self.cs_pin, GPIO.HIGH)

    def read_register(self, reg_addr):
        GPIO.output(self.cs_pin, GPIO.LOW)
        result = self.spi.xfer2([reg_addr | 0x80, 0x00])
        GPIO.output(self.cs_pin, GPIO.HIGH)
        return result[1]

    def read_lhr_data(self):
        GPIO.output(self.cs_pin, GPIO.LOW)
        data = self.spi.xfer2([
            REG_LHR_DATA_MSB | 0x80,
            0x00, 0x00, 0x00
        ])
        GPIO.output(self.cs_pin, GPIO.HIGH)

        # Combine 3 LSBs of the data
        value = ((data[1] << 16) | (data[2] << 8) | data[3]) & 0x3FFFFF
        return value

    def set_func_mode(self, mode):
        current = self.read_register(REG_FUNC_CONFIG)
        new_val = (current & 0xFC) | (mode & 0x03)
        self.write_register(REG_FUNC_CONFIG, new_val)

    def enable_lhr_mode(self):
        print("Entering Sleep Mode...")
        self.set_func_mode(FUNC_MODE_SLEEP)
        sleep(0.05)

        print("Configuring LHR mode...")
        self.write_register(REG_MUX_CONFIG, 0x02)       # LHR mode
        self.write_register(REG_DIGITAL_CONFIG, 0x0C)   # Recommended for LHR
        sleep(0.05)

        print("Starting Conversions...")
        self.set_func_mode(FUNC_MODE_ACTIVE)
        sleep(0.1)

    def read_inductance(self):
        raw_lhr = self.read_lhr_data()
        # Placeholder conversion constant (you must calibrate!)
        k_constant = 1e-8
        inductance = k_constant / raw_lhr if raw_lhr != 0 else 0
        return raw_lhr, inductance

    def cleanup(self):
        self.spi.close()
        GPIO.cleanup()

# ---- MAIN PROGRAM ----
if __name__ == "__main__":
    ldc = LDC1101()
    try:
        ldc.enable_lhr_mode()
        print("Reading inductance values (Ctrl+C to stop)...")
        while True:
            value, inductance = ldc.read_inductance()
            print(f"LHR Value: {value}, Inductance: {inductance:.6f} H")
            sleep(0.5)
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        ldc.cleanup()
