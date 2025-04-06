from time import sleep
import spidev
import RPi.GPIO as GPIO

# Register addresses from datasheet
LDC1101_REG_L_DATA_MSB = 0x22
LDC1101_REG_L_DATA_LSB = 0x23
LDC1101_REG_LHR_DATA_LSB2 = 0x25
LDC1101_REG_CFG_POWER_STATE = 0x0F
LDC1101_REG_MUX_CONFIG = 0x1B

# Mode values
LDC1101_FUNC_MODE_SLEEP = 0x00
LDC1101_FUNC_MODE_ACTIVE = 0x01
LDC1101_LHR_MODE_ENABLE = 0x02

LDC1101_SPI_READ = 0x80

class LDC1101:
    def __init__(self, spi_bus=0, spi_device=0, cs_pin=8, int_pin=18):
        self.spi = spidev.SpiDev()
        self.spi.open(spi_bus, spi_device)
        self.spi.max_speed_hz = 100000  # 100 kHz
        self.spi.mode = 0b00

        self.cs_pin = cs_pin
        self.int_pin = int_pin

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(cs_pin, GPIO.OUT)
        GPIO.setup(int_pin, GPIO.IN)
        GPIO.output(cs_pin, GPIO.HIGH)

    def cleanup(self):
        self.spi.close()
        GPIO.cleanup()

    def write_register(self, reg, value):
        GPIO.output(self.cs_pin, GPIO.LOW)
        self.spi.xfer2([reg & 0x7F, value])
        GPIO.output(self.cs_pin, GPIO.HIGH)

    def read_register(self, reg):
        GPIO.output(self.cs_pin, GPIO.LOW)
        result = self.spi.xfer2([reg | LDC1101_SPI_READ, 0x00])
        GPIO.output(self.cs_pin, GPIO.HIGH)
        return result[1]

    def set_power_mode(self, mode):
        self.write_register(LDC1101_REG_CFG_POWER_STATE, mode)

    def enable_lhr_mode(self):
        self.set_power_mode(LDC1101_FUNC_MODE_SLEEP)
        sleep(0.05)
        self.set_power_mode(LDC1101_FUNC_MODE_ACTIVE)
        sleep(0.05)
        self.write_register(LDC1101_REG_MUX_CONFIG, LDC1101_LHR_MODE_ENABLE)
        sleep(0.1)

    def get_lhr_data(self):
        msb = self.read_register(LDC1101_REG_L_DATA_MSB)
        mid = self.read_register(LDC1101_REG_L_DATA_LSB)
        lsb = self.read_register(LDC1101_REG_LHR_DATA_LSB2)
        return (msb << 16) | (mid << 8) | lsb

    def get_lhr_inductance(self, k_constant=1.0):
        raw_val = self.get_lhr_data()
        if raw_val == 0:
            return float('inf')
        return k_constant / raw_val

# Example usage
if __name__ == "__main__":
    ldc = LDC1101(cs_pin=8, int_pin=18)
    try:
        print("Starting LDC1101 in LHR mode...")
        ldc.enable_lhr_mode()
        sleep(0.1)

        while True:
            lhr_val = ldc.get_lhr_data()
            print(f"LHR Raw: {lhr_val}")

            # Use your own calibration constant
            inductance = ldc.get_lhr_inductance(k_constant=1000000.0)
            print(f"Estimated Inductance: {inductance:.6f} H")
            sleep(0.5)

    except KeyboardInterrupt:
        print("Stopped.")
    finally:
        ldc.cleanup()
