from time import sleep
import spidev
import RPi.GPIO as GPIO

# Register addresses
REG_START_CONFIG = 0x0B
REG_MUX_CONFIG = 0x1B
REG_L_DATA_MSB = 0x22
REG_L_DATA_LSB = 0x23
REG_LHR_DATA_LSB2 = 0x25

# FUNC_MODE values for START_CONFIG[1:0]
FUNC_MODE_ACTIVE = 0b00
FUNC_MODE_SLEEP = 0b01
FUNC_MODE_SHUTDOWN = 0b10  # Not used here

READ_FLAG = 0x80

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
        result = self.spi.xfer2([reg | READ_FLAG, 0x00])
        GPIO.output(self.cs_pin, GPIO.HIGH)
        return result[1]

    def set_func_mode(self, mode):
        # Read full byte to preserve other bits (if any), update only bits [1:0]
        val = self.read_register(REG_START_CONFIG)
        new_val = (val & 0xFC) | (mode & 0x03)  # Clear bits 0-1, set new FUNC_MODE
        self.write_register(REG_START_CONFIG, new_val)
        sleep(0.05)

    def enable_lhr_mode(self):
        print("Setting to Sleep Mode...")
        self.set_func_mode(FUNC_MODE_SLEEP)

        print("Writing MUX_CONFIG to enable LHR mode...")
        self.write_register(REG_MUX_CONFIG, 0x02)  # LHR mode

        print("Switching to Active Mode...")
        self.set_func_mode(FUNC_MODE_ACTIVE)
        sleep(0.1)

    def get_lhr_data(self):
        msb = self.read_register(REG_L_DATA_MSB)
        mid = self.read_register(REG_L_DATA_LSB)
        lsb = self.read_register(REG_LHR_DATA_LSB2)
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
        print("Initializing LDC1101 in LHR mode...")
        ldc.enable_lhr_mode()

        while True:
            lhr_val = ldc.get_lhr_data()
            print(f"LHR Raw: {lhr_val}")

            inductance = ldc.get_lhr_inductance(k_constant=1_000_000.0)
            print(f"Estimated Inductance: {inductance:.6f} H")
            sleep(0.5)

    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        ldc.cleanup()
