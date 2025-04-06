import spidev
import time
import RPi.GPIO as GPIO

# SPI settings
SPI_BUS = 0
SPI_DEVICE = 0
SPI_MODE = 0b00  # SPI mode (CPOL = 0, CPHA = 0)

# LDC1101 register addresses
REG = {
    "START_CONFIG": 0x0B,
    "RP_SET": 0x01,
    "TC1": 0x02,
    "TC2": 0x03,
    "DIG_CONFIG": 0x04,
    "ALT_CONFIG": 0x05,
    "RP_THRESH_H_LSB": 0x06,
    "RP_THRESH_H_MSB": 0x07,
    "RP_THRESH_L_LSB": 0x08,
    "RP_THRESH_L_MSB": 0x09,
    "INTB_MODE": 0x0A,
    "D_CONF": 0x0C,
    "L_THRESH_HI_LSB": 0x16,
    "L_THRESH_HI_MSB": 0x17,
    "L_THRESH_LO_LSB": 0x18,
    "L_THRESH_LO_MSB": 0x19,
    "STATUS": 0x20,
    "RP_DATA_LSB": 0x21,
    "RP_DATA_MSB": 0x22,
    "L_DATA_LSB": 0x23,
    "L_DATA_MSB": 0x24,
    "LHR_RCOUNT_LSB": 0x30,
    "LHR_RCOUNT_MSB": 0x31,
    "LHR_OFFSET_LSB": 0x32,
    "LHR_OFFSET_MSB": 0x33,
    "LHR_CONFIG": 0x34,
    "LHR_DATA_LSB": 0x38,
    "LHR_DATA_MID": 0x39,
    "LHR_DATA_MSB": 0x3A,
    "LHR_STATUS": 0x3B,
    "RID": 0x3E,
    "CHIP_ID": 0x3F
}

# Initialize SPI (empty, to be configured later)
spi = spidev.SpiDev()

# Define the GPIO pins for the LDC1101
CS_PIN = 8
SCK_PIN = 11
MISO_PIN = 9
MOSI_PIN = 10

# Initialize the GPIO library
GPIO.setmode(GPIO.BCM)
GPIO.setup(CS_PIN, GPIO.OUT)
GPIO.setup(SCK_PIN, GPIO.OUT)
GPIO.setup(MISO_PIN, GPIO.IN)
GPIO.setup(MOSI_PIN, GPIO.OUT)

# Device status indicators
DEVICE_ERROR = 0x01
DEVICE_OK = 0x00

# Power modes
ACTIVE_CONVERSION_MODE = 0x00
SLEEP_MODE = 0x01
SHUTDOWN_MODE = 0x02

# Write to LDC1101 register
def write_register(reg_addr, value):
    GPIO.output(CS_PIN, GPIO.LOW)
    spi.xfer2([reg_addr & 0x7F, value])  # Write command (MSB = 0)
    GPIO.output(CS_PIN, GPIO.HIGH)

# Read from LDC1101 register
def read_register(reg_addr):
    GPIO.output(CS_PIN, GPIO.LOW)
    result = spi.xfer2([reg_addr | 0x80, 0x00])  # Read command (MSB = 1)
    GPIO.output(CS_PIN, GPIO.HIGH)
    return result[1]

# Try multiple SPI speeds until initialization succeeds
def initialize_ldc1101():
    global spi
    spi_speeds = [5000, 10000, 20000, 50000, 100000, 200000, 500000]

    print("Trying SPI speeds to initialize LDC1101...")

    for speed in spi_speeds:
        try:
            spi.close()
            spi = spidev.SpiDev()
            spi.open(SPI_BUS, SPI_DEVICE)
            spi.max_speed_hz = speed
            spi.mode = SPI_MODE

            time.sleep(0.1)
            chip_id = read_register(REG["CHIP_ID"])

            if chip_id == 0xD4:
                print(f"Initialized at SPI speed: {speed} Hz (CHIP ID: 0x{chip_id:02X})")
                break
            else:
                print(f"Speed {speed} Hz: Unexpected CHIP ID: 0x{chip_id:02X}")
        except Exception as e:
            print(f"Speed {speed} Hz: SPI error: {e}")
    else:
        print("Initialization failed at all SPI speeds.")
        return DEVICE_ERROR

    # Configure registers
    write_register(REG["RP_SET"], 0x07)
    write_register(REG["TC1"], 0x90)
    write_register(REG["TC2"], 0xA0)
    write_register(REG["DIG_CONFIG"], 0x03)
    write_register(REG["ALT_CONFIG"], 0x00)
    write_register(REG["RP_THRESH_H_MSB"], 0x00)
    write_register(REG["RP_THRESH_L_LSB"], 0x00)
    write_register(REG["RP_THRESH_L_MSB"], 0x00)
    write_register(REG["INTB_MODE"], 0x00)
    write_register(REG["START_CONFIG"], SLEEP_MODE)
    write_register(REG["D_CONF"], 0x00)
    write_register(REG["L_THRESH_HI_LSB"], 0x00)
    write_register(REG["L_THRESH_HI_MSB"], 0x00)
    write_register(REG["L_THRESH_LO_LSB"], 0x00)
    write_register(REG["L_THRESH_LO_MSB"], 0x00)
    write_register(REG["LHR_RCOUNT_LSB"], 0x00)
    write_register(REG["LHR_RCOUNT_MSB"], 0x00)
    write_register(REG["LHR_OFFSET_LSB"], 0x00)
    write_register(REG["LHR_OFFSET_MSB"], 0x00)
    write_register(REG["LHR_CONFIG"], 0x00)
    time.sleep(0.1)
    return DEVICE_OK

def enable_powermode(mode):
    write_register(REG["START_CONFIG"], mode)

def enable_lmode():
    write_register(REG["ALT_CONFIG"], 0x01)
    write_register(REG["D_CONF"], 0x01)

def enable_rpmode():
    write_register(REG["ALT_CONFIG"], 0x02)
    write_register(REG["D_CONF"], 0x00)

def enable_lhrmode():
    write_register(REG["LHR_RCOUNT_LSB"], 0x00)
    write_register(REG["LHR_RCOUNT_MSB"], 0x80)
    write_register(REG["LHR_OFFSET_LSB"], 0x00)
    write_register(REG["LHR_OFFSET_MSB"], 0x00)
    write_register(REG["LHR_CONFIG"], 0x01)

def getstatus():
    return read_register(REG["STATUS"])

def getrpdata():
    value = read_register(REG["RP_DATA_MSB"]) << 8
    value |= read_register(REG["RP_DATA_LSB"])
    return value

def getldata():
    value = read_register(REG["L_DATA_MSB"]) << 8
    value |= read_register(REG["L_DATA_LSB"])
    return value

def getlhrdata():
    value = read_register(REG["LHR_DATA_MSB"])
    value = (value << 8) | read_register(REG["LHR_DATA_MID"])
    value = (value << 8) | read_register(REG["LHR_DATA_LSB"])
    return value

def display_all_registers():
    for name, addr in REG.items():
        value = read_register(addr)
        print(f"{name} (0x{addr:02X}): 0x{value:02X}")

def main():
    if initialize_ldc1101() != DEVICE_OK:
        print("Failed to initialize LDC1101.")
        return

    print("LDC1101 initialized. Entering LHR mode...")
    enable_powermode(ACTIVE_CONVERSION_MODE)
    enable_lhrmode()
    time.sleep(1)

    while True:
        lhr_val = getlhrdata()
        print(f"LHR Data: {lhr_val}")
        time.sleep(0.5)

# Run main
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        spi.close()
        GPIO.cleanup()
