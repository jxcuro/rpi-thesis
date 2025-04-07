import spidev
import time
import RPi.GPIO as GPIO

# === SPI and GPIO Config ===
SPI_BUS = 0
SPI_DEVICE = 0
SPI_SPEED = 50000  # 50 kHz
SPI_MODE = 0b00    # Mode 0: CPOL = 0, CPHA = 0

CS_PIN = 8   # GPIO8 (CE0)
SCK_PIN = 11 # Clock
MISO_PIN = 9 # MISO
MOSI_PIN = 10 # MOSI

# === LDC1101 Register Addresses ===
REG = {
    "START_CONFIG": 0x0B,
    "RP_SET": 0x01,
    "TC1": 0x02,
    "TC2": 0x03,
    "DIG_CONFIG": 0x04,
    "ALT_CONFIG": 0x05,
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
    "CHIP_ID": 0x3F,
}

# === Device Constants ===
DEVICE_OK = 0x00
DEVICE_ERROR = 0x01

# Power Modes
ACTIVE_MODE = 0x00
SLEEP_MODE = 0x01
SHUTDOWN_MODE = 0x02

# === SPI Init ===
spi = spidev.SpiDev()
spi.open(SPI_BUS, SPI_DEVICE)
spi.max_speed_hz = SPI_SPEED
spi.mode = SPI_MODE

# === GPIO Init ===
GPIO.setmode(GPIO.BCM)
GPIO.setup(CS_PIN, GPIO.OUT)
GPIO.setup(SCK_PIN, GPIO.OUT)
GPIO.setup(MISO_PIN, GPIO.IN)
GPIO.setup(MOSI_PIN, GPIO.OUT)

# === Helper Functions ===

def write_register(addr, value):
    GPIO.output(CS_PIN, GPIO.LOW)
    spi.xfer2([addr & 0x7F, value])
    GPIO.output(CS_PIN, GPIO.HIGH)

def read_register(addr):
    GPIO.output(CS_PIN, GPIO.LOW)
    resp = spi.xfer2([addr | 0x80, 0x00])
    GPIO.output(CS_PIN, GPIO.HIGH)
    return resp[1]

# === Initialization ===

def initialize_ldc1101():
    chip_id = read_register(REG["CHIP_ID"])
    if chip_id != 0xD4:
        print(f"Unexpected CHIP ID: 0x{chip_id:02X}")
        return DEVICE_ERROR

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

# === Mode Config ===

def set_powermode(mode):
    write_register(REG["START_CONFIG"], mode)

def enable_lhr_mode():
    write_register(REG["LHR_RCOUNT_LSB"], 0x00)
    write_register(REG["LHR_RCOUNT_MSB"], 0x80)
    write_register(REG["LHR_OFFSET_LSB"], 0x00)
    write_register(REG["LHR_OFFSET_MSB"], 0x00)
    write_register(REG["LHR_CONFIG"], 0x01)

# === Data Fetch ===

def read_lhr_data():
    msb = read_register(REG["LHR_DATA_MSB"])
    mid = read_register(REG["LHR_DATA_MID"])
    lsb = read_register(REG["LHR_DATA_LSB"])
    return (msb << 16) | (mid << 8) | lsb

# === Debug ===

def display_all_registers():
    print("=== LDC1101 Register Dump ===")
    for name, addr in REG.items():
        try:
            val = read_register(addr)
            print(f"{name:20s} (0x{addr:02X}): 0x{val:02X}")
        except Exception as e:
            print(f"{name:20s} (0x{addr:02X}): Error: {e}")

# === Main ===

def main():
    if initialize_ldc1101() != DEVICE_OK:
        print("Initialization failed.")
        return

    print("LDC1101 initialized successfully. Switching to LHR mode...")
    set_powermode(ACTIVE_MODE)
    enable_lhr_mode()
    time.sleep(1)

    try:
        while True:
            lhr = read_lhr_data()
            print(f"LHR Data: {lhr}")
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nTerminated by user.")
    finally:
        spi.close()
        GPIO.cleanup()

if __name__ == '__main__':
    main()
