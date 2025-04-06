import spidev
import time
import RPi.GPIO as GPIO

# SPI settings
SPI_BUS = 0
SPI_DEVICE = 0
SPI_SPEED = 50000  # 50 kHz
SPI_MODE = 0b00    # CPOL = 0, CPHA = 0

# LDC1101 register addresses
START_CONFIG_REG = 0x0B
RP_SET_REG = 0x01
TC1_REG = 0x02
TC2_REG = 0x03
DIG_CONFIG_REG = 0x04
ALT_CONFIG_REG = 0x05
D_CONF_REG = 0x0C
LHR_RCOUNT_LSB_REG = 0x30
LHR_RCOUNT_MSB_REG = 0x31
LHR_OFFSET_LSB_REG = 0x32
LHR_OFFSET_MSB_REG = 0x33
LHR_CONFIG_REG = 0x34
LHR_DATA_LSB_REG = 0x38
LHR_DATA_MID_REG = 0x39
LHR_DATA_MSB_REG = 0x3A
CHIP_ID_REG = 0x3F

# GPIO Chip Select pin
CS_PIN = 8

# Device status
DEVICE_OK = 0x00
DEVICE_ERROR = 0x01

# Power modes
ACTIVE_CONVERSION_MODE = 0x00

# GPIO setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(CS_PIN, GPIO.OUT)
GPIO.output(CS_PIN, GPIO.HIGH)

# SPI init
spi = spidev.SpiDev()
spi.open(SPI_BUS, SPI_DEVICE)
spi.max_speed_hz = SPI_SPEED
spi.mode = SPI_MODE

# Write to register
def write_register(reg_addr, value):
    GPIO.output(CS_PIN, GPIO.LOW)
    spi.xfer2([reg_addr & 0x7F, value])  # MSB=0 for write
    GPIO.output(CS_PIN, GPIO.HIGH)

# Read from register
def read_register(reg_addr):
    GPIO.output(CS_PIN, GPIO.LOW)
    result = spi.xfer2([reg_addr | 0x80, 0x00])  # MSB=1 for read
    GPIO.output(CS_PIN, GPIO.HIGH)
    return result[1]

# Initialize LDC1101
def initialize_ldc1101():
    chip_id = read_register(CHIP_ID_REG)
    if chip_id != 0xD4:
        return DEVICE_ERROR

    write_register(RP_SET_REG, 0x07)
    write_register(TC1_REG, 0x90)
    write_register(TC2_REG, 0xA0)
    write_register(DIG_CONFIG_REG, 0x03)
    write_register(ALT_CONFIG_REG, 0x00)
    write_register(D_CONF_REG, 0x00)
    write_register(START_CONFIG_REG, ACTIVE_CONVERSION_MODE)
    time.sleep(0.1)
    return DEVICE_OK

# Configure LHR Mode
def enable_lhrmode():
    write_register(ALT_CONFIG_REG, 0x03)         # Enable LHR mode
    write_register(D_CONF_REG, 0x00)             # Confirm LHR mode is selected
    write_register(LHR_RCOUNT_LSB_REG, 0x00)     # RCOUNT = 0x0800 (2048)
    write_register(LHR_RCOUNT_MSB_REG, 0x08)
    write_register(LHR_OFFSET_LSB_REG, 0x00)
    write_register(LHR_OFFSET_MSB_REG, 0x00)
    write_register(LHR_CONFIG_REG, 0x01)         # Enable LHR data output
    time.sleep(0.1)

# Get LHR data (24-bit)
def get_lhr_data():
    msb = read_register(LHR_DATA_MSB_REG)
    mid = read_register(LHR_DATA_MID_REG)
    lsb = read_register(LHR_DATA_LSB_REG)
    return (msb << 16) | (mid << 8) | lsb

# Main function
def main():
    if initialize_ldc1101() != DEVICE_OK:
        print("Failed to initialize LDC1101.")
        return

    print("LDC1101 initialized. Entering LHR mode...")
    enable_lhrmode()
    time.sleep(0.5)

    while True:
        lhr_val = get_lhr_data()
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
