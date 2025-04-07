import spidev
import time
import RPi.GPIO as GPIO

# SPI settings
SPI_BUS = 0
SPI_DEVICE = 0
SPI_SPEED = 50000  # 50 kHz clock speed
SPI_MODE = 0b00  # SPI mode (CPOL = 0, CPHA = 0)

# LDC1101 register addresses
START_CONFIG_REG = 0x0B
RP_SET_REG = 0x01
TC1_REG = 0x02
TC2_REG = 0x03
DIG_CONFIG_REG = 0x04
ALT_CONFIG_REG = 0x05
RP_THRESH_H_LSB_REG = 0x06
RP_THRESH_H_MSB_REG = 0x07
RP_THRESH_L_LSB_REG = 0x08
RP_THRESH_L_MSB_REG = 0x09
INTB_MODE_REG = 0x0A
D_CONF_REG = 0x0C
L_THRESH_HI_LSB_REG = 0x16
L_THRESH_HI_MSB_REG = 0x17
L_THRESH_LO_LSB_REG = 0x18
L_THRESH_LO_MSB_REG = 0x19
STATUS_REG = 0x20
RP_DATA_LSB_REG = 0x21
RP_DATA_MSB_REG = 0x22
L_DATA_LSB_REG = 0x23
L_DATA_MSB_REG = 0x24
LHR_RCOUNT_LSB_REG = 0x30
LHR_RCOUNT_MSB_REG = 0x31
LHR_OFFSET_LSB_REG = 0x32
LHR_OFFSET_MSB_REG = 0x33
LHR_CONFIG_REG = 0x34
LHR_DATA_LSB_REG = 0x38
LHR_DATA_MID_REG = 0x39
LHR_DATA_MSB_REG = 0x3A
LHR_STATUS_REG = 0x3B
RID_REG = 0x3E
CHIP_ID_REG = 0x3F

# Initialize SPI
spi = spidev.SpiDev()
spi.open(SPI_BUS, SPI_DEVICE)
spi.max_speed_hz = SPI_SPEED
spi.mode = SPI_MODE

# Define the GPIO pins for the LDC1101
CS_PIN = 8   # Chip Select pin (example GPIO pin)
SCK_PIN = 11 # Clock pin (example GPIO pin)
MISO_PIN = 9 # MISO pin (example GPIO pin)
MOSI_PIN = 10 # MOSI pin (example GPIO pin)
PWM_PIN = 12  # GPIO12 / Pin 32

# Initialize the GPIO library
GPIO.setmode(GPIO.BCM)  # Use Broadcom pin numbering

# Setup the GPIO pins for SPI
GPIO.setup(CS_PIN, GPIO.OUT)
GPIO.setup(PWM_PIN, GPIO.OUT)

# Setup PWM pin
pwm = GPIO.PWM(PWM_PIN, 3000000)  # Start with 3 MHz for testing
pwm.start(50)  # 50% duty cycle

# Device status indicators
DEVICE_ERROR = 0x01
DEVICE_OK = 0x00

# Configure Power State (RW)
ACTIVE_CONVERSION_MODE = 0x00
SLEEP_MODE = 0x01
SHUTDOWN_MODE = 0x02

# Function to write to register
def write_register(reg_addr, value):
    GPIO.output(CS_PIN, GPIO.LOW)
    time.sleep(0.1)
    spi.xfer2([reg_addr & 0x7F, value])  # Write command
    time.sleep(0.1)
    GPIO.output(CS_PIN, GPIO.HIGH)

# Function to read register
def read_register(reg_addr):
    GPIO.output(CS_PIN, GPIO.LOW)
    time.sleep(0.1)
    result = spi.xfer2([reg_addr | 0x80, 0x00])  # Read command
    time.sleep(0.1)
    GPIO.output(CS_PIN, GPIO.HIGH)
    return result[1]

def initialize_ldc1101():
    chip_id = read_register(CHIP_ID_REG)
    if chip_id != 0xD4:
        return DEVICE_ERROR

    write_register(RP_SET_REG, 0x07)
    write_register(TC1_REG, 0x90)
    write_register(TC2_REG, 0xA0)
    write_register(DIG_CONFIG_REG, 0x03)
    write_register(ALT_CONFIG_REG, 0x00)
    write_register(RP_THRESH_H_MSB_REG, 0x00)
    write_register(RP_THRESH_L_LSB_REG, 0x00)
    write_register(RP_THRESH_L_MSB_REG, 0x00)
    write_register(INTB_MODE_REG, 0x00)
    write_register(START_CONFIG_REG, SLEEP_MODE)
    write_register(D_CONF_REG, 0x00)
    write_register(L_THRESH_HI_LSB_REG, 0x00)
    write_register(L_THRESH_HI_MSB_REG, 0x00)
    write_register(L_THRESH_LO_LSB_REG, 0x00)
    write_register(L_THRESH_LO_MSB_REG, 0x00)
    write_register(LHR_RCOUNT_LSB_REG, 0x00)
    write_register(LHR_RCOUNT_MSB_REG, 0x00)
    write_register(LHR_OFFSET_LSB_REG, 0x00)
    write_register(LHR_OFFSET_MSB_REG, 0x00)
    write_register(LHR_CONFIG_REG, 0x00)
    time.sleep(0.1)
    return DEVICE_OK

def enable_powermode(mode):
    write_register(START_CONFIG_REG, mode)

def enable_lmode():
    write_register(START_CONFIG_REG, SLEEP_MODE)
    write_register(ALT_CONFIG_REG, 0x01)
    write_register(D_CONF_REG, 0x01)
    write_register(START_CONFIG_REG, ACTIVE_CONVERSION_MODE)

def enable_rpmode():
    write_register(START_CONFIG_REG, SLEEP_MODE)
    write_register(ALT_CONFIG_REG, 0x02)
    write_register(D_CONF_REG, 0x00)
    write_register(START_CONFIG_REG, ACTIVE_CONVERSION_MODE)

def enable_lhrmode():
    write_register(START_CONFIG_REG, SLEEP_MODE)
    write_register(0x01, 0x07)
    write_register(0x04, 0xE7)
    write_register(0x30, 0x4A)
    write_register(0x31, 0x01)
    write_register(0xB8, 0x00)  # Not documented, kept as-is
    write_register(0x0B, 0x00)
    write_register(START_CONFIG_REG, ACTIVE_CONVERSION_MODE)

def getstatus():
    return read_register(STATUS_REG)

def getrpdata():
    value = read_register(RP_DATA_MSB_REG)
    value = (value << 8) | read_register(RP_DATA_LSB_REG)
    return value

def getldata():
    value = read_register(L_DATA_MSB_REG)
    value = (value << 8) | read_register(L_DATA_LSB_REG)
    return value

def getlhrdata():
    value = read_register(LHR_DATA_MSB_REG)
    value = (value << 8) | read_register(LHR_DATA_MID_REG)
    value = (value << 8) | read_register(LHR_DATA_LSB_REG)
    return value

def display_all_registers():
    register_addresses = [
        0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A,
        0x0B, 0x0C, 0x16, 0x17, 0x18, 0x19, 0x20, 0x21, 0x22, 0x23, 0x24,
        0x30, 0x31, 0x32, 0x33, 0x34, 0x38, 0x39, 0x3A, 0x3B, 0x3E, 0x3F
    ]
    for addr in register_addresses:
        value = read_register(addr)
        print(f"Register 0x{addr:02X}: 0x{value:02X}")

def main():
    if initialize_ldc1101() != DEVICE_OK:
        print("Failed to initialize LDC1101.")
        return

    print("LDC1101 initialized. Starting frequency sweep to enter LHR mode...")

    # Frequency sweep: 2 MHz to 10 MHz
    for freq in range(2000000, 10000001, 500000):
        print(f"\nSetting PWM frequency to {freq / 1_000_000:.1f} MHz")
        pwm.ChangeFrequency(freq)
        time.sleep(0.5)

        enable_lhrmode()
        time.sleep(0.5)

        lhr_val = getlhrdata()
        print(f"LHR Data at {freq / 1_000_000:.1f} MHz: {lhr_val}")

        if lhr_val != 0:
            print(f"LHR ACTIVE at {freq / 1_000_000:.1f} MHz")
            break
        else:
            print("No LHR data")

    print("\nFinished frequency sweep. Holding current frequency.\n")

    # Continue displaying LHR values
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
        pwm.stop()
        spi.close()
        GPIO.cleanup()
