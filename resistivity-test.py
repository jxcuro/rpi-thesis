import spidev
import time
import RPi.GPIO as GPIO
import pigpio

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
PWM_PIN = 12  # GPIO12 / Pin 32

# Initialize the GPIO library
GPIO.setmode(GPIO.BCM)  # Use Broadcom pin numbering

# Setup the GPIO pins for SPI
GPIO.setup(CS_PIN, GPIO.OUT)
GPIO.setup(PWM_PIN, GPIO.OUT)

# Setup PWM pin
pi = pigpio.pi()
pi.set_mode(PWM_PIN, pigpio.OUTPUT)

# Device status indicators
DEVICE_ERROR = 0x01
DEVICE_OK = 0x00

# Power modes
ACTIVE_CONVERSION_MODE = 0x00
SLEEP_MODE = 0x01

# Function to write to register
def write_register(reg_addr, value):
    GPIO.output(CS_PIN, GPIO.LOW)
    time.sleep(0.1)  # Wait for the register to be updated
    spi.xfer2([reg_addr & 0x7F, value])  # Send write command (MSB = 0)
    time.sleep(0.1)  # Wait for the register to be updated
    GPIO.output(CS_PIN, GPIO.HIGH)

# Function to read register
def read_register(reg_addr):
    GPIO.output(CS_PIN, GPIO.LOW)
    time.sleep(0.1)  # Wait for the register to be updated
    result = spi.xfer2([reg_addr | 0x80, 0x00])  # Send read command (MSB = 1)
    time.sleep(0.1)  # Wait for the register to be updated
    GPIO.output(CS_PIN, GPIO.HIGH)
    return result[1]  # Return data from the register

def initialize_ldc1101():
    chip_id = read_register(CHIP_ID_REG)
    if chip_id != 0xD4:
        return DEVICE_ERROR

    # Default Init
    write_register(RP_SET_REG, 0x07)
    write_register(TC1_REG, 0x90)
    write_register(TC2_REG, 0xA0)
    write_register(DIG_CONFIG_REG, 0x03)
    write_register(ALT_CONFIG_REG, 0x00)  # 0x01 if needed
    write_register(RP_THRESH_H_MSB_REG, 0x00)
    write_register(RP_THRESH_L_LSB_REG, 0x00)
    write_register(RP_THRESH_L_MSB_REG, 0x00)
    write_register(INTB_MODE_REG, 0x00)
    write_register(START_CONFIG_REG, SLEEP_MODE)
    time.sleep(0.1)
    return DEVICE_OK

def enable_lhrmode():
    write_register(START_CONFIG_REG, SLEEP_MODE)
    # LHR application settings
    write_register(0x01, 0x07)
    write_register(0x04, 0xE7)
    write_register(0x30, 0x4A)
    write_register(0x31, 0x01)
    write_register(0xB8, 0x00)  # Not typical but kept as-is
    write_register(0x0B, 0x00)
    write_register(START_CONFIG_REG, ACTIVE_CONVERSION_MODE)

def getlhrdata():
    value = read_register(0x3A)
    value = (value << 8) | read_register(0x39)
    value = (value << 8) | read_register(0x38)
    return value

def perform_frequency_sweep(start_freq, end_freq, step_size, delay=0.5):
    # Perform frequency sweep
    results = []
    for freq in range(start_freq, end_freq + 1, step_size):
        pi.hardware_PWM(PWM_PIN, freq, 500000)  # 50% duty cycle
        time.sleep(delay)  # Wait for the LDC1101 to respond
        lhr_val = getlhrdata()
        results.append((freq, lhr_val))
        print(f"Freq: {freq} Hz, LHR Data: {lhr_val}")
    
    # Optionally store the results in a file
    with open('frequency_sweep_results.csv', 'w') as f:
        f.write("Frequency, LHR Data\n")
        for freq, lhr_val in results:
            f.write(f"{freq}, {lhr_val}\n")

def main():
    if initialize_ldc1101() != DEVICE_OK:
        print("Failed to initialize LDC1101.")
        return

    print("LDC1101 initialized. Entering LHR mode...")
    enable_lhrmode()
    time.sleep(1)

    # Perform frequency sweep from 1 kHz to 10 MHz with a step of 1 kHz
    perform_frequency_sweep(1000, 10000000, 1000)

# Run main
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        pi.hardware_PWM(PWM_PIN, 0, 0)  # Turn off PWM
        pi.stop()
        spi.close()
        GPIO.cleanup()
