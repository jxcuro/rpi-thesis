import spidev
import RPi.GPIO as GPIO
import time

# Define GPIO settings for Chip Select (CS)
CS_PIN = 10  # GPIO10 (CE0)

# SPI Settings
SPI_CHANNEL = 0  # SPI Channel 0 (CE0)
SPI_SPEED = 500000  # SPI Speed in Hz

# Register Definitions
LDC1101_REG_CFG_RP_MEASUREMENT_DYNAMIC_RANGE = 0x01
LDC1101_REG_CFG_INTERNAL_TIME_CONSTANT_1 = 0x02
LDC1101_REG_CFG_INTERNAL_TIME_CONSTANT_2 = 0x03
LDC1101_REG_CFG_RP_L_CONVERSION_INTERVAL = 0x04
LDC1101_REG_CFG_ADDITIONAL_DEVICE = 0x05
LDC1101_REG_RP_THRESH_H_LSB = 0x06
LDC1101_REG_RP_THRESH_H_MSB = 0x07
LDC1101_REG_RP_THRESH_L_LSB = 0x08
LDC1101_REG_RP_THRESH_L_MSB = 0x09
LDC1101_REG_CFG_INTB_MODE = 0x0A
LDC1101_REG_CFG_POWER_STATE = 0x0B
LDC1101_REG_AMPLITUDE_CONTROL_REQUIREMENT = 0x0C
LDC1101_REG_L_THRESH_HI_LSB = 0x16
LDC1101_REG_L_THRESH_HI_MSB = 0x17
LDC1101_REG_L_THRESH_LO_LSB = 0x18
LDC1101_REG_L_THRESH_LO_MSB = 0x19
LDC1101_REG_RP_L_MEASUREMENT_STATUS = 0x20
LDC1101_REG_RP_DATA_LSB = 0x21
LDC1101_REG_RP_DATA_MSB = 0x22
LDC1101_REG_L_DATA_LSB = 0x23
LDC1101_REG_L_DATA_MSB = 0x24
LDC1101_REG_LHR_RCOUNT_LSB = 0x30
LDC1101_REG_LHR_RCOUNT_MSB = 0x31
LDC1101_REG_LHR_OFFSET_LSB = 0x32
LDC1101_REG_LHR_OFFSET_MSB = 0x33
LDC1101_REG_CFG_LHR = 0x34
LDC1101_REG_LHR_DATA_LSB = 0x38
LDC1101_REG_LHR_DATA_MID = 0x39
LDC1101_REG_LHR_DATA_MSB = 0x3A
LDC1101_REG_LHR_STATUS = 0x3B
LDC1101_REG_DEVICE_RID_VALUE = 0x3E
LDC1101_REG_CHIP_ID = 0x3F

# Initialize SPI and GPIO
spi = spidev.SpiDev()
spi.open(SPI_CHANNEL, 0)  # Open SPI channel 0 (CE0)
spi.max_speed_hz = SPI_SPEED

GPIO.setmode(GPIO.BCM)
GPIO.setup(CS_PIN, GPIO.OUT)
GPIO.output(CS_PIN, GPIO.HIGH)  # Deactivate chip select initially

# Function to write a byte to a specific register
def ldc1101_write_byte(addr, data):
    GPIO.output(CS_PIN, GPIO.LOW)  # Activate chip select
    spi.xfer([addr, data])  # Send register address and data
    GPIO.output(CS_PIN, GPIO.HIGH)  # Deactivate chip select

# Function to read a byte from a specific register
def ldc1101_read_byte(addr):
    GPIO.output(CS_PIN, GPIO.LOW)  # Activate chip select
    response = spi.xfer([0x80 | addr, 0x00])  # Send register address with read flag
    GPIO.output(CS_PIN, GPIO.HIGH)  # Deactivate chip select
    return response[1]  # Return the received data

# Function to initialize the LDC1101 sensor
def ldc1101_init():
    chip_id = ldc1101_read_byte(LDC1101_REG_CHIP_ID)
    if chip_id != 0xD4:  # Check chip ID
        print(f"Error: Expected chip ID 0xD4, got 0x{chip_id:02X}")
        return False

    # Initialize registers with default values
    ldc1101_write_byte(LDC1101_REG_CFG_RP_MEASUREMENT_DYNAMIC_RANGE, 0x07)
    ldc1101_write_byte(LDC1101_REG_CFG_INTERNAL_TIME_CONSTANT_1, 0x90)
    ldc1101_write_byte(LDC1101_REG_CFG_INTERNAL_TIME_CONSTANT_2, 0xA0)
    ldc1101_write_byte(LDC1101_REG_CFG_RP_L_CONVERSION_INTERVAL, 0x03)
    ldc1101_write_byte(LDC1101_REG_CFG_ADDITIONAL_DEVICE, 0x00)
    ldc1101_write_byte(LDC1101_REG_RP_THRESH_H_MSB, 0x00)
    ldc1101_write_byte(LDC1101_REG_RP_THRESH_L_LSB, 0x00)
    ldc1101_write_byte(LDC1101_REG_RP_THRESH_L_MSB, 0x00)
    ldc1101_write_byte(LDC1101_REG_CFG_INTB_MODE, 0x00)
    ldc1101_write_byte(LDC1101_REG_CFG_POWER_STATE, 0x01)  # Sleep mode
    ldc1101_write_byte(LDC1101_REG_AMPLITUDE_CONTROL_REQUIREMENT, 0x00)
    ldc1101_write_byte(LDC1101_REG_L_THRESH_HI_LSB, 0x00)
    ldc1101_write_byte(LDC1101_REG_L_THRESH_HI_MSB, 0x00)
    ldc1101_write_byte(LDC1101_REG_L_THRESH_LO_LSB, 0x00)
    ldc1101_write_byte(LDC1101_REG_L_THRESH_LO_MSB, 0x00)
    ldc1101_write_byte(LDC1101_REG_LHR_RCOUNT_LSB, 0x00)
    ldc1101_write_byte(LDC1101_REG_LHR_RCOUNT_MSB, 0x00)
    ldc1101_write_byte(LDC1101_REG_LHR_OFFSET_LSB, 0x00)
    ldc1101_write_byte(LDC1101_REG_LHR_OFFSET_MSB, 0x00)
    ldc1101_write_byte(LDC1101_REG_CFG_LHR, 0x00)

    time.sleep(0.1)  # Wait for sensor to stabilize
    return True

# Function to set the power mode of the sensor
def ldc1101_set_power_mode(mode):
    ldc1101_write_byte(LDC1101_REG_CFG_POWER_STATE, mode)

# Function to switch to L Mode (L measurement mode)
def ldc1101_go_to_L_mode():
    ldc1101_write_byte(LDC1101_REG_CFG_ADDITIONAL_DEVICE, 0x01)
    ldc1101_write_byte(LDC1101_REG_AMPLITUDE_CONTROL_REQUIREMENT, 0x01)

# Function to switch to RP Mode (RP measurement mode)
def ldc1101_go_to_RP_mode():
    ldc1101_write_byte(LDC1101_REG_CFG_ADDITIONAL_DEVICE, 0x00)
    ldc1101_write_byte(LDC1101_REG_AMPLITUDE_CONTROL_REQUIREMENT, 0x00)

# Main program
if __name__ == "__main__":
    if not ldc1101_init():
        print("Failed to initialize LDC1101 sensor.")
    else:
        print("LDC1101 sensor initialized successfully.")
