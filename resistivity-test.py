import spidev
import time

# Constants for LDC1101 Registers (same as in C code)
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

# SPI Setup
spi = spidev.SpiDev()
spi.open(0, 0)  # SPI bus 0, CS 0 (Change based on your setup)
spi.max_speed_hz = 50000  # Adjust to your need

# Function to write a byte to a specific register
def ldc1101_write_byte(addr, data):
    spi.xfer2([addr, data])

# Function to read a byte from a specific register
def ldc1101_read_byte(addr):
    result = spi.xfer2([0x80 | addr, 0x00])
    return result[1]

# Initialize LDC1101 by checking the chip ID and setting default values
def ldc1101_init():
    chip_id = ldc1101_read_byte(LDC1101_REG_CHIP_ID)
    if chip_id != 0xD4:
        return "DEVICE_ERROR"
    
    # Default Initialization
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
    
    time.sleep(0.1)
    return "DEVICE_OK"

# Function to set the LHR mode
def ldc1101_set_LHR_mode():
    ldc1101_write_byte(LDC1101_REG_CFG_LHR, 0x80)  # Set LHR mode (0x80 for LHR mode)
    time.sleep(0.1)

# Function to read inductance data in LHR mode
def ldc1101_read_inductance_LHR():
    # Ensure LHR mode is enabled
    ldc1101_set_LHR_mode()
    
    # Read the LHR data registers
    lhr_data_lsb = ldc1101_read_byte(LDC1101_REG_LHR_DATA_LSB)
    lhr_data_mid = ldc1101_read_byte(LDC1101_REG_LHR_DATA_MID)
    lhr_data_msb = ldc1101_read_byte(LDC1101_REG_LHR_DATA_MSB)
    
    # Combine the bytes into the inductance value
    inductance_value = (lhr_data_msb << 16) | (lhr_data_mid << 8) | lhr_data_lsb
    
    # Return the inductance value
    return inductance_value

# Example Usage
init_status = ldc1101_init()
print(f"Initialization Status: {init_status}")

# Read inductance value in LHR mode
inductance_value = ldc1101_read_inductance_LHR()
print(f"Inductance Value (in LHR mode): {inductance_value}")
