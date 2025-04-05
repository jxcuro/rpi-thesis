import spidev
import time

# Constants for LDC1101 Registers (same as in C code)
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
    
    # Default Initialization (you can customize this as needed)
    ldc1101_write_byte(LDC1101_REG_CFG_LHR, 0x00)  # Start with LHR off
    time.sleep(0.1)
    return "DEVICE_OK"

# Function to set the LHR mode
def ldc1101_set_LHR_mode():
    ldc1101_write_byte(LDC1101_REG_CFG_LHR, 0x80)  # Set LHR mode (0x80 for LHR mode)
    time.sleep(0.1)

# Function to check the LHR status
def ldc1101_check_LHR_status():
    status = ldc1101_read_byte(LDC1101_REG_LHR_STATUS)
    # Check if the measurement is complete or if there's an error
    # Status bit 0: 1 indicates measurement is ready
    if status & 0x01:
        return True
    else:
        return False

# Function to read inductance data in LHR mode
def ldc1101_read_inductance_LHR():
    # Ensure LHR mode is enabled
    ldc1101_set_LHR_mode()

    # Wait until the measurement is ready
    retries = 5
    while retries > 0:
        if ldc1101_check_LHR_status():
            break
        time.sleep(0.1)  # Wait 100ms before checking again
        retries -= 1

    if retries == 0:
        print("LHR measurement not ready in time.")
        return None

    # Read the LHR data registers
    lhr_data_lsb = ldc1101_read_byte(LDC1101_REG_LHR_DATA_LSB)
    lhr_data_mid = ldc1101_read_byte(LDC1101_REG_LHR_DATA_MID)
    lhr_data_msb = ldc1101_read_byte(LDC1101_REG_LHR_DATA_MSB)

    # Combine the bytes into the inductance value
    inductance_value = (lhr_data_msb << 16) | (lhr_data_mid << 8) | lhr_data_lsb

    return inductance_value

# Example Usage
init_status = ldc1101_init()
print(f"Initialization Status: {init_status}")

# Read inductance value in LHR mode
inductance_value = ldc1101_read_inductance_LHR()
if inductance_value is not None:
    print(f"Inductance Value (in LHR mode): {inductance_value}")
else:
    print("Failed to read inductance.")
