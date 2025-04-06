import spidev
import time

# Define LDC1101 registers (Including LHR related ones)
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

DEVICE_ERROR = 0x01
DEVICE_OK = 0x00

# Initialize SPI
spi = spidev.SpiDev()
spi.open(0, 0)  # Bus 0, Device 0
spi.max_speed_hz = 50000
spi.mode = 0b00  # SPI mode 0 (CPOL=0, CPHA=0)

# Function to write a byte to a register
def ldc1101_write_byte(address, data):
    write_data = [address, data]
    spi.xfer2(write_data)

# Function to read a byte from a register
def ldc1101_read_byte(address):
    write_data = [0x80 | address]
    response = spi.xfer2(write_data)
    return response[1]

# Initialize LDC1101 for LHR mode
def ldc1101_init_lhr():
    chip_id = ldc1101_read_byte(LDC1101_REG_CHIP_ID)
    if chip_id != 0xD4:
        return DEVICE_ERROR
    
    # Set up LHR mode by configuring the LHR mode register (0x34)
    ldc1101_write_byte(LDC1101_REG_CFG_LHR, 0x01)  # Enable LHR mode
    
    # Default initialization for other registers
    ldc1101_write_byte(LDC1101_REG_CFG_RP_MEASUREMENT_DYNAMIC_RANGE, 0x07)
    ldc1101_write_byte(LDC1101_REG_CFG_INTERNAL_TIME_CONSTANT_1, 0x90)
    ldc1101_write_byte(LDC1101_REG_CFG_INTERNAL_TIME_CONSTANT_2, 0xA0)
    ldc1101_write_byte(LDC1101_REG_CFG_RP_L_CONVERSION_INTERVAL, 0x03)
    ldc1101_write_byte(LDC1101_REG_CFG_ADDITIONAL_DEVICE, 0x00)  # 0x01
    ldc1101_write_byte(LDC1101_REG_RP_THRESH_H_MSB, 0x00)
    ldc1101_write_byte(LDC1101_REG_RP_THRESH_L_LSB, 0x00)
    ldc1101_write_byte(LDC1101_REG_RP_THRESH_L_MSB, 0x00)
    ldc1101_write_byte(LDC1101_REG_CFG_INTB_MODE, 0x00)
    ldc1101_write_byte(LDC1101_REG_CFG_POWER_STATE, 0x01)  # Sleep mode
    ldc1101_write_byte(LDC1101_REG_AMPLITUDE_CONTROL_REQUIREMENT, 0x00)  # 0x01
    ldc1101_write_byte(LDC1101_REG_L_THRESH_HI_LSB, 0x00)
    ldc1101_write_byte(LDC1101_REG_L_THRESH_HI_MSB, 0x00)
    ldc1101_write_byte(LDC1101_REG_L_THRESH_LO_LSB, 0x00)
    ldc1101_write_byte(LDC1101_REG_L_THRESH_LO_MSB, 0x00)
    ldc1101_write_byte(LDC1101_REG_LHR_RCOUNT_LSB, 0x00)
    ldc1101_write_byte(LDC1101_REG_LHR_RCOUNT_MSB, 0x00)
    ldc1101_write_byte(LDC1101_REG_LHR_OFFSET_LSB, 0x00)
    ldc1101_write_byte(LDC1101_REG_LHR_OFFSET_MSB, 0x00)
    
    time.sleep(0.1)
    
    return DEVICE_OK

# Set power mode (Active/Sleep/Shutdown)
def ldc1101_set_power_mode(mode):
    ldc1101_write_byte(LDC1101_REG_CFG_POWER_STATE, mode)

# Get LHR data (reading 3 bytes and combining them)
def ldc1101_get_lhr_data():
    data_lsb = ldc1101_read_byte(LDC1101_REG_LHR_DATA_LSB)
    data_mid = ldc1101_read_byte(LDC1101_REG_LHR_DATA_MID)
    data_msb = ldc1101_read_byte(LDC1101_REG_LHR_DATA_MSB)
    data = (data_msb << 16) | (data_mid << 8) | data_lsb
    return data

# Main
if __name__ == "__main__":
    if ldc1101_init_lhr() == DEVICE_OK:
        print("LDC1101 initialized successfully in LHR mode")
        ldc1101_set_power_mode(0x00)  # Active mode

        while True:
            lhr_data = ldc1101_get_lhr_data()
            print(f"LHR Data: {lhr_data}")
            time.sleep(1)
    else:
        print("Failed to initialize LDC1101")
