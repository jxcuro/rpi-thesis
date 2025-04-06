import time
import spidev

# SPI Settings
SPI_BUS = 0          # SPI Bus (typically 0 or 1)
SPI_DEVICE = 0       # SPI Device (usually 0)
SPI_SPEED = 100000   # 100 kHz SPI speed, can adjust based on requirements
SPI_MODE = 0b00      # SPI Mode (CPOL = 0, CPHA = 0)

# Initialize SPI
spi = spidev.SpiDev()
spi.open(SPI_BUS, SPI_DEVICE)
spi.max_speed_hz = SPI_SPEED
spi.mode = SPI_MODE

# LDC1101 Register Addresses
REG_RP_SET = 0x01
REG_TC1 = 0x02
REG_TC2 = 0x03
REG_DIG_CONFIG = 0x04
REG_ALT_CONFIG = 0x05
REG_RP_THRESH_H_LSB = 0x06
REG_RP_THRESH_H_MSB = 0x07
REG_RP_THRESH_L_LSB = 0x08
REG_RP_THRESH_L_MSB = 0x09
REG_INTB_MODE = 0x0A
REG_START_CONFIG = 0x0B
REG_D_CONF = 0x0C
REG_L_THRESH_HI_LSB = 0x16
REG_L_THRESH_HI_MSB = 0x17
REG_L_THRESH_LO_LSB = 0x18
REG_L_THRESH_LO_MSB = 0x19
REG_STATUS = 0x20
REG_RP_DATA_LSB = 0x21
REG_RP_DATA_MSB = 0x22
REG_L_DATA_LSB = 0x23
REG_L_DATA_MSB = 0x24
REG_LHR_RCOUNT_LSB = 0x30
REG_LHR_RCOUNT_MSB = 0x31
REG_LHR_OFFSET_LSB = 0x32
REG_LHR_OFFSET_MSB = 0x33
REG_LHR_CONFIG = 0x34
REG_LHR_DATA_LSB = 0x38
REG_LHR_DATA_MID = 0x39
REG_LHR_DATA_MSB = 0x3A
REG_LHR_STATUS = 0x3B
REG_RID = 0x3E
REG_CHIP_ID = 0x3F

# Default register values as per your data
DEFAULT_RP_SET = 0x07
DEFAULT_TC1 = 0x90
DEFAULT_TC2 = 0xA0
DEFAULT_DIG_CONFIG = 0x03
DEFAULT_ALT_CONFIG = 0x00
DEFAULT_RP_THRESH = 0x00  # RP_THRESH_LSB, RP_THRESH_MSB, etc.
DEFAULT_LHR_CONFIG = 0x00  # LHR Configuration

# Function to write to a register
def write_register(reg_addr, value):
    spi.xfer2([reg_addr & 0x7F, value])  # Send write command (MSB = 0)

# Function to read from a register
def read_register(reg_addr):
    result = spi.xfer2([reg_addr | 0x80, 0x00])  # Send read command (MSB = 1)
    return result[1]  # Return data from the register

# Function to initialize LDC1101 registers
def initialize_ldc1101():
    print("Initializing LDC1101 Registers...")

    # Write default value to RP_SET register
    write_register(REG_RP_SET, DEFAULT_RP_SET)

    # Configure TC1 and TC2 for internal time constants
    write_register(REG_TC1, DEFAULT_TC1)
    write_register(REG_TC2, DEFAULT_TC2)

    # Configure Digital settings
    write_register(REG_DIG_CONFIG, DEFAULT_DIG_CONFIG)

    # Configure Additional settings
    write_register(REG_ALT_CONFIG, DEFAULT_ALT_CONFIG)

    # RP Threshold settings (High and Low) (default values)
    write_register(REG_RP_THRESH_H_LSB, DEFAULT_RP_THRESH)
    write_register(REG_RP_THRESH_H_MSB, DEFAULT_RP_THRESH)
    write_register(REG_RP_THRESH_L_LSB, DEFAULT_RP_THRESH)
    write_register(REG_RP_THRESH_L_MSB, DEFAULT_RP_THRESH)

    # Configure INTB Mode (Interrupt reporting on SDO pin)
    write_register(REG_INTB_MODE, 0x00)  # Default value, can modify as needed

    # Power-up the device by writing to the START_CONFIG register
    write_register(REG_START_CONFIG, 0x01)

    # Configure Sensor Amplitude Control (D_CONF)
    write_register(REG_D_CONF, 0x00)

    # Configure L Threshold High and Low settings (default)
    write_register(REG_L_THRESH_HI_LSB, 0x00)
    write_register(REG_L_THRESH_HI_MSB, 0x00)
    write_register(REG_L_THRESH_LO_LSB, 0x00)
    write_register(REG_L_THRESH_LO_MSB, 0x00)

    # Configure High Resolution LHR settings
    write_register(REG_LHR_RCOUNT_LSB, 0x00)
    write_register(REG_LHR_RCOUNT_MSB, 0x00)
    write_register(REG_LHR_OFFSET_LSB, 0x00)
    write_register(REG_LHR_OFFSET_MSB, 0x00)

    # Configure LHR Mode
    write_register(REG_LHR_CONFIG, DEFAULT_LHR_CONFIG)

    print("LDC1101 Initialization Complete.")

# Function to read and print the Status register
def read_status():
    status = read_register(REG_STATUS)
    print(f"Status Register: 0x{status:02X}")

# Function to read High Resolution Inductance Data
def read_lhr_data():
    lhr_data_lsb = read_register(REG_LHR_DATA_LSB)
    lhr_data_mid = read_register(REG_LHR_DATA_MID)
    lhr_data_msb = read_register(REG_LHR_DATA_MSB)
    print(f"LHR Data (LSB, MID, MSB): 0x{lhr_data_lsb:02X}, 0x{lhr_data_mid:02X}, 0x{lhr_data_msb:02X}")

# Main function to initialize the LDC1101 and read data
def main():
    # Initialize LDC1101 by configuring necessary registers
    initialize_ldc1101()

    # Wait for a moment to allow the device to stabilize
    time.sleep(0.5)

    # Read and print the status register
    read_status()

    # Read High-Resolution Inductance Data
    read_lhr_data()

# Run the main function
if __name__ == '__main__':
    main()

    # Cleanup
    spi.close()
