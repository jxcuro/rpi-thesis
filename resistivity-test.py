import time
import spidev

# SPI Settings
SPI_BUS = 0          # SPI Bus (typically 0 or 1)
SPI_DEVICE = 0       # SPI Device (usually 0)
SPI_SPEED = 8000000   # 100 kHz SPI speed, can adjust based on requirements
SPI_MODE = 0b00      # SPI Mode (CPOL = 0, CPHA = 0)

# Initialize SPI
spi = spidev.SpiDev()
spi.open(SPI_BUS, SPI_DEVICE)
spi.max_speed_hz = SPI_SPEED
spi.mode = SPI_MODE

# LDC1101 Register Addresses
REGISTER_MAP = {
    0x01: "RP_SET",
    0x02: "TC1",
    0x03: "TC2",
    0x04: "DIG_CONFIG",
    0x05: "ALT_CONFIG",
    0x06: "RP_THRESH_H_LSB",
    0x07: "RP_THRESH_H_MSB",
    0x08: "RP_THRESH_L_LSB",
    0x09: "RP_THRESH_L_MSB",
    0x0A: "INTB_MODE",
    0x0B: "START_CONFIG",
    0x0C: "D_CONF",
    0x16: "L_THRESH_HI_LSB",
    0x17: "L_THRESH_HI_MSB",
    0x18: "L_THRESH_LO_LSB",
    0x19: "L_THRESH_LO_MSB",
    0x20: "STATUS",
    0x21: "RP_DATA_LSB",
    0x22: "RP_DATA_MSB",
    0x23: "L_DATA_LSB",
    0x24: "L_DATA_MSB",
    0x30: "LHR_RCOUNT_LSB",
    0x31: "LHR_RCOUNT_MSB",
    0x32: "LHR_OFFSET_LSB",
    0x33: "LHR_OFFSET_MSB",
    0x34: "LHR_CONFIG",
    0x38: "LHR_DATA_LSB",
    0x39: "LHR_DATA_MID",
    0x3A: "LHR_DATA_MSB",
    0x3B: "LHR_STATUS",
    0x3E: "RID",
    0x3F: "CHIP_ID"
}

# Default register values as per your data
DEFAULT_VALUES = {
    0x01: 0x07,  # RP_SET
    0x02: 0x90,  # TC1
    0x03: 0xA0,  # TC2
    0x04: 0x03,  # DIG_CONFIG
    0x05: 0x00,  # ALT_CONFIG
    0x06: 0x00,  # RP_THRESH_H_LSB
    0x07: 0x00,  # RP_THRESH_H_MSB
    0x08: 0x00,  # RP_THRESH_L_LSB
    0x09: 0x00,  # RP_THRESH_L_MSB
    0x0A: 0x00,  # INTB_MODE
    0x0B: 0x01,  # START_CONFIG
    0x0C: 0x00,  # D_CONF
    0x16: 0x00,  # L_THRESH_HI_LSB
    0x17: 0x00,  # L_THRESH_HI_MSB
    0x18: 0x00,  # L_THRESH_LO_LSB
    0x19: 0x00,  # L_THRESH_LO_MSB
    0x20: 0x00,  # STATUS
    0x21: 0x00,  # RP_DATA_LSB
    0x22: 0x00,  # RP_DATA_MSB
    0x23: 0x00,  # L_DATA_LSB
    0x24: 0x00,  # L_DATA_MSB
    0x30: 0x00,  # LHR_RCOUNT_LSB
    0x31: 0x00,  # LHR_RCOUNT_MSB
    0x32: 0x00,  # LHR_OFFSET_LSB
    0x33: 0x00,  # LHR_OFFSET_MSB
    0x34: 0x00,  # LHR_CONFIG
    0x38: 0x00,  # LHR_DATA_LSB
    0x39: 0x00,  # LHR_DATA_MID
    0x3A: 0x00,  # LHR_DATA_MSB
    0x3B: 0x00,  # LHR_STATUS
    0x3E: 0x02,  # RID
    0x3F: 0xD4   # CHIP_ID
}

# Function to write to a register
def write_register(reg_addr, value):
    print(f"Writing 0x{value:02X} to Register 0x{reg_addr:02X}")
    spi.xfer2([reg_addr & 0x7F, value])  # Send write command (MSB = 0)

# Function to read from a register
def read_register(reg_addr):
    result = spi.xfer2([reg_addr | 0x80, 0x00])  # Send read command (MSB = 1)
    return result[1]  # Return data from the register

# Function to initialize LDC1101 registers
def initialize_ldc1101():
    print("Initializing LDC1101 Registers...")

    # Write default values to all registers with delays
    for reg_addr, default_value in DEFAULT_VALUES.items():
        write_register(reg_addr, default_value)
        time.sleep(0.1)  # Short delay to ensure proper writing

    print("LDC1101 Initialization Complete.")

# Function to display all register values
def display_all_registers():
    print("\nReading All Registers:")
    for reg_addr in REGISTER_MAP:
        register_name = REGISTER_MAP[reg_addr]
        register_value = read_register(reg_addr)
        print(f"{register_name} (0x{reg_addr:02X}): 0x{register_value:02X}")

# Main function to initialize the LDC1101 and read all registers
def main():
    # Initialize LDC1101 by configuring necessary registers
    initialize_ldc1101()

    # Wait for a moment to allow the device to stabilize
    time.sleep(0.5)

    # Display all register values
    display_all_registers()

# Run the main function
if __name__ == '__main__':
    main()

    # Cleanup
    spi.close()
