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
    0x0B: 0x01,  # START_CONFIG - Will be updated in initialization
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
    0x30: 0x04,  # LHR_RCOUNT_LSB - Updated for proper LHR operation
    0x31: 0x00,  # LHR_RCOUNT_MSB
    0x32: 0x00,  # LHR_OFFSET_LSB
    0x33: 0x00,  # LHR_OFFSET_MSB
    0x34: 0x01,  # LHR_CONFIG - Updated to enable LHR measurement
    0x38: 0x00,  # LHR_DATA_LSB
    0x39: 0x00,  # LHR_DATA_MID
    0x3A: 0x00,  # LHR_DATA_MSB
    0x3B: 0x00,  # LHR_STATUS
    0x3E: 0x02,  # RID
    0x3F: 0xD4   # CHIP_ID
}

# Function to write to a register
def write_register(reg_addr, value):
    print(f"Writing 0x{value:02X} to Register 0x{reg_addr:02X} ({REGISTER_MAP.get(reg_addr, 'Unknown')})")
    spi.xfer2([reg_addr & 0x7F, value])  # Send write command (MSB = 0)
    time.sleep(0.01)  # Small delay after each write

# Function to read from a register
def read_register(reg_addr):
    result = spi.xfer2([reg_addr | 0x80, 0x00])  # Send read command (MSB = 1)
    value = result[1]
    print(f"Read 0x{value:02X} from Register 0x{reg_addr:02X} ({REGISTER_MAP.get(reg_addr, 'Unknown')})")
    return value

# Function to verify a register write
def verify_register(reg_addr, expected_value):
    actual_value = read_register(reg_addr)
    if actual_value != expected_value:
        print(f"WARNING: Register 0x{reg_addr:02X} ({REGISTER_MAP.get(reg_addr, 'Unknown')}) "
              f"value 0x{actual_value:02X} does not match expected 0x{expected_value:02X}")
        return False
    return True

# Function to initialize LDC1101 registers
def initialize_ldc1101():
    print("Initializing LDC1101 Registers...")

    # First, verify chip ID to ensure communication is working
    chip_id = read_register(0x3F)
    if chip_id != 0xD4:
        print(f"ERROR: Invalid CHIP_ID: 0x{chip_id:02X}, expected 0xD4")
        print("Check connections and SPI configuration")
        return False

    # Write default values to all registers with delays
    for reg_addr, default_value in DEFAULT_VALUES.items():
        write_register(reg_addr, default_value)
        time.sleep(0.1)  # Delay to ensure proper register updating

    # Special configuration for LHR operation
    # Configure LHR_CONFIG register for proper operation
    write_register(0x34, 0x01)  # Enable LHR measurement mode
    time.sleep(0.1)
    
    # Configure LHR_RCOUNT registers
    write_register(0x30, 0x04)  # LHR_RCOUNT_LSB
    write_register(0x31, 0x00)  # LHR_RCOUNT_MSB
    time.sleep(0.1)
    
    # Set START_CONFIG to enable both RP and LHR measurements
    write_register(0x0B, 0x03)  # Enable both RP and LHR measurements
    time.sleep(0.2)

    # Verify critical register settings
    critical_registers = [0x0B, 0x30, 0x31, 0x34]
    expected_values = [0x03, 0x04, 0x00, 0x01]
    
    for reg, expected in zip(critical_registers, expected_values):
        if not verify_register(reg, expected):
            print(f"WARNING: Critical register 0x{reg:02X} verification failed")

    print("LDC1101 Initialization Complete.")
    return True

# Function to display all register values
def display_all_registers():
    print("\nReading All Registers:")
    for reg_addr in sorted(REGISTER_MAP.keys()):
        register_name = REGISTER_MAP[reg_addr]
        register_value = read_register(reg_addr)
        print(f"{register_name} (0x{reg_addr:02X}): 0x{register_value:02X}")

# Function to monitor LHR_STATUS until it is ready
def monitor_lhr_status():
    print("Monitoring LHR_STATUS...")
    max_attempts = 20  # Prevent infinite loop
    attempts = 0
    
    while attempts < max_attempts:
        lhr_status = read_register(0x3B)  # Read LHR_STATUS
        print(f"LHR_STATUS: 0x{lhr_status:02X}")

        # Check if bit 0 is set (0x01), indicating data is ready
        # Note: Adjust this condition based on the actual LDC1101 datasheet
        if lhr_status & 0x01:
            print("LHR data is ready.")
            return True
            
        attempts += 1
        time.sleep(0.5)  # Wait before checking again
    
    print("WARNING: LHR_STATUS never indicated ready state after multiple attempts")
    return False

# Function to read LHR data
def read_lhr_data():
    lsb = read_register(0x38)
    mid = read_register(0x39)
    msb = read_register(0x3A)
    
    # Combine the bytes into a 24-bit value
    lhr_value = (msb << 16) | (mid << 8) | lsb
    
    print(f"LHR Data: 0x{lhr_value:06X} ({lhr_value})")
    return lhr_value

# Main function to initialize the LDC1101, monitor LHR_STATUS, and read all registers
def main():
    try:
        print("LDC1101 SPI Interface - Revised Version")
        print("======================================")
        
        # Initialize LDC1101 by configuring necessary registers
        if not initialize_ldc1101():
            print("Initialization failed. Exiting.")
            return
        
        # Wait for a moment to allow the device to stabilize
        print("Waiting for device to stabilize...")
        time.sleep(1)
        
        # Monitor the LHR_STATUS register
        if monitor_lhr_status():
            # Read LHR data if status indicates it's ready
            read_lhr_data()
        
        # Display all register values
        display_all_registers()
        
        print("Operation completed successfully.")
        
    except Exception as e:
        print(f"ERROR: {str(e)}")
    finally:
        # Cleanup
        spi.close()
        print("SPI connection closed.")

# Run the main function
if __name__ == '__main__':
    main()
