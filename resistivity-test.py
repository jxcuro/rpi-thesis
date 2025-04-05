import time
import smbus

# Initialize I2C bus
bus = smbus.SMBus(1)

# LDC1101 I2C address
LDC1101_ADDRESS = 0x2A

def read_register(address, register):
    try:
        return bus.read_byte_data(address, register)
    except Exception as e:
        print(f"Error reading register 0x{register:02X}: {e}")
        return None

def check_LHR_status():
    # Read the LHR_STATUS register (0x01)
    status = read_register(LDC1101_ADDRESS, 0x01)
    if status is not None:
        print(f"LHR_STATUS (0x01): 0x{status:02X}")
        if status & 0x01 == 0:
            print("LHR Data Ready: Unread data is available.")
        else:
            print("LHR Data Ready: No unread data.")
        
        # Check for any errors in the status register
        if status & 0x10:
            print("Zero Count Error (ERR_ZC): Occurred.")
        else:
            print("Zero Count Error (ERR_ZC): No error.")
        
        if status & 0x08:
            print("Conversion Over-range Error (ERR_OR): Occurred.")
        else:
            print("Conversion Over-range Error (ERR_OR): No error.")
        
        if status & 0x04:
            print("Conversion Under-range Error (ERR_UR): Occurred.")
        else:
            print("Conversion Under-range Error (ERR_UR): No error.")
        
        if status & 0x02:
            print("Conversion Overflow Error (ERR_OF): Occurred.")
        else:
            print("Conversion Overflow Error (ERR_OF): No error.")

def configure_LHR():
    # Example: Configure LHR registers (0x0E, 0x0F, 0x10) and ensure it's done correctly.
    # These settings depend on your specific use case; below is just an example.
    print("Configuring LHR registers...")

    # Example: Write to register 0x0E (LHR configuration)
    bus.write_byte_data(LDC1101_ADDRESS, 0x0E, 0x01)  # Write some example configuration byte
    print("Wrote to 0x0E: 0x01")
    
    # Example: Write to register 0x0F (LHR configuration)
    bus.write_byte_data(LDC1101_ADDRESS, 0x0F, 0x02)  # Another example configuration
    print("Wrote to 0x0F: 0x02")
    
    # Example: Write to register 0x10 (LHR configuration)
    bus.write_byte_data(LDC1101_ADDRESS, 0x10, 0x03)  # Another example configuration
    print("Wrote to 0x10: 0x03")

def check_conversion_data():
    # Read the LHR Conversion Data register (0x38) for the latest result
    data = read_register(LDC1101_ADDRESS, 0x38)
    if data is not None:
        print(f"LHR Conversion Data (0x38): {data}")
    else:
        print("No conversion data available.")
    
def wait_for_data_ready():
    print("Waiting for LHR data to be ready...")
    while True:
        status = read_register(LDC1101_ADDRESS, 0x01)
        if status & 0x01 == 0:  # LHR_DRDY == 0 means data is ready
            print("Data is ready!")
            break
        else:
            print("Waiting for data to be ready...")
        time.sleep(0.1)

# Main function to configure and check status
def main():
    print("Checking initial LHR_STATUS...")
    check_LHR_status()
    
    print("\nConfiguring LHR...")
    configure_LHR()
    
    print("\nWaiting for data to be ready...")
    wait_for_data_ready()
    
    print("\nChecking LHR_STATUS again after configuration...")
    check_LHR_status()
    
    print("\nReading conversion data...")
    check_conversion_data()

# Run the main function
if __name__ == "__main__":
    main()
