import spidev
import time

# Define the SPI bus and device
spi = spidev.SpiDev()
spi.open(0, 0)  # Bus 0, Device 0 (change if necessary)
spi.max_speed_hz = 50000  # Adjust as needed
spi.mode = 0b00  # SPI Mode 0

# LDC1101 register addresses
REGISTER_FUNC_MODE = 0x00  # Functional mode register
REGISTER_STATUS = 0x01  # Status register
REGISTER_RESET = 0x7F  # Register to reset the LDC1101 (Shutdown mode)

# Function to write a value to a register
def write_register(register, value):
    spi.xfer2([register, value])

# Function to read a single byte from a register
def read_register(register):
    response = spi.xfer2([register | 0x80, 0x00])  # Send register address with read flag
    return response[1]

# Reset the LDC1101
def reset_ldc1101():
    print("Resetting LDC1101...")
    write_register(REGISTER_RESET, 0x80)  # Send reset command (shutdown)
    time.sleep(0.1)  # Wait for reset to complete
    print("LDC1101 reset complete.")
    
# Set LDC1101 to active mode
def set_active_mode():
    print("Setting LDC1101 to active mode...")
    write_register(REGISTER_FUNC_MODE, 0x01)  # Set the LDC1101 to active mode
    time.sleep(0.1)  # Wait for mode change
    functional_mode = read_register(REGISTER_FUNC_MODE)
    print(f"Functional Mode: {hex(functional_mode)}")
    
    if functional_mode == 0x01:
        print("LDC1101 is now in active mode.")
    else:
        print("Failed to set LDC1101 to active mode. Please check the configuration.")

# Main function to test the LDC1101
def main():
    reset_ldc1101()
    set_active_mode()

if __name__ == "__main__":
    main()
