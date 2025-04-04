import spidev
import time

# Define the SPI bus and device
spi = spidev.SpiDev()
spi.open(0, 0)  # Bus 0, Device 0 (Change to the correct SPI bus and device if needed)
spi.max_speed_hz = 50000  # Adjust the speed if necessary (max speed for LDC1101 is 1 MHz)
spi.mode = 0b00  # SPI Mode 0 (CPOL = 0, CPHA = 0)

# LDC1101 register addresses
REGISTER_CHIP_ID = 0x3F  # Read Chip ID register
REGISTER_LHRDATA = 0x38  # LHR (Inductance) data register (Low byte)

# Function to read a single byte from a register
def read_register(register):
    # Send the register address with the Read bit (0x80 for read operation)
    response = spi.xfer2([register | 0x80, 0x00])  # Send register address with read flag
    return response[1]  # Return the byte read

# Function to read 2 bytes of LHR data (Inductance)
def read_lhr_data():
    # LHR data is spread across two registers: 0x38 and 0x39
    high_byte = read_register(REGISTER_LHRDATA)
    low_byte = read_register(REGISTER_LHRDATA + 1)
    # Combine the two bytes into a single value
    lhr_value = (high_byte << 8) | low_byte
    return lhr_value

# Function to read the Chip ID
def read_chip_id():
    chip_id = read_register(REGISTER_CHIP_ID)
    return chip_id

# Main function
def main():
    print("Testing LDC1101...")
    
    # Read the Chip ID (should return 0xD2 if everything is working)
    chip_id = read_chip_id()
    print(f"Chip ID: {hex(chip_id)}")
    
    # Check if the Chip ID is correct
    if chip_id == 0xD2:
        print("LDC1101 Chip ID matches. Proceeding with data read...")
    else:
        print("Error: Invalid Chip ID. Please check the connection.")
        return
    
    # Read LHR data (Inductance)
    print("Reading LHR data...")
    lhr_value = read_lhr_data()
    print(f"LHR Data: {lhr_value}")
    
    # Display the result
    if lhr_value > 0:
        print("LHR data read successfully.")
    else:
        print("Error: Invalid LHR data. Check the sensor and connections.")

# Run the test
if __name__ == "__main__":
    main()
