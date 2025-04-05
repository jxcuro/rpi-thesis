import time
import spidev

# Constants for LDC1101 Register Addresses and Configuration
LDC1101_REG_CFG_LHR = 0x00   # Replace with the correct register address for LHR configuration
LDC1101_REG_L_DATA_MSB = 0x01  # Replace with actual register address for L data MSB
LDC1101_REG_L_DATA_LSB = 0x02  # Replace with actual register address for L data LSB
LDC1101_FUNC_MODE_ACTIVE_CONVERSION_MODE = 0x01  # Example value
LDC1101_FUNC_MODE_SLEEP_MODE = 0x00  # Example value

# Initialize SPI
spi = spidev.SpiDev()
spi.open(0, 0)  # Open SPI device 0, CS 0
spi.max_speed_hz = 50000
spi.mode = 0b00  # SPI mode 0 (CPOL=0, CPHA=0)

# Function to write to a register on the LDC1101
def write_register(register, value):
    data = [register, value]
    spi.xfer2(data)

# Function to read from a register on the LDC1101
def read_register(register):
    data = [register | 0x80, 0x00]  # Read operation: setting the MSB of the register address
    response = spi.xfer2(data)
    return response[1]  # Return the read value

# Initialize LDC1101 for LHR Mode
def initialize_ldc1101():
    # Set up LHR mode (replace with appropriate register settings)
    write_register(LDC1101_REG_CFG_LHR, 0x01)  # Example value for LHR mode
    time.sleep(0.1)  # Wait for configuration to take effect

    # Set the sensor to active conversion mode
    write_register(LDC1101_REG_CFG_LHR, LDC1101_FUNC_MODE_ACTIVE_CONVERSION_MODE)
    time.sleep(0.1)

# Get L Data from LDC1101
def get_l_data():
    msb = read_register(LDC1101_REG_L_DATA_MSB)
    lsb = read_register(LDC1101_REG_L_DATA_LSB)
    l_data = (msb << 8) | lsb  # Combine MSB and LSB to get 16-bit data
    return l_data

# Main Function
def main():
    initialize_ldc1101()
    while True:
        l_data = get_l_data()
        print(f"L Data: {l_data}")
        time.sleep(1)  # Wait before reading again

if __name__ == "__main__":
    main()
