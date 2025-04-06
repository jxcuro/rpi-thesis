import spidev
import time

# SPI settings
SPI_BUS = 0
SPI_DEVICE = 0
SPI_SPEED = 50000  # 50 kHz clock speed
SPI_MODE = 0b00    # SPI mode (CPOL = 0, CPHA = 0)

# LDC1101 register addresses (based on the provided register map)
RP_SET_REG = 0x01  # RP_SET register address
LHR_CONFIG_REG = 0x34  # LHR_CONFIG register address

# Initialize SPI
spi = spidev.SpiDev()
spi.open(SPI_BUS, SPI_DEVICE)
spi.max_speed_hz = SPI_SPEED
spi.mode = SPI_MODE

# Function to write to register
def write_register(reg_addr, value):
    spi.xfer2([reg_addr & 0x7F, value])  # Send write command (MSB = 0)

# Function to read register
def read_register(reg_addr):
    result = spi.xfer2([reg_addr | 0x80, 0x00])  # Send read command (MSB = 1)
    return result[1]  # Return data from the register

# Function to initialize the LDC1101
def initialize_ldc1101():
    # Configure RP_SET register (writing 0x07 as per default)
    write_register(RP_SET_REG, 0x07)
    time.sleep(0.1)  # Wait for the register to be updated

    # Configure LHR_CONFIG register (Set SENSOR_DIV to 0x01 to divide by 2)
    write_register(LHR_CONFIG_REG, 0x01)  # Set SENSOR_DIV to 0x01 (divide by 2)
    time.sleep(0.1)  # Wait for the register to be updated

    # Display updated register values
    display_all_registers()

# Function to display all register values
def display_all_registers():
    print("Reading all registers...")

    # List of all register addresses
    register_addresses = [
        0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A,
        0x0B, 0x0C, 0x16, 0x17, 0x18, 0x19, 0x20, 0x21, 0x22, 0x23, 0x24,
        0x30, 0x31, 0x32, 0x33, 0x34, 0x38, 0x39, 0x3A, 0x3B, 0x3E, 0x3F
    ]
    
    # Read and display each register value
    for addr in register_addresses:
        value = read_register(addr)
        print(f"Register 0x{addr:02X}: 0x{value:02X}")

# Main function to initialize the LDC1101 and display register values
def main():
    # Initialize LDC1101 by configuring necessary registers
    initialize_ldc1101()

    # Additional actions like configuring other registers or reading sensor data
    time.sleep(1)  # Wait after initialization

    # Display all register values after configuration
    display_all_registers()

# Run the main function
if __name__ == '__main__':
    main()

    # Cleanup
    spi.close()
