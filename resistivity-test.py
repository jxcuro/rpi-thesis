import time
import spidev

# Initialize SPI interface
spi = spidev.SpiDev()
spi.open(0, 0)  # Bus 0, Device 0
spi.max_speed_hz = 1000000  # 1 MHz SPI clock speed

# Function to write a value to a register
def write_register(register, value):
    spi.xfer2([register, value])

# Function to read a value from a register
def read_register(register):
    response = spi.xfer2([register, 0x00])
    return response[1]

# Initialize necessary registers
def initialize_registers():
    # Set LHR_CONFIG to 0x00 to reset configuration
    write_register(0x34, 0x00)  # LHR_CONFIG reset

    # Wait for a while to ensure settings take effect
    time.sleep(0.1)

    # Check if LHR_CONFIG is set to 0x00
    lhr_config = read_register(0x34)
    print(f"LHR_CONFIG (0x34): {hex(lhr_config)}")

    # Set LHR_CONFIG to 0x01 for enabling high-resolution L measurement
    write_register(0x34, 0x01)
    
    # Wait again for the configuration to take effect
    time.sleep(0.1)

    # Check if the LHR_CONFIG is properly set to 0x01
    lhr_config = read_register(0x34)
    print(f"LHR_CONFIG (0x34) after setting: {hex(lhr_config)}")

# Function to read all registers for debugging
def read_all_registers():
    print("Reading all registers:")
    for reg in range(0x00, 0x40):  # Checking a range of registers
        value = read_register(reg)
        print(f"Register {hex(reg)}: {hex(value)}")

# Run the initialization and register reading functions
initialize_registers()
read_all_registers()
