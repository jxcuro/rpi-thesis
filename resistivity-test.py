import time
import spidev

# Define the LDC1101 SPI register addresses (update these as per datasheet)
LDC1101_FUNC_MODE = 0x0B   # Function Mode register
LDC1101_STATUS_REG = 0x00  # Status register
LDC1101_POR_READ = 0x10   # Power-On Reset status register

# Initialize SPI
spi = spidev.SpiDev()
spi.open(0, 0)  # Bus 0, Device 0 (ensure correct bus and device numbers)
spi.max_speed_hz = 5000  # Set a lower SPI speed for debugging (can increase later)
spi.mode = 0b00  # SPI Mode 0 (CPOL=0, CPHA=0)

# Function to read a register
def read_register(register):
    response = spi.xfer2([register | 0x80, 0x00])  # Read operation (MSB set to 1)
    return response[1]  # Return the register value (2nd byte)

# Function to write to a register
def write_register(register, value):
    spi.xfer2([register, value])  # Write operation

# Check if the LDC1101 is out of reset
def check_reset():
    por_read = read_register(LDC1101_POR_READ)
    print(f"POR_READ: {por_read:02x}")
    if por_read & 0x01:
        print("LDC1101 is still in reset!")
    else:
        print("LDC1101 is out of reset.")

# Set FUNC_MODE to 0x01 (active mode) and verify
def set_active_mode():
    print("Setting LDC1101 to active mode...")
    write_register(LDC1101_FUNC_MODE, 0x01)
    time.sleep(0.05)  # Wait a bit to allow mode change

    # Read FUNC_MODE multiple times to confirm it's set to 0x01
    func_mode = read_register(LDC1101_FUNC_MODE)
    print(f"FUNC_MODE: {func_mode:02x}")

    if func_mode != 0x01:
        print("Error: LDC1101 is not in active mode!")
    else:
        print("LDC1101 is in active mode.")

# Read status register to verify SPI communication
def check_status():
    status = read_register(LDC1101_STATUS_REG)
    print(f"Status register: {status:02x}")
    if status == 0x00:
        print("Status is normal.")
    else:
        print(f"Warning: Status indicates error or unexpected condition. Status: {status:02x}")

# Read inductance data (for debugging, you may need to use LDC1101's other registers for actual inductance)
def read_inductance():
    inductance_data = read_register(LDC1101_STATUS_REG)  # Replace with correct register for inductance
    print(f"Inductance Data: {inductance_data}")
    if inductance_data == 0:
        print("Warning: Inductance value is 0, check connections and configuration.")
    else:
        print(f"Inductance: {inductance_data}")

# Main Debugging Sequence
def debug_ldc1101():
    # Step 1: Check if the LDC1101 is out of reset
    check_reset()

    # Step 2: Set the LDC1101 to active mode
    set_active_mode()

    # Step 3: Check SPI communication by reading the status register
    check_status()

    # Step 4: Read and display inductance data (if possible)
    read_inductance()

# Run the debugging sequence
debug_ldc1101()

# Close the SPI connection
spi.close()
