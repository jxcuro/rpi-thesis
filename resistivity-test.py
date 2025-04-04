import time
import spidev

# Define LDC1101 register addresses
LDC1101_FUNC_MODE_REG = 0x00
LDC1101_L_DATA_LSB_REG = 0x23
LDC1101_L_DATA_MSB_REG = 0x24
LDC1101_STATUS_REG = 0x01

# SPI setup
spi = spidev.SpiDev()
spi.open(0, 0)  # Bus 0, Device 0 (check your wiring to ensure correct SPI bus and device)
spi.max_speed_hz = 50000  # Set the SPI speed (adjust as needed)
spi.mode = 0b00  # SPI Mode 0 (CPOL=0, CPHA=0)

# Function to write to a register
def write_register(register, value):
    spi.xfer2([register & 0xFF, value & 0xFF])

# Function to read from a register
def read_register(register):
    response = spi.xfer2([register & 0xFF, 0x00])  # Send register address with dummy data (0x00)
    return response[1]  # The response data is in the second byte

# Function to initialize the LDC1101 and set it to active mode
def initialize_ldc1101():
    for attempt in range(5):  # Try to set the device 5 times
        print(f"Attempt {attempt + 1}: Setting LDC1101 to active mode...")
        
        # Set FUNC_MODE register to 0x01 for Active mode (according to datasheet)
        write_register(LDC1101_FUNC_MODE_REG, 0x01)
        time.sleep(0.5)  # Wait for a longer time (500ms) to stabilize
        
        # Verify FUNC_MODE after setting it
        func_mode = read_register(LDC1101_FUNC_MODE_REG)
        print(f"FUNC_MODE after setting: {func_mode:#04x}")  # Ensure it's 0x01
        
        # Check if the device has transitioned to active mode
        status = read_register(LDC1101_STATUS_REG)
        print(f"Status Register after setting active mode: {status:#04x}")
        
        if status != 0x00:
            print("Device has exited reset mode and is active.")
            break
        else:
            print("Device still in reset mode, retrying...")
        
        # If it has not exited reset mode, wait and retry
        time.sleep(1)

# Function to read the inductance value
def read_inductance():
    # Read the lower and upper 8 bits of inductance data
    lsb = read_register(LDC1101_L_DATA_LSB_REG)
    msb = read_register(LDC1101_L_DATA_MSB_REG)

    print(f"LSB: {lsb}, MSB: {msb}")  # Print both LSB and MSB values

    # Combine the MSB and LSB to form the full inductance value (16-bit)
    inductance_value = (msb << 8) | lsb
    print(f"Inductance Data: {inductance_value}")

    if inductance_value == 0:
        print("Warning: Inductance value is 0, check connections and configuration.")
    else:
        print(f"Inductance: {inductance_value} (raw data)")

# Main function to run the test
def main():
    initialize_ldc1101()  # Initialize LDC1101 and set it to active mode
    time.sleep(0.5)  # Wait a little before taking a reading
    read_inductance()  # Read and print the inductance data

# Run the test
if __name__ == "__main__":
    main()
