import time
import spidev  # SPI interface library

# SPI setup
spi = spidev.SpiDev()
spi.open(0, 0)  # Open SPI bus 0, device 0 (adjust if needed)
spi.max_speed_hz = 50000  # Adjust SPI speed if necessary

# Constants for commands based on datasheet
CMD_INIT = 0x0F  # Initialization command (example, adjust with actual)
CMD_RP_DATA = 0x01  # Command to read RP data (impedance + inductance)
CMD_LHR_DATA = 0x02  # Command to read high-resolution inductance (LHR mode)
CMD_POWER_MODE = 0x10  # Command to set power mode
CMD_SLEEP_MODE = 0x00  # Command for sleep mode
CMD_ACTIVE_MODE = 0x01  # Command for active mode

# Function to send a command via SPI and read data
def spi_write_read(command, num_bytes=2):
    # Send the command and receive the response
    response = spi.xfer2([command] + [0x00] * (num_bytes - 1))
    return response

# Function to set power mode (active, sleep, or shutdown)
def set_power_mode(mode):
    print(f"Setting power mode to: {mode}")
    spi_write_read(CMD_POWER_MODE, 1)
    time.sleep(0.1)

# Function to initialize LDC1101
def ldc1101_init():
    print("Initializing LDC1101...")
    # Set to active mode
    set_power_mode(CMD_ACTIVE_MODE)
    # Add any specific initialization commands here based on datasheet
    time.sleep(0.5)

# Function to read RP (Inductance + Impedance) data
def read_rp_data():
    print("Reading RP data...")
    response = spi_write_read(CMD_RP_DATA, 2)  # Adjust command if needed
    rp_data = (response[0] << 8) | response[1]
    return rp_data

# Function to read high-resolution inductance (LHR mode)
def read_lhr_data():
    print("Reading LHR data...")
    response = spi_write_read(CMD_LHR_DATA, 2)  # Adjust command if needed
    lhr_data = (response[0] << 8) | response[1]
    return lhr_data

# Function to check if the sensor is communicating correctly
def check_spi_communication():
    print("Testing SPI communication...")
    response = spi_write_read(0x01, 2)  # Simple read command to test communication
    print(f"SPI response: {response}")
    if response == [0, 0]:
        print("No valid response received. Check SPI wiring and initialization.")
    else:
        print("SPI communication seems fine.")

# Function to display readings from the LDC1101
def display_readings():
    print("Starting sensor data readout...")
    
    # Check SPI communication
    check_spi_communication()
    
    # Initialize the LDC1101
    ldc1101_init()
    
    # Wait for sensor to stabilize
    time.sleep(1)
    
    # Read RP data
    rp_data = read_rp_data()
    print(f"RP Data (Inductance + Impedance): {rp_data}")
    
    if rp_data == 0:
        print("RP data is zero. Ensure the sensor is in range of a conductive object.")
    
    # Optional: Read high-resolution inductance data if LHR mode is enabled
    lhr_data = read_lhr_data()
    print(f"LHR Data (High-Resolution Inductance): {lhr_data}")
    
    if lhr_data == 0:
        print("LHR data is zero. Ensure the clock signal is provided if using LHR mode.")

# Main script execution
if __name__ == "__main__":
    # Set power mode to active
    set_power_mode(CMD_ACTIVE_MODE)
    
    # Display sensor readings
    display_readings()
