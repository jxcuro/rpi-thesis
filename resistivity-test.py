import time
import spidev  # SPI library for Raspberry Pi

# SPI setup
spi = spidev.SpiDev()
spi.open(0, 0)  # Open SPI bus 0, device 0
spi.max_speed_hz = 50000  # Set SPI speed to 50 kHz
spi.mode = 0b00  # SPI mode 0 (CPOL=0, CPHA=0)

# Commands (Check datasheet for exact commands)
CMD_INIT = 0x0F  # Initialization command (replace with the correct one)
CMD_READ_RP = 0x01  # Command to read RP (Impedance + Inductance)
CMD_READ_LHR = 0x02  # Command to read LHR (High-Resolution Inductance)
CMD_POWER_MODE = 0x10  # Command to set power mode

# Function to send command and receive response
def send_command(command, num_bytes=2):
    response = spi.xfer2([command] + [0x00] * (num_bytes - 1))
    return response

# Function to initialize sensor and set power mode to active
def initialize_sensor():
    print("Initializing sensor...")
    send_command(CMD_INIT, 1)  # Initialize the sensor
    time.sleep(0.5)  # Give it some time to initialize

# Function to check the sensor's SPI communication
def check_spi_communication():
    print("Checking SPI communication...")
    response = send_command(0x00, 2)  # Send a simple command (adjust as needed)
    print(f"SPI response: {response}")
    if response == [0, 0]:
        print("Error: No response from LDC1101. Check SPI wiring or sensor power.")
    else:
        print("SPI communication is working.")

# Function to read the RP data (Inductance + Impedance)
def read_rp_data():
    print("Reading RP data...")
    response = send_command(CMD_READ_RP, 2)  # Command to read RP data
    rp_data = (response[0] << 8) | response[1]  # Combine bytes
    print(f"RP Data (Inductance + Impedance): {rp_data}")
    return rp_data

# Function to read the LHR data (High-Resolution Inductance)
def read_lhr_data():
    print("Reading LHR data...")
    response = send_command(CMD_READ_LHR, 2)  # Command to read LHR data
    lhr_data = (response[0] << 8) | response[1]  # Combine bytes
    print(f"LHR Data (High-Resolution Inductance): {lhr_data}")
    return lhr_data

# Main troubleshooting function
def run_debug():
    # Step 1: Check SPI Communication
    check_spi_communication()
    
    # Step 2: Initialize the sensor and set power mode to active
    initialize_sensor()
    
    # Step 3: Give sensor time to stabilize
    time.sleep(1)
    
    # Step 4: Read RP and LHR data
    rp_data = read_rp_data()
    if rp_data == 0:
        print("RP data is 0. Please check the sensor is in range of a conductive object.")
    
    lhr_data = read_lhr_data()
    if lhr_data == 0:
        print("LHR data is 0. Ensure the sensor is powered and in active mode.")
    
# Main script execution
if __name__ == "__main__":
    run_debug()
