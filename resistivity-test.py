import spidev
import time

# SPI setup
spi = spidev.SpiDev()
spi.open(0, 0)  # Bus 0, Device 0
spi.max_speed_hz = 50000  # Set SPI speed (adjust as necessary)
spi.mode = 0b00  # Set SPI Mode (Mode 0)

# LDC1101 commands and registers
LDC1101_CONFIG_CMD = 0x00     # Command to configure (change as needed)
LDC1101_READ_RP_CMD = 0x10    # Command to read RP data
LDC1101_READ_L_CMD = 0x11     # Command to read L data
LDC1101_SHUTDOWN_CMD = 0x0C   # Command to shut down the sensor (use if needed)
LDC1101_SLEEP_CMD = 0x0D      # Command to put the sensor in sleep mode
LDC1101_MODE_RP_L = 0x01      # Example mode for RP+L

# Function to initialize LDC1101 (using initialization logic from mikroSDK)
def ldc1101_init():
    """Initializes the LDC1101 sensor."""
    print("Initializing LDC1101...")
    
    # Ensure the chip is in Sleep mode for configuration
    ldc1101_setPowerMode('sleep')
    
    # Set the LDC1101 to RP+L mode (impedance and inductance mode)
    ldc1101_setMode('RP+L')
    
    # After configuring, switch to Active mode
    ldc1101_setPowerMode('active')
    
    time.sleep(0.1)  # Allow the sensor some time to initialize

def ldc1101_setPowerMode(mode):
    """Sets the power mode of the LDC1101."""
    if mode == 'sleep':
        print("Setting LDC1101 to Sleep mode")
        spi.xfer2([LDC1101_SLEEP_CMD])
    elif mode == 'active':
        print("Setting LDC1101 to Active mode")
        spi.xfer2([0x00])  # This would be the command to set the sensor to Active mode, based on the SDK function.

def ldc1101_setMode(mode):
    """Sets the mode of the LDC1101."""
    if mode == 'RP+L':
        print("Setting LDC1101 to RP+L mode")
        spi.xfer2([LDC1101_MODE_RP_L])  # Send RP+L mode command
    elif mode == 'LHR':
        print("Setting LDC1101 to LHR mode")
        spi.xfer2([0x02])  # Example command for LHR mode (you may need to adjust this based on datasheet)

def ldc1101_read_register(register):
    """Reads a single register from the LDC1101."""
    response = spi.xfer2([register | 0x80, 0x00])  # Send the register address and read command
    return response[1]  # The second byte is the data from the register

def ldc1101_read_data():
    """Reads RP and L data from LDC1101."""
    rp_data = ldc1101_read_register(LDC1101_READ_RP_CMD)
    l_data = ldc1101_read_register(LDC1101_READ_L_CMD)
    
    print(f"Inductive Data (RP): {rp_data}")
    print(f"Inductive Data (L): {l_data}")

def main():
    ldc1101_init()  # Initialize the sensor
    
    while True:
        ldc1101_read_data()  # Read inductive data
        time.sleep(1)  # Wait for a second before reading again

if __name__ == "__main__":
    main()
