import spidev
import time

# SPI setup
spi = spidev.SpiDev()
spi.open(0, 0)  # Bus 0, Device 0
spi.max_speed_hz = 50000  # Set SPI speed (adjust as necessary)
spi.mode = 0b00  # Set SPI Mode (Mode 0)

# LDC1101 commands and registers
LDC1101_STATUS_REG = 0x00  # Status register address
LDC1101_READ_RP_CMD = 0x10  # Command to read RP data
LDC1101_READ_L_CMD = 0x11  # Command to read L data
LDC1101_SHUTDOWN_CMD = 0x0C  # Command to shut down the sensor
LDC1101_SLEEP_CMD = 0x0D  # Command to put the sensor in sleep mode
LDC1101_MODE_RP_L = 0x01  # RP+L mode

# Function to initialize LDC1101
def ldc1101_init():
    """Initializes the LDC1101 sensor."""
    print("Initializing LDC1101...")
    
    # Ensure the chip is in Sleep mode for configuration
    ldc1101_setPowerMode('sleep')
    
    # Set the LDC1101 to RP+L mode (impedance and inductance mode)
    ldc1101_setMode('RP+L')
    
    # After configuring, switch to Active mode
    ldc1101_setPowerMode('active')
    
    # Verify if it's in active mode
    ldc1101_checkStatus()
    
    time.sleep(0.5)  # Allow the sensor some time to stabilize

def ldc1101_setPowerMode(mode):
    """Sets the power mode of the LDC1101."""
    if mode == 'sleep':
        print("Setting LDC1101 to Sleep mode")
        spi.xfer2([LDC1101_SLEEP_CMD])  # Sleep mode command
    elif mode == 'active':
        print("Setting LDC1101 to Active mode")
        # Active mode doesn't require any specific command for now
        pass

def ldc1101_setMode(mode):
    """Sets the mode of the LDC1101."""
    if mode == 'RP+L':
        print("Setting LDC1101 to RP+L mode")
        spi.xfer2([LDC1101_MODE_RP_L])  # RP+L mode command

def ldc1101_checkStatus():
    """Reads the status register to check the current mode."""
    response = spi.xfer2([LDC1101_STATUS_REG | 0x80, 0x00])  # Read status register
    status_byte = response[1]
    print(f"Status Register: 0x{status_byte:02X}")
    
    # Check the status byte for Active Mode (Bit 3 indicates 'Active')
    if (status_byte & 0x08) != 0:
        print("LDC1101 is in Active mode")
    else:
        print("LDC1101 is NOT in Active mode")

def ldc1101_read_register(register):
    """Reads a single register from the LDC1101."""
    response = spi.xfer2([register | 0x80, 0x00])  # Send the register address and read command
    return response[1]  # The second byte is the data from the register

def ldc1101_read_data():
    """Reads RP and L data from LDC1101."""
    rp_data = ldc1101_read_register(LDC1101_READ_RP_CMD)
    l_data = ldc1101_read_register(LDC1101_READ_L_CMD)
    
    print(f"Raw Inductive Data (RP): {rp_data}")
    print(f"Raw Inductive Data (L): {l_data}")
    
    if rp_data == 0 or l_data == 0:
        print("Warning: Data returned is zero. Ensure that the sensor is correctly connected and the object is near the sensor.")
    else:
        print(f"Inductive Data (RP): {rp_data}")
        print(f"Inductive Data (L): {l_data}")

def main():
    ldc1101_init()  # Initialize the sensor
    
    while True:
        ldc1101_read_data()  # Read inductive data
        time.sleep(1)  # Wait for a second before reading again

if __name__ == "__main__":
    main()
