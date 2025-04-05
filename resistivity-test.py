import spidev
import time

# Initialize SPI
spi = spidev.SpiDev()
spi.open(0, 0)  # Open SPI bus 0, device 0 (CS pin is 0 by default)
spi.max_speed_hz = 50000  # Set SPI speed (adjust as needed)
spi.mode = 0b00  # SPI Mode 0 (CPOL = 0, CPHA = 0)

def ldc1101_getRPData():
    """Reads RP data from the LDC1101."""
    # Send the read command for RP data (replace with your actual command)
    response = spi.xfer2([0x10, 0x00, 0x00])  # Replace 0x10 with actual command to read RP data
    rp_data = (response[1] << 8) | response[2]  # Assuming the response is 16-bit
    return rp_data

def print_log(message):
    """Simulates logging the message to a console."""
    print(message)

def applicationTask():
    """Main application loop to read and log RP data."""
    while True:
        RP_Data = ldc1101_getRPData()  # Get RP data from LDC1101
        demoText = str(RP_Data)  # Convert RP data to string
        
        # Log the data
        print_log(f"Inductive Linear Position: {demoText}")
        
        # Delay to simulate Delay_100ms()
        time.sleep(0.1)  # 100ms delay

# Start the task loop
applicationTask()
