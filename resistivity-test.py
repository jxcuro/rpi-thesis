import time
import spidev
import csv

# Initialize SPI
spi = spidev.SpiDev()
spi.open(0, 0)  # Using SPI bus 0, CS pin 0
spi.max_speed_hz = 50000
spi.mode = 0b00  # SPI mode 0 (CPOL=0, CPHA=0)

# Define chip select pin (GPIO) if needed
# GPIO.setmode(GPIO.BCM)
# GPIO.setup(CS_PIN, GPIO.OUT)

# Function to initialize LDC1101 (power mode, etc.)
def ldc1101_init():
    # Set power mode, configure SPI settings, etc.
    # This function will initialize the LDC1101 for communication
    pass

# Function to read RP data (Inductance + Impedance)
def ldc1101_getRPData():
    # Send command to the LDC1101 to read RP data
    # Read the response from the LDC1101
    data = spi.xfer2([0x00])  # Placeholder for the command
    rp_data = data[0]  # Read RP data from response
    return rp_data

# Function to read inductance data
def ldc1101_getInductance():
    # Send command to read inductance
    data = spi.xfer2([0x01])  # Placeholder for the inductance command
    inductance = data[0]  # Read inductance value
    return inductance

# Function to log data to CSV
def log_data(rp_data, inductance):
    with open("metal_data.csv", mode="a") as file:
        writer = csv.writer(file)
        writer.writerow([time.strftime('%Y-%m-%d %H:%M:%S'), rp_data, inductance])

# Main loop
def main():
    ldc1101_init()
    
    try:
        while True:
            # Read RP data (Inductance + Impedance)
            rp_data = ldc1101_getRPData()
            
            # Read inductance data
            inductance = ldc1101_getInductance()
            
            # Display data on LCD (adjust based on your LCD setup)
            print(f"RP Data: {rp_data}, Inductance: {inductance}")
            
            # Log the data to CSV
            log_data(rp_data, inductance)
            
            # Wait before next reading (adjust delay as needed)
            time.sleep(1)
    
    except KeyboardInterrupt:
        print("Program interrupted.")
        spi.close()

if __name__ == "__main__":
    main()
