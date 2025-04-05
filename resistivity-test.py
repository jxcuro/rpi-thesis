import time
import RPi.GPIO as GPIO
import spidev

# Set the GPIO mode
GPIO.setmode(GPIO.BCM)

# Define the GPIO pin for CS (Chip Select)
CS_PIN = 24

# Set CS_PIN as an output pin
GPIO.setup(CS_PIN, GPIO.OUT)

# Initialize SPI
spi = spidev.SpiDev()
spi.open(0, 0)  # Bus 0, Device 0 (CE0)
spi.max_speed_hz = 1000000  # 1 MHz for example

# Function to communicate with the LDC1101
def communicate_with_ldc1101():
    # Set CS low to select the device
    GPIO.output(CS_PIN, GPIO.LOW)
    
    # Send a dummy byte (e.g., 0x00) to trigger the communication
    response = spi.xfer2([0x00])  # Replace with actual LDC1101 commands
    
    # Set CS high to deselect the device
    GPIO.output(CS_PIN, GPIO.HIGH)
    
    return response

# Example usage
try:
    while True:
        response = communicate_with_ldc1101()
        print("Received data:", response)
        time.sleep(1)
except KeyboardInterrupt:
    print("Exiting program.")
finally:
    # Clean up GPIO and close SPI
    GPIO.cleanup()
    spi.close()
