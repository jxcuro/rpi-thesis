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
spi.max_speed_hz = 1000000  # 1 MHz for example, adjust if needed

# Function to write to the LDC1101
def write_ldc1101_register(register_address, data):
    # Set CS low to start the communication
    GPIO.output(CS_PIN, GPIO.LOW)
    
    # Send register address and data (write mode: no 0x80 bit)
    spi.xfer2([register_address, data])
    
    # Set CS high to end the communication
    GPIO.output(CS_PIN, GPIO.HIGH)

# Function to read from the LDC1101
def read_ldc1101_register(register_address):
    # Set CS low to start the communication
    GPIO.output(CS_PIN, GPIO.LOW)
    
    # Send register address with 0x80 bit to indicate a read
    response = spi.xfer2([register_address | 0x80, 0x00])
    
    # Set CS high to end the communication
    GPIO.output(CS_PIN, GPIO.HIGH)

    return response

# Test Writing to Register 0x00 and Reading It Back
try:
    test_value = 0x42  # Example test value to write
    register = 0x00  # Example register address for testing

    while True:
        write_ldc1101_register(register, test_value)  # Write test value to register
        print(f"Written {hex(test_value)} to register {hex(register)}")

        response = read_ldc1101_register(register)  # Read back from register
        print(f"Read {response} from register {hex(register)}")

        if response != [test_value]:
            print(f"Error: Response {response} doesn't match written value!")
        
        time.sleep(1)  # Sleep for a second between tests

except KeyboardInterrupt:
    print("Exiting program.")
finally:
    # Clean up GPIO and close SPI
    GPIO.cleanup()
    spi.close()
