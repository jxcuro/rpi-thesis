import spidev
import time
import RPi.GPIO as GPIO

# Pin configuration for Raspberry Pi
CS_PIN = 8  # Chip select (CS)
MOSI_PIN = 10  # MOSI
MISO_PIN = 9  # MISO
SCK_PIN = 11  # SCK

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(CS_PIN, GPIO.OUT)

# SPI setup
spi = spidev.SpiDev()
spi.open(0, 0)  # Open SPI bus 0, device 0 (CS connected to GPIO 8)
spi.max_speed_hz = 1000000  # SPI speed (1 MHz)
spi.mode = 0b01  # SPI mode 1 (CPOL=0, CPHA=1)

# Function to set LDC1101 to Sleep mode (important for configuration)
def set_sleep_mode():
    # Writing the command to set LDC1101 to Sleep mode (FUNC_MODE = b01)
    # Assuming you have a method to write to the register (e.g., FUNC_MODE in register 0x0B)
    FUNC_MODE_REG = 0x0B
    FUNC_MODE_SLEEP = 0x01
    write_register(FUNC_MODE_REG, FUNC_MODE_SLEEP)
    time.sleep(0.1)  # Wait for the LDC1101 to enter sleep mode

# Function to set LDC1101 to Active mode
def set_active_mode():
    # Writing the command to set LDC1101 to Active mode (FUNC_MODE = b00)
    FUNC_MODE_REG = 0x0B
    FUNC_MODE_ACTIVE = 0x00
    write_register(FUNC_MODE_REG, FUNC_MODE_ACTIVE)
    time.sleep(0.1)  # Wait for LDC1101 to activate and start conversions

# Function to write to the LDC1101 register
def write_register(register, value):
    # Send the register address and value to the LDC1101
    GPIO.output(CS_PIN, GPIO.LOW)  # Pull CS low to start communication
    spi.xfer2([register, value])
    GPIO.output(CS_PIN, GPIO.HIGH)  # Pull CS high to end communication

# Function to read from the LDC1101 register
def read_register(register):
    # Send the register address to read from and get the response
    GPIO.output(CS_PIN, GPIO.LOW)
    response = spi.xfer2([register, 0x00])  # Read from the register
    GPIO.output(CS_PIN, GPIO.HIGH)
    return response[1]  # Return the data byte (second byte of response)

# Function to read inductance (assuming we want the LHR_DATA register)
def read_inductance():
    LHR_MSB_REG = 0x10  # Example register for LHR data MSB (adjust if different)
    LHR_LSB_REG = 0x11  # Example register for LHR data LSB (adjust if different)
    
    # Read MSB and LSB values
    msb = read_register(LHR_MSB_REG)
    lsb = read_register(LHR_LSB_REG)
    
    # Combine MSB and LSB to get the full inductance value
    inductance = (msb << 8) | lsb
    
    # Convert the raw value to inductance (based on LDC1101 datasheet scaling factor)
    inductance_henries = inductance * 0.241723  # Example scaling factor (check datasheet)
    
    return inductance_henries

# Main program
try:
    # Step 1: Set LDC1101 to Sleep Mode for configuration
    set_sleep_mode()
    
    # Step 2: Configure the LDC1101 as needed (e.g., setting registers)
    # You can set other registers here as per your configuration needs
    
    # Step 3: Set LDC1101 to Active Mode
    set_active_mode()
    
    # Step 4: Read Inductance
    inductance = read_inductance()
    print(f"Inductance: {inductance:.6f} H")
    
finally:
    # Clean up GPIO
    GPIO.cleanup()

