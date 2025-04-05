import spidev
import time
import RPi.GPIO as GPIO

# Set up GPIO for CS pin
CS_PIN = 24  # GPIO 24 for Chip Select (CE0)
GPIO.setmode(GPIO.BCM)
GPIO.setup(CS_PIN, GPIO.OUT)
GPIO.output(CS_PIN, GPIO.HIGH)  # Ensure CS is high initially

# Initialize SPI
spi = spidev.SpiDev()
spi.open(0, 0)  # Bus 0, Device 0
spi.max_speed_hz = 5000  # SPI speed
spi.mode = 0b11  # SPI Mode 3 (CPOL=1, CPHA=1)

# Function to manually control CS (Chip Select) pin
def select_chip():
    GPIO.output(CS_PIN, GPIO.LOW)  # CS goes LOW to select the chip

def deselect_chip():
    GPIO.output(CS_PIN, GPIO.HIGH)  # CS goes HIGH to deselect the chip

# Function to write data to a register
def write_register(register, value):
    select_chip()  # Assert CS low before sending
    response = spi.xfer2([register | 0x80, value])  # Write to register
    deselect_chip()  # Deassert CS after transfer
    print(f"Write: Register {hex(register)} = {hex(value)}, Response: {response}")

# Function to read data from a register
def read_register(register):
    select_chip()  # Assert CS low before reading
    response = spi.xfer2([register, 0x00])  # Read register
    deselect_chip()  # Deassert CS after transfer
    print(f"Read: Register {hex(register)} -> {hex(response[1])}")
    return response[1]

# Initialize LDC1101 (if needed)
def init_ldc1101():
    print("Initializing LDC1101...")
    write_register(0x01, 0x00)  # Soft reset command to register 0x01
    time.sleep(0.1)  # Wait for the device to reset
    write_register(0x00, 0x01)  # Set control register to 0x01 as an example
    time.sleep(0.1)

# Read the Chip ID (for testing)
def read_chip_id():
    return read_register(0x3F)  # Chip ID register

# Main flow
init_ldc1101()

# Read and print the chip ID
chip_id = read_chip_id()
print(f"LDC1101 Chip ID: {hex(chip_id)}")

# Try reading status register for additional debug info
status = read_register(0x08)  # Status register
print(f"Status Register: {hex(status)}")
