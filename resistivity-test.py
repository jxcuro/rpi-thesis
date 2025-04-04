import time
import spidev
import RPi.GPIO as GPIO

# LDC1101 SPI configuration
LDC1101_CS_PIN = 24   # Chip select (CE0 on Raspberry Pi)
LDC1101_SCK_PIN = 23  # SPI Clock
LDC1101_MISO_PIN = 21 # SPI MISO (Master In Slave Out)
LDC1101_MOSI_PIN = 19 # SPI MOSI (Master Out Slave In)

# Initialize GPIO and SPI interface
GPIO.setmode(GPIO.BCM)
GPIO.setup(LDC1101_CS_PIN, GPIO.OUT, initial=GPIO.HIGH)

spi = spidev.SpiDev()
spi.open(0, 0)  # Bus 0, Device 0 (CE0)
spi.max_speed_hz = 50000  # Set SPI clock speed
spi.mode = 0b00  # SPI Mode 0

# LDC1101 Register addresses
LDC1101_STATUS_REG = 0x00
LDC1101_RP_DATA_LSB = 0x21
LDC1101_RP_DATA_MSB = 0x22
LDC1101_FUNC_MODE = 0x0B
LDC1101_POR_READ = 0x0A

# Function to read from a register
def read_register(reg):
    GPIO.output(LDC1101_CS_PIN, GPIO.LOW)
    response = spi.xfer2([reg, 0x00])  # Send register address and dummy byte
    GPIO.output(LDC1101_CS_PIN, GPIO.HIGH)
    return response[1]  # Return the byte read from the register

# Function to write to a register
def write_register(reg, value):
    GPIO.output(LDC1101_CS_PIN, GPIO.LOW)
    spi.xfer2([reg, value])  # Write value to register
    GPIO.output(LDC1101_CS_PIN, GPIO.HIGH)
    time.sleep(0.01)  # Small delay for register write to complete

# Function to read a 16-bit value (for inductance data)
def read_inductance():
    # Read LSB first, then MSB (LDC1101 data registers)
    lsb = read_register(LDC1101_RP_DATA_LSB)
    msb = read_register(LDC1101_RP_DATA_MSB)
    inductance = (msb << 8) | lsb
    return inductance

# Check if the LDC1101 is out of reset
def check_reset():
    por_read = read_register(LDC1101_POR_READ)
    if por_read & 0x01:
        print("LDC1101 is in reset mode.")
    else:
        print("LDC1101 is out of reset.")

# Check the status register for errors
def check_status():
    status = read_register(LDC1101_STATUS_REG)
    print(f"Status register: {status:02x}")
    if status & 0x01:
        print("Error: Conversion error.")
    if status & 0x02:
        print("Error: Oscillation failure.")
    if status & 0x04:
        print("Error: Frequency out of range.")
    if status & 0x08:
        print("Error: Power-on-reset is active.")

# Set the device to active mode
def set_active_mode():
    func_mode = read_register(LDC1101_FUNC_MODE)
    print(f"Functional Mode (before): {func_mode:02x}")
    if func_mode != 0x01:
        print("Setting LDC1101 to active mode.")
        write_register(LDC1101_FUNC_MODE, 0x01)  # Set to active mode
        time.sleep(0.05)  # Wait for the device to transition into active mode
    else:
        print("LDC1101 is already in active mode.")

# Main function for testing the LDC1101
def test_ldc1101():
    print("Testing LDC1101...")

    # Check if the device is in reset
    check_reset()

    # Check the status register for any error conditions
    check_status()

    # Set LDC1101 to active mode
    set_active_mode()

    # Attempt to read inductance data
    inductance = read_inductance()
    print(f"Inductance Data: {inductance}")
    if inductance == 0:
        print("Warning: Inductance value is 0, check connections and configuration.")

    # Check if the device is properly configured (Functional Mode)
    func_mode = read_register(LDC1101_FUNC_MODE)
    print(f"Functional Mode: {func_mode:02x}")
    if func_mode != 0x01:  # Expected mode is active (0x01)
        print("Warning: LDC1101 is not in active mode. Check FUNC_MODE register configuration.")

    print("LDC1101 Test Complete.")

# Run the test
try:
    test_ldc1101()
except Exception as e:
    print(f"Error: {e}")
finally:
    GPIO.cleanup()  # Clean up GPIO when done
    spi.close()  # Close SPI interface
