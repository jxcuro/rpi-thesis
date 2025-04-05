import spidev
import time

# Initialize SPI communication
spi = spidev.SpiDev()
spi.open(0, 0)  # Open SPI bus 0, device 0 (CS pin)
spi.max_speed_hz = 50000  # Set the SPI speed, adjust as needed

# LDC1101 Register Addresses
LDC1101_MODE_REG = 0x00  # Mode register (RP_SET, etc.)
LDC1101_L_DATA_LSB = 0x23  # Lower 8 bits of inductance measurement result
LDC1101_L_DATA_MSB = 0x24  # Upper 8 bits of inductance measurement result

# Function to write to the LDC1101 register
def write_register(reg, value):
    # LSB and MSB are combined in one byte pair
    value_bytes = [(value >> 8) & 0xFF, value & 0xFF]
    spi.xfer([reg, value_bytes[0], value_bytes[1]])

# Function to read from the LDC1101 register
def read_register(reg):
    response = spi.xfer([reg, 0x00, 0x00])  # Send read command
    return (response[1] << 8) | response[2]  # Combine MSB and LSB

# Function to start a measurement and get the inductance value
def get_inductance():
    # Set the LDC1101 to LHR mode by writing the corresponding mode value
    write_register(LDC1101_MODE_REG, 0x12)  # Set the correct mode for LHR
    time.sleep(0.1)  # Wait for the sensor to stabilize

    # Read the inductance measurement result from the registers
    lsb = read_register(LDC1101_L_DATA_LSB)
    msb = read_register(LDC1101_L_DATA_MSB)
    
    # Combine MSB and LSB to get the full 16-bit inductance value
    inductance_raw = (msb << 8) | lsb
    print(f"Raw Inductance Value: {inductance_raw}")

    # Convert raw value to inductance (in microhenries)
    inductance_value = inductance_raw * 0.1  # Assuming scaling factor; adjust based on calibration
    print(f"Inductance (uH): {inductance_value:.2f}")

    return inductance_value

# Main loop to continuously measure inductance
try:
    while True:
        inductance = get_inductance()
        time.sleep(1)  # Delay for next measurement

except KeyboardInterrupt:
    print("Program terminated.")
    spi.close()  # Close SPI connection on exit
