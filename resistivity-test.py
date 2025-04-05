import spidev
import time

# Initialize SPI connection
spi = spidev.SpiDev()
spi.open(0, 0)  # Bus 0, Device 0
spi.max_speed_hz = 50000
spi.mode = 0b00  # SPI mode 0 (CPOL=0, CPHA=0)

# Function to write a register
def write_register(addr, value):
    # The MSB bit is 0 for write commands
    addr = addr & 0x7F  # Ensure MSB is 0 for write
    response = spi.xfer2([addr, value])
    time.sleep(0.01)  # Delay for SPI stability
    return response

# Function to read a register
def read_register(addr):
    # The MSB bit is 1 for read commands
    addr = addr | 0x80  # Set MSB to 1 for read
    response = spi.xfer2([addr, 0x00])  # Send dummy byte to read data
    time.sleep(0.01)  # Delay for SPI stability
    return response[1]  # Return the received data (second byte)

# Function to configure the LDC1101 for LHR measurement
def configure_ldc1101():
    # Sleep mode needed to configure the LDC chip
    write_register(0x0B, 0x01)

    # Register settings for LHR application
    write_register(0x01, 0x07)  # Set RP_SET (0x01) register value
    write_register(0x04, 0xE7)  # Set DIG_CONFIG (0x04) register value
    write_register(0x30, 0x4A)  # Set LHR_RCOUNT_LSB (0x30) register value
    write_register(0x31, 0x01)  # Set LHR_RCOUNT_MSB (0x31) register value

    # Send command to read LHR data (0x38 register)
    write_register(0xB8, 0x00)  # Command to read register 0x38

    # Return to conversion mode
    write_register(0x0B, 0x00)

# Function to read LHR data from registers
def read_lhr_data():
    # Read the LHR data from register 0x38, 0x39, 0x3A
    lhr_data_1 = read_register(0x38)  # Read from register 0x38
    lhr_data_2 = read_register(0x39)  # Read from register 0x39
    lhr_data_3 = read_register(0x3A)  # Read from register 0x3A
    print(f"LHR Data: 0x{lhr_data_1:02X}, 0x{lhr_data_2:02X}, 0x{lhr_data_3:02X}")
    return (lhr_data_1, lhr_data_2, lhr_data_3)

# Main function
def main():
    configure_ldc1101()
    # Wait a little for the conversion to complete
    time.sleep(0.1)
    lhr_data = read_lhr_data()
    print(f"Retrieved LHR Data: {lhr_data}")

# Run the main function
if __name__ == "__main__":
    main()
