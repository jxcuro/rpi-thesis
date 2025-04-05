import spidev
import time

# Define SPI parameters
SPI_BUS = 0
SPI_DEVICE = 0
SPI_SPEED = 10000  # 10 kHz for safe timing
SPI_MODE = 0
SPI_BITS = 8

# Initialize SPI
spi = spidev.SpiDev()
spi.open(SPI_BUS, SPI_DEVICE)
spi.max_speed_hz = SPI_SPEED
spi.mode = SPI_MODE
spi.bits_per_word = SPI_BITS

# Function to read data from LDC1101 register
def read_register(register):
    response = spi.xfer2([register | 0x80, 0x00])  # 0x80 enables read
    print(f"Read from 0x{register:02X}: 0x{response[1]:02X}")
    return response[1]

# Function to write data to LDC1101 register
def write_register(register, value):
    response = spi.xfer2([register & 0x7F, value])  # 0x7F disables read operation
    print(f"Writing to 0x{register:02X}: 0x{value:02X}, Response: 0x{response[1]:02X}")

# Step 1: Delay after power-up to allow initialization (0.8 ms)
time.sleep(0.001)  # Wait for 1 ms to ensure proper initialization

# Step 2: Set to Active Mode (0x00) for LHR measurement
write_register(0x0B, 0x00)  # Start conversion in active mode
time.sleep(0.01)  # Allow time for conversion to begin

# Step 3: Set the required configuration registers for LHR measurement
# RPMAX (0x01) for RP range
write_register(0x01, 0x75)  # RPMAX = 0x75 (as per settings)
time.sleep(0.01)

# RPMIN (0x02) for RP minimum setting
write_register(0x02, 0x05)  # RPMIN = 1.5 kΩ (0x05)
time.sleep(0.01)

# MIN_FREQ (0x03) for minimum frequency setting
write_register(0x03, 0x0E)  # MIN_FREQ = 4.0 MHz (0x0E)
time.sleep(0.01)

# DIG_CONF (0x04) for digital config
write_register(0x04, 0xE7)  # Set for LHR measurement
time.sleep(0.01)

# RCOUNT (0x05) for sample rate setting
write_register(0x05, 0x4A)  # RCOUNT_LSB = 0x4A
write_register(0x06, 0x01)  # RCOUNT_MSB = 0x01
time.sleep(0.01)

# Step 4: Poll for data ready (LHR_STATUS: DRDYB) from register 0x3B
while True:
    status = read_register(0x3B)
    if status & 0x01:  # Check if DRDYB (bit 0) is set, meaning data is ready
        print("Data is ready")
        break
    time.sleep(0.01)

# Step 5: Read LHR conversion results from registers 0x38, 0x39, 0x3A
lhr_data_low = read_register(0x38)  # LHR data lower byte
lhr_data_high = read_register(0x39)  # LHR data upper byte

# Combine the high and low bytes to form the 16-bit result
lhr_value = (lhr_data_high << 8) | lhr_data_low
print(f"LHR Inductance Measurement Value: {lhr_value}")

# Step 6: Optionally, put the LDC1101 into sleep or shutdown mode if no more measurements are needed
write_register(0x0B, 0x02)  # Sleep mode (0x02)
time.sleep(0.01)

# Close SPI connection
spi.close()
