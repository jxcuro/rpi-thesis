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

# Function to read all registers for debugging
def read_all_registers():
    print("Reading all registers:")
    for reg in range(0x00, 0x40):  # Checking a range of registers
        value = read_register(reg)
        print(f"Register 0x{reg:02X}: 0x{value:02X}")

# Step 1: Delay after power-up to allow initialization (0.8 ms)
time.sleep(0.001)  # Wait for 1 ms to ensure proper initialization

# Step 2: Write to RP_SET (0x01) for sensor configuration (RPMAX setting)
write_register(0x01, 0x75)  # RPMAX value (0x75)
time.sleep(0.01)

# Step 3: Set the sensor for L-only measurements by configuring RPMIN (0x01)
write_register(0x01, 0x05)  # RPMIN (1.5 kΩ) value (0x05)
time.sleep(0.01)

# Step 4: Set MIN_FREQ (0x02) to 4.0 MHz (0x1110)
write_register(0x02, 0x0E)  # MIN_FREQ = 0x1110 (4 MHz)
time.sleep(0.01)

# Step 5: Configure DIG_CONFIG (0x04) for the RP+L conversion
write_register(0x04, 0xE7)  # DIG_CONFIG value (0xE7)
time.sleep(0.01)

# Step 6: Set RCOUNT for 3 kSPS (value 5280 or 0x014A)
write_register(0x30, 0x4A)  # LHR_RCOUNT_LSB (0x4A)
write_register(0x31, 0x01)  # LHR_RCOUNT_MSB (0x01)
time.sleep(0.01)

# Step 7: Configure START_CONFIG (0x0B) to active mode (0x00)
write_register(0x0B, 0x00)  # Active mode (0x00)
time.sleep(0.01)

# Step 8: Poll for DRDYB (Data Ready) from LHR_STATUS (0x3B) before reading results
# Poll until DRDYB bit is 1
while True:
    status = read_register(0x3B)  # Read LHR_STATUS
    if status & 0x01:  # Check if DRDYB (bit 0) is set
        print("Conversion Complete!")
        break
    time.sleep(0.01)  # Wait before checking again

# Step 9: Read LHR conversion results from registers 0x38, 0x39, 0x3A
lhr_data_lsb = read_register(0x38)  # LHR_DATA_LSB
lhr_data_mid = read_register(0x39)  # LHR_DATA_MID
lhr_data_msb = read_register(0x3A)  # LHR_DATA_MSB

# Combine the LHR data to form the full inductance measurement
lhr_data = (lhr_data_msb << 16) | (lhr_data_mid << 8) | lhr_data_lsb
print(f"LHR Inductance Measurement Data: 0x{lhr_data:06X}")

# Verify the changes by reading the relevant configuration registers
rp_set_value = read_register(0x01)
lhr_rcount_lsb_value = read_register(0x30)
lhr_rcount_msb_value = read_register(0x31)
dig_config_value = read_register(0x04)
start_config_value = read_register(0x0B)

# Print the register values for debugging
print(f"RP_SET (0x01) Value: 0x{rp_set_value:02X}")
print(f"LHR_RCOUNT_LSB (0x30) Value: 0x{lhr_rcount_lsb_value:02X}")
print(f"LHR_RCOUNT_MSB (0x31) Value: 0x{lhr_rcount_msb_value:02X}")
print(f"DIG_CONFIG (0x04) Value: 0x{dig_config_value:02X}")
print(f"START_CONFIG (0x0B) Value: 0x{start_config_value:02X}")

# Read and print all registers to debug
read_all_registers()

# Close SPI connection
spi.close()
