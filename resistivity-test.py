import spidev
import time

# SPI setup
SPI_BUS = 0
SPI_DEVICE = 0
SPI_SPEED = 10000  # 10 kHz
SPI_MODE = 0
SPI_BITS = 8

# Initialize SPI
spi = spidev.SpiDev()
spi.open(SPI_BUS, SPI_DEVICE)
spi.max_speed_hz = SPI_SPEED
spi.mode = SPI_MODE
spi.bits_per_word = SPI_BITS

# Function to read a register
def read_register(register):
    response = spi.xfer2([register | 0x80, 0x00])  # MSB=1 for read
    print(f"Read from 0x{register:02X}: 0x{response[1]:02X}")
    return response[1]

# Function to write to a register
def write_register(register, value):
    response = spi.xfer2([register & 0x7F, value])  # MSB=0 for write
    print(f"Writing to 0x{register:02X}: 0x{value:02X}, Response: 0x{response[1]:02X}")

# Delay for power-up
time.sleep(0.001)

# Step 1: Read DEVICE_ID (0x3F) to ensure the LDC1101 is working properly
device_id = read_register(0x3F)
print(f"DEVICE_ID (0x3F) Value: 0x{device_id:02X}")

# Step 2: Set START_CONFIG (0x0B) to Active Mode (0x00)
write_register(0x0B, 0x00)  # Set to Active Conversion Mode (b00)
time.sleep(0.01)  # Wait for the mode to take effect

# Step 3: Set DIG_CONFIG (0x04) for LHR measurements
# RPMAX = 0x75, RPMIN = 0xB1 (1.5 kΩ), MIN_FREQ = 4 MHz, RESP_TIME = 0x07 (don’t care)
write_register(0x04, 0xE7)  # Set the DIG_CONFIG register for LHR (MIN_FREQ = 4.0 MHz and RESP_TIME)
time.sleep(0.01)  # Give time for the change to take effect

# Step 4: Set RP_SET (0x01) with RPMAX = 0x75 (b011) and RPMIN = 0xB1 (1.5 kΩ)
write_register(0x01, 0x75)  # RPMAX setting (RP_SET)
time.sleep(0.01)

# Step 5: Set the LHR_RCOUNT register for sample rate
write_register(0x30, 0x4A)  # LHR_RCOUNT_LSB
write_register(0x31, 0x01)  # LHR_RCOUNT_MSB
time.sleep(0.01)

# Step 6: Verify the configuration by reading back important registers
start_config_value = read_register(0x0B)
dig_config_value = read_register(0x04)
rp_set_value = read_register(0x01)
lhr_rcount_lsb_value = read_register(0x30)
lhr_rcount_msb_value = read_register(0x31)

# Print the results to verify
print(f"START_CONFIG (0x0B) Value: 0x{start_config_value:02X}")
print(f"DIG_CONFIG (0x04) Value: 0x{dig_config_value:02X}")
print(f"RP_SET (0x01) Value: 0x{rp_set_value:02X}")
print(f"LHR_RCOUNT_LSB (0x30) Value: 0x{lhr_rcount_lsb_value:02X}")
print(f"LHR_RCOUNT_MSB (0x31) Value: 0x{lhr_rcount_msb_value:02X}")

# Step 7: Poll LHR_STATUS.DRDYB (0x3B: bit0) for conversion result
# This could be done by checking the LHR_STATUS register for the conversion completion
lhr_status = read_register(0x3B)
print(f"LHR_STATUS (0x3B) Value: 0x{lhr_status:02X}")

# Step 8: After conversion, read the results from 0x38, 0x39, and 0x3A
lhr_result_lsb = read_register(0x38)
lhr_result_msb = read_register(0x39)
lhr_result_full = read_register(0x3A)

print(f"LHR Results: LSB: 0x{lhr_result_lsb:02X}, MSB: 0x{lhr_result_msb:02X}, Full: 0x{lhr_result_full:02X}")

# Close SPI connection
spi.close()
