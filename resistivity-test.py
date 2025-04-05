import spidev
import time

# Define SPI parameters
SPI_BUS = 0
SPI_DEVICE = 0
SPI_SPEED = 10000  # Safe lower speed
SPI_MODE = 0
SPI_BITS = 8

# Initialize SPI
spi = spidev.SpiDev()
spi.open(SPI_BUS, SPI_DEVICE)
spi.max_speed_hz = SPI_SPEED
spi.mode = SPI_MODE
spi.bits_per_word = SPI_BITS

# Function to read from LDC1101 register
def read_register(register):
    response = spi.xfer2([register | 0x80, 0x00])
    print(f"Read 0x{register:02X}: 0x{response[1]:02X}")
    return response[1]

# Function to write to LDC1101 register
def write_register(register, value):
    response = spi.xfer2([register & 0x7F, value])
    print(f"Wrote 0x{value:02X} to 0x{register:02X}, Response: 0x{response[1]:02X}")

# Step 1: Delay after power-up
time.sleep(0.001)

# Step 2: Configure registers for LHR-only measurement (based on datasheet example)
write_register(0x0B, 0x01)  # START_CONFIG: Active mode (not 0x00 — thanks for the correction!)
time.sleep(0.01)

write_register(0x01, 0x75)  # RP_SET: RPMAX/RPMIN, HIGH_Q_SENSOR disabled
time.sleep(0.01)

write_register(0x04, 0xE7)  # DIG_CONFIG: MIN_FREQ=4.0MHz, RPMIN=1.5kΩ
time.sleep(0.01)

write_register(0x30, 0x4A)  # LHR_RCOUNT_LSB (330 = 0x014A)
write_register(0x31, 0x01)  # LHR_RCOUNT_MSB
time.sleep(0.01)

# Step 3: Confirm configuration
read_register(0x0B)  # START_CONFIG
read_register(0x01)  # RP_SET
read_register(0x04)  # DIG_CONFIG
read_register(0x30)  # LHR_RCOUNT_LSB
read_register(0x31)  # LHR_RCOUNT_MSB

# Close SPI
spi.close()
