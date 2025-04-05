import spidev
import time

# Define SPI parameters
SPI_BUS = 0
SPI_DEVICE = 0
SPI_SPEED = 10000
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
    return response[1]

# Read the Inductance Data Registers (L_DATA_LSB and L_DATA_MSB)
l_data_lsb = read_register(0x23)  # L_DATA_LSB (low byte)
l_data_msb = read_register(0x24)  # L_DATA_MSB (high byte)

# Combine the two bytes to get the full inductance value
inductance_value = (l_data_msb << 8) | l_data_lsb

print(f"Inductance Value: {inductance_value}")

# Close SPI connection
spi.close()
