import spidev
import time

# Initialize SPI
spi = spidev.SpiDev()
spi.open(0, 0)  # SPI bus 0, device 0 (CE0)
spi.max_speed_hz = 50000  # 50 kHz is slow but enough for LDC1101
spi.mode = 0  # Mode 0 (CPOL=0, CPHA=0)

# Function to read RP+L data from LDC1101
def read_ldc1101_RP():
    # Send a 16-bit command to the LDC1101 (for example, single-ended reading)
    command = [0x80]  # Example command for reading RP+L data (16-bit)
    response = spi.xfer2(command)
    
    # Return the received data (this should be the result from LDC1101)
    return response

# Test reading from LDC1101 (RP+L mode)
while True:
    print("Reading RP+L data from LDC1101...")
    result = read_ldc1101_RP()  # Read RP+L data
    print("LDC1101 Response (RP Data):", result)
    time.sleep(1)

spi.close()
