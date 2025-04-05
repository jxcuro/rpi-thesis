import spidev
import time

# Initialize SPI
spi = spidev.SpiDev()
spi.open(0, 0)  # Bus 0, CE0 (Chip Select 0)
spi.max_speed_hz = 500000  # 500kHz is safe
spi.mode = 0b00

# LDC1101 Register Addresses
REG_MODE_CONFIG      = 0x01
REG_RCOUNT_LSB       = 0x08
REG_RCOUNT_MSB       = 0x09
REG_SETTLECOUNT_LSB  = 0x10
REG_SETTLECOUNT_MSB  = 0x11
REG_CLOCK_DIVIDERS   = 0x1E
REG_MUX_CONFIG       = 0x1B
REG_DRIVE_CURRENT    = 0x1C
REG_CONFIG           = 0x1A
REG_RP_MSB           = 0x20
REG_RP_LSB           = 0x21
REG_STATUS           = 0x0B  # Status Register (important for debugging)

# SPI write function (send data to the LDC1101)
def write_register(register, value):
    print(f"Writing to Register 0x{register:02X} Value 0x{value:02X}")
    spi.xfer2([register & 0x7F, value])  # MSB = 0 for write

# SPI read function (read data from LDC1101)
def read_register(register):
    result = spi.xfer2([0x80 | register, 0x00])  # MSB = 1 for read
    print(f"Reading Register 0x{register:02X}, Received: 0x{result[1]:02X}")
    return result[1]

def init_ldc1101():
    """Initialize the LDC1101 sensor in Active mode."""
    print("Initializing LDC1101...")
    
    # Put in Standby mode
    write_register(REG_MODE_CONFIG, 0x01)
    time.sleep(0.1)
    
    # Set RCOUNT (set for medium range measurement)
    write_register(REG_RCOUNT_MSB, 0x02)
    write_register(REG_RCOUNT_LSB, 0x58)

    # Set SETTLECOUNT (set for moderate settling time)
    write_register(REG_SETTLECOUNT_MSB, 0x00)
    write_register(REG_SETTLECOUNT_LSB, 0x0A)

    # Clock Dividers (optional, adjust if necessary)
    write_register(REG_CLOCK_DIVIDERS, 0x00)

    # Set RP+L mode (default)
    write_register(REG_MUX_CONFIG, 0x00)

    # Drive current (set to maximum for stable measurements)
    write_register(REG_DRIVE_CURRENT, 0xF4)

    # CONFIG register (optional)
    write_register(REG_CONFIG, 0x00)

    # Set to ACTIVE mode (crucial for taking measurements)
    write_register(REG_MODE_CONFIG, 0x02)
    time.sleep(0.1)
    
    print("LDC1101 Initialized in ACTIVE mode.")

def read_rp():
    """Read RP (Resistive Position) from LDC1101"""
    msb = read_register(REG_RP_MSB)
    lsb = read_register(REG_RP_LSB)
    rp = (msb << 8) | lsb
    return rp

def read_status():
    """Read the STATUS register to check for errors or status flags."""
    status = read_register(REG_STATUS)
    return status

# Main Execution
init_ldc1101()
print("LDC1101 Initialized. Starting RP measurement...")

# Run continuous reading and debug every few seconds
while True:
    print("\n--- Reading Sensor ---")
    # Check the STATUS register (important for debugging)
    status = read_status()
    print(f"STATUS Register (0x0B): {status}")

    # Read the RP value (MSB + LSB combined)
    rp = read_rp()
    print(f"RP Measurement: {rp}")

    # Check if RP value is stuck at 31744
    if rp == 31744:
        print("RP value is stuck. Possible sensor or environmental issue.")
        print("Ensure that an inductive object is near the sensor.")

    # Sleep for 2 seconds
    time.sleep(2)
