import spidev
import time

# SPI setup
spi = spidev.SpiDev()
spi.open(0, 0)  # Bus 0, CE0
spi.max_speed_hz = 500000
spi.mode = 0b00

# LDC1101 register addresses
REG_START_CONFIG     = 0x01
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
REG_STATUS           = 0x19  # Status register (check datasheet for flag positions)

# SPI read/write functions
def write_register(reg, value):
    # Write operation: MSB is 0.
    spi.xfer2([reg & 0x7F, value])

def read_register(reg):
    # Read operation: set MSB to 1 and use a dummy byte (0x00)
    return spi.xfer2([reg | 0x80, 0x00])[1]

# LDC1101 initialization function
def init_ldc1101():
    # Reset sequence: Sleep then Standby mode.
    write_register(REG_START_CONFIG, 0x03)  # Sleep mode
    time.sleep(0.05)
    write_register(REG_START_CONFIG, 0x01)  # Standby mode
    time.sleep(0.05)

    # Set RCOUNT (measurement resolution), e.g., RCOUNT = 0x0858.
    write_register(REG_RCOUNT_MSB, 0x08)
    write_register(REG_RCOUNT_LSB, 0x58)

    # Set SETTLECOUNT (stabilization time for the LC circuit)
    write_register(REG_SETTLECOUNT_MSB, 0x00)
    write_register(REG_SETTLECOUNT_LSB, 0x0A)

    # Set CLOCK_DIVIDERS (default using internal clock)
    write_register(REG_CLOCK_DIVIDERS, 0x00)

    # Set MUX_CONFIG to RP+L mode
    write_register(REG_MUX_CONFIG, 0x00)

    # Set DRIVE_CURRENT to maximum to help start oscillation
    write_register(REG_DRIVE_CURRENT, 0xFF)

    # Set CONFIG to default (0x00)
    write_register(REG_CONFIG, 0x00)

    # Set START_CONFIG to Active Conversion Mode (FUNC_MODE = b00)
    write_register(REG_START_CONFIG, 0x00)
    time.sleep(0.1)  # Allow time for the device to settle

# Function to check if the conversion is complete using the STATUS register
def is_conversion_complete():
    status = read_register(REG_STATUS)
    print("Raw status register value:", hex(status))
    # Update: Try checking bit 7 (0x80) for conversion complete instead of bit 0.
    return (status & 0x80) != 0

# Read RP values from LDC1101
def read_rp():
    msb = read_register(REG_RP_MSB)
    lsb = read_register(REG_RP_LSB)
    return (msb << 8) | lsb

# Main execution: initialize the LDC1101 and continuously read RP values.
init_ldc1101()
print("LDC1101 initialized in Active RP+L mode. Reading RP values...")

while True:
    if is_conversion_complete():
        rp = read_rp()
        print(f"RP Measurement: {rp}")
    else:
        print("Waiting for conversion to complete...")
    
    time.sleep(0.5)
