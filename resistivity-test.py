import spidev
import time

# SPI setup
spi = spidev.SpiDev()
spi.open(0, 0)  # Using bus 0, chip select 0
spi.max_speed_hz = 500000  # Try reducing to 100000 if issues persist
spi.mode = 0b00  # Verify this mode with your device datasheet

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
REG_STATUS           = 0x19  # Status register (check datasheet for details)

# SPI read/write functions
def write_register(reg, value):
    # Write operation: MSB must be 0.
    spi.xfer2([reg & 0x7F, value])
    
def read_register(reg):
    # Read operation: set MSB to 1; dummy byte clocks in the response.
    return spi.xfer2([reg | 0x80, 0x00])[1]

# LDC1101 initialization function
def init_ldc1101():
    # Reset sequence: set to Sleep mode, then Standby.
    write_register(REG_START_CONFIG, 0x03)  # Sleep mode
    time.sleep(0.05)
    write_register(REG_START_CONFIG, 0x01)  # Standby mode
    time.sleep(0.05)
    
    # Configure RCOUNT (example: 0x0858)
    write_register(REG_RCOUNT_MSB, 0x08)
    write_register(REG_RCOUNT_LSB, 0x58)
    
    # Configure SETTLECOUNT (stabilization time)
    write_register(REG_SETTLECOUNT_MSB, 0x00)
    write_register(REG_SETTLECOUNT_LSB, 0x0A)
    
    # Set CLOCK_DIVIDERS (internal clock)
    write_register(REG_CLOCK_DIVIDERS, 0x00)
    
    # Set MUX_CONFIG to RP+L mode
    write_register(REG_MUX_CONFIG, 0x00)
    
    # Set DRIVE_CURRENT to maximum (helps start oscillation)
    write_register(REG_DRIVE_CURRENT, 0xFF)
    
    # Set CONFIG to default (0x00)
    write_register(REG_CONFIG, 0x00)
    
    # Activate conversion mode
    write_register(REG_START_CONFIG, 0x00)  # Active Conversion Mode
    time.sleep(0.1)  # Increase delay if needed for proper startup

# Function to read and print all registers for debugging
def debug_read_all_registers():
    print("Debug: Reading registers 0x01 to 0x21:")
    for reg in range(0x01, 0x22):
        val = read_register(reg)
        print("Reg 0x{:02X}: 0x{:02X}".format(reg, val))
        time.sleep(0.05)

# Main execution
init_ldc1101()
print("LDC1101 initialization complete.")

# Print raw status register
status = read_register(REG_STATUS)
print("Raw Status register value: 0x{:02X}".format(status))

# Debug: Read and print all registers
debug_read_all_registers()

# Optionally, if you expect conversion data, try reading RP values
def read_rp():
    msb = read_register(REG_RP_MSB)
    lsb = read_register(REG_RP_LSB)
    return (msb << 8) | lsb

rp = read_rp()
print("RP Measurement: {}".format(rp))
