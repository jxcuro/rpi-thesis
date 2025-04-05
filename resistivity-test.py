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
REG_INTERRUPT_STATUS = 0x04
REG_INTERRUPT_ENABLE = 0x06

def write_register(register, value):
    """Writes a value to a specific register."""
    spi.xfer2([register & 0x7F, value])  # MSB = 0 for write

def read_register(register):
    """Reads the value from a specific register."""
    result = spi.xfer2([0x80 | register, 0x00])  # MSB = 1 for read
    return result[1]

def init_ldc1101():
    """Initializes the LDC1101 sensor in RP+L mode."""
    # Put in Standby mode
    write_register(REG_MODE_CONFIG, 0x01)
    time.sleep(0.1)

    # Set RCOUNT
    write_register(REG_RCOUNT_MSB, 0x02)
    write_register(REG_RCOUNT_LSB, 0x58)

    # Set SETTLECOUNT
    write_register(REG_SETTLECOUNT_MSB, 0x00)
    write_register(REG_SETTLECOUNT_LSB, 0x0A)

    # Clock Dividers
    write_register(REG_CLOCK_DIVIDERS, 0x00)

    # Set RP+L mode
    write_register(REG_MUX_CONFIG, 0x00)

    # Drive current
    write_register(REG_DRIVE_CURRENT, 0xF4)

    # CONFIG (optional - default settings are usually fine)
    write_register(REG_CONFIG, 0x00)

    # Set to ACTIVE mode
    write_register(REG_MODE_CONFIG, 0x02)
    time.sleep(0.1)

def read_rp():
    """Reads RP data from the sensor and returns it."""
    msb = read_register(REG_RP_MSB)
    lsb = read_register(REG_RP_LSB)
    return (msb << 8) | lsb

def read_interrupt_status():
    """Reads the interrupt status register to check if an interrupt occurred."""
    return read_register(REG_INTERRUPT_STATUS)

def enable_interrupt():
    """Enables interrupt reporting for the LDC1101."""
    write_register(REG_INTERRUPT_ENABLE, 0x01)

def disable_interrupt():
    """Disables interrupt reporting for the LDC1101."""
    write_register(REG_INTERRUPT_ENABLE, 0x00)

# --- Main Execution ---
init_ldc1101()
print("LDC1101 Initialized in RP+L Mode. Reading RP values:")

# Enable interrupts to report when RP values exceed a threshold
enable_interrupt()

while True:
    rp = read_rp()
    interrupt_status = read_interrupt_status()
    
    print(f"RP Measurement: {rp}")
    
    # Check if interrupt occurred
    if interrupt_status:
        print("Interrupt Triggered! Something changed.")
        disable_interrupt()  # Disable the interrupt to prevent constant triggering
        enable_interrupt()   # Re-enable the interrupt for future monitoring
    
    time.sleep(0.5)
