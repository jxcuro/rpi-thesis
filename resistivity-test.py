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

def write_register(register, value):
    """Write value to register."""
    spi.xfer2([register & 0x7F, value])  # MSB = 0 for write

def read_register(register):
    """Read value from register."""
    result = spi.xfer2([0x80 | register, 0x00])  # MSB = 1 for read
    return result[1]

def init_ldc1101():
    """Initialize the LDC1101 sensor in Active mode."""
    print("Initializing LDC1101...")
    
    # Put in Standby mode
    write_register(REG_MODE_CONFIG, 0x01)
    time.sleep(0.1)
    
    # Increase RCOUNT for better measurement sensitivity
    write_register(REG_RCOUNT_MSB, 0x05)  # Increase RCOUNT to higher value (0x05)
    write_register(REG_RCOUNT_LSB, 0xF4)  # Increase RCOUNT to higher value (0xF4)

    # Increase SETTLECOUNT for better stabilization
    write_register(REG_SETTLECOUNT_MSB, 0x00)
    write_register(REG_SETTLECOUNT_LSB, 0x20)  # Increase settling time

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
    """Read the RP value (Inductive position)."""
    msb = read_register(REG_RP_MSB)
    lsb = read_register(REG_RP_LSB)
    return (msb << 8) | lsb

def check_registers():
    """Check if registers are being updated."""
    for reg in [REG_MODE_CONFIG, REG_RCOUNT_MSB, REG_RCOUNT_LSB, REG_SETTLECOUNT_MSB, REG_SETTLECOUNT_LSB, REG_RP_MSB, REG_RP_LSB]:
        value = read_register(reg)
        print(f"Register {hex(reg)}: {hex(value)}")

# --- Main Execution ---
init_ldc1101()

# Check if registers are being updated
check_registers()

print("LDC1101 Initialized in ACTIVE mode. Reading RP values:")

while True:
    rp = read_rp()
    print(f"RP Measurement: {rp}")
    check_registers()  # Check registers again to see if they are updating
    time.sleep(0.5)
