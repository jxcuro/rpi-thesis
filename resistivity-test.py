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

def write_register(register, value):
    spi.xfer2([register & 0x7F, value])  # MSB = 0 for write

def read_register(register):
    result = spi.xfer2([0x80 | register, 0x00])  # MSB = 1 for read
    return result[1]

def init_ldc1101():
    """Initialize LDC1101 sensor in proper mode."""
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

    # Clock Dividers (optional, you can adjust if needed)
    write_register(REG_CLOCK_DIVIDERS, 0x00)

    # Set RP+L mode (default)
    write_register(REG_MUX_CONFIG, 0x00)

    # Drive current (set to max for good measurements)
    write_register(REG_DRIVE_CURRENT, 0xF4)

    # CONFIG register (ensure default settings)
    write_register(REG_CONFIG, 0x00)

    # Set to ACTIVE mode
    write_register(REG_MODE_CONFIG, 0x02)
    time.sleep(0.1)
    print("LDC1101 Initialized in ACTIVE mode.")

def read_rp():
    """Read the RP (Resistive Position) data from LDC1101."""
    msb = read_register(REG_RP_MSB)
    lsb = read_register(REG_RP_LSB)
    rp = (msb << 8) | lsb
    return rp

def read_status():
    """Check the STATUS register to look for errors or flags."""
    status = read_register(REG_STATUS)
    return status

def debug_sensor():
    """Debug the sensor by reading its status and registers."""
    print("Reading the STATUS register (0x0B)...")
    status = read_status()
    print(f"STATUS Register (0x0B): {status}")

    print("Reading other registers...")
    # Read and display important registers for debugging
    mode = read_register(REG_MODE_CONFIG)
    mux_config = read_register(REG_MUX_CONFIG)
    print(f"MODE_CONFIG (0x01): {mode}")
    print(f"MUX_CONFIG (0x1B): {mux_config}")

    # Check RP data (MSB + LSB)
    rp = read_rp()
    print(f"RP Measurement: {rp}")

# Main Execution
init_ldc1101()
print("LDC1101 Initialized. Starting measurement...")

# Run continuous reading and debug every few seconds
while True:
    debug_sensor()
    rp = read_rp()
    print(f"RP Measurement: {rp}")

    # Check if RP value is stuck at 31744, indicating no change in the environment
    if rp == 31744:
        print("RP value is stuck. Possible sensor or environmental issue.")
        print("Check if an inductive object is near the sensor.")
    
    # Read and log every 2 seconds
    time.sleep(2)
