import spidev
import time
import RPi.GPIO as GPIO

# SPI settings
SPI_BUS = 0
SPI_DEVICE = 0
# Consider increasing SPI speed if needed, up to 8 MHz according to datasheet
SPI_SPEED = 1000000  # 1 MHz clock speed (Increased from 50kHz)
SPI_MODE = 0b00      # SPI mode (CPOL = 0, CPHA = 0) - Matches LDC1101 requirement

# LDC1101 register addresses
RP_SET_REG = 0x01
TC1_REG = 0x02
TC2_REG = 0x03
DIG_CONFIG_REG = 0x04
ALT_CONFIG_REG = 0x05
RP_THRESH_H_LSB_REG = 0x06
RP_THRESH_H_MSB_REG = 0x07
RP_THRESH_L_LSB_REG = 0x08
RP_THRESH_L_MSB_REG = 0x09
INTB_MODE_REG = 0x0A
START_CONFIG_REG = 0x0B
D_CONF_REG = 0x0C
L_THRESH_HI_LSB_REG = 0x16
L_THRESH_HI_MSB_REG = 0x17
L_THRESH_LO_LSB_REG = 0x18
L_THRESH_LO_MSB_REG = 0x19
STATUS_REG = 0x20
RP_DATA_LSB_REG = 0x21
RP_DATA_MSB_REG = 0x22
L_DATA_LSB_REG = 0x23
L_DATA_MSB_REG = 0x24
LHR_RCOUNT_LSB_REG = 0x30
LHR_RCOUNT_MSB_REG = 0x31
LHR_OFFSET_LSB_REG = 0x32
LHR_OFFSET_MSB_REG = 0x33
LHR_CONFIG_REG = 0x34
LHR_DATA_LSB_REG = 0x38
LHR_DATA_MID_REG = 0x39
LHR_DATA_MSB_REG = 0x3A
LHR_STATUS_REG = 0x3B
RID_REG = 0x3E
CHIP_ID_REG = 0x3F

# Initialize SPI
spi = spidev.SpiDev()
spi.open(SPI_BUS, SPI_DEVICE)
spi.max_speed_hz = SPI_SPEED
spi.mode = SPI_MODE

# Define the GPIO pins for the LDC1101
CS_PIN = 8   # Chip Select pin (BCM numbering)
# SCK, MISO, MOSI are handled by spidev, no need to define/setup manually

# Initialize the GPIO library
GPIO.setwarnings(False) # Disable warnings if channels are already in use
GPIO.setmode(GPIO.BCM)  # Use Broadcom pin numbering

# Setup the GPIO pins for SPI Chip Select
GPIO.setup(CS_PIN, GPIO.OUT)
GPIO.output(CS_PIN, GPIO.HIGH) # Deselect device initially

# Device status indicators
DEVICE_ERROR = 0x01
DEVICE_OK = 0x00

# Configure Power State (FUNC_MODE in START_CONFIG_REG 0x0B)
ACTIVE_CONVERSION_MODE = 0x00
SLEEP_MODE = 0x01
SHUTDOWN_MODE = 0x02 # Requires SHUTDOWN_EN=1 in ALT_CONFIG (0x05)

# --- Sensor Specific Configuration ---
# !! IMPORTANT !!
# These values are ESTIMATED for the MIKROE-3240 LDC1101 Click Board's
# onboard ~1-inch PCB coil. Precise measurements are always preferred.
# Assumptions: f_sensor ~4MHz, C_sensor ~220pF, Rp range ~3k-24k Ohm.
# You may need to FINE-TUNE these based on observed performance.

SENSOR_RP_SET_VAL = 0x25    # Estimated: RPMAX=24k (b010), RPMIN=3k (b101)
SENSOR_TC1_VAL = 0x9B       # Estimated: C1=3pF (b10), R1=64.9k (b1'1011) for f_min ~4MHz
SENSOR_TC2_VAL = 0xFB       # Estimated: C2=24pF (b11), R2=56.0k (b11'1011) for RPMIN=3k, C=220pF
SENSOR_DIG_CONF_VAL = 0xE4  # Estimated: MIN_FREQ=4MHz (b1110), RESP_TIME=768 (b100)

# --- SPI Communication Functions ---

# Reduced sleep time - 100ms is very long for SPI.
# Datasheet specifies t_IAG (CSB high time between accesses) as 100ns minimum.
# A small delay might still be useful for stability on some platforms.
SPI_DELAY = 0.0001 # 100 microseconds - Adjust if needed, can potentially be much lower

def write_register(reg_addr, value):
    """Writes a byte value to the specified register."""
    GPIO.output(CS_PIN, GPIO.LOW) # Select device
    time.sleep(SPI_DELAY)
    spi.xfer2([reg_addr & 0x7F, value]) # Send write command (MSB=0) + address + data
    time.sleep(SPI_DELAY)
    GPIO.output(CS_PIN, GPIO.HIGH) # Deselect device
    time.sleep(SPI_DELAY) # Ensure CS high time (t_IAG)

def read_register(reg_addr):
    """Reads a byte value from the specified register."""
    GPIO.output(CS_PIN, GPIO.LOW) # Select device
    time.sleep(SPI_DELAY)
    # Send read command (MSB=1) + address, then clock in data
    result = spi.xfer2([reg_addr | 0x80, 0x00])
    time.sleep(SPI_DELAY)
    GPIO.output(CS_PIN, GPIO.HIGH) # Deselect device
    time.sleep(SPI_DELAY) # Ensure CS high time (t_IAG)
    return result[1] # Return received data byte

# --- LDC1101 Configuration Functions ---

def initialize_ldc1101():
    """Initializes the LDC1101 with basic settings and puts it to sleep."""
    print("Initializing LDC1101...")
    try:
        chip_id = read_register(CHIP_ID_REG)
        print(f"Read Chip ID: 0x{chip_id:02X}")
        if chip_id != 0xD4:
            print("Error: Invalid Chip ID. Expected 0xD4.")
            return DEVICE_ERROR

        # --- Configure Sensor Specific Registers ---
        # Use the calculated/estimated values defined above
        print("Applying sensor-specific configuration (Estimated for MIKROE-3240)...")
        print(f"  Writing RP_SET (0x01) = 0x{SENSOR_RP_SET_VAL:02X}")
        write_register(RP_SET_REG, SENSOR_RP_SET_VAL)
        print(f"  Writing TC1 (0x02) = 0x{SENSOR_TC1_VAL:02X}")
        write_register(TC1_REG, SENSOR_TC1_VAL)
        print(f"  Writing TC2 (0x03) = 0x{SENSOR_TC2_VAL:02X}")
        write_register(TC2_REG, SENSOR_TC2_VAL)
        print(f"  Writing DIG_CONFIG (0x04) = 0x{SENSOR_DIG_CONF_VAL:02X}")
        write_register(DIG_CONFIG_REG, SENSOR_DIG_CONF_VAL)

        # --- Configure Other Registers (Defaults or specific needs) ---
        print("Applying general configuration...")
        write_register(ALT_CONFIG_REG, 0x00) # Standard operation (L-optimal off, Shutdown disabled)
        write_register(D_CONF_REG, 0x00)     # Require amplitude regulation for RP
        write_register(INTB_MODE_REG, 0x00)  # INTB disabled

        # Thresholds (set to 0 initially)
        write_register(RP_THRESH_H_LSB_REG, 0x00)
        write_register(RP_THRESH_H_MSB_REG, 0x00)
        write_register(RP_THRESH_L_LSB_REG, 0x00)
        write_register(RP_THRESH_L_MSB_REG, 0x00)
        write_register(L_THRESH_HI_LSB_REG, 0x00)
        write_register(L_THRESH_HI_MSB_REG, 0x00)
        write_register(L_THRESH_LO_LSB_REG, 0x00)
        write_register(L_THRESH_LO_MSB_REG, 0x00)

        # LHR Registers (set to 0 initially, not used in RP mode focus)
        write_register(LHR_RCOUNT_LSB_REG, 0x00)
        write_register(LHR_RCOUNT_MSB_REG, 0x00)
        write_register(LHR_OFFSET_LSB_REG, 0x00)
        write_register(LHR_OFFSET_MSB_REG, 0x00)
        write_register(LHR_CONFIG_REG, 0x00)

        # --- Set Initial Power Mode ---
        # Start in SLEEP mode after configuration
        print("Setting initial power mode to SLEEP...")
        write_register(START_CONFIG_REG, SLEEP_MODE)
        time.sleep(0.01) # Short delay after configuration

        print("LDC1101 Initialized Successfully.")
        return DEVICE_OK

    except Exception as e:
        print(f"Error during initialization: {e}")
        return DEVICE_ERROR

def set_power_mode(mode):
    """Sets the power/functional mode (Active, Sleep, Shutdown)."""
    # For Shutdown, ensure ALT_CONFIG SHUTDOWN_EN is 1 and CLKIN is stopped
    if mode == SHUTDOWN_MODE:
         alt_conf = read_register(ALT_CONFIG_REG)
         if not (alt_conf & 0x02): # Check if SHUTDOWN_EN (bit 1) is set
             print("Warning: SHUTDOWN_EN is not set in ALT_CONFIG. Enabling it.")
             write_register(ALT_CONFIG_REG, alt_conf | 0x02)
         print("Entering Shutdown. Ensure CLKIN is stopped externally.")
    elif mode == ACTIVE_CONVERSION_MODE:
        print("Entering Active Conversion Mode.")
    elif mode == SLEEP_MODE:
        print("Entering Sleep Mode.")

    write_register(START_CONFIG_REG, mode)
    time.sleep(0.01) # Allow mode transition

def enable_rpmode():
    """Configures registers specifically for starting RP+L mode measurements."""
    # Most RP configuration (RP_SET, TC1, TC2, DIG_CONF) is done in initialize_ldc1101
    # Ensure ALT_CONFIG and D_CONF are correct for RP+L mode
    print("Ensuring configuration for RP+L mode...")
    write_register(ALT_CONFIG_REG, 0x00) # LOPTIMAL=0 (bit 0), SHUTDOWN_EN=0 (bit 1)
    write_register(D_CONF_REG, 0x00)     # DOK_REPORT=0 (bit 0) -> requires amplitude lock
    # Set device to active mode to start conversions
    set_power_mode(ACTIVE_CONVERSION_MODE)

# --- Data Reading Functions ---

def get_status():
    """Reads the STATUS register (0x20)."""
    return read_register(STATUS_REG)

def get_rp_data():
    """Reads the 16-bit RP data. Reads LSB first as required by datasheet."""
    # IMPORTANT: Read LSB (0x21) before MSB (0x22) to latch data correctly!
    lsb = read_register(RP_DATA_LSB_REG)
    msb = read_register(RP_DATA_MSB_REG)
    value = (msb << 8) | lsb
    return value

def get_l_data():
    """Reads the 16-bit L data. Reads LSB first as required by datasheet."""
    # IMPORTANT: Read LSB (0x23) before MSB (0x24) (after reading RP LSB 0x21)
    lsb = read_register(L_DATA_LSB_REG)
    msb = read_register(L_DATA_MSB_REG)
    value = (msb << 8) | lsb
    return value

def get_lhr_data():
    """Reads the 24-bit LHR data. Reads LSB->MID->MSB as required."""
    # IMPORTANT: Read LSB (0x38) -> MID (0x39) -> MSB (0x3A)
    lsb = read_register(LHR_DATA_LSB_REG)
    mid = read_register(LHR_DATA_MID_REG)
    msb = read_register(LHR_DATA_MSB_REG)
    value = (msb << 16) | (mid << 8) | lsb
    return value

# --- Utility Functions ---

def display_all_registers():
    """Reads and prints the values of all configuration and status registers."""
    print("\n--- LDC1101 Register Dump ---")
    register_addresses = [
        0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A,
        0x0B, 0x0C, 0x16, 0x17, 0x18, 0x19, 0x20, # Status added
        # 0x21-0x24 are data, skip here
        0x30, 0x31, 0x32, 0x33, 0x34,
        # 0x38-0x3A are data, skip here
        0x3B, # LHR Status added
        0x3E, 0x3F
    ]
    config_regs = {}
    for addr in register_addresses:
        try:
            # Ensure device is not in shutdown before reading non-essential regs
            power_state = read_register(START_CONFIG_REG) & 0x03
            if power_state == SHUTDOWN_MODE and addr not in [CHIP_ID_REG, RID_REG, START_CONFIG_REG]:
                 config_regs[f"0x{addr:02X}"] = "Shutdown"
                 continue

            value = read_register(addr)
            config_regs[f"0x{addr:02X}"] = f"0x{value:02X}"
            # print(f"Register 0x{addr:02X}: 0x{value:02X}")
        except Exception as e:
            print(f"Error reading register 0x{addr:02X}: {e}")
            config_regs[f"0x{addr:02X}"] = "Error"

    # Print formatted output
    print("Configuration Registers:")
    print(f"  RP_SET(0x01): {config_regs.get('0x01', 'N/A')}  TC1(0x02): {config_regs.get('0x02', 'N/A')}  TC2(0x03): {config_regs.get('0x03', 'N/A')}")
    print(f"  DIG_CONF(0x04): {config_regs.get('0x04', 'N/A')}  ALT_CONF(0x05): {config_regs.get('0x05', 'N/A')}  D_CONF(0x0C): {config_regs.get('0x0C', 'N/A')}")
    print(f"  INTB_MODE(0x0A): {config_regs.get('0x0A', 'N/A')} START_CONF(0x0B): {config_regs.get('0x0B', 'N/A')}")
    print("RP Thresholds:")
    print(f"  RP_HI(0x07-06): {config_regs.get('0x07', 'N/A')}{config_regs.get('0x06', 'N/A')}  RP_LO(0x09-08): {config_regs.get('0x09', 'N/A')}{config_regs.get('0x08', 'N/A')}")
    print("L Thresholds:")
    print(f"  L_HI(0x17-16): {config_regs.get('0x17', 'N/A')}{config_regs.get('0x16', 'N/A')}  L_LO(0x19-18): {config_regs.get('0x19', 'N/A')}{config_regs.get('0x18', 'N/A')}")
    print("LHR Config:")
    print(f"  LHR_RCOUNT(0x31-30): {config_regs.get('0x31', 'N/A')}{config_regs.get('0x30', 'N/A')}")
    print(f"  LHR_OFFSET(0x33-32): {config_regs.get('0x33', 'N/A')}{config_regs.get('0x32', 'N/A')}")
    print(f"  LHR_CONFIG(0x34): {config_regs.get('0x34', 'N/A')}")
    print("Status & ID:")
    print(f"  STATUS(0x20): {config_regs.get('0x20', 'N/A')}  LHR_STATUS(0x3B): {config_regs.get('0x3B', 'N/A')}")
    print(f"  RID(0x3E): {config_regs.get('0x3E', 'N/A')}  CHIP_ID(0x3F): {config_regs.get('0x3F', 'N/A')}")
    print("-----------------------------")


def main():
    """Main function to initialize and read RP data."""
    if initialize_ldc1101() != DEVICE_OK:
        print("Failed to initialize LDC1101. Exiting.")
        return

    # Display initial configuration after initialization (device is in Sleep)
    display_all_registers()

    print("Entering RP+L mode...")
    enable_rpmode() # Enable RP+L mode and start conversions
    time.sleep(0.1) # Allow time for first conversion cycle to start/complete

    print("Starting RP data acquisition loop...")
    read_count = 0
    while True:
        try:
            # 1. Check Status Register
            status = get_status()
            # Check for Data Ready (Bit 6 DRDYB == 0 means ready)
            data_ready = not (status & 0b01000000)
            # Check for Sensor Oscillation Error (Bit 7 NO_SENSOR_OSC == 1 means error)
            osc_error = bool(status & 0b10000000)

            if osc_error:
                print("!!! WARNING: Sensor Oscillation Error detected (STATUS bit 7 = 1) !!!")
                # Consider re-initializing or specific error handling
                # Maybe display registers to see if config changed
                display_all_registers()


            if data_ready:
                # 2. Read RP Data (Correct Order)
                rp_val = get_rp_data()
                read_count += 1
                print(f"Read #{read_count}: RP Data = {rp_val} (Status: 0x{status:02X})")
                # Optional: Read L data as well if needed
                # l_val = get_l_data()
                # print(f"L Data: {l_val}")
            else:
                # Optional: Indicate waiting if needed, or just sleep
                # print(f"Waiting for data... (Status: 0x{status:02X})")
                pass

            # Adjust sleep time based on expected conversion rate (RESP_TIME)
            # Conversion Time = Response Time / (3 * f_SENSOR)
            # Example: RESP_TIME=768, f_SENSOR=4MHz -> ~64us. Sleep(0.05) is safe.
            time.sleep(0.05) # Read data at ~20Hz rate

        except Exception as e:
            print(f"Error during data acquisition: {e}")
            # Consider adding retry logic or breaking the loop
            break

# Run main
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        # Cleanup
        print("Placing device in Sleep mode...")
        try: # Try to put device to sleep before cleanup
            set_power_mode(SLEEP_MODE)
        except Exception as e:
            print(f"Could not set sleep mode: {e}")

        print("Closing SPI and cleaning up GPIO...")
        if 'spi' in globals() and spi:
            try:
                spi.close()
            except Exception as e:
                 print(f"Error closing SPI: {e}")
        try:
            GPIO.cleanup()
        except Exception as e:
            print(f"Error cleaning up GPIO: {e}")
        print("Cleanup complete.")
