import spidev
import time

# Define LDC1101 Registers
START_CONFIG = 0x0B
LHR_STATUS = 0x3B
LHR_RCOUNT_LSB = 0x30
LHR_RCOUNT_MSB = 0x31
LHR_RESULT_LSB = 0x38
LHR_RESULT_MSB = 0x39
LHR_RESULT_XLSB = 0x3A

# Setup SPI
spi = spidev.SpiDev()
spi.open(0, 0)  # Open SPI bus 0, device 0
spi.max_speed_hz = 50000
spi.mode = 0b00

def write_register(register, value):
    """Write value to register."""
    spi.xfer2([register, value])

def read_register(register):
    """Read value from register."""
    result = spi.xfer2([register, 0x00])
    return result[1]

def read_inductance():
    """Read inductance from LDC1101."""
    # Set the LDC1101 to active mode (start conversion)
    write_register(START_CONFIG, 0x00)

    # Poll the DRDYB flag in LHR_STATUS to check if conversion is complete
    while True:
        status = read_register(LHR_STATUS)
        if status & 0x01:  # DRDYB is bit 0
            break
        time.sleep(0.01)  # Polling delay

    # Read the result registers (LHR_RESULT_LSB, LHR_RESULT_MSB, LHR_RESULT_XLSB)
    lsb = read_register(LHR_RESULT_LSB)
    msb = read_register(LHR_RESULT_MSB)
    xlsb = read_register(LHR_RESULT_XLSB)

    # Combine the results from LSB, MSB, and XLSB
    inductance_raw = (msb << 8) | lsb
    inductance_raw = (inductance_raw << 8) | xlsb

    # Calculate inductance (raw value might need scaling or calibration, depending on sensor)
    # For now, just return the raw value
    return inductance_raw

def main():
    """Main function to measure inductance."""
    while True:
        inductance = read_inductance()
        print(f"Inductance (raw value): {inductance}")
        time.sleep(1)

if __name__ == "__main__":
    main()
