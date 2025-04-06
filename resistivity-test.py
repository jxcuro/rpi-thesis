import spidev
import time

# SPI Setup
spi = spidev.SpiDev()
spi.open(0, 0)  # Open SPI bus 0, device 0 (CS0)
spi.max_speed_hz = 50000  # SPI speed (adjust as necessary)
spi.mode = 0b00  # SPI mode 0 (CPOL=0, CPHA=0)

# Register Addresses
LHR_DATA_REG = [0x38, 0x39, 0x3A]  # LHR data registers (0x38, 0x39, 0x3A)
LHR_OFFSET_REG = [0x32, 0x33]  # LHR offset registers (0x32, 0x33)
LHR_CONFIG_REG = 0x34  # LHR configuration register (Sensor divider)
RP_SET_REG = 0x01  # RPMIN register
RCOUNT_REG = [0x30, 0x31]  # RCOUNT register for LHR conversion time

# Constants (for the example)
CLKIN_FREQ = 16_000_000  # 16 MHz CLKIN (change if using a different frequency)
SENSOR_DIV = 0x01  # SENSOR_DIV value (0x01 = divide by 2)

# Helper function to read a single byte from a register
def read_register(reg):
    return spi.xfer2([reg | 0x80, 0x00])[1]  # OR with 0x80 for read operation

# Helper function to write a byte to a register
def write_register(reg, value):
    spi.xfer2([reg & 0x7F, value])  # Mask reg with 0x7F for write operation

# Read LHR data from registers 0x38, 0x39, 0x3A
def read_lhr_data():
    lhr_data = []
    for reg in LHR_DATA_REG:
        lhr_data.append(read_register(reg))
    # Combine the three bytes into a single 24-bit value
    lhr_value = (lhr_data[2] << 16) | (lhr_data[1] << 8) | lhr_data[0]
    return lhr_value

# Calculate sensor frequency from LHR data
def calculate_sensor_frequency(lhr_data):
    # Read LHR offset (0x32, 0x33)
    lhr_offset = (read_register(LHR_OFFSET_REG[1]) << 8) | read_register(LHR_OFFSET_REG[0])
    
    # Sensor Divider (read from LHR_CONFIG)
    sensor_div = read_register(LHR_CONFIG_REG) & 0x03  # Bits [1:0] for SENSOR_DIV
    
    # Calculate sensor frequency using the formula
    sensor_frequency = (lhr_data / lhr_offset) * (sensor_div / CLKIN_FREQ)
    return sensor_frequency

# Calculate inductance from sensor frequency
def calculate_inductance(sensor_frequency, sensor_capacitance):
    inductance = 1 / (sensor_capacitance * sensor_frequency**2)
    return inductance

# Set RPMIN for LHR measurements
def set_rpmin(rp_value):
    # Write RPMIN value to RP_SET register
    write_register(RP_SET_REG, rp_value)

# Set RCOUNT for conversion time and resolution
def set_rcount(rcount_value):
    # Write RCOUNT value (2 bytes) to RCOUNT registers
    write_register(RCOUNT_REG[0], rcount_value & 0xFF)
    write_register(RCOUNT_REG[1], (rcount_value >> 8) & 0xFF)

# Main code to interact with the LDC1101 and perform calculations
def main():
    # Configure LDC1101 settings
    # Set RPMIN to appropriate value for your sensor (example: 3 kΩ)
    set_rpmin(0b101)  # RPMIN = 3 kΩ
    
    # Set RCOUNT for desired resolution (example: full resolution)
    set_rcount(0xFFFF)  # Maximum resolution
    
    # Read LHR data and calculate sensor frequency
    lhr_data = read_lhr_data()
    if lhr_data == 0x000000:
        print("LHR conversion error or data is not valid.")
        return
    
    print(f"LHR Data: {hex(lhr_data)}")
    
    # Calculate sensor frequency (example using LHR data)
    sensor_frequency = calculate_sensor_frequency(lhr_data)
    print(f"Sensor Frequency: {sensor_frequency} Hz")
    
    # Calculate inductance (example: assuming sensor capacitance = 1 nF)
    sensor_capacitance = 1e-9  # 1 nF
    inductance = calculate_inductance(sensor_frequency, sensor_capacitance)
    print(f"Inductance: {inductance} H")
    
    # Wait for the next cycle or termination
    time.sleep(1)

if __name__ == "__main__":
    while True:
        main()
        time.sleep(1)  # Wait for the next cycle
