import spidev
import time

# SPI setup
spi = spidev.SpiDev()
spi.open(0, 0)  # Bus 0, Chip Select 0
spi.max_speed_hz = 10000  # 10kHz SPI
spi.mode = 1  # SPI mode 1 (CPOL=0, CPHA=1)

def write_register(address, value):
    spi.xfer2([address & 0x7F, value])
    print(f"Wrote 0x{value:02X} to register 0x{address:02X}")

def read_register(address):
    response = spi.xfer2([address | 0x80, 0x00])
    value = response[1]
    print(f"Read  0x{value:02X} from register 0x{address:02X}")
    return value

def read_lhr_data():
    data = spi.xfer2([0x38 | 0x80, 0x00, 0x00, 0x00])  # LHR_DATA_LSB to LHR_DATA_MSB
    lhr = (data[1] << 16) | (data[2] << 8) | data[3]
    print(f"LHR Raw Value: 0x{lhr:06X}")
    return lhr

def is_data_ready():
    status = read_register(0x3B)  # LHR_STATUS
    return (status & 0x01) == 0  # DRDYB is active low

def initialize_ldc1101():
    print("Initializing LDC1101 in LHR mode...")

    # Set RP_SET for 6.33 kΩ to 5.91 kΩ range (0x75)
    write_register(0x01, 0x75)

    # DIG_CONFIG for LHR, HIGH_Q_SENSOR disabled, RPMIN=1.5kΩ, MIN_FREQ=4MHz
    write_register(0x04, 0xE7)

    # LHR_RCOUNT = 0x014A (330 decimal)
    write_register(0x30, 0x4A)  # LSB
    write_register(0x31, 0x01)  # MSB

    # Set to sleep mode first
    write_register(0x0B, 0x01)
    time.sleep(0.05)

    # Then activate
    write_register(0x0B, 0x00)
    print("Configuration complete. Waiting for LHR data...")

def main():
    initialize_ldc1101()

    try:
        while True:
            if is_data_ready():
                lhr = read_lhr_data()
                if lhr == 0:
                    print("Warning: LHR value is zero. Check sensor or wiring.")
                else:
                    print(f"Measured LHR Value: 0x{lhr:06X}")
            else:
                print("Data not ready.")
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("Exiting...")
        spi.close()

if __name__ == "__main__":
    main()
