import spidev
import time

# SPI settings
SPI_BUS = 0
SPI_DEVICE = 0
SPI_SPEED_HZ = 10000  # 10 kHz

# Create SPI instance
spi = spidev.SpiDev()
spi.open(SPI_BUS, SPI_DEVICE)
spi.max_speed_hz = SPI_SPEED_HZ
spi.mode = 1  # CPOL=0, CPHA=1 for LDC1101

def write_register(addr, value):
    spi.xfer2([addr & 0x7F, value])  # MSB=0 for write

def read_register(addr):
    response = spi.xfer2([addr | 0x80, 0x00])  # MSB=1 for read
    return response[1]

def read_lhr_data():
    data = spi.xfer2([0x38 | 0x80, 0x00, 0x00, 0x00])  # Read 3 bytes from 0x38, 0x39, 0x3A
    raw = (data[1] << 16) | (data[2] << 8) | data[3]
    return raw

def is_data_ready():
    status = read_register(0x3B)
    return (status & 0x01) == 0  # DRDYB == 0 means data ready

def init_ldc1101_lhr_mode():
    print("Configuring LDC1101 for LHR (inductance) mode...")

    write_register(0x01, 0x75)  # RP_SET: set RPMAX
    write_register(0x04, 0xE7)  # DIG_CONFIG: MIN_FREQ=4MHz, RPMIN=1.5kΩ, LHR mode

    # Set LHR RCOUNT = 330 (0x014A)
    write_register(0x30, 0x4A)  # LHR_RCOUNT_LSB
    write_register(0x31, 0x01)  # LHR_RCOUNT_MSB

    # Optional: Sleep then Active to reset state
    write_register(0x0B, 0x01)  # START_CONFIG: sleep
    time.sleep(0.1)
    write_register(0x0B, 0x00)  # START_CONFIG: active

    print("Initialization complete. Waiting for data...")

def main():
    init_ldc1101_lhr_mode()

    try:
        while True:
            if is_data_ready():
                lhr_value = read_lhr_data()
                print(f"LHR Raw Value: {lhr_value} (0x{lhr_value:06X})")
            else:
                print("Waiting for data...")
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Exiting...")
        spi.close()

if __name__ == "__main__":
    main()
