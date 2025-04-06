import spidev
import time

# LDC1011 expected device ID
EXPECTED_DEVICE_ID = 0xD4

# Register address for Device ID (0x0F for LDC1011)
DEVICE_ID_REGISTER = 0x3F

# SPI settings
spi_bus = 0
spi_device = 0

# SPI speeds to test (in Hz)
spi_speeds = [
    100000,     # 100 kHz
    200000,
    500000,
    1000000,    # 1 MHz
    2000000,
    4000000,
    8000000,    # 8 MHz
    10000000,
    12000000,   # LDC1011 max is 12 MHz
    15000000
]

def read_register(spi, reg_addr):
    # Read operation: bit 7 = 1
    reg_read = 0x80 | reg_addr
    response = spi.xfer2([reg_read, 0x00])
    return response[1]

def test_spi_speed(speed):
    spi = spidev.SpiDev()
    spi.open(spi_bus, spi_device)
    spi.max_speed_hz = speed
    spi.mode = 1  # SPI mode 1 for LDC1011

    try:
        device_id = read_register(spi, DEVICE_ID_REGISTER)
        print(f"Speed {speed} Hz -> Device ID: 0x{device_id:02X}")
        return device_id == EXPECTED_DEVICE_ID
    except Exception as e:
        print(f"Speed {speed} Hz -> Error: {e}")
        return False
    finally:
        spi.close()

# Sweep through speeds
print("Starting SPI speed sweep for LDC1011...\n")
for speed in spi_speeds:
    is_success = test_spi_speed(speed)
    if is_success:
        print(f"Supported speed: {speed} Hz\n")
    else:
        print(f"Unsupported speed: {speed} Hz\n")

print("SPI sweep complete.")
