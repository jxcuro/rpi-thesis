import spidev
import time

# SPI settings
SPI_BUS = 0
SPI_DEVICE = 0
SPI_SPEED = 100000  # Conservative speed
SPI_MODE = 1  # LDC1101 uses SPI mode 1

# LDC1101 Register Map
CHIP_ID_REG      = 0x3F
STATUS_REG       = 0x20
RCOUNT_REG       = 0x01
FUNC_MODE_REG    = 0x0F
INDUCTANCE_REG   = 0x1A
ERROR_CONFIG_REG = 0x21

# SPI helper functions
def spi_setup():
    spi = spidev.SpiDev()
    spi.open(SPI_BUS, SPI_DEVICE)
    spi.max_speed_hz = SPI_SPEED
    spi.mode = SPI_MODE
    return spi

def read_register(spi, addr):
    read_cmd = [0x80 | addr, 0x00]
    response = spi.xfer2(read_cmd)
    return response[1]

def write_register(spi, addr, value):
    write_cmd = [addr & 0x7F, value]
    spi.xfer2(write_cmd)

def ldc1101_diagnostics():
    spi = spi_setup()

    print("\n[ LDC1101 Diagnostic Tool ]")
    print("---------------------------")

    try:
        chip_id = read_register(spi, CHIP_ID_REG)
        print(f"CHIP_ID Register (0x3F): 0x{chip_id:02X}")
        if chip_id == 0x00:
            print("Chip did not respond. Possible communication or power issue.")
        elif chip_id != 0xD4:
            print("Unexpected CHIP ID. Check datasheet for your module version.")
        else:
            print("LDC1101 responded with correct CHIP ID.")

        # Read and interpret FUNC_MODE
        func_mode = read_register(spi, FUNC_MODE_REG)
        print(f"FUNC_MODE (0x0F): 0x{func_mode:02X}")
        if func_mode == 0x00:
            print("Not in active mode.")
        elif func_mode == 0x01:
            print("Mode: Active Conversion")
        else:
            print(f"Mode: Unknown or custom (0x{func_mode:02X})")

        # Status Register
        status = read_register(spi, STATUS_REG)
        print(f"STATUS (0x20): 0x{status:02X}")

        # Error Config
        error_cfg = read_register(spi, ERROR_CONFIG_REG)
        print(f"ERROR_CONFIG (0x21): 0x{error_cfg:02X}")

        # Inductance Value
        inductance = read_register(spi, INDUCTANCE_REG)
        print(f"INDUCTANCE_DATA_MSB (0x1A): 0x{inductance:02X}")

        # Attempt to set active mode (if not already)
        print("\nAttempting to set FUNC_MODE to active (0x01)...")
        write_register(spi, FUNC_MODE_REG, 0x01)
        time.sleep(0.1)
        mode_check = read_register(spi, FUNC_MODE_REG)
        print(f"FUNC_MODE after write: 0x{mode_check:02X}")
        if mode_check != 0x01:
            print("Failed to enter active mode.")
        else:
            print("Active mode set.")

    except Exception as e:
        print(f"\n[ERROR] SPI communication failed: {e}")

    finally:
        spi.close()

# Run diagnostic
if __name__ == "__main__":
    ldc1101_diagnostics()
