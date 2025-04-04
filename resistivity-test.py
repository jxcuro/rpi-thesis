import spidev
import time

spi = spidev.SpiDev()
spi.open(0, 0)  # Bus 0, CS 0
spi.max_speed_hz = 500000
spi.mode = 1  # SPI mode 1 (CPOL=0, CPHA=1)

def read_register(address):
    read_cmd = [address & 0x7F, 0x00]  # MSB=0 for read
    response = spi.xfer2(read_cmd)
    return response[1]

def write_register(address, value):
    write_cmd = [address | 0x80, value]  # MSB=1 for write
    spi.xfer2(write_cmd)

def debug_ldc1101():
    print("Starting LDC1101 SPI debug...")

    # Check CHIP_ID and RDEVICE_ID
    chip_id = read_register(0x3F)
    rdevice_id = read_register(0x3E)
    print(f"CHIP_ID (0x3F): 0x{chip_id:02X}")
    print(f"RDEVICE_ID (0x3E): 0x{rdevice_id:02X}")

    if chip_id != 0xD4:
        print("Error: CHIP_ID is incorrect. Check wiring and power.")
        return

    # Set to Active Mode
    print("Setting to Active mode (FUNC_MODE)...")
    write_register(0x01, 0x01)  # FUNC_MODE = Active

    # Verify FUNC_MODE
    func_mode = read_register(0x01)
    print(f"FUNC_MODE (0x01): 0x{func_mode:02X}")
    if func_mode != 0x01:
        print("Error: Failed to set FUNC_MODE to Active.")
    else:
        print("FUNC_MODE set to Active.")

    # Read STATUS register
    status = read_register(0x00)
    print(f"STATUS (0x00): 0x{status:02X}")

    # Read a result (optional test)
    rp_data = (
        (read_register(0x20) << 16) |
        (read_register(0x21) << 8) |
        read_register(0x22)
    )
    print(f"RP_DATA: {rp_data}")

    print("LDC1101 debug complete.")

debug_ldc1101()
