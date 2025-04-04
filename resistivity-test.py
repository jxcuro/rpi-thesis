import spidev
import time

# SPI setup
spi = spidev.SpiDev()
spi.open(0, 0)  # Bus 0, device 0 (CE0)
spi.max_speed_hz = 500000
spi.mode = 1  # SPI mode 1: CPOL=0, CPHA=1

# Register addresses
REGISTER_CHIP_ID = 0x3E
REGISTER_FUNC_MODE = 0x0F
REGISTER_STATUS = 0x01

# SPI Read
def read_register(register):
    read_command = 0x80 | (register & 0x7F)
    response = spi.xfer2([read_command, 0x00])
    return response[1]

# SPI Write
def write_register(register, value):
    write_command = register & 0x7F
    spi.xfer2([write_command, value])

# ---- Start of Debug Sequence ----
print("🔍 Starting LDC1101 SPI Debug\n")

# Step 1: Check Chip ID
chip_id = read_register(REGISTER_CHIP_ID)
print(f"📦 Chip ID (0x3E): {hex(chip_id)}")
if chip_id == 0xD4:
    print("✅ LDC1101 is responding correctly!\n")
else:
    print("❌ Unexpected Chip ID. SPI or wiring may be wrong.\n")

# Step 2: Write to FUNC_MODE register
print("⚙️  Writing 0x01 to FUNC_MODE (Active mode)...")
write_register(REGISTER_FUNC_MODE, 0x01)
time.sleep(0.1)
func_mode = read_register(REGISTER_FUNC_MODE)
print(f"🔁 FUNC_MODE readback: {hex(func_mode)}")
if func_mode == 0x01:
    print("✅ FUNC_MODE successfully set to ACTIVE.\n")
else:
    print("❌ FUNC_MODE write failed.\n")

# Step 3: Read STATUS register
status = read_register(REGISTER_STATUS)
print(f"📊 STATUS Register (0x01): {hex(status)}")

# Cleanup
spi.close()
print("\n✅ Debug complete.")
