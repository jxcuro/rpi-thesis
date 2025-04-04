REGISTER_FUNC_MODE = 0x0F

def write_register(register, value):
    spi = spidev.SpiDev()
    spi.open(0, 0)
    spi.max_speed_hz = 500000
    spi.mode = 1
    spi.xfer2([register & 0x7F, value])
    spi.close()

# Set mode to active
write_register(REGISTER_FUNC_MODE, 0x01)
time.sleep(0.1)
mode = read_register(REGISTER_FUNC_MODE)
print(f"FUNC_MODE after write: {hex(mode)}")
