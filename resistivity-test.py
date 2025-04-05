import spidev
import time

# SPI setup
spi = spidev.SpiDev()
spi.open(0, 0)  # Open SPI bus 0, device 0 (CE0)
spi.max_speed_hz = 50000  # Set SPI speed (adjust if needed)
spi.mode = 0b00  # SPI mode 0

# LDC1101 Power Mode Command
ACTIVE_MODE_CMD = 0x00  # Command to set Active mode

# Power Control Register address
POWER_MODE_REGISTER = 0x01

def set_power_mode(mode):
    """
    Set the LDC1101 power mode.
    :param mode: Power mode (0x00 for Active mode).
    """
    # Send the command to set the power mode (register 0x01 for power control)
    response = spi.xfer2([POWER_MODE_REGISTER, mode])
    time.sleep(0.1)  # Wait for the mode to take effect
    return response

def read_status_register():
    """
    Read the status register (0x00) to confirm the current mode.
    """
    status_register = 0x00  # Status register address
    status = spi.xfer2([status_register, 0x00])  # Send dummy byte to read
    return status

# Set the LDC1101 to Active mode
set_power_mode(ACTIVE_MODE_CMD)

# Wait briefly to ensure mode change
time.sleep(0.5)

# Read and print the status register to verify the mode
status = read_status_register()
print("Status Register (after mode change):", status)
