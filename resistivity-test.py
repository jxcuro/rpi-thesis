import spidev
import time

# SPI setup
spi = spidev.SpiDev()
spi.open(0, 0)  # Open SPI bus 0, device 0 (CE0)
spi.max_speed_hz = 50000  # Set SPI speed (adjust if needed)
spi.mode = 0b00  # SPI mode 0

# LDC1101 Power Mode Command
ACTIVE_MODE_CMD = 0x00  # Command to set Active mode

def set_power_mode(mode):
    """
    Set the LDC1101 power mode.
    :param mode: Power mode (0x00 for Active mode).
    """
    # Send the command to set the power mode (register 0x01 for power control)
    power_mode_register = 0x01  # Register for power control
    response = spi.xfer2([power_mode_register, mode])
    time.sleep(0.1)  # Wait for the mode to take effect

    # Optionally, read back the register to confirm it is in active mode
    return response

# Set the LDC1101 to Active mode
response = set_power_mode(ACTIVE_MODE_CMD)

# Print the response to confirm the mode change
print("Response:", response)
