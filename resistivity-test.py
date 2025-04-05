import RPi.GPIO as GPIO
import spidev

# Set up SPI interface
spi = spidev.SpiDev()
spi.open(0, 0)  # SPI bus 0, device 0 (CE0)
spi.max_speed_hz = 50000
spi.mode = 1  # SPI Mode 1

# Set up GPIO for CS (CE0 pin)
GPIO.setmode(GPIO.BOARD)
CS_PIN = 8  # Pin for chip select (CE0)
GPIO.setup(CS_PIN, GPIO.OUT)
GPIO.output(CS_PIN, GPIO.HIGH)  # Keep it high initially

# Toggle CS low before communication and high after
GPIO.output(CS_PIN, GPIO.LOW)  # Start communication
response = spi.xfer2([0x00])  # Example to read the status register (0x00)
GPIO.output(CS_PIN, GPIO.HIGH)  # End communication

print("Response:", response)

# Clean up GPIO
GPIO.cleanup()
