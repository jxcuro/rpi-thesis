import spidev
import time
import RPi.GPIO as GPIO

# SPI and GPIO setup
spi = spidev.SpiDev()
spi.open(0, 0)  # Open SPI bus 0, device (CS) 0
spi.max_speed_hz = 1000000  # 1 MHz
spi.mode = 1  # SPI Mode 1 (CPOL = 0, CPHA = 1)

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
cs_pin = 8  # CE0 is GPIO8
GPIO.setup(cs_pin, GPIO.OUT)
GPIO.output(cs_pin, GPIO.HIGH)

def read_register(register_address):
    # Construct the read command
    read_cmd = [0x80 | (register_address & 0x3F), 0x00]  # 1st byte = read command, 2nd byte = dummy
    GPIO.output(cs_pin, GPIO.LOW)
    time.sleep(0.001)
    response = spi.xfer2(read_cmd)
    time.sleep(0.001)
    GPIO.output(cs_pin, GPIO.HIGH)
    return response[1]  # Data is returned in the second byte

print("Reading LDC1101 Registers (0x00 to 0x3F):")
for reg in range(0x00, 0x40):  # Loop from 0x00 to 0x3F
    val = read_register(reg)
    print(f"Register 0x{reg:02X} = 0x{val:02X}")

spi.close()
