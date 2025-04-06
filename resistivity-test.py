import spidev
import time
import RPi.GPIO as GPIO

CS_PIN = 8  # Use GPIO8 (CE0)

GPIO.setmode(GPIO.BCM)
GPIO.setup(CS_PIN, GPIO.OUT)
GPIO.output(CS_PIN, GPIO.HIGH)

spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 8000000
spi.mode = 1  # SPI Mode 1: CPOL=0, CPHA=1

def spi_transfer(data):
    GPIO.output(CS_PIN, GPIO.LOW)
    resp = spi.xfer2(data)
    GPIO.output(CS_PIN, GPIO.HIGH)
    time.sleep(0.001)
    return resp

def write_register(addr, value):
    spi_transfer([addr & 0x7F, value])

def read_register(addr):
    return spi_transfer([addr | 0x80, 0x00])[1]

def configure_lhr_mode():
    write_register(0x25, 0x01)
    time.sleep(0.01)

    write_register(0x1A, 0x02)
    write_register(0x12, 0xFF)
    write_register(0x13, 0xFF)
    write_register(0x14, 0x03)
    write_register(0x1E, 0xFF)
    write_register(0x1F, 0xFF)
    write_register(0x20, 0x0F)
    write_register(0x24, 0x20)

    write_register(0x25, 0x00)
    time.sleep(0.01)

def dump_registers():
    print("Register Dump (0x00 to 0x3F):")
    for addr in range(0x00, 0x40):
        val = read_register(addr)
        print(f"0x{addr:02X}: 0x{val:02X}")

configure_lhr_mode()
dump_registers()
