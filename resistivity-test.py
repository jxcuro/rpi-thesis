import RPi.GPIO as GPIO
import time

CS_PIN = 8  # GPIO8 / Pin 24

GPIO.setmode(GPIO.BCM)
GPIO.setup(CS_PIN, GPIO.OUT)

print("Toggling CS pin...")
for i in range(10):
    GPIO.output(CS_PIN, GPIO.LOW)
    time.sleep(0.5)
    GPIO.output(CS_PIN, GPIO.HIGH)
    time.sleep(0.5)
