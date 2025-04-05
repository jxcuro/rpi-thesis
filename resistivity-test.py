# Reset LDC1101
def reset_ldc1101():
    GPIO.output(CS_PIN, GPIO.LOW)  # Pull CS low to start communication
    spi.xfer2([0x03, 0x01])  # Send the reset command
    GPIO.output(CS_PIN, GPIO.HIGH)  # Pull CS high to end communication
    time.sleep(0.1)  # Wait for reset to take effect

reset_ldc1101()  # Perform reset before setting power mode
