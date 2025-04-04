import spidev
import time

# Open SPI (SPI bus 0, device 0)
spi = spidev.SpiDev()
spi.open(0, 0)  # SPI bus 0, device 0 (CE0)

# Try different SPI settings
modes = [0, 1, 2, 3]
speeds = [500000, 1000000, 2000000]  # Experiment with different speeds

for mode in modes:
    for speed in speeds:
        print(f"Testing mode {mode} and speed {speed}Hz")
        spi.mode = mode
        spi.max_speed_hz = speed
        
        # Send and receive data
        response = spi.xfer([0x01, 0x02])  # Send two bytes
        print(f"Response with mode {mode} and speed {speed}: {response}")

# Close SPI after use
spi.close()
