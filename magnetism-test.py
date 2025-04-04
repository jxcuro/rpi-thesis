import time
import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

# Initialize I2C and ADS1115
i2c = busio.I2C(board.SCL, board.SDA)
ads = ADS.ADS1115(i2c)

# Use A0 channel for Hall sensor
hall_sensor = AnalogIn(ads, ADS.P0)

print("Reading idle voltage from Hall sensor...")
print("Make sure no magnets or metal are near the sensor.")

try:
    while True:
        voltage = hall_sensor.voltage
        print(f"Idle Voltage: {voltage:.4f} V")
        time.sleep(0.5)  # Read every half-second

except KeyboardInterrupt:
    print("\nCalibration stopped.")
    print("You can take the average of the displayed values as your IDLE_VOLTAGE.")
