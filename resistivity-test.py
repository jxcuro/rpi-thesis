import spidev
import RPi.GPIO as GPIO
import time
import math

class LDC1101:
    # Register addresses
    REG_RP_SET = 0x01
    REG_TC1 = 0x02
    REG_TC2 = 0x03
    REG_DIG_CONFIG = 0x04
    REG_ALT_CONFIG = 0x05
    REG_START_CONFIG = 0x0B
    REG_D_CONFIG = 0x0C
    REG_STATUS = 0x20
    REG_RP_DATA_LSB = 0x21
    REG_RP_DATA_MSB = 0x22
    REG_L_DATA_LSB = 0x23
    REG_L_DATA_MSB = 0x24
    REG_LHR_RCOUNT_LSB = 0x30
    REG_LHR_RCOUNT_MSB = 0x31
    REG_LHR_OFFSET_LSB = 0x32
    REG_LHR_OFFSET_MSB = 0x33
    REG_LHR_CONFIG = 0x34
    REG_LHR_DATA_LSB = 0x38
    REG_LHR_DATA_MID = 0x39
    REG_LHR_DATA_MSB = 0x3A
    REG_LHR_STATUS = 0x3B
    
    def __init__(self, bus=0, device=0, clkin_pin=12, clkin_freq=16000000):
        # Initialize SPI
        self.spi = spidev.SpiDev()
        self.spi.open(bus, device)  # Using SPI0.0 (CE0)
        self.spi.max_speed_hz = 1000000
        self.spi.mode = 0
        
        # Initialize CLKIN pin (PCM_CLK - Pin 12)
        self.clkin_pin = clkin_pin
        self.clkin_freq = clkin_freq
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.clkin_pin, GPIO.OUT)
        
        # Set up PWM for CLKIN
        self.pwm = GPIO.PWM(self.clkin_pin, clkin_freq // 1000)  # Frequency in kHz
        self.pwm.start(50)  # 50% duty cycle
        
        # Initialize the sensor
        self.initialize()
    
    def __del__(self):
        self.pwm.stop()
        GPIO.cleanup()
        self.spi.close()
    
    def read_register(self, reg_addr):
        # Set the read bit (MSB = 1)
        addr = 0x80 | reg_addr
        result = self.spi.xfer2([addr, 0x00])
        return result[1]
    
    def write_register(self, reg_addr, value):
        # Clear the read bit (MSB = 0)
        self.spi.xfer2([reg_addr, value])
    
    def initialize(self):
        # Put device in sleep mode for configuration
        self.write_register(self.REG_START_CONFIG, 0x01)
        
        # Configure for RP+L measurement
        # Set RP_MIN to 1.5 kΩ and RP_MAX to 12 kΩ
        self.write_register(self.REG_RP_SET, 0x36)
        
        # Set internal time constants
        # These values are based on a typical sensor with ~2MHz resonant frequency
        self.write_register(self.REG_TC1, 0x90)  # Default value
        self.write_register(self.REG_TC2, 0xA0)  # Default value
        
        # Configure digital settings
        # Set MIN_FREQ for ~2MHz and RESP_TIME to 384
        self.write_register(self.REG_DIG_CONFIG, 0x83)
        
        # Configure LHR mode for high-resolution inductance measurement
        # Set RCOUNT to 0x0FFF for good resolution
        self.write_register(self.REG_LHR_RCOUNT_LSB, 0xFF)
        self.write_register(self.REG_LHR_RCOUNT_MSB, 0x0F)
        
        # Set LHR_OFFSET to 0
        self.write_register(self.REG_LHR_OFFSET_LSB, 0x00)
        self.write_register(self.REG_LHR_OFFSET_MSB, 0x00)
        
        # Set LHR_CONFIG to no divider
        self.write_register(self.REG_LHR_CONFIG, 0x00)
        
        # Put device in active mode
        self.write_register(self.REG_START_CONFIG, 0x00)
        
        # Wait for oscillation to stabilize
        time.sleep(0.1)
    
    def read_inductance_rp_l(self):
        # Read L_DATA registers (standard resolution)
        l_lsb = self.read_register(self.REG_L_DATA_LSB)
        l_msb = self.read_register(self.REG_L_DATA_MSB)
        
        # Combine bytes to get L_DATA
        l_data = (l_msb << 8) | l_lsb
        
        # Calculate sensor frequency
        # f_SENSOR = (f_CLKIN * RESP_TIME) / (3 * L_DATA)
        resp_time = 384  # Based on our configuration
        if l_data == 0:
            return float('inf')  # Avoid division by zero
        
        f_sensor = (self.clkin_freq * resp_time) / (3 * l_data)
        
        # Calculate inductance (assuming a fixed capacitance)
        # L = 1 / (C * (2π * f_SENSOR)²)
        # Assuming C = 270 pF (typical value, adjust based on your sensor)
        c_sensor = 270e-12  # 270 pF
        
        if f_sensor == 0:
            return float('inf')  # Avoid division by zero
        
        inductance = 1 / (c_sensor * (2 * math.pi * f_sensor) ** 2)
        return inductance * 1e6  # Convert to μH
    
    def read_inductance_lhr(self):
        # Read LHR_DATA registers (high resolution)
        lhr_lsb = self.read_register(self.REG_LHR_DATA_LSB)
        lhr_mid = self.read_register(self.REG_LHR_DATA_MID)
        lhr_msb = self.read_register(self.REG_LHR_DATA_MSB)
        
        # Combine bytes to get LHR_DATA
        lhr_data = (lhr_msb << 16) | (lhr_mid << 8) | lhr_lsb
        
        # Check if data is valid
        if lhr_data == 0 or lhr_data == 0xFFFFFF:
            return float('inf')  # Invalid data
        
        # Calculate sensor frequency
        # f_SENSOR = f_CLKIN * 2^SENSOR_DIV * LHR_DATA / 2^24
        sensor_div = 0  # Based on our configuration
        f_sensor = self.clkin_freq * (2 ** sensor_div) * lhr_data / (2 ** 24)
        
        # Calculate inductance (assuming a fixed capacitance)
        # L = 1 / (C * (2π * f_SENSOR)²)
        # Assuming C = 270 pF (typical value, adjust based on your sensor)
        c_sensor = 270e-12  # 270 pF
        
        if f_sensor == 0:
            return float('inf')  # Avoid division by zero
        
        inductance = 1 / (c_sensor * (2 * math.pi * f_sensor) ** 2)
        return inductance * 1e6  # Convert to μH
    
    def is_object_present(self, baseline, threshold=0.1):
        """
        Detect if an object is present based on inductance change
        
        Args:
            baseline: Baseline inductance with no object present
            threshold: Threshold for detection (fractional change)
            
        Returns:
            True if object is detected, False otherwise
        """
        current = self.read_inductance_lhr()
        if current == float('inf'):
            return False
            
        # Calculate relative change
        change = abs(current - baseline) / baseline
        
        return change > threshold

# Example usage
if __name__ == "__main__":
    try:
        # Initialize the sensor
        sensor = LDC1101()
        
        # Calibration - measure baseline inductance with no object
        print("Calibrating... Please ensure no objects are near the sensor")
        time.sleep(2)
        
        # Take multiple readings and average them for baseline
        baseline_readings = []
        for _ in range(10):
            inductance = sensor.read_inductance_lhr()
            if inductance != float('inf'):
                baseline_readings.append(inductance)
            time.sleep(0.1)
        
        if not baseline_readings:
            print("Error: Could not get valid baseline readings")
            exit(1)
            
        baseline = sum(baseline_readings) / len(baseline_readings)
        print(f"Baseline inductance: {baseline:.3f} μH")
        
        print("\nReady to detect objects. Place objects near the sensor...")
        print("Press Ctrl+C to exit")
        
        # Continuous measurement loop
        while True:
            # Read inductance using high-resolution method
            inductance_lhr = sensor.read_inductance_lhr()
            
            # Read inductance using standard method
            inductance_rp_l = sensor.read_inductance_rp_l()
            
            # Detect object presence
            object_present = sensor.is_object_present(baseline)
            
            # Print results
            print(f"Inductance (LHR): {inductance_lhr:.3f} μH | ", end="")
            print(f"Inductance (RP+L): {inductance_rp_l:.3f} μH | ", end="")
            print(f"Object Present: {'Yes' if object_present else 'No'}")
            
            time.sleep(0.5)
            
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        if 'sensor' in locals():
            del sensor
        GPIO.cleanup()
