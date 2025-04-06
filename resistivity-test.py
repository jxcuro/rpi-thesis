#!/usr/bin/env python3
"""
LDC1101 Driver for Raspberry Pi 4B
Configured for LHR mode with specific pin connections
"""

import time
import spidev
import RPi.GPIO as GPIO

# Register definitions
class LDC1101_REG:
    CFG_RP_MEASUREMENT_DYNAMIC_RANGE = 0x01
    CFG_INTERNAL_TIME_CONSTANT_1 = 0x02
    CFG_INTERNAL_TIME_CONSTANT_2 = 0x03
    CFG_RP_L_CONVERSION_INTERVAL = 0x04
    CFG_ADDITIONAL_DEVICE = 0x05
    RP_THRESH_H_LSB = 0x06
    RP_THRESH_H_MSB = 0x07
    RP_THRESH_L_LSB = 0x08
    RP_THRESH_L_MSB = 0x09
    CFG_INTB_MODE = 0x0A
    CFG_POWER_STATE = 0x0B
    AMPLITUDE_CONTROL_REQUIREMENT = 0x0C
    L_THRESH_HI_LSB = 0x16
    L_THRESH_HI_MSB = 0x17
    L_THRESH_LO_LSB = 0x18
    L_THRESH_LO_MSB = 0x19
    RP_L_MEASUREMENT_STATUS = 0x20
    RP_DATA_LSB = 0x21
    RP_DATA_MSB = 0x22
    L_DATA_LSB = 0x23
    L_DATA_MSB = 0x24
    LHR_RCOUNT_LSB = 0x30
    LHR_RCOUNT_MSB = 0x31
    LHR_OFFSET_LSB = 0x32
    LHR_OFFSET_MSB = 0x33
    CFG_LHR = 0x34
    LHR_DATA_LSB = 0x38
    LHR_DATA_MID = 0x39
    LHR_DATA_MSB = 0x3A
    LHR_STATUS = 0x3B
    DEVICE_RID_VALUE = 0x3E
    CHIP_ID = 0x3F

# Constants for RP_SET Field
class LDC1101_RP_SET:
    RP_MAX_IS_DRIVEN = 0x00
    RP_MAX_CURRENT_IS_IGNORED = 0x80
    RP_MAX_96KOhm = 0x00
    RP_MAX_48KOhm = 0x10
    RP_MAX_24KOhm = 0x20
    RP_MAX_12KOhm = 0x30
    RP_MAX_6KOhm = 0x40
    RP_MAX_3KOhm = 0x50
    RP_MAX_1_5KOhm = 0x60
    RP_MAX_0_75KOh = 0x70
    RP_MIN_96KOhm = 0x00
    RP_MIN_48KOhm = 0x01
    RP_MIN_24KOhm = 0x02
    RP_MIN_12KOhm = 0x03
    RP_MIN_6KOhm = 0x04
    RP_MIN_3KOhm = 0x05
    RP_MIN_1_5KOhm = 0x06
    RP_MIN_0_75KOh = 0x07

# Configure Internal Time Constant 1
class LDC1101_TC1:
    C1_0_75pF = 0x00
    C1_1_5pF = 0x40
    C1_3pF = 0x80
    C1_6pF = 0xC0
    R1_417kOhm = 0x00
    R1_212_7kOhm = 0x10
    R1_21_1kOhm = 0x1F

# Configure Internal Time Constant 2
class LDC1101_TC2:
    C2_3pF = 0x00
    C2_6pF = 0x40
    C2_12pF = 0x80
    C2_24pF = 0xC0
    R2_835kOhm = 0x00
    R2_426_4kOhm = 0x20
    R2_30_5kOhm = 0x3F

# Configure RP+L conversion interval
class LDC1101_DIG_CFG:
    MIN_FREQ_500kHz = 0x00
    MIN_FREQ_8MHz = 0xF0
    RESP_TIME_192s = 0x02
    RESP_TIME_384s = 0x03
    RESP_TIME_768s = 0x04
    RESP_TIME_1536s = 0x05
    RESP_TIME_3072s = 0x06
    RESP_TIME_6144s = 0x07

# Configure additional device settings
class LDC1101_ALT_CFG:
    SHUTDOWN_ENABLE = 0x02
    SHUTDOWN_DISABLE = 0x00
    L_OPTIMAL_DISABLED = 0x00
    L_OPTIMAL_ENABLE = 0x01

# Configure INTB reporting on SDO pin
class LDC1101_INTB_MODE:
    DONT_REPORT_INTB_ON_SDO_PIN = 0x00
    REPORT_INTB_ON_SDO_PIN = 0x80
    REPORT_LHR_DATA_READY = 0x20
    L_CONVERSION_TO_L_THRESHOLDS = 0x10
    L_CONVERSION_TO_L_HIGH_THRESHOLDS = 0x08
    REPORT_RP_L_DATA_READY = 0x04
    RP_CONVERSION_TO_L_THRESHOLDS = 0x02
    RP_CONVERSION_TO_L_HIGH_THRESHOLDS = 0x01
    NO_OUTPUT = 0x00

# Configure Power State
class LDC1101_FUNC_MODE:
    ACTIVE_CONVERSION_MODE = 0x00
    SLEEP_MODE = 0x01
    SHUTDOWN_MODE = 0x02

# High Resolution L Configuration
class LDC1101_LHR_CFG:
    FREQUENCY_NOT_DIVIDED = 0x00
    FREQUENCY_DIVIDED_BY_2 = 0x01
    FREQUENCY_DIVIDED_BY_4 = 0x02
    FREQUENCY_DIVIDED_BY_8 = 0x03

# Status codes
DEVICE_ERROR = 0x01
DEVICE_OK = 0x00

class LDC1101:
    def __init__(self, cs_pin=24, pwm_pin=12, spi_bus=0, spi_device=0):
        """
        Initialize the LDC1101 driver
        
        Args:
            cs_pin: GPIO pin number for chip select (default 24 for CE0)
            pwm_pin: GPIO pin number for PWM (default 12 for PCM_CLK)
            spi_bus: SPI bus number (default 0)
            spi_device: SPI device number (default 0)
        """
        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        
        # Convert from physical pin numbers to BCM if needed
        # For Raspberry Pi 4B:
        # Physical pin 24 = BCM 8 (CE0)
        # Physical pin 12 = BCM 18 (PCM_CLK)
        self.cs_pin = 8  # BCM pin for physical pin 24
        self.pwm_pin = 18  # BCM pin for physical pin 12
        
        GPIO.setup(self.cs_pin, GPIO.OUT)
        GPIO.output(self.cs_pin, GPIO.HIGH)
        
        # Setup PWM pin if needed
        GPIO.setup(self.pwm_pin, GPIO.IN)  # Set as input initially
        
        # Initialize SPI
        self.spi = spidev.SpiDev()
        self.spi.open(spi_bus, spi_device)
        self.spi.max_speed_hz = 1000000  # 1MHz
        self.spi.mode = 0  # Mode 0: CPOL=0, CPHA=0
    
    def __del__(self):
        """Cleanup when object is deleted"""
        try:
            self.spi.close()
            GPIO.cleanup()
        except:
            pass
    
    def write_byte(self, addr, data):
        """
        Write a byte to the specified register address
        
        Args:
            addr: Register address
            data: Byte to write
        """
        GPIO.output(self.cs_pin, GPIO.LOW)
        self.spi.xfer2([addr, data])
        GPIO.output(self.cs_pin, GPIO.HIGH)
    
    def read_byte(self, addr):
        """
        Read a byte from the specified register address
        
        Args:
            addr: Register address
            
        Returns:
            Byte read from the register
        """
        GPIO.output(self.cs_pin, GPIO.LOW)
        result = self.spi.xfer2([0x80 | addr, 0])
        GPIO.output(self.cs_pin, GPIO.HIGH)
        return result[1]
    
    def init(self):
        """
        Initialize the LDC1101 device
        
        Returns:
            DEVICE_OK if successful, DEVICE_ERROR otherwise
        """
        # Check chip ID
        chip_id = self.read_byte(LDC1101_REG.CHIP_ID)
        if chip_id != 0xD4:
            print(f"Error: Invalid chip ID: 0x{chip_id:02X}, expected 0xD4")
            return DEVICE_ERROR
        
        # Default initialization
        self.write_byte(LDC1101_REG.CFG_RP_MEASUREMENT_DYNAMIC_RANGE, 0x07)
        self.write_byte(LDC1101_REG.CFG_INTERNAL_TIME_CONSTANT_1, 0x90)
        self.write_byte(LDC1101_REG.CFG_INTERNAL_TIME_CONSTANT_2, 0xA0)
        self.write_byte(LDC1101_REG.CFG_RP_L_CONVERSION_INTERVAL, 0x03)
        self.write_byte(LDC1101_REG.CFG_ADDITIONAL_DEVICE, 0x00)
        self.write_byte(LDC1101_REG.RP_THRESH_H_MSB, 0x00)
        self.write_byte(LDC1101_REG.RP_THRESH_L_LSB, 0x00)
        self.write_byte(LDC1101_REG.RP_THRESH_L_MSB, 0x00)
        self.write_byte(LDC1101_REG.CFG_INTB_MODE, 0x00)
        self.write_byte(LDC1101_REG.CFG_POWER_STATE, LDC1101_FUNC_MODE.SLEEP_MODE)
        self.write_byte(LDC1101_REG.AMPLITUDE_CONTROL_REQUIREMENT, 0x00)
        self.write_byte(LDC1101_REG.L_THRESH_HI_LSB, 0x00)
        self.write_byte(LDC1101_REG.L_THRESH_HI_MSB, 0x00)
        self.write_byte(LDC1101_REG.L_THRESH_LO_LSB, 0x00)
        self.write_byte(LDC1101_REG.L_THRESH_LO_MSB, 0x00)
        self.write_byte(LDC1101_REG.LHR_RCOUNT_LSB, 0x00)
        self.write_byte(LDC1101_REG.LHR_RCOUNT_MSB, 0x00)
        self.write_byte(LDC1101_REG.LHR_OFFSET_LSB, 0x00)
        self.write_byte(LDC1101_REG.LHR_OFFSET_MSB, 0x00)
        self.write_byte(LDC1101_REG.CFG_LHR, 0x00)
        time.sleep(0.1)  # 100ms delay
        
        return DEVICE_OK
    
    def set_power_mode(self, mode):
        """
        Set the power mode of the device
        
        Args:
            mode: Power mode (use LDC1101_FUNC_MODE constants)
        """
        self.write_byte(LDC1101_REG.CFG_POWER_STATE, mode)
    
    def go_to_l_mode(self):
        """Switch to L mode"""
        self.write_byte(LDC1101_REG.CFG_ADDITIONAL_DEVICE, 0x01)
        self.write_byte(LDC1101_REG.AMPLITUDE_CONTROL_REQUIREMENT, 0x01)
    
    def go_to_rp_mode(self):
        """Switch to RP mode"""
        self.write_byte(LDC1101_REG.CFG_ADDITIONAL_DEVICE, 0x02)
        self.write_byte(LDC1101_REG.AMPLITUDE_CONTROL_REQUIREMENT, 0x00)
    
    def set_lhr_mode(self):
        """
        Configure the device for LHR (High Resolution L) mode
        """
        # Set LHR configuration
        # Using no frequency division for maximum resolution
        self.write_byte(LDC1101_REG.CFG_LHR, LDC1101_LHR_CFG.FREQUENCY_NOT_DIVIDED)
        
        # Set RCOUNT (reference count) - higher values give better resolution
        # Setting to a moderate value (0x0800 = 2048)
        self.write_byte(LDC1101_REG.LHR_RCOUNT_LSB, 0x00)
        self.write_byte(LDC1101_REG.LHR_RCOUNT_MSB, 0x08)
        
        # Set offset to zero initially
        self.write_byte(LDC1101_REG.LHR_OFFSET_LSB, 0x00)
        self.write_byte(LDC1101_REG.LHR_OFFSET_MSB, 0x00)
        
        # Configure interrupt to report LHR data ready
        self.write_byte(LDC1101_REG.CFG_INTB_MODE, LDC1101_INTB_MODE.REPORT_LHR_DATA_READY)
        
        # Set to active conversion mode
        self.set_power_mode(LDC1101_FUNC_MODE.ACTIVE_CONVERSION_MODE)
    
    def get_status(self):
        """
        Get the status register value
        
        Returns:
            Status register value
        """
        return self.read_byte(LDC1101_REG.RP_L_MEASUREMENT_STATUS)
    
    def get_lhr_status(self):
        """
        Get the LHR status register value
        
        Returns:
            LHR status register value
        """
        return self.read_byte(LDC1101_REG.LHR_STATUS)
    
    def get_rp_data(self):
        """
        Get the RP data
        
        Returns:
            16-bit RP data value
        """
        msb = self.read_byte(LDC1101_REG.RP_DATA_MSB)
        lsb = self.read_byte(LDC1101_REG.RP_DATA_LSB)
        return (msb << 8) | lsb
    
    def get_l_data(self):
        """
        Get the L data
        
        Returns:
            16-bit L data value
        """
        msb = self.read_byte(LDC1101_REG.L_DATA_MSB)
        lsb = self.read_byte(LDC1101_REG.L_DATA_LSB)
        return (msb << 8) | lsb
    
    def get_lhr_data(self):
        """
        Get the LHR (High Resolution L) data
        
        Returns:
            24-bit LHR data value
        """
        msb = self.read_byte(LDC1101_REG.LHR_DATA_MSB)
        mid = self.read_byte(LDC1101_REG.LHR_DATA_MID)
        lsb = self.read_byte(LDC1101_REG.LHR_DATA_LSB)
        return (msb << 16) | (mid << 8) | lsb
    
    def get_pwm_state(self):
        """
        Get the PWM pin state
        
        Returns:
            PWM pin state
        """
        return GPIO.input(self.pwm_pin)


# Example usage with continuous data reading
if __name__ == "__main__":
    try:
        print("Initializing LDC1101 in LHR mode...")
        
        # Initialize the LDC1101 with the correct pin mappings
        ldc = LDC1101()
        
        # Initialize the device
        status = ldc.init()
        if status == DEVICE_OK:
            print("LDC1101 initialized successfully")
            
            # Set to LHR mode
            ldc.set_lhr_mode()
            print("LDC1101 set to LHR mode")
            
            # Read and display data continuously
            print("Reading LHR data (Press Ctrl+C to stop)...")
            print("Time (s)\tLHR Data\tStatus")
            
            start_time = time.time()
            while True:
                current_time = time.time() - start_time
                lhr_data = ldc.get_lhr_data()
                lhr_status = ldc.get_lhr_status()
                
                print(f"{current_time:.2f}\t{lhr_data}\t0x{lhr_status:02X}")
                time.sleep(0.5)
                
        else:
            print("Failed to initialize LDC1101")
            
    except KeyboardInterrupt:
        print("\nProgram terminated by user")
    finally:
        # Set to sleep mode when done
        if 'ldc' in locals():
            ldc.set_power_mode(LDC1101_FUNC_MODE.SLEEP_MODE)
            print("Set to sleep mode")
