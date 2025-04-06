import time
import spidev
import RPi.GPIO as GPIO



# @ SPI

# Define SPI parameters for the Raspberry Pi 4
SPI_BUS = 0  # SPI bus 0 (could be 1 depending on your setup)
SPI_DEVICE = 0  # SPI device (usually 0 for SPI0)
SPI_SPEED_HZ = 50000  # SPI clock speed in Hz
SPI_MODE = 0b00  # SPI Mode (0: CPOL=0, CPHA=0)

# Initialize SPI connection
spi = spidev.SpiDev()

# Define configuration flags to simulate conditional compilation from C code.
__LDC1101_DRV_SPI__ = True

def hal_spiMap(spiObj):
    # Open the SPI bus and device
    spiObj.open(SPI_BUS, SPI_DEVICE)

    # Set SPI mode (CPOL=0, CPHA=0)
    spiObj.mode = SPI_MODE

    # Set the SPI speed (clock speed)
    spiObj.max_speed_hz = SPI_SPEED_HZ



# @ GPIO

# Define the GPIO pins for the LDC1101
CS_PIN = 8   # Chip Select pin (example GPIO pin)
SCK_PIN = 11 # Clock pin (example GPIO pin)
MISO_PIN = 9 # MISO pin (example GPIO pin)
MOSI_PIN = 10 # MOSI pin (example GPIO pin)

# Initialize the GPIO library
GPIO.setmode(GPIO.BCM)  # Use Broadcom pin numbering

# Setup the GPIO pins for SPI
GPIO.setup(CS_PIN, GPIO.OUT)   # Chip Select as output
GPIO.setup(SCK_PIN, GPIO.OUT)  # Clock as output
GPIO.setup(MISO_PIN, GPIO.IN)  # MISO as input
GPIO.setup(MOSI_PIN, GPIO.OUT) # MOSI as output

# Function to map GPIO pins for LDC1101
def hal_gpioMap():
    # Set default values or configurations
    GPIO.output(CS_PIN, GPIO.HIGH)  # Default state for CS pin
    GPIO.output(SCK_PIN, GPIO.LOW)  # Default state for Clock pin

def hal_gpio_csSet(value):
    if value == 1:
        GPIO.output(CS_PIN, GPIO.HIGH)  # CS high (deselect)
    elif value == 0:
        GPIO.output(CS_PIN, GPIO.LOW)   # CS low (select)
    else:
        print("Invalid value for CS (must be 0 or 1)")



# @ Register

# Global variables to simulate SPI register map and current read address for SPI simulation.
register_map = {
    0x01: 0x07,  # RP_SET (Configure RP Measurement Dynamic Range)
    0x02: 0x90,  # TC1 (Configure Internal Time Constant 1)
    0x03: 0xA0,  # TC2 (Configure Internal Time Constant 2)
    0x04: 0x03,  # DIG_CONFIG (Configure RP+L conversion interval)
    0x05: 0x00,  # ALT_CONFIG (Configure additional device settings)
    0x06: 0x00,  # RP_THRESH_H_LSB (RP_THRESHOLD High Setting – bits 7:0)
    0x07: 0x00,  # RP_THRESH_H_MSB (RP_THRESHOLD High Setting – bits 15:8)
    0x08: 0x00,  # RP_THRESH_L_LSB (RP_THRESHOLD Low Setting – bits 7:0)
    0x09: 0x00,  # RP_THRESH_L_MSB (RP_THRESHOLD Low Setting – bits 15:8)
    0x0A: 0x00,  # INTB_MODE (Configure INTB reporting on SDO pin)
    0x0B: 0x01,  # START_CONFIG (Configure Power State)
    0x0C: 0x00,  # D_CONF (Sensor Amplitude Control Requirement)
    0x16: 0x00,  # L_THRESH_HI_LSB (L_THRESHOLD High Setting – bits 7:0)
    0x17: 0x00,  # L_THRESH_HI_MSB (L_THRESHOLD High Setting – bits 15:8)
    0x18: 0x00,  # L_THRESH_LO_LSB (L_THRESHOLD Low Setting – bits 7:0)
    0x19: 0x00,  # L_THRESH_LO_MSB (L_THRESHOLD Low Setting – bits 15:8)
    0x20: 0x00,  # STATUS (Report RP+L measurement status)
    0x21: 0x00,  # RP_DATA_LSB (RP Conversion Result Data Output - bits 7:0)
    0x22: 0x00,  # RP_DATA_MSB (RP Conversion Result Data Output - bits 15:8)
    0x23: 0x00,  # L_DATA_LSB (L Conversion Result Data Output - bits 7:0)
    0x24: 0x00,  # L_DATA_MSB (L Conversion Result Data Output - bits 15:8)
    0x30: 0x00,  # LHR_RCOUNT_LSB (High Resolution L Reference Count – bits 7:0)
    0x31: 0x00,  # LHR_RCOUNT_MSB (High Resolution L Reference Count – bits 15:8)
    0x32: 0x00,  # LHR_OFFSET_LSB (High Resolution L Offset – bits 7:0)
    0x33: 0x00,  # LHR_OFFSET_MSB (High Resolution L Offset – bits 15:8)
    0x34: 0x00,  # LHR_CONFIG (High Resolution L Configuration)
    0x38: 0x00,  # LHR_DATA_LSB (High Resolution L Conversion Result Data output - bits 7:0)
    0x39: 0x00,  # LHR_DATA_MID (High Resolution L Conversion Result Data output - bits 15:8)
    0x3A: 0x00,  # LHR_DATA_MSB (High Resolution L Conversion Result Data output - bits 23:16)
    0x3B: 0x00,  # LHR_STATUS (High Resolution L Measurement Status)
    0x3E: 0x02,  # RID (Device RID value)
    0x3F: 0xD4   # CHIP_ID (Device ID value)
}

_current_read_addr = None

def hal_spiWrite(writeData, count):
    global register_map, _current_read_addr
    # In this simulation, if count equals 2 then we assume a register write;
    # if count equals 1, then we assume a register address is being sent for a read.
    if count == 2:
        # writeData[0] is the register address, writeData[1] is the data to write.
        register_map[writeData[0]] = writeData[1]
    elif count == 1:
        # For a read, store the address (mask off the MSB).
        _current_read_addr = writeData[0] & 0x7F
    # Otherwise do nothing.

def hal_spiRead(readBuffer, count):
    global register_map, _current_read_addr
    # In this simulation, read 'count' bytes from the register map based on _current_read_addr.
    if count == 1:
        # If reading the chip ID register, simulate the chip id 0xD4.
        if _current_read_addr == _LDC1101_REG_CHIP_ID:
            value = 0xD4
        else:
            value = register_map.get(_current_read_addr, 0)
        readBuffer[0] = value
    # Reset current read address after reading.
    _current_read_addr = None

def hal_gpio_intGet():
    # Simulate reading the GPIO interrupt.
    # Return 0 to indicate no interrupt.
    return 0

def Delay_100ms():
    # Delay for 100 milliseconds.
    time.sleep(0.1)

# Register
_LDC1101_REG_CFG_RP_MEASUREMENT_DYNAMIC_RANGE = 0x01
_LDC1101_REG_CFG_INTERNAL_TIME_CONSTANT_1 = 0x02
_LDC1101_REG_CFG_INTERNAL_TIME_CONSTANT_2 = 0x03
_LDC1101_REG_CFG_RP_L_CONVERSION_INTERVAL = 0x04
_LDC1101_REG_CFG_ADDITIONAL_DEVICE = 0x05
_LDC1101_REG_RP_THRESH_H_LSB = 0x06
_LDC1101_REG_RP_THRESH_H_MSB = 0x07
_LDC1101_REG_RP_THRESH_L_LSB = 0x08
_LDC1101_REG_RP_THRESH_L_MSB = 0x09
_LDC1101_REG_CFG_INTB_MODE = 0x0A
_LDC1101_REG_CFG_POWER_STATE = 0x0B
_LDC1101_REG_AMPLITUDE_CONTROL_REQUIREMENT = 0x0C
_LDC1101_REG_L_THRESH_HI_LSB = 0x16
_LDC1101_REG_L_THRESH_HI_MSB = 0x17
_LDC1101_REG_L_THRESH_LO_LSB = 0x18
_LDC1101_REG_L_THRESH_LO_MSB = 0x19
_LDC1101_REG_RP_L_MEASUREMENT_STATUS = 0x20
_LDC1101_REG_RP_DATA_LSB = 0x21
_LDC1101_REG_RP_DATA_MSB = 0x22
_LDC1101_REG_L_DATA_LSB = 0x23
_LDC1101_REG_L_DATA_MSB = 0x24
_LDC1101_REG_LHR_RCOUNT_LSB = 0x30
_LDC1101_REG_LHR_RCOUNT_MSB = 0x31
_LDC1101_REG_LHR_OFFSET_LSB = 0x32
_LDC1101_REG_LHR_OFFSET_MSB = 0x33
_LDC1101_REG_CFG_LHR = 0x34
_LDC1101_REG_LHR_DATA_LSB = 0x38
_LDC1101_REG_LHR_DATA_MID = 0x39
_LDC1101_REG_LHR_DATA_MSB = 0x3A
_LDC1101_REG_LHR_STATUS = 0x3B
_LDC1101_REG_DEVICE_RID_VALUE = 0x3E
_LDC1101_REG_CHIP_ID = 0x3F

# Register RP_SET Field Descriptions (RW)
_LDC1101_RP_SET_RP_MAX_IS_DRIVEN = 0x00
_LDC1101_RP_SET_RP_MAX_CURRENT_IS_IGNORED = 0x80
_LDC1101_RP_SET_RP_MAX_96KOhm = 0x00
_LDC1101_RP_SET_RP_MAX_48KOhm = 0x10
_LDC1101_RP_SET_RP_MAX_24KOhm = 0x20
_LDC1101_RP_SET_RP_MAX_12KOhm = 0x30
_LDC1101_RP_SET_RP_MAX_6KOhm = 0x40
_LDC1101_RP_SET_RP_MAX_3KOhm = 0x50
_LDC1101_RP_SET_RP_MAX_1_5KOhm = 0x60
_LDC1101_RP_SET_RP_MAX_0_75KOh = 0x70
_LDC1101_RP_SET_RP_MIN_96KOhm = 0x00
_LDC1101_RP_SET_RP_MIN_48KOhm = 0x01
_LDC1101_RP_SET_RP_MIN_24KOhm = 0x02
_LDC1101_RP_SET_RP_MIN_12KOhm = 0x03
_LDC1101_RP_SET_RP_MIN_6KOhm = 0x04
_LDC1101_RP_SET_RP_MIN_3KOhm = 0x05
_LDC1101_RP_SET_RP_MIN_1_5KOhm = 0x06
_LDC1101_RP_SET_RP_MIN_0_75KOh = 0x07

# Configure Internal Time Constant 1 (RW)
_LDC1101_TC1_C1_0_75pF = 0x00
_LDC1101_TC1_C1_1_5pF = 0x40
_LDC1101_TC1_C1_3pF = 0x80
_LDC1101_TC1_C1_6pF = 0xC0
_LDC1101_TC1_R1_417kOhm = 0x00
_LDC1101_TC1_R1_212_7kOhm = 0x10
_LDC1101_TC1_R1_21_1kOhm = 0x1F

# Configure Internal Time Constant 2 (RW)
_LDC1101_TC2_C2_3pF = 0x00
_LDC1101_TC2_C2_6pF = 0x40
_LDC1101_TC2_C2_12pF = 0x80
_LDC1101_TC2_C2_24pF = 0xC0
_LDC1101_TC2_R2_835kOhm = 0x00
_LDC1101_TC2_R2_426_4kOhm = 0x20
_LDC1101_TC2_R2_30_5kOhm = 0x3F

# Configure RP+L conversion interval (RW)
_LDC1101_DIG_CFG_MIN_FREQ_500kHz = 0x00
_LDC1101_DIG_CFG_MIN_FREQ_8MHz = 0xF0
_LDC1101_DIG_CFG_RESP_TIME_192s = 0x02
_LDC1101_DIG_CFG_RESP_TIME_384s = 0x03
_LDC1101_DIG_CFG_RESP_TIME_768s = 0x04
_LDC1101_DIG_CFG_RESP_TIME_1536s = 0x05
_LDC1101_DIG_CFG_RESP_TIME_3072s = 0x06
_LDC1101_DIG_CFG_RESP_TIME_6144s = 0x07

# Configure additional device settings (RW)
_LDC1101_ALT_CFG_SHUTDOWN_ENABLE = 0x02
_LDC1101_ALT_CFG_SHUTDOWN_DISABLE = 0x00
_LDC1101_ALT_CFG_L_OPTIMAL_DISABLED = 0x00
_LDC1101_ALT_CFG_L_OPTIMAL_ENABLE = 0x01

# Configure INTB reporting on SDO pin (RW)
_LDC1101_INTB_MODE_DONT_REPORT_INTB_ON_SDO_PIN = 0x00
_LDC1101_INTB_MODE_REPORT_INTB_ON_SDO_PIN = 0x80
_LDC1101_INTB_MODE_REPORT_LHR_DATA_READY = 0x20
_LDC1101_INTB_MODE_L_CONVERSION_TO_L_THRESHOLDS = 0x10
_LDC1101_INTB_MODE_L_CONVERSION_TO_L_HIGH_THRESHOLDS = 0x08
_LDC1101_INTB_MODE_REPORT_RP_L_DATA_READY = 0x04
_LDC1101_INTB_MODE_RP_CONVERSION_TO_L_THRESHOLDS = 0x02
_LDC1101_INTB_MODE_RP_CONVERSION_TO_L_HIGH_THRESHOLDS = 0x01
_LDC1101_INTB_MODE_NO_OUTPUT = 0x00

# Configure Power State (RW)
_LDC1101_FUNC_MODE_ACTIVE_CONVERSION_MODE = 0x00
_LDC1101_FUNC_MODE_SLEEP_MODE = 0x01
_LDC1101_FUNC_MODE_SHUTDOWN_MODE = 0x02

# High Resolution L Configuration (RW)
_LDC1101_LHR_CFG_FREQUENCY_NOT_DIVIDED = 0x00
_LDC1101_LHR_CFG_FREQUENCY_DIVIDED_BY_2 = 0x01
_LDC1101_LHR_CFG_FREQUENCY_DIVIDED_BY_4 = 0x02
_LDC1101_LHR_CFG_FREQUENCY_DIVIDED_BY_8 = 0x03

# Device status indicators
DEVICE_ERROR = 0x01
DEVICE_OK = 0x00

# ----------------------------------------------------------- IMPLEMENTATION
def ldc1101_writeByte(addr, _data):
    writeReg = [0] * 2

    writeReg[0] = addr
    writeReg[1] = _data

    hal_gpio_csSet(0)
    hal_spiWrite(writeReg, 2)
    hal_gpio_csSet(1)


def ldc1101_readByte(addr):
    writeReg = [0] * 2
    readReg = [0] * 2

    writeReg[0] = (0x80 | addr)

    hal_gpio_csSet(0)
    hal_spiWrite(writeReg, 1)
    hal_spiRead(readReg, 1)
    hal_gpio_csSet(1)

    return readReg[0]


def ldc1101_init():
    chip_id = 0

    chip_id = ldc1101_readByte(_LDC1101_REG_CHIP_ID)
    if chip_id != 0xD4:
        return DEVICE_ERROR

    # Default Init
    ldc1101_writeByte(_LDC1101_REG_CFG_RP_MEASUREMENT_DYNAMIC_RANGE, 0x07)
    ldc1101_writeByte(_LDC1101_REG_CFG_INTERNAL_TIME_CONSTANT_1, 0x90)
    ldc1101_writeByte(_LDC1101_REG_CFG_INTERNAL_TIME_CONSTANT_2, 0xA0)
    ldc1101_writeByte(_LDC1101_REG_CFG_RP_L_CONVERSION_INTERVAL, 0x03)
    ldc1101_writeByte(_LDC1101_REG_CFG_ADDITIONAL_DEVICE, 0x00)  # 0x01
    ldc1101_writeByte(_LDC1101_REG_RP_THRESH_H_MSB, 0x00)
    ldc1101_writeByte(_LDC1101_REG_RP_THRESH_L_LSB, 0x00)
    ldc1101_writeByte(_LDC1101_REG_RP_THRESH_L_MSB, 0x00)
    ldc1101_writeByte(_LDC1101_REG_CFG_INTB_MODE, 0x00)
    ldc1101_writeByte(_LDC1101_REG_CFG_POWER_STATE, _LDC1101_FUNC_MODE_SLEEP_MODE)
    ldc1101_writeByte(_LDC1101_REG_AMPLITUDE_CONTROL_REQUIREMENT, 0x00)  # 0x01
    ldc1101_writeByte(_LDC1101_REG_L_THRESH_HI_LSB, 0x00)
    ldc1101_writeByte(_LDC1101_REG_L_THRESH_HI_MSB, 0x00)
    ldc1101_writeByte(_LDC1101_REG_L_THRESH_LO_LSB, 0x00)
    ldc1101_writeByte(_LDC1101_REG_L_THRESH_LO_MSB, 0x00)
    ldc1101_writeByte(_LDC1101_REG_LHR_RCOUNT_LSB, 0x00)
    ldc1101_writeByte(_LDC1101_REG_LHR_RCOUNT_MSB, 0x00)
    ldc1101_writeByte(_LDC1101_REG_LHR_OFFSET_LSB, 0x00)
    ldc1101_writeByte(_LDC1101_REG_LHR_OFFSET_MSB, 0x00)
    ldc1101_writeByte(_LDC1101_REG_CFG_LHR, 0x00)
    Delay_100ms()

    return DEVICE_OK


def ldc1101_setPowerMode(mode):
    ldc1101_writeByte(_LDC1101_REG_CFG_POWER_STATE, mode)


def ldc1101_goTo_Lmode():
    ldc1101_writeByte(_LDC1101_REG_CFG_ADDITIONAL_DEVICE, 0x01)
    ldc1101_writeByte(_LDC1101_REG_AMPLITUDE_CONTROL_REQUIREMENT, 0x01)


def ldc1101_goTo_RPmode():
    ldc1101_writeByte(_LDC1101_REG_CFG_ADDITIONAL_DEVICE, 0x02)
    ldc1101_writeByte(_LDC1101_REG_AMPLITUDE_CONTROL_REQUIREMENT, 0x00)


def ldc1101_getStatus():
    status = 0

    status = ldc1101_readByte(0x20)
    return status


def ldc1101_getRPData():
    _data = 0

    _data = ldc1101_readByte(0x22)
    _data = _data << 8
    _data = _data | ldc1101_readByte(0x21)

    return _data


def ldc1101_getLData():
    _data = 0

    _data = ldc1101_readByte(0x24)
    _data = _data << 8
    _data = _data | ldc1101_readByte(0x23)

    return _data


def ldc1101_getInterrupt():
    return hal_gpio_intGet()

def main():
    hal_spiMap(spi)
    hal_gpioMap()
    hal_gpio_csSet(1)
    ldc1101_init()
    ldc1101_setPowerMode(_LDC1101_FUNC_MODE_ACTIVE_CONVERSION_MODE)
    ldc1101_goTo_Lmode()
    ldc1101_getLData()

# Run the main function
if __name__ == '__main__':
    main()

    # Cleanup
    spi.close()
    GPIO.cleanup()




