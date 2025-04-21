import tkinter as tk
import cv2
from PIL import Image, ImageTk
import time
import os
import uuid
from datetime import datetime

import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

import spidev
import RPi.GPIO as GPIO

# ==================================
# === Constants and Configuration ===
# ==================================

# --- Camera ---
CAMERA_INDEX = 0 # Usually 0 for default USB webcam

# --- Hall Sensor (ADS1115) ---
HALL_ADC_CHANNEL = ADS.P0
SENSITIVITY_V_PER_TESLA = 0.0004
SENSITIVITY_V_PER_MILLITESLA = SENSITIVITY_V_PER_TESLA * 1000
IDLE_VOLTAGE = 1.7256  # Initial guess, will be updated by calibration

# --- Inductive Sensor (LDC1101) ---
# SPI settings
SPI_BUS = 0
SPI_DEVICE = 0
SPI_SPEED = 500000  # Increased speed (500kHz) - adjust if needed
SPI_MODE = 0b00     # SPI mode (CPOL = 0, CPHA = 0)

# LDC1101 GPIO Pins (Using BCM numbering)
CS_PIN = 8   # Chip Select pin for LDC1101
# SCK, MISO, MOSI usually handled by spidev, but define if needed elsewhere
# SCK_PIN = 11
# MISO_PIN = 9
# MOSI_PIN = 10

# LDC1101 Register Addresses
START_CONFIG_REG = 0x0B
RP_SET_REG = 0x01
TC1_REG = 0x02
TC2_REG = 0x03
DIG_CONFIG_REG = 0x04
ALT_CONFIG_REG = 0x05
RP_THRESH_H_LSB_REG = 0x06
RP_THRESH_H_MSB_REG = 0x07
RP_THRESH_L_LSB_REG = 0x08
RP_THRESH_L_MSB_REG = 0x09
INTB_MODE_REG = 0x0A
D_CONF_REG = 0x0C
L_THRESH_HI_LSB_REG = 0x16
L_THRESH_HI_MSB_REG = 0x17
L_THRESH_LO_LSB_REG = 0x18
L_THRESH_LO_MSB_REG = 0x19
STATUS_REG = 0x20
RP_DATA_LSB_REG = 0x21
RP_DATA_MSB_REG = 0x22
L_DATA_LSB_REG = 0x23
L_DATA_MSB_REG = 0x24
LHR_RCOUNT_LSB_REG = 0x30
LHR_RCOUNT_MSB_REG = 0x31
LHR_OFFSET_LSB_REG = 0x32
LHR_OFFSET_MSB_REG = 0x33
LHR_CONFIG_REG = 0x34
LHR_DATA_LSB_REG = 0x38
LHR_DATA_MID_REG = 0x39
LHR_DATA_MSB_REG = 0x3A
LHR_STATUS_REG = 0x3B
RID_REG = 0x3E
CHIP_ID_REG = 0x3F

# LDC1101 Device Status Indicators
DEVICE_ERROR = 0x01
DEVICE_OK = 0x00

# LDC1101 Power States
ACTIVE_CONVERSION_MODE = 0x00
SLEEP_MODE = 0x01
SHUTDOWN_MODE = 0x02

# --- Calibration ---
IDLE_RP_VALUE = 0 # Initial guess, updated by calibration

# --- File Saving ---
SAVE_PATH = os.path.join(os.path.expanduser('~'), "Pictures", "Thesis")

# =========================
# === Hardware Setup ===
# =========================

# --- Initialize Camera ---
print("Initializing Camera...")
camera = cv2.VideoCapture(CAMERA_INDEX)
if not camera.isOpened():
    print(f"Error: Could not open camera at index {CAMERA_INDEX}")
    # Handle error appropriately, maybe exit or disable camera features
    camera = None # Indicate camera failed

# --- Initialize I2C and ADS1115 (Hall Sensor ADC) ---
print("Initializing I2C and ADS1115...")
try:
    i2c = busio.I2C(board.SCL, board.SDA)
    ads = ADS.ADS1115(i2c)
    hall_sensor = AnalogIn(ads, HALL_ADC_CHANNEL)
    print("ADS1115 Initialized.")
except Exception as e:
    print(f"Error initializing I2C/ADS1115: {e}")
    hall_sensor = None # Indicate ADS/Hall sensor failed

# --- Initialize SPI, GPIO, and LDC1101 (Inductive Sensor) ---
print("Initializing GPIO, SPI and LDC1101...")
spi = None
ldc_initialized = False
try:
    # Initialize GPIO
    GPIO.setmode(GPIO.BCM)  # Use Broadcom pin numbering
    GPIO.setup(CS_PIN, GPIO.OUT)
    GPIO.output(CS_PIN, GPIO.HIGH) # Deselect LDC1101 initially
    print("GPIO Initialized.")

    # Initialize SPI
    spi = spidev.SpiDev()
    spi.open(SPI_BUS, SPI_DEVICE)
    spi.max_speed_hz = SPI_SPEED
    spi.mode = SPI_MODE
    print(f"SPI Initialized (Bus={SPI_BUS}, Device={SPI_DEVICE}, Speed={SPI_SPEED}Hz).")

except Exception as e:
    print(f"Error initializing GPIO/SPI: {e}")
    spi = None # Indicate SPI failed


# =========================
# === LDC1101 Functions ===
# =========================

def ldc_write_register(reg_addr, value):
    """Writes a byte value to an LDC1101 register via SPI."""
    if not spi: return # Don't try if SPI init failed
    try:
        GPIO.output(CS_PIN, GPIO.LOW)  # Select chip
        # time.sleep(0.001) # Short delay if required by hardware, often not needed
        spi.xfer2([reg_addr & 0x7F, value]) # Send write command (MSB=0) + data
        # time.sleep(0.001)
        GPIO.output(CS_PIN, GPIO.HIGH) # Deselect chip
    except Exception as e:
        print(f"Error writing to LDC register 0x{reg_addr:02X}: {e}")

def ldc_read_register(reg_addr):
    """Reads a byte value from an LDC1101 register via SPI."""
    if not spi: return 0 # Return default value if SPI init failed
    try:
        GPIO.output(CS_PIN, GPIO.LOW)  # Select chip
        # time.sleep(0.001)
        # Send read command (MSB=1) + dummy byte to clock out data
        result = spi.xfer2([reg_addr | 0x80, 0x00])
        # time.sleep(0.001)
        GPIO.output(CS_PIN, GPIO.HIGH) # Deselect chip
        return result[1]  # Return the second byte received (the data)
    except Exception as e:
        print(f"Error reading LDC register 0x{reg_addr:02X}: {e}")
        return 0 # Return default value on error

def initialize_ldc1101():
    """Initializes the LDC1101 with custom settings for RP mode."""
    global ldc_initialized
    if not spi:
        print("SPI not available, skipping LDC1101 initialization.")
        return DEVICE_ERROR

    print("Checking LDC1101 Chip ID...")
    chip_id = ldc_read_register(CHIP_ID_REG)
    print(f"LDC1101 Chip ID Read: 0x{chip_id:02X} (Expected: 0xD4)")
    if chip_id != 0xD4:
        print("Error: Invalid LDC1101 Chip ID.")
        return DEVICE_ERROR

    print("Configuring LDC1101 Registers for RP Mode...")
    # Custom Init for High Sensitivity Metal Detection in RP Mode (adjust as needed)
    ldc_write_register(RP_SET_REG, 0x1B)       # RP_MIN = 12kΩ, RP_MAX = 24kΩ
    ldc_write_register(TC1_REG, 0x80)          # TC1: R1 = ~264kΩ, C1 = 3pF
    ldc_write_register(TC2_REG, 0x88)          # TC2: R2 = ~375kΩ, C2 = 12pF
    ldc_write_register(DIG_CONFIG_REG, 0x07)   # Longest conversion time (RESP_TIME = 6144)
    ldc_write_register(ALT_CONFIG_REG, 0x02)   # Set to RP Mode initially
    ldc_write_register(D_CONF_REG, 0x00)       # RP Mode specific config
    ldc_write_register(INTB_MODE_REG, 0x00)    # Disable interrupt pin reporting

    # Clear thresholds (optional, good practice)
    ldc_write_register(RP_THRESH_H_LSB_REG, 0x00)
    ldc_write_register(RP_THRESH_H_MSB_REG, 0x00)
    ldc_write_register(RP_THRESH_L_LSB_REG, 0x00)
    ldc_write_register(RP_THRESH_L_MSB_REG, 0x00)
    ldc_write_register(L_THRESH_HI_LSB_REG, 0x00)
    ldc_write_register(L_THRESH_HI_MSB_REG, 0x00)
    ldc_write_register(L_THRESH_LO_LSB_REG, 0x00)
    ldc_write_register(L_THRESH_LO_MSB_REG, 0x00)

    # LHR Mode settings (clear even if not used)
    ldc_write_register(LHR_RCOUNT_LSB_REG, 0x00)
    ldc_write_register(LHR_RCOUNT_MSB_REG, 0x00)
    ldc_write_register(LHR_OFFSET_LSB_REG, 0x00)
    ldc_write_register(LHR_OFFSET_MSB_REG, 0x00)
    ldc_write_register(LHR_CONFIG_REG, 0x00)

    # Start in Sleep Mode initially
    ldc_write_register(START_CONFIG_REG, SLEEP_MODE)
    time.sleep(0.05) # Allow time for registers to settle

    print("LDC1101 Configuration Complete.")
    ldc_initialized = True
    return DEVICE_OK

def enable_ldc_powermode(mode):
    """Sets the power mode of the LDC1101."""
    print(f"Setting LDC1101 Power Mode to: {mode}")
    ldc_write_register(START_CONFIG_REG, mode)
    time.sleep(0.02) # Allow mode transition

def enable_ldc_rpmode():
    """Configures and enables RP conversion mode."""
    print("Enabling LDC1101 RP Mode...")
    ldc_write_register(ALT_CONFIG_REG, 0x02) # Select RP Mode
    ldc_write_register(D_CONF_REG, 0x00)    # RP specific setting
    enable_ldc_powermode(ACTIVE_CONVERSION_MODE) # Start conversions
    print("LDC1101 RP Mode Active.")

def get_ldc_rpdata():
    """Reads the 16-bit RP data from the LDC1101."""
    if not spi or not ldc_initialized: return 0
    msb = ldc_read_register(RP_DATA_MSB_REG)
    lsb = ldc_read_register(RP_DATA_LSB_REG)
    value = (msb << 8) | lsb
    return value

# ======================
# === GUI Functions ===
# ======================

def capture_photo():
    """Captures an image from the camera, reads magnetism, and saves it."""
    if not camera:
         feedback_label.config(text="Camera not available", fg="red")
         return

    capture_button.config(state=tk.DISABLED)
    feedback_label.config(text="Capturing photo...", fg="orange")
    window.update() # Update GUI immediately

    ret, frame = camera.read()
    if not ret:
        feedback_label.config(text="Failed to capture photo", fg="red")
        capture_button.config(state=tk.NORMAL)
        return

    # --- Process Image ---
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    img = Image.fromarray(frame_rgb)
    img_resized = img.resize((640, 480)) # Resize for consistency if needed

    # --- Get Magnetism Value ---
    magnetism_value_str = "N/A"
    unit = ""
    if hall_sensor:
        try:
            voltage = hall_sensor.voltage
            adjusted_voltage = voltage - IDLE_VOLTAGE
            magnetism_mT = adjusted_voltage / SENSITIVITY_V_PER_MILLITESLA

            if abs(magnetism_mT) < 1:
                magnetism_uT = magnetism_mT * 1000
                magnetism_value_str = f"{magnetism_uT:.2f}"
                unit = "uT"
            else:
                magnetism_value_str = f"{magnetism_mT:.2f}"
                unit = "mT"
        except Exception as e:
            print(f"Error reading Hall Sensor: {e}")
            magnetism_value_str = "ReadError"
            unit = ""
    else:
        magnetism_value_str = "NoSensor"
        unit = ""


    # --- Save Image ---
    os.makedirs(SAVE_PATH, exist_ok=True)
    unique_id = uuid.uuid4().hex[:8]
    # Include magnetism in filename
    file_name = f"mag_{magnetism_value_str}{unit}_id_{unique_id}.jpg"
    file_path = os.path.join(SAVE_PATH, file_name)

    try:
        img_resized.save(file_path)
        feedback_label.config(text=f"Photo Saved: {file_name}", fg="green")
    except Exception as e:
        feedback_label.config(text=f"Error saving photo: {e}", fg="red")
        print(f"Error saving photo to {file_path}: {e}")

    # Re-enable button after a delay
    window.after(2000, lambda: capture_button.config(state=tk.NORMAL))
    window.after(2000, lambda: feedback_label.config(text="")) # Clear feedback


def calibrate_sensors():
    """Calibrates idle voltage for Hall sensor and idle RP value for LDC1101."""
    global IDLE_VOLTAGE, IDLE_RP_VALUE
    feedback_text = ""
    feedback_color = "blue"

    # Calibrate Hall Sensor
    if hall_sensor:
        try:
            IDLE_VOLTAGE = hall_sensor.voltage
            feedback_text += f"Calibrated Hall Idle: {IDLE_VOLTAGE:.4f} V\n"
        except Exception as e:
            feedback_text += f"Hall Cal Error: {e}\n"
            feedback_color = "orange"
            print(f"Error calibrating Hall Sensor: {e}")
    else:
        feedback_text += "Hall Sensor N/A\n"
        feedback_color = "orange"


    # Calibrate LDC1101
    if ldc_initialized:
        try:
            # Take a few readings and average? Or just one? Taking one for simplicity.
            current_rp = get_ldc_rpdata()
            IDLE_RP_VALUE = current_rp
            feedback_text += f"Calibrated LDC RP Idle: {IDLE_RP_VALUE}"
        except Exception as e:
            feedback_text += f"LDC Cal Error: {e}"
            feedback_color = "red" if feedback_color != "orange" else "orange"
            print(f"Error calibrating LDC1101: {e}")
    else:
        feedback_text += "LDC Sensor N/A"
        feedback_color = "red" if feedback_color != "orange" else "orange"


    feedback_label.config(text=feedback_text.strip(), fg=feedback_color)
    # Optionally clear feedback after a delay
    window.after(4000, lambda: feedback_label.config(text=""))


def update_camera_feed():
    """Updates the camera feed label in the GUI."""
    if camera:
        ret, frame = camera.read()
        if ret:
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(frame_rgb)
            img_resized = img.resize((640, 480)) # Resize displayed image
            img_tk = ImageTk.PhotoImage(img_resized)
            camera_label.img_tk = img_tk # Keep reference
            camera_label.configure(image=img_tk)
        else:
            # Optionally display an error message on the label if frame read fails
            pass
    else:
         # Optionally display "Camera not available" on the label
         pass
    window.after(60, update_camera_feed) # Update approx 16 FPS


def update_magnetism():
    """Updates the magnetism reading label in the GUI."""
    if hall_sensor:
        try:
            voltage = hall_sensor.voltage
            adjusted_voltage = voltage - IDLE_VOLTAGE
            magnetism_mT = adjusted_voltage / SENSITIVITY_V_PER_MILLITESLA

            if abs(magnetism_mT) < 1:
                magnetism_uT = magnetism_mT * 1000
                magnetism_label.config(text=f"Magnetism: {magnetism_uT:.2f} µT")
            else:
                magnetism_label.config(text=f"Magnetism: {magnetism_mT:.2f} mT")
        except Exception as e:
             magnetism_label.config(text="Magnetism: Error")
             # print(f"Error reading Hall sensor voltage: {e}") # Avoid flooding console
    else:
        magnetism_label.config(text="Magnetism: N/A")

    window.after(100, update_magnetism) # Update magnetism less frequently than camera feed


def update_ldc_reading():
    """Updates the LDC1101 RP value label in the GUI."""
    if ldc_initialized:
        try:
            rp_val = get_ldc_rpdata()
            # Optionally display difference from idle: diff = rp_val - IDLE_RP_VALUE
            ldc_label.config(text=f"LDC RP Value: {rp_val}")
        except Exception as e:
            ldc_label.config(text="LDC RP Value: Error")
            # print(f"Error reading LDC RP data: {e}") # Avoid flooding console
    else:
        ldc_label.config(text="LDC RP Value: N/A")

    window.after(100, update_ldc_reading) # Update LDC reading at same rate as magnetism

# ======================
# === GUI Setup ===
# ======================
print("Setting up GUI...")
window = tk.Tk()
window.title("Camera Feed with Magnetism & Inductance Measurement")

# Main Frame
frame = tk.Frame(window)
frame.pack(padx=10, pady=10, fill='both', expand=True)

# Camera Feed Label (Left)
camera_label = tk.Label(frame)
camera_label.grid(row=0, column=0, padx=10, pady=10, rowspan=6) # Span more rows

# Controls Frame (Right)
controls_frame = tk.Frame(frame)
controls_frame.grid(row=0, column=1, padx=10, pady=10, sticky='nw') # Align top-west

# Feedback Label
feedback_label = tk.Label(controls_frame, text="", fg="green", justify=tk.LEFT, font=("Helvetica", 12))
feedback_label.grid(row=0, column=0, pady=(0, 10), sticky='w') # Align west

# Magnetism Label
magnetism_label = tk.Label(controls_frame, text="Magnetism: Initializing...", font=("Helvetica", 14))
magnetism_label.grid(row=1, column=0, pady=5, sticky='w')

# LDC1101 RP Label
ldc_label = tk.Label(controls_frame, text="LDC RP Value: Initializing...", font=("Helvetica", 14))
ldc_label.grid(row=2, column=0, pady=5, sticky='w') # Place below magnetism

# Capture Button
capture_button = tk.Button(controls_frame, text="Capture Photo", command=capture_photo, height=2, width=20,
                           font=("Helvetica", 14))
capture_button.grid(row=3, column=0, pady=15)

# Calibrate Button
calibrate_button = tk.Button(controls_frame, text="Calibrate Sensors", command=calibrate_sensors,
                             height=2, width=20, font=("Helvetica", 12))
calibrate_button.grid(row=4, column=0, pady=5)

# ==========================
# === Main Execution ===
# ==========================

def run_application():
    # Initialize LDC1101 (if SPI available)
    if spi:
        if initialize_ldc1101() == DEVICE_OK:
            enable_ldc_rpmode() # Put into active RP measurement mode
        else:
            feedback_label.config(text="LDC1101 Init Failed!", fg="red")
            # Update LDC label to show failure
            ldc_label.config(text="LDC RP Value: Failed Init")

    # Start the update loops for GUI elements
    update_camera_feed()
    update_magnetism()
    update_ldc_reading()

    print("Starting Tkinter main loop...")
    window.mainloop() # Blocks until window is closed

# --- Cleanup ---
def cleanup_resources():
    print("Cleaning up resources...")
    if camera and camera.isOpened():
        print("Releasing camera...")
        camera.release()
    cv2.destroyAllWindows()
    print("Closing SPI...")
    if spi:
        spi.close()
    print("Cleaning up GPIO...")
    GPIO.cleanup()
    print("Cleanup complete.")

# --- Run with Cleanup ---
if __name__ == '__main__':
    try:
        run_application()
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        cleanup_resources()
