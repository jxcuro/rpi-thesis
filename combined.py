# Combined Code: Camera, Magnetism (ADS1115/Hall), and Inductance (LDC1101) Measurement
# VERSION WITH ENHANCED LDC ERROR HANDLING AND MOVING AVERAGE

import tkinter as tk
import cv2
from PIL import Image, ImageTk
import time
import os
import uuid
from datetime import datetime
import traceback # For detailed error printing
from collections import deque # For moving average

# --- I2C/ADS1115 Imports ---
import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

# --- SPI/LDC1101 Imports ---
import spidev
import RPi.GPIO as GPIO

# ==================================
# === Constants and Configuration ===
# ==================================

# --- Camera ---
CAMERA_INDEX = 0

# --- Hall Sensor (ADS1115) ---
HALL_ADC_CHANNEL = ADS.P0
SENSITIVITY_V_PER_TESLA = 0.0004
SENSITIVITY_V_PER_MILLITESLA = SENSITIVITY_V_PER_TESLA * 1000
IDLE_VOLTAGE = 1.7256

# --- Inductive Sensor (LDC1101) ---
SPI_BUS = 0
SPI_DEVICE = 0
SPI_SPEED = 500000
SPI_MODE = 0b00
CS_PIN = 8

# LDC1101 Registers (Add STATUS_REG if not already present)
STATUS_REG = 0x20
# ... (keep other register definitions)
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
# STATUS_REG = 0x20 # Already defined above
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

# LDC1101 Device Status
DEVICE_ERROR = 0x01
DEVICE_OK = 0x00

# LDC1101 Power States
ACTIVE_CONVERSION_MODE = 0x00
SLEEP_MODE = 0x01
SHUTDOWN_MODE = 0x02

# --- Averaging Filter ---
LDC_AVG_WINDOW_SIZE = 10 # Number of readings to average
ldc_readings_buffer = deque(maxlen=LDC_AVG_WINDOW_SIZE)
last_known_good_rp = 0 # To display if errors occur

# --- Calibration ---
IDLE_RP_VALUE = 0

# --- File Saving ---
SAVE_PATH = os.path.join(os.path.expanduser('~'), "Pictures", "Thesis")

# =========================
# === Hardware Setup ===
# =========================
# (Keep the hardware setup section largely the same, ensuring print statements for init steps)
print("Initializing Hardware...")
# --- Camera ---
camera = None
try:
    camera = cv2.VideoCapture(CAMERA_INDEX)
    if not camera.isOpened():
        print(f"Error: Could not open camera at index {CAMERA_INDEX}")
        camera = None
    else:
        print("Camera Initialized.")
except Exception as e:
    print(f"Error initializing Camera: {e}")
    camera = None

# --- I2C/ADS1115 ---
hall_sensor = None
try:
    i2c = busio.I2C(board.SCL, board.SDA)
    ads = ADS.ADS1115(i2c)
    hall_sensor = AnalogIn(ads, HALL_ADC_CHANNEL)
    print("ADS1115 Initialized.")
except Exception as e:
    print(f"Error initializing I2C/ADS1115: {e}")
    hall_sensor = None

# --- SPI/GPIO/LDC1101 ---
spi = None
ldc_initialized = False
try:
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(CS_PIN, GPIO.OUT)
    GPIO.output(CS_PIN, GPIO.HIGH)
    print("GPIO Initialized.")

    spi = spidev.SpiDev()
    spi.open(SPI_BUS, SPI_DEVICE)
    spi.max_speed_hz = SPI_SPEED
    spi.mode = SPI_MODE
    print(f"SPI Initialized (Bus={SPI_BUS}, Device={SPI_DEVICE}, Speed={SPI_SPEED}Hz).")
except Exception as e:
    print(f"Error initializing GPIO/SPI: {e}")
    spi = None

# =========================
# === LDC1101 Functions ===
# =========================

def ldc_write_register(reg_addr, value):
    """Writes a byte value to an LDC1101 register via SPI with error handling."""
    if not spi: return False
    try:
        GPIO.output(CS_PIN, GPIO.LOW)
        spi.xfer2([reg_addr & 0x7F, value])
        GPIO.output(CS_PIN, GPIO.HIGH)
        return True
    except Exception as e:
        print(f"ERROR: LDC Write Failed! Reg=0x{reg_addr:02X}, Val=0x{value:02X}. Error: {e}")
        # traceback.print_exc() # Uncomment for full error details if needed
        try: # Attempt to deselect chip even on error
            GPIO.output(CS_PIN, GPIO.HIGH)
        except Exception: pass
        return False

def ldc_read_register(reg_addr):
    """Reads a byte value from an LDC1101 register via SPI with error handling."""
    if not spi: return None # Return None on SPI init failure
    try:
        GPIO.output(CS_PIN, GPIO.LOW)
        result = spi.xfer2([reg_addr | 0x80, 0x00])
        GPIO.output(CS_PIN, GPIO.HIGH)
        return result[1]
    except Exception as e:
        print(f"ERROR: LDC Read Failed! Reg=0x{reg_addr:02X}. Error: {e}")
        # traceback.print_exc() # Uncomment for full error details if needed
        try: # Attempt to deselect chip even on error
            GPIO.output(CS_PIN, GPIO.HIGH)
        except Exception: pass
        return None # Return None on read failure

def initialize_ldc1101():
    """Initializes the LDC1101 with error checks."""
    global ldc_initialized
    if not spi:
        print("SPI not available, skipping LDC1101 initialization.")
        return DEVICE_ERROR

    print("Checking LDC1101 Chip ID...")
    chip_id = ldc_read_register(CHIP_ID_REG)
    if chip_id is None: # Check if read failed
         print("ERROR: Failed to read LDC1101 Chip ID.")
         return DEVICE_ERROR

    print(f"LDC1101 Chip ID Read: 0x{chip_id:02X} (Expected: 0xD4)")
    if chip_id != 0xD4:
        print("ERROR: Invalid LDC1101 Chip ID.")
        return DEVICE_ERROR

    print("Configuring LDC1101 Registers for RP Mode...")
    # Simplified config sequence with checks
    config_ok = True
    config_ok &= ldc_write_register(RP_SET_REG, 0x1B)
    config_ok &= ldc_write_register(TC1_REG, 0x80)
    config_ok &= ldc_write_register(TC2_REG, 0x88)
    config_ok &= ldc_write_register(DIG_CONFIG_REG, 0x07) # Longest conversion time
    config_ok &= ldc_write_register(ALT_CONFIG_REG, 0x02) # Set to RP Mode initially
    config_ok &= ldc_write_register(D_CONF_REG, 0x00)
    config_ok &= ldc_write_register(INTB_MODE_REG, 0x00)
    # ... (add other registers if critical, checking config_ok)

    if not config_ok:
        print("ERROR: Failed to write one or more LDC1101 configurations.")
        return DEVICE_ERROR

    # Start in Sleep Mode initially
    if not ldc_write_register(START_CONFIG_REG, SLEEP_MODE):
         print("ERROR: Failed to set LDC1101 to Sleep Mode.")
         return DEVICE_ERROR

    time.sleep(0.05)
    print("LDC1101 Configuration Complete.")
    ldc_initialized = True
    return DEVICE_OK

def enable_ldc_powermode(mode):
    """Sets the power mode of the LDC1101."""
    print(f"Setting LDC1101 Power Mode to: {mode}")
    if not ldc_write_register(START_CONFIG_REG, mode):
        print(f"ERROR: Failed to set LDC power mode to {mode}")
    time.sleep(0.02)

def enable_ldc_rpmode():
    """Configures and enables RP conversion mode."""
    print("Enabling LDC1101 RP Mode...")
    ok = True
    ok &= ldc_write_register(ALT_CONFIG_REG, 0x02) # Select RP Mode
    ok &= ldc_write_register(D_CONF_REG, 0x00)    # RP specific setting
    if ok:
        enable_ldc_powermode(ACTIVE_CONVERSION_MODE) # Start conversions
        print("LDC1101 RP Mode Active.")
    else:
        print("ERROR: Failed to configure LDC for RP Mode.")


def get_ldc_rpdata():
    """Reads the 16-bit RP data from the LDC1101 with error checking."""
    global last_known_good_rp
    if not spi or not ldc_initialized: return None

    # Optional: Read status register first
    # status = ldc_read_register(STATUS_REG)
    # if status is not None and status != 0: # Check datasheet for error bits
    #     print(f"Warning: LDC Status Register = 0x{status:02X}")
        # Handle specific status flags if needed

    msb = ldc_read_register(RP_DATA_MSB_REG)
    lsb = ldc_read_register(RP_DATA_LSB_REG)

    if msb is None or lsb is None:
        print("ERROR: Failed to read full RP data.")
        return None # Indicate read failure

    value = (msb << 8) | lsb
    last_known_good_rp = value # Store the latest good value
    return value

# ======================
# === GUI Functions ===
# ======================
# (capture_photo, calibrate_sensors, update_camera_feed, update_magnetism functions remain largely the same,
# but ensure they also handle cases where their respective sensors (`camera`, `hall_sensor`) are None)

# --- Modify calibrate_sensors to use the robust get_ldc_rpdata ---
def calibrate_sensors():
    """Calibrates idle voltage for Hall sensor and idle RP value for LDC1101."""
    global IDLE_VOLTAGE, IDLE_RP_VALUE
    feedback_text = ""
    feedback_color = "blue"

    # Calibrate Hall Sensor
    if hall_sensor:
        try:
            # Maybe average a few readings?
            voltages = [hall_sensor.voltage for _ in range(5)]
            time.sleep(0.05)
            IDLE_VOLTAGE = sum(voltages) / len(voltages)
            feedback_text += f"Cal. Hall Idle: {IDLE_VOLTAGE:.4f} V\n"
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
            # Read a few values and average for calibration
            rp_readings = []
            for _ in range(10): # Take 10 readings for calibration average
                val = get_ldc_rpdata()
                if val is not None:
                    rp_readings.append(val)
                time.sleep(0.02) # Short delay between reads

            if rp_readings:
                IDLE_RP_VALUE = int(sum(rp_readings) / len(rp_readings))
                feedback_text += f"Cal. LDC RP Idle: {IDLE_RP_VALUE}"
                ldc_readings_buffer.clear() # Clear buffer after calibration
                ldc_readings_buffer.extend(rp_readings) # Prime buffer
            else:
                 feedback_text += "LDC Cal Read Error"
                 feedback_color = "red" if feedback_color != "orange" else "orange"

        except Exception as e:
            feedback_text += f"LDC Cal Error: {e}"
            feedback_color = "red" if feedback_color != "orange" else "orange"
            print(f"Error calibrating LDC1101: {e}")
            # traceback.print_exc()
    else:
        feedback_text += "LDC Sensor N/A"
        feedback_color = "red" if feedback_color != "orange" else "orange"


    feedback_label.config(text=feedback_text.strip(), fg=feedback_color)
    window.after(5000, lambda: feedback_label.config(text="")) # Clear feedback


# --- Modify update_ldc_reading for averaging and error handling ---
def update_ldc_reading():
    """Updates the LDC1101 RP value label using a moving average and handles errors."""
    global ldc_readings_buffer # Use global buffer

    current_rp_value = None
    error_occurred = False

    if ldc_initialized:
        try:
            # Attempt to get a new reading
            current_rp_value = get_ldc_rpdata()

            if current_rp_value is not None:
                # Add to buffer if reading was successful
                ldc_readings_buffer.append(current_rp_value)
            else:
                # Read failed, don't add to buffer
                error_occurred = True
                print("LDC read returned None, skipping average update.")

        except Exception as e:
            error_occurred = True
            print(f"ERROR during LDC update/read: {e}")
            # traceback.print_exc() # Uncomment for full details

    else:
        # LDC not initialized, display N/A
        ldc_label.config(text="LDC RP Value: N/A")
        # Reschedule the update check
        window.after(200, update_ldc_reading) # Check again after 200ms
        return

    # Calculate average if buffer has readings
    display_value = "---"
    if ldc_readings_buffer:
        average_rp = int(sum(ldc_readings_buffer) / len(ldc_readings_buffer))
        display_value = f"{average_rp}"
    elif error_occurred:
        display_value = f"Err (Last: {last_known_good_rp})" # Show last good value on error
    else:
        display_value = "Waiting..." # If buffer is empty initially

    # Update Label
    status_text = ""
    if error_occurred and current_rp_value is None: status_text = " (Read Error!)"
    elif len(ldc_readings_buffer) < LDC_AVG_WINDOW_SIZE: status_text = f" (Avg {len(ldc_readings_buffer)}/{LDC_AVG_WINDOW_SIZE})"

    ldc_label.config(text=f"LDC RP Value: {display_value}{status_text}")


    # Crucial: Reschedule the function call even if errors occurred (unless fatal)
    # Use a slightly longer interval to reduce load and potentially noise
    window.after(150, update_ldc_reading) # Update ~6-7 times per second


# ======================
# === GUI Setup ===
# ======================
# (GUI Setup remains the same)
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
feedback_label = tk.Label(controls_frame, text="", fg="green", justify=tk.LEFT, font=("Helvetica", 12), wraplength=250) # Allow wrapping
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
            ldc_label.config(text="LDC RP Value: Failed Init")

    # Start the update loops for GUI elements
    if camera: update_camera_feed()
    if hall_sensor: update_magnetism()
    update_ldc_reading() # Start LDC updates regardless of init status (it handles N/A)

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
        try:
            spi.close()
        except Exception as e:
            print(f"Error closing SPI: {e}")
    print("Cleaning up GPIO...")
    try:
        GPIO.cleanup() # Important: Use try-except as cleanup might fail if pins weren't set up
    except Exception as e:
        print(f"Error during GPIO cleanup: {e}")
    print("Cleanup complete.")

# --- Run with Cleanup ---
if __name__ == '__main__':
    try:
        run_application()
    except Exception as e:
        print(f"An unexpected FATAL error occurred in the main application thread:")
        traceback.print_exc() # Print full traceback for fatal errors
    finally:
        cleanup_resources()
