# Combined Code: Camera, Magnetism, Inductance Measurement with Dataset Creation
# Version: Enhanced GUI and Accuracy

import tkinter as tk
from tkinter import ttk # For themed widgets
from tkinter import font as tkFont # For font control
import cv2
from PIL import Image, ImageTk
import time
import os
import uuid
import csv
from datetime import datetime
import statistics # For mean calculation if needed

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

# --- Accuracy ---
NUM_SAMPLES_PER_UPDATE = 3 # Number of reads to average per update cycle
NUM_SAMPLES_CALIBRATION = 7 # Number of reads to average for calibration

# --- Camera ---
CAMERA_INDEX = 0
IMG_WIDTH = 640
IMG_HEIGHT = 480

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
LDC_CHIP_ID = 0xD4
# Register Addresses
START_CONFIG_REG, RP_SET_REG, TC1_REG, TC2_REG, DIG_CONFIG_REG = 0x0B, 0x01, 0x02, 0x03, 0x04
ALT_CONFIG_REG, D_CONF_REG, INTB_MODE_REG = 0x05, 0x0C, 0x0A
RP_DATA_MSB_REG, RP_DATA_LSB_REG = 0x22, 0x21
CHIP_ID_REG = 0x3F
# Power States
ACTIVE_CONVERSION_MODE, SLEEP_MODE = 0x00, 0x01

# --- Calibration ---
IDLE_RP_VALUE = 0

# --- Dataset / File Saving ---
PROJECT_FOLDER_NAME = "Project_Dataset"
IMAGES_FOLDER_NAME = "images"
METADATA_FILENAME = "metadata.csv"
METADATA_HEADER = ['image_path', 'magnetism_mT', 'ldc_rp', 'target_material']
# Using script directory
try:
    BASE_PATH = os.path.dirname(os.path.abspath(__file__))
except NameError: # Handle if run interactively where __file__ is not defined
    BASE_PATH = os.getcwd()
PROJECT_PATH = os.path.join(BASE_PATH, PROJECT_FOLDER_NAME)
IMAGES_PATH = os.path.join(PROJECT_PATH, IMAGES_FOLDER_NAME)
METADATA_PATH = os.path.join(PROJECT_PATH, METADATA_FILENAME)

# --- Target Materials ---
TARGET_OPTIONS = [
    "Copper", "Steel", "Iron", "Aluminum", "Zinc",
    "Brass", "Gold", "Silver", "Bronze", "Others"
]

# --- Global Hardware Objects ---
camera = None
i2c = None
ads = None
hall_sensor = None
spi = None
ldc_initialized = False

# --- GUI Globals ---
window = None
camera_label = None
controls_frame = None
feedback_label = None
magnetism_label = None
ldc_label = None
target_var = None
capture_button = None
calibrate_button = None
# Font Objects
label_font = None
readout_font = None
button_font = None
feedback_font = None
title_font = None


# =========================
# === Directory Setup =====
# =========================
def setup_project_directory():
    """Creates the main project and images directories if they don't exist."""
    print(f"Ensuring dataset directory exists at: {PROJECT_PATH}")
    try:
        os.makedirs(PROJECT_PATH, exist_ok=True)
        os.makedirs(IMAGES_PATH, exist_ok=True)
        print("Dataset directories checked/created.")
        return True
    except OSError as e:
        print(f"Error creating directories: {e}")
        return False

# =========================
# === Hardware Setup ===
# =========================
def initialize_hardware():
    """Initializes Camera, I2C/ADS1115, SPI/GPIO/LDC1101."""
    global camera, i2c, ads, hall_sensor, spi, ldc_initialized
    print("--- Initializing Hardware ---")
    # Camera
    print("Initializing Camera...")
    camera = cv2.VideoCapture(CAMERA_INDEX)
    if not camera.isOpened(): print(f"Error: Could not open camera {CAMERA_INDEX}"); camera = None
    # I2C/ADS1115
    print("Initializing I2C and ADS1115...")
    try:
        i2c = busio.I2C(board.SCL, board.SDA)
        ads = ADS.ADS1115(i2c)
        hall_sensor = AnalogIn(ads, HALL_ADC_CHANNEL)
        print("ADS1115 Initialized.")
    except Exception as e: print(f"Error initializing I2C/ADS1115: {e}"); hall_sensor = None
    # SPI/GPIO/LDC1101
    print("Initializing GPIO, SPI...")
    try:
        GPIO.setwarnings(False); GPIO.setmode(GPIO.BCM)
        GPIO.setup(CS_PIN, GPIO.OUT); GPIO.output(CS_PIN, GPIO.HIGH)
        print("GPIO Initialized.")
        spi = spidev.SpiDev(); spi.open(SPI_BUS, SPI_DEVICE)
        spi.max_speed_hz = SPI_SPEED; spi.mode = SPI_MODE
        print(f"SPI Initialized (Bus={SPI_BUS}, Dev={SPI_DEVICE}).")
        if initialize_ldc1101(): enable_ldc_rpmode()
        else: print("LDC1101 Initialization Failed.")
    except Exception as e: print(f"Error initializing GPIO/SPI: {e}"); spi = None; ldc_initialized = False
    print("--- Hardware Initialization Complete ---")

# =========================
# === LDC1101 Functions ===
# =========================
def ldc_write_register(reg_addr, value):
    if not spi: return
    try: GPIO.output(CS_PIN, GPIO.LOW); spi.xfer2([reg_addr & 0x7F, value]); GPIO.output(CS_PIN, GPIO.HIGH)
    except Exception as e: print(f"Error writing LDC 0x{reg_addr:02X}: {e}")

def ldc_read_register(reg_addr):
    if not spi: return 0
    try: GPIO.output(CS_PIN, GPIO.LOW); result = spi.xfer2([reg_addr | 0x80, 0x00]); GPIO.output(CS_PIN, GPIO.HIGH); return result[1]
    except Exception as e: print(f"Error reading LDC 0x{reg_addr:02X}: {e}"); return 0

def initialize_ldc1101():
    global ldc_initialized
    if not spi: return False
    print("Checking LDC1101 Chip ID...")
    chip_id = ldc_read_register(CHIP_ID_REG)
    print(f"LDC1101 Chip ID Read: 0x{chip_id:02X} (Expected: 0x{LDC_CHIP_ID:02X})")
    if chip_id != LDC_CHIP_ID: return False
    print("Configuring LDC1101 for RP Mode...")
    ldc_write_register(RP_SET_REG, 0x1B); ldc_write_register(TC1_REG, 0x80); ldc_write_register(TC2_REG, 0x88)
    ldc_write_register(DIG_CONFIG_REG, 0x07); ldc_write_register(ALT_CONFIG_REG, 0x02)
    ldc_write_register(D_CONF_REG, 0x00); ldc_write_register(INTB_MODE_REG, 0x00)
    ldc_write_register(START_CONFIG_REG, SLEEP_MODE); time.sleep(0.05)
    print("LDC1101 Configuration Complete.")
    ldc_initialized = True; return True

def enable_ldc_powermode(mode):
    if not spi: return; print(f"Setting LDC1101 Power Mode to: {mode}")
    ldc_write_register(START_CONFIG_REG, mode); time.sleep(0.02)

def enable_ldc_rpmode():
    if not spi: return; print("Enabling LDC1101 RP Mode...")
    ldc_write_register(ALT_CONFIG_REG, 0x02); ldc_write_register(D_CONF_REG, 0x00)
    enable_ldc_powermode(ACTIVE_CONVERSION_MODE); print("LDC1101 RP Mode Active.")

def get_ldc_rpdata():
    if not spi or not ldc_initialized: return None
    try: msb = ldc_read_register(RP_DATA_MSB_REG); lsb = ldc_read_register(RP_DATA_LSB_REG); return (msb << 8) | lsb
    except Exception as e: print(f"Error in get_ldc_rpdata: {e}"); return None

# =========================
# === CSV Handling =========
# =========================
def append_metadata(image_rel_path, mag_mT, rp_value, target):
    file_exists = os.path.isfile(METADATA_PATH)
    try:
        with open(METADATA_PATH, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            if not file_exists or os.path.getsize(METADATA_PATH) == 0:
                writer.writerow(METADATA_HEADER)
            mag_mT_str = f"{mag_mT:.4f}" if mag_mT is not None else "N/A"
            rp_value_str = str(rp_value) if rp_value is not None else "N/A"
            writer.writerow([image_rel_path, mag_mT_str, rp_value_str, target])
        return True
    except IOError as e: print(f"Error writing metadata: {e}"); return False
    except Exception as e: print(f"Unexpected CSV error: {e}"); return False

# ============================
# === Sensor Reading (Avg) ===
# ============================
def get_averaged_hall_voltage():
    """Reads the Hall sensor multiple times and returns the average voltage."""
    if not hall_sensor: return None
    readings = []
    for _ in range(NUM_SAMPLES_PER_UPDATE):
        try:
            readings.append(hall_sensor.voltage)
            # time.sleep(0.005) # Small delay if ADC needs settling time
        except Exception:
            pass # Ignore read errors for averaging, rely on filtering
    if not readings: return None
    return sum(readings) / len(readings)

def get_averaged_rp_data():
    """Reads the LDC RP value multiple times and returns the average."""
    if not ldc_initialized: return None
    readings = []
    for _ in range(NUM_SAMPLES_PER_UPDATE):
        val = get_ldc_rpdata()
        if val is not None:
            readings.append(val)
        # time.sleep(0.005) # Small delay if needed
    if not readings: return None
    return sum(readings) / len(readings)


# ======================
# === GUI Functions ===
# ======================
def capture_and_save_data():
    global feedback_label
    if not camera: feedback_label.config(text="Camera not available", foreground="#E53935"); return

    capture_button.config(state=tk.DISABLED)
    feedback_label.config(text="Capturing data...", foreground="#FFA726") # Orange
    window.update()

    ret, frame = camera.read()
    if not ret:
        feedback_label.config(text="Failed to capture photo", foreground="#E53935") # Red
        capture_button.config(state=tk.NORMAL); return

    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB); img = Image.fromarray(frame_rgb)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S"); unique_id = uuid.uuid4().hex[:6]
    image_filename = f"{timestamp}_{unique_id}.jpg"
    image_save_path = os.path.join(IMAGES_PATH, image_filename)
    image_relative_path = os.path.join(IMAGES_FOLDER_NAME, image_filename)

    # Get Averaged Sensor Readings
    avg_voltage = get_averaged_hall_voltage()
    current_mag_mT = None
    if avg_voltage is not None:
        try: adjusted_voltage = avg_voltage - IDLE_VOLTAGE; current_mag_mT = adjusted_voltage / SENSITIVITY_V_PER_MILLITESLA
        except Exception: current_mag_mT = None # Handle potential division errors etc.

    current_rp_val = get_averaged_rp_data()
    if current_rp_val is not None: current_rp_val = int(current_rp_val) # Store RP as integer

    selected_target = target_var.get()

    save_success = False
    try: img.save(image_save_path); print(f"Image saved: {image_save_path}"); save_success = True
    except Exception as e:
        feedback_label.config(text=f"Error saving image: {e}", foreground="#E53935")
        print(f"Error saving photo: {e}"); capture_button.config(state=tk.NORMAL); return

    meta_success = append_metadata(image_relative_path, current_mag_mT, current_rp_val, selected_target)
    if meta_success: feedback_label.config(text=f"Data Added: {image_filename}", foreground="#66BB6A") # Green
    else: feedback_label.config(text="Image saved, metadata FAILED", foreground="#E53935")

    window.after(1500, lambda: capture_button.config(state=tk.NORMAL))
    window.after(3000, lambda: feedback_label.config(text=""))

def calibrate_sensors():
    global IDLE_VOLTAGE, IDLE_RP_VALUE, feedback_label
    feedback_text = ""; feedback_color = "#29B6F6" # Blue

    # Calibrate Hall Sensor
    if hall_sensor:
        voltages = []
        feedback_label.config(text="Calibrating Hall...", foreground="#FFA726")
        window.update()
        for _ in range(NUM_SAMPLES_CALIBRATION):
            try: voltages.append(hall_sensor.voltage); time.sleep(0.05)
            except Exception: pass
        if voltages: IDLE_VOLTAGE = sum(voltages) / len(voltages); feedback_text += f"Hall Idle: {IDLE_VOLTAGE:.4f} V\n"
        else: feedback_text += "Hall Cal Error: No valid readings\n"; feedback_color = "#FFA726" # Orange
    else: feedback_text += "Hall Sensor N/A\n"; feedback_color = "#FFA726"

    # Calibrate LDC1101
    if ldc_initialized:
        rp_readings = []
        feedback_label.config(text="Calibrating Hall...\nCalibrating LDC...", foreground="#FFA726")
        window.update()
        for _ in range(NUM_SAMPLES_CALIBRATION):
            val = get_ldc_rpdata()
            if val is not None: rp_readings.append(val)
            time.sleep(0.05)
        if rp_readings: IDLE_RP_VALUE = int(sum(rp_readings) / len(rp_readings)); feedback_text += f"LDC RP Idle: {IDLE_RP_VALUE}"
        else: feedback_text += "LDC Cal Error: No valid readings\n"; feedback_color = "#E53935" # Red
    else: feedback_text += "LDC Sensor N/A"; feedback_color = "#E53935" if feedback_color != "#FFA726" else "#FFA726"

    feedback_label.config(text=feedback_text.strip(), foreground=feedback_color)
    window.after(4000, lambda: feedback_label.config(text=""))

def update_camera_feed():
    if camera:
        ret, frame = camera.read()
        if ret:
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(frame_rgb).resize((IMG_WIDTH, IMG_HEIGHT))
            img_tk = ImageTk.PhotoImage(img)
            camera_label.img_tk = img_tk; camera_label.configure(image=img_tk)
    else:
         if not hasattr(camera_label, 'no_cam_img'):
             placeholder = Image.new('RGB', (IMG_WIDTH, IMG_HEIGHT), color = '#BDBDBD') # Grey
             # Add text to placeholder?
             camera_label.no_cam_img = ImageTk.PhotoImage(placeholder)
         camera_label.configure(image=camera_label.no_cam_img)
    window.after(50, update_camera_feed) # ~20 FPS

def update_magnetism():
    # Get averaged reading
    avg_voltage = get_averaged_hall_voltage()
    if avg_voltage is not None:
        try:
            adjusted_voltage = avg_voltage - IDLE_VOLTAGE
            magnetism_mT = adjusted_voltage / SENSITIVITY_V_PER_MILLITESLA
            if abs(magnetism_mT) < 1:
                magnetism_uT = magnetism_mT * 1000
                magnetism_label.config(text=f"{magnetism_uT:.2f} µT")
            else:
                magnetism_label.config(text=f"{magnetism_mT:.2f} mT")
        except Exception: magnetism_label.config(text="Error")
    else:
        # Check if sensor exists but read failed vs sensor not initialized
        magnetism_label.config(text="N/A" if not hall_sensor else "Error")

    window.after(100, update_magnetism) # Update ~10 times/sec

def update_ldc_reading():
    # Get averaged reading
    avg_rp_val = get_averaged_rp_data()
    if avg_rp_val is not None:
        # Display as integer for RP
        ldc_label.config(text=f"{int(avg_rp_val)}")
    else:
        # Check if sensor exists but read failed vs sensor not initialized
        ldc_label.config(text="N/A" if not ldc_initialized else "Error")

    window.after(100, update_ldc_reading) # Update ~10 times/sec

# ======================
# === GUI Setup ========
# ======================
def setup_gui():
    global window, camera_label, controls_frame, feedback_label
    global magnetism_label, ldc_label, target_var, capture_button, calibrate_button
    global label_font, readout_font, button_font, feedback_font, title_font

    window = tk.Tk()
    window.title("Sensor Data Acquisition Tool")
    window.geometry("1100x700") # Adjusted size

    # --- Configure Style & Fonts ---
    style = ttk.Style()
    try: style.theme_use('clam') # Or 'alt', 'default', 'classic'
    except tk.TclError: print("Clam theme not available, using default.")

    # Fonts (adjust sizes as needed)
    title_font = tkFont.Font(family="Helvetica", size=14, weight="bold")
    label_font = tkFont.Font(family="Helvetica", size=11)
    readout_font = tkFont.Font(family="Consolas", size=14, weight="bold") # Monospaced for numbers
    button_font = tkFont.Font(family="Helvetica", size=11, weight="bold")
    feedback_font = tkFont.Font(family="Helvetica", size=10)

    style.configure("TLabel", font=label_font, padding=3)
    style.configure("TButton", font=button_font, padding=(10, 8)) # Pad X, Y
    style.configure("TMenubutton", font=label_font, padding=5)
    style.configure("TLabelframe", padding=10)
    style.configure("TLabelframe.Label", font=tkFont.Font(family="Helvetica", size=12, weight="bold")) # Labelframe title font
    style.configure("Feedback.TLabel", font=feedback_font, padding=5) # Custom style for feedback
    style.configure("Readout.TLabel", font=readout_font, padding=(5, 2)) # Custom style for sensor values
    style.configure("Unit.TLabel", font=label_font, padding=(0, 2)) # Style for units label

    # --- Main Frame ---
    main_frame = ttk.Frame(window, padding="15 15 15 15")
    main_frame.pack(fill=tk.BOTH, expand=True)
    main_frame.columnconfigure(0, weight=3) # Camera feed wider (60-75%)
    main_frame.columnconfigure(1, weight=1) # Controls narrower (25-40%)
    main_frame.rowconfigure(0, weight=1)

    # --- Camera Feed Label (Left) ---
    camera_label = ttk.Label(main_frame, text="Initializing Camera...", anchor="center")
    camera_label.grid(row=0, column=0, padx=(0, 15), pady=0, sticky="nsew")
    # Add a border around camera feed for definition
    camera_label.configure(borderwidth=1, relief="sunken")

    # --- Controls Frame (Right) ---
    controls_frame = ttk.Frame(main_frame)
    controls_frame.grid(row=0, column=1, padx=(0, 0), pady=0, sticky="nsew")
    controls_frame.columnconfigure(0, weight=1) # Make internal column expandable

    # --- Widgets in Controls Frame ---
    row_idx = 0

    # Feedback Area (above Labelframes)
    feedback_label = ttk.Label(controls_frame, text="", style="Feedback.TLabel", anchor="w")
    feedback_label.grid(row=row_idx, column=0, sticky="ew", pady=(0, 10))
    row_idx += 1

    # Sensor Readings Group
    readings_frame = ttk.Labelframe(controls_frame, text=" Sensor Readings ", padding="15 10 15 10")
    readings_frame.grid(row=row_idx, column=0, sticky="new", pady=(0, 15))
    readings_frame.columnconfigure(0, weight=0) # Label column
    readings_frame.columnconfigure(1, weight=1) # Value column
    row_idx += 1

    # Magnetism
    ttk.Label(readings_frame, text="Magnetism:").grid(row=0, column=0, sticky="w", padx=(0, 5))
    magnetism_label = ttk.Label(readings_frame, text="Init...", style="Readout.TLabel", anchor="e")
    magnetism_label.grid(row=0, column=1, sticky="ew")

    # LDC RP
    ttk.Label(readings_frame, text="LDC RP:").grid(row=1, column=0, sticky="w", padx=(0, 5), pady=(5,0))
    ldc_label = ttk.Label(readings_frame, text="Init...", style="Readout.TLabel", anchor="e")
    ldc_label.grid(row=1, column=1, sticky="ew", pady=(5,0))

    # Data Capture Group
    actions_frame = ttk.Labelframe(controls_frame, text=" Data Capture ", padding="15 10 15 15")
    actions_frame.grid(row=row_idx, column=0, sticky="new", pady=(0, 15))
    actions_frame.columnconfigure(0, weight=1) # Allow widgets to expand horizontally
    row_idx += 1

    # Target Material
    target_title_label = ttk.Label(actions_frame, text="Target Material:")
    target_title_label.grid(row=0, column=0, sticky="w", pady=(0, 3))

    target_var = tk.StringVar(window)
    target_var.set(TARGET_OPTIONS[0])
    target_dropdown = ttk.OptionMenu(actions_frame, target_var, TARGET_OPTIONS[0], *TARGET_OPTIONS, style="TMenubutton")
    target_dropdown.grid(row=1, column=0, sticky="ew", pady=(0, 15))

    # Separator
    ttk.Separator(actions_frame, orient='horizontal').grid(row=2, column=0, sticky='ew', pady=(0, 15))

    # Capture Button
    capture_button = ttk.Button(actions_frame, text="Capture & Add Data", command=capture_and_save_data)
    capture_button.grid(row=3, column=0, sticky="ew", pady=(0, 8))

    # Calibrate Button
    calibrate_button = ttk.Button(actions_frame, text="Calibrate Sensors", command=calibrate_sensors)
    calibrate_button.grid(row=4, column=0, sticky="ew")

    # Add spacer at the bottom of controls_frame to push content up if needed
    # ttk.Frame(controls_frame).grid(row=row_idx, column=0, sticky="nsew")
    # controls_frame.rowconfigure(row_idx, weight=1)


# ==========================
# === Main Execution =======
# ==========================
def run_application():
    if not setup_project_directory(): print("Fatal Error: Could not create directories. Exiting."); return
    setup_gui()
    update_camera_feed(); update_magnetism(); update_ldc_reading()
    print("Starting Tkinter main loop...")
    window.mainloop()

# --- Cleanup ---
def cleanup_resources():
    print("Cleaning up resources...")
    if camera and camera.isOpened(): print("Releasing camera..."); camera.release()
    cv2.destroyAllWindows()
    if spi: print("Closing SPI..."); spi.close()
    try: print("Cleaning up GPIO..."); GPIO.cleanup()
    except Exception as e: print(f"Note: GPIO cleanup error: {e}")
    print("Cleanup complete.")

# --- Run with Hardware Init and Cleanup ---
if __name__ == '__main__':
    initialize_hardware()
    try: run_application()
    except Exception as e: print(f"Unexpected error: {e}"); import traceback; traceback.print_exc()
    finally: cleanup_resources()
