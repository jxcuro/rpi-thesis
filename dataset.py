# Combined Code: Camera, Magnetism, Inductance Measurement with Dataset Creation
# Version: 1.5.1 - Relocated Feedback Label to Status Bar for Compactness

import tkinter as tk
from tkinter import ttk
from tkinter import font as tkFont
import cv2
from PIL import Image, ImageTk # Pillow for image manipulation
import time
import os
import csv
from datetime import datetime
import statistics
from collections import deque
import re # For regex in file naming

# --- I2C/ADS1115 Imports ---
try:
    import board
    import busio
    import adafruit_ads1x15.ads1115 as ADS
    from adafruit_ads1x15.analog_in import AnalogIn
    I2C_ENABLED = True
except ImportError:
    print("Warning: I2C/ADS1115 libraries not found. Magnetism disabled.")
    I2C_ENABLED = False

# --- SPI/LDC1101 Imports ---
try:
    import spidev
    import RPi.GPIO as GPIO
    SPI_ENABLED = True
except ImportError:
    print("Warning: SPI/GPIO libraries not found. LDC disabled.")
    SPI_ENABLED = False

# ==================================
# === Constants and Configuration ===
# ==================================

# --- Accuracy / Stability / Speed (Optimization) ---
NUM_SAMPLES_PER_UPDATE = 3
NUM_SAMPLES_CALIBRATION = 5
GUI_UPDATE_INTERVAL_MS = 250
LDC_DISPLAY_BUFFER_SIZE = 5

# --- Image Saving ---
SAVE_IMG_WIDTH = 224
SAVE_IMG_HEIGHT = 224

# --- Camera ---
CAMERA_INDEX = 0
DISPLAY_IMG_WIDTH = 640
DISPLAY_IMG_HEIGHT = 480

# --- Hall Sensor (ADS1115) ---
HALL_ADC_CHANNEL = ADS.P0 if I2C_ENABLED else None
SENSITIVITY_V_PER_TESLA = 0.0004
SENSITIVITY_V_PER_MILLITESLA = SENSITIVITY_V_PER_TESLA * 1000
IDLE_VOLTAGE = 1.7256

# --- Inductive Sensor (LDC1101) ---
SPI_BUS = 0; SPI_DEVICE = 0; SPI_SPEED = 500000; SPI_MODE = 0b00
CS_PIN = 8; LDC_CHIP_ID = 0xD4
START_CONFIG_REG, RP_SET_REG, TC1_REG, TC2_REG, DIG_CONFIG_REG = 0x0B, 0x01, 0x02, 0x03, 0x04
ALT_CONFIG_REG, D_CONF_REG, INTB_MODE_REG = 0x05, 0x0C, 0x0A
RP_DATA_MSB_REG, RP_DATA_LSB_REG = 0x22, 0x21; CHIP_ID_REG = 0x3F
ACTIVE_CONVERSION_MODE, SLEEP_MODE = 0x00, 0x01

# --- Calibration ---
IDLE_RP_VALUE = 0

# --- Dataset / File Saving ---
PROJECT_FOLDER_NAME = "Project_Dataset"
IMAGES_FOLDER_NAME = "images"
METADATA_FILENAME = "metadata.csv"
METADATA_HEADER = [
    'image_filename', 'magnetism_mT', 'ldc_rp', 'delta_rp', 'target_material',
    'is_coated', 'is_dilapidated', 'is_degraded'
]
try: BASE_PATH = os.path.dirname(os.path.abspath(__file__))
except NameError: BASE_PATH = os.getcwd()
PROJECT_PATH = os.path.join(BASE_PATH, PROJECT_FOLDER_NAME)
IMAGES_PATH = os.path.join(PROJECT_PATH, IMAGES_FOLDER_NAME)
METADATA_PATH = os.path.join(PROJECT_PATH, METADATA_FILENAME)
FILENAME_PREFIX = "data_"

# --- Target Materials ---
TARGET_OPTIONS = [
    "Copper", "Steel", "Aluminum", "Others"
]

# --- Global Hardware Objects ---
camera = None; i2c = None; ads = None; hall_sensor = None
spi = None; ldc_initialized = False

# --- Global State ---
RP_DISPLAY_BUFFER = deque(maxlen=LDC_DISPLAY_BUFFER_SIZE)
object_counter = 0
shot_counter = 0

# --- GUI Globals ---
window = None; camera_label = None; controls_frame = None; feedback_label = None # feedback_label still global
magnetism_label = None; ldc_label = None; target_var = None
capture_button = None; calibrate_button = None
coated_var = None; dilapidated_var = None; degraded_var = None
new_object_var = None
next_filename_label = None
label_font = None; readout_font = None; button_font = None
feedback_font = None; title_font = None; check_font = None
filename_display_font = None

# =========================
# === File/State Utils ====
# =========================
# (Functions get_last_object_index, initialize_global_state, setup_project_directory remain the same as v1.5.0)
def get_last_object_index(img_dir, prefix):
    """Finds the highest object index based on existing files like data_OBJ_SHOT.jpg."""
    max_obj_index = 0
    if not os.path.isdir(img_dir):
        print(f"Info: Image directory not found ({img_dir}). Starting object index at 1.")
        return 0
    try:
        filename_pattern_str = f"^{prefix}(\\d+)_(\\d+)\\.jpg$"
        filename_pattern = re.compile(filename_pattern_str)
        print(f"Scanning {img_dir} for files matching pattern: {filename_pattern_str}")
        found_files = False
        for filename in os.listdir(img_dir):
            match = filename_pattern.match(filename)
            if match:
                found_files = True
                try:
                    obj_index = int(match.group(1))
                    max_obj_index = max(max_obj_index, obj_index)
                except ValueError:
                    print(f"Warning: Could not parse object index from filename '{filename}'.")
                    continue
        if not found_files:
             print("No existing data files found matching the pattern.")
    except OSError as e:
        print(f"Warning: Could not read image directory {img_dir}: {e}")
    print(f"Determined last used object index: {max_obj_index}")
    return max_obj_index

def initialize_global_state():
    """Initializes state variables including object/shot counters."""
    global object_counter, shot_counter
    if not setup_project_directory():
         print("Fatal Error: Directory setup failed. Cannot initialize state.")
         return False
    print("Finding last used object index...")
    object_counter = get_last_object_index(IMAGES_PATH, FILENAME_PREFIX)
    shot_counter = 0
    print(f"Starting with Object Counter = {object_counter} (Next new object will be {object_counter + 1})")
    return True

def setup_project_directory():
    """Creates the main project and images directories if they don't exist."""
    print(f"Ensuring dataset directory exists at: {PROJECT_PATH}")
    try:
        os.makedirs(PROJECT_PATH, exist_ok=True)
        os.makedirs(IMAGES_PATH, exist_ok=True)
        return True
    except OSError as e:
        print(f"Error creating directories: {e}")
        return False

# =========================
# === Hardware Setup ===
# =========================
# (initialize_hardware remains the same as v1.5.0)
def initialize_hardware():
    """Initializes Camera, I2C/ADS1115, SPI/GPIO/LDC1101."""
    global camera, i2c, ads, hall_sensor, spi, ldc_initialized
    print("--- Initializing Hardware ---")
    try:
        camera = cv2.VideoCapture(CAMERA_INDEX)
        if camera and not camera.isOpened(): raise ValueError("Could not open camera")
    except Exception as e:
        print(f"Error opening camera {CAMERA_INDEX}: {e}")
        camera = None

    if I2C_ENABLED:
        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            ads = ADS.ADS1115(i2c)
            hall_sensor = AnalogIn(ads, HALL_ADC_CHANNEL)
            print("ADS1115 Initialized.")
        except Exception as e:
            print(f"Error initializing I2C/ADS1115: {e}")
            hall_sensor = None
    else:
        print("Skipping I2C/ADS1115 setup.")

    if SPI_ENABLED:
        try:
            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(CS_PIN, GPIO.OUT)
            GPIO.output(CS_PIN, GPIO.HIGH)
            print("GPIO Initialized.")
            spi = spidev.SpiDev()
            spi.open(SPI_BUS, SPI_DEVICE)
            spi.max_speed_hz = SPI_SPEED
            spi.mode = SPI_MODE
            print(f"SPI Initialized (Bus={SPI_BUS}, Dev={SPI_DEVICE}).")
            if initialize_ldc1101():
                enable_ldc_rpmode()
            else:
                print("LDC1101 Initialization Failed.")
        except Exception as e:
            print(f"Error initializing GPIO/SPI: {e}")
            spi = None
            ldc_initialized = False
    else:
        print("Skipping SPI/GPIO/LDC1101 setup.")
    print("--- Hardware Initialization Complete ---")

# =========================
# === LDC1101 Functions ===
# =========================
# (LDC functions remain the same as v1.5.0)
def ldc_write_register(reg_addr, value):
    if not spi: return
    try:
        GPIO.output(CS_PIN, GPIO.LOW)
        spi.xfer2([reg_addr & 0x7F, value])
        GPIO.output(CS_PIN, GPIO.HIGH)
    except Exception as e: print(f"Error writing LDC 0x{reg_addr:02X}: {e}")

def ldc_read_register(reg_addr):
    if not spi: return 0
    try:
        GPIO.output(CS_PIN, GPIO.LOW)
        result = spi.xfer2([reg_addr | 0x80, 0x00])
        GPIO.output(CS_PIN, GPIO.HIGH)
        return result[1]
    except Exception as e: print(f"Error reading LDC 0x{reg_addr:02X}: {e}"); return 0

def initialize_ldc1101():
    global ldc_initialized
    ldc_initialized = False
    if not spi: return False
    chip_id = ldc_read_register(CHIP_ID_REG)
    if chip_id != LDC_CHIP_ID:
        print(f"LDC Chip ID Mismatch: Read 0x{chip_id:02X}, Expected 0x{LDC_CHIP_ID:02X}")
        return False
    print("Configuring LDC1101...")
    ldc_write_register(RP_SET_REG, 0x1B)
    ldc_write_register(TC1_REG, 0x80)
    ldc_write_register(TC2_REG, 0x88)
    ldc_write_register(DIG_CONFIG_REG, 0x07)
    ldc_write_register(ALT_CONFIG_REG, 0x00)
    ldc_write_register(D_CONF_REG, 0x00)
    ldc_write_register(INTB_MODE_REG, 0x00)
    ldc_write_register(START_CONFIG_REG, SLEEP_MODE)
    time.sleep(0.05)
    print("LDC1101 Configured.")
    ldc_initialized = True
    return True

def enable_ldc_powermode(mode):
    if not spi: return
    ldc_write_register(START_CONFIG_REG, mode)
    time.sleep(0.02)

def enable_ldc_rpmode():
    if not spi or not ldc_initialized: return
    print("Enabling LDC RP Mode...")
    ldc_write_register(ALT_CONFIG_REG, 0x00)
    ldc_write_register(D_CONF_REG, 0x00)
    enable_ldc_powermode(ACTIVE_CONVERSION_MODE)
    print("LDC RP Mode Active.")

def get_ldc_rpdata():
    if not spi or not ldc_initialized: return None
    try:
        msb = ldc_read_register(RP_DATA_MSB_REG)
        lsb = ldc_read_register(RP_DATA_LSB_REG)
        value = (msb << 8) | lsb
        return value
    except Exception as e: print(f"Error in get_ldc_rpdata: {e}"); return None

# =========================
# === CSV Handling =========
# =========================
# (append_metadata remains the same as v1.5.0)
def append_metadata(image_filename, mag_mT, rp_value, delta_rp, target, is_coated, is_dilapidated, is_degraded):
    file_exists = os.path.isfile(METADATA_PATH)
    try:
        with open(METADATA_PATH, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            if not file_exists or os.path.getsize(METADATA_PATH) == 0:
                writer.writerow(METADATA_HEADER)
            mag_mT_str = f"{mag_mT:.4f}" if mag_mT is not None else "N/A"
            rp_value_str = str(rp_value) if rp_value is not None else "N/A"
            delta_rp_str = str(delta_rp) if delta_rp is not None else "N/A"
            writer.writerow([
                image_filename, mag_mT_str, rp_value_str, delta_rp_str, target,
                is_coated, is_dilapidated, is_degraded
            ])
        return True
    except IOError as e: print(f"Error writing metadata: {e}"); return False
    except Exception as e: print(f"Unexpected CSV error: {e}"); return False

# ============================
# === Sensor Reading (Avg) ===
# ============================
# (Sensor reading functions remain the same as v1.5.0)
def get_averaged_hall_voltage(num_samples=NUM_SAMPLES_PER_UPDATE):
    if not hall_sensor: return None
    readings = [hall_sensor.voltage for _ in range(num_samples) if hall_sensor] # More compact reading
    return sum(readings) / len(readings) if readings else None

def get_averaged_rp_data(num_samples=NUM_SAMPLES_PER_UPDATE):
    if not ldc_initialized: return None
    readings = [get_ldc_rpdata() for _ in range(num_samples)]
    valid_readings = [r for r in readings if r is not None]
    return sum(valid_readings) / len(valid_readings) if valid_readings else None


# ======================
# === GUI Functions ===
# ======================

# (update_next_filename_display, capture_and_save_data, calibrate_sensors,
#  update_camera_feed, update_magnetism, update_ldc_reading remain the same logic as v1.5.0,
#  only the GUI setup changes where feedback_label is placed)

def update_next_filename_display():
    """Updates the GUI label showing the next filename to be generated."""
    global next_filename_label, new_object_var, object_counter, shot_counter
    if not next_filename_label or not new_object_var: return

    try:
        is_new = new_object_var.get()
        if is_new == 1:
            display_obj = object_counter + 1
            display_shot = 1
        else:
            display_obj = object_counter if object_counter > 0 else 1
            display_shot = shot_counter + 1

        next_fn = f"{FILENAME_PREFIX}{display_obj}_{display_shot}.jpg"
        next_filename_label.config(text=f"Next File: {next_fn}")
    except tk.TclError:
         next_filename_label.config(text="Next File: Initializing...") # Should resolve quickly
    except Exception as e:
         print(f"Error updating filename display: {e}")
         next_filename_label.config(text="Next File: Error")

def capture_and_save_data():
    """Captures image, reads sensors, updates counters, saves files, appends metadata."""
    global feedback_label, capture_button, window, target_var, IDLE_RP_VALUE
    global coated_var, dilapidated_var, degraded_var
    global object_counter, shot_counter, new_object_var

    if not camera:
        feedback_label.config(text="Camera not available", foreground="#E53935")
        return

    capture_button.config(state=tk.DISABLED)
    feedback_label.config(text="Capturing data...", foreground="#FFA726") # Update status bar
    window.update()

    ret, frame = camera.read()
    if not ret:
        feedback_label.config(text="Failed to capture photo", foreground="#E53935")
        capture_button.config(state=tk.NORMAL)
        return

    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    img_raw = Image.fromarray(frame_rgb)
    try:
        img_resized_for_save = img_raw.resize((SAVE_IMG_WIDTH, SAVE_IMG_HEIGHT), Image.Resampling.LANCZOS)
    except AttributeError:
        img_resized_for_save = img_raw.resize((SAVE_IMG_WIDTH, SAVE_IMG_HEIGHT), Image.LANCZOS)
    except Exception as e:
        print(f"Error resizing image: {e}")
        feedback_label.config(text=f"Resize Error: {e}", foreground="#E53935")
        capture_button.config(state=tk.NORMAL)
        return

    is_new_object = new_object_var.get()
    if is_new_object == 1:
        object_counter += 1
        shot_counter = 1
        print(f"Starting new object series: Object {object_counter}")
    else:
        if object_counter == 0: object_counter = 1 # Ensure first object is 1
        shot_counter += 1
        print(f"Continuing object {object_counter}, taking shot {shot_counter}")

    image_filename = f"{FILENAME_PREFIX}{object_counter}_{shot_counter}.jpg"
    image_save_path = os.path.join(IMAGES_PATH, image_filename)

    avg_voltage = get_averaged_hall_voltage(num_samples=NUM_SAMPLES_CALIBRATION)
    current_mag_mT = None
    if avg_voltage is not None:
        try: current_mag_mT = (avg_voltage - IDLE_VOLTAGE) / SENSITIVITY_V_PER_MILLITESLA
        except Exception: current_mag_mT = None

    current_rp_val = get_averaged_rp_data(num_samples=NUM_SAMPLES_CALIBRATION)
    delta_rp = None
    if current_rp_val is not None:
        current_rp_val = int(current_rp_val)
        if IDLE_RP_VALUE != 0: delta_rp = current_rp_val - IDLE_RP_VALUE

    selected_target = target_var.get()
    is_coated = coated_var.get()
    is_dilapidated = dilapidated_var.get()
    is_degraded = degraded_var.get()

    save_success = False
    try:
        img_resized_for_save.save(image_save_path)
        print(f"Image saved: {image_save_path}")
        save_success = True
    except Exception as e:
        print(f"Error saving photo: {e}")
        feedback_label.config(text=f"Save Error: {e}", foreground="#E53935")
        capture_button.config(state=tk.NORMAL)
        update_next_filename_display()
        return

    meta_success = append_metadata(
        image_filename, current_mag_mT, current_rp_val, delta_rp, selected_target,
        is_coated, is_dilapidated, is_degraded
    )

    if meta_success:
        feedback_label.config(text=f"Data Added: {image_filename}", foreground="#66BB6A")
        new_object_var.set(0) # Default to same object next time
    else:
        feedback_label.config(text="Image saved, metadata FAILED", foreground="#E53935")

    update_next_filename_display()
    window.after(1500, lambda: capture_button.config(state=tk.NORMAL))
    # Clear feedback from status bar after a longer delay
    window.after(5000, lambda: feedback_label.config(text=""))

def calibrate_sensors():
    """Calibrates idle voltage for Hall sensor and idle RP value for LDC1101."""
    global IDLE_VOLTAGE, IDLE_RP_VALUE, feedback_label, window
    feedback_text = ""
    feedback_color = "#29B6F6"
    feedback_label.config(text="Calibrating...", foreground="#FFA726") # Update status bar
    window.update()

    cal_voltages = []
    if hall_sensor:
        for _ in range(NUM_SAMPLES_CALIBRATION):
            try:
                cal_voltages.append(hall_sensor.voltage)
                time.sleep(0.05)
            except Exception: pass
        if cal_voltages:
            IDLE_VOLTAGE = sum(cal_voltages) / len(cal_voltages)
            feedback_text += f"Hall Idle: {IDLE_VOLTAGE:.4f} V  "
        else:
            feedback_text += "Hall Cal Error  "
            feedback_color = "#FFA726"
    else:
        feedback_text += "Hall N/A  "
        feedback_color = "#FFA726"

    cal_rp_readings = []
    if ldc_initialized:
        # Update status bar progressively if needed, or just wait till end
        # feedback_label.config(text=feedback_text + "LDC...", foreground="#FFA726")
        # window.update()
        for _ in range(NUM_SAMPLES_CALIBRATION):
            val = get_ldc_rpdata()
            if val is not None: cal_rp_readings.append(val)
            time.sleep(0.05)
        if cal_rp_readings:
            IDLE_RP_VALUE = int(sum(cal_rp_readings) / len(cal_rp_readings))
            feedback_text += f"LDC Idle: {IDLE_RP_VALUE}"
        else:
            feedback_text += "LDC Cal Error"
            feedback_color = "#E53935" if feedback_color != "#FFA726" else "#FFA726"
    else:
        feedback_text += "LDC N/A"
        feedback_color = "#E53935" if feedback_color != "#FFA726" else "#FFA726"

    feedback_label.config(text=feedback_text.strip(), foreground=feedback_color)
    window.after(5000, lambda: feedback_label.config(text="")) # Clear status bar

def update_camera_feed():
    global camera_label, window
    if camera:
        ret, frame = camera.read()
        if ret:
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(frame_rgb).resize((DISPLAY_IMG_WIDTH, DISPLAY_IMG_HEIGHT))
            img_tk = ImageTk.PhotoImage(img)
            camera_label.img_tk = img_tk
            camera_label.configure(image=img_tk)
    else:
         if not hasattr(camera_label, 'no_cam_img'):
             placeholder = Image.new('RGB', (DISPLAY_IMG_WIDTH, DISPLAY_IMG_HEIGHT), color='#BDBDBD')
             camera_label.no_cam_img = ImageTk.PhotoImage(placeholder)
         camera_label.configure(image=camera_label.no_cam_img)
    window.after(50, update_camera_feed)

def update_magnetism():
    global magnetism_label, window
    avg_voltage = get_averaged_hall_voltage()
    if avg_voltage is not None:
        try:
            mag_mT = (avg_voltage - IDLE_VOLTAGE) / SENSITIVITY_V_PER_MILLITESLA
            unit = "mT"
            value = mag_mT
            if abs(mag_mT) < 1:
                unit = "µT"
                value = mag_mT * 1000
            magnetism_label.config(text=f"{value:.2f} {unit}")
        except Exception: magnetism_label.config(text="Error")
    else: magnetism_label.config(text="N/A" if not hall_sensor else "Error")
    window.after(GUI_UPDATE_INTERVAL_MS, update_magnetism)

def update_ldc_reading():
    global ldc_label, window, RP_DISPLAY_BUFFER
    avg_rp_val = get_averaged_rp_data()
    display_rp_text = "N/A"
    if not ldc_initialized: display_rp_text = "N/A"
    elif avg_rp_val is not None:
        RP_DISPLAY_BUFFER.append(avg_rp_val)
        if RP_DISPLAY_BUFFER:
            display_rp_text = f"{int(sum(RP_DISPLAY_BUFFER) / len(RP_DISPLAY_BUFFER))}"
        else: display_rp_text = "..."
    else: RP_DISPLAY_BUFFER.clear(); display_rp_text = "Error"
    ldc_label.config(text=display_rp_text)
    window.after(GUI_UPDATE_INTERVAL_MS, update_ldc_reading)

# ======================
# === GUI Setup ========
# ======================
def setup_gui():
    """Sets up the Tkinter GUI elements, with feedback label in a status bar."""
    global window, camera_label, controls_frame, feedback_label, magnetism_label, ldc_label
    global target_var, capture_button, calibrate_button, coated_var, dilapidated_var, degraded_var
    global new_object_var, next_filename_label
    global label_font, readout_font, button_font, feedback_font, title_font, check_font, filename_display_font

    window = tk.Tk()
    window.title("Sensor Data Acquisition Tool v1.5.1") # Updated version
    window.geometry("1100x700") # Initial size, might not need adjustment now

    # --- Style & Fonts ---
    style = ttk.Style()
    if 'clam' in style.theme_names(): style.theme_use('clam')
    else: style.theme_use('default')

    title_font = tkFont.Font(family="Helvetica", size=14, weight="bold")
    label_font = tkFont.Font(family="Helvetica", size=11)
    readout_font = tkFont.Font(family="Consolas", size=14, weight="bold")
    button_font = tkFont.Font(family="Helvetica", size=11, weight="bold")
    feedback_font = tkFont.Font(family="Helvetica", size=10) # Font for status bar
    check_font = tkFont.Font(family="Helvetica", size=10)
    filename_display_font = tkFont.Font(family="Consolas", size=9)

    style.configure("TLabel", font=label_font, padding=3)
    style.configure("TButton", font=button_font, padding=(10, 8))
    style.configure("TMenubutton", font=label_font, padding=5)
    style.configure("TLabelframe", padding=10)
    style.configure("TLabelframe.Label", font=tkFont.Font(family="Helvetica", size=12, weight="bold"))
    # Removed Feedback.TLabel style as it's now a standard label in status bar
    style.configure("Readout.TLabel", font=readout_font, padding=(5, 2))
    style.configure("Unit.TLabel", font=label_font, padding=(0, 2))
    style.configure("TCheckbutton", font=check_font, padding=3)
    style.configure("Filename.TLabel", font=filename_display_font, padding=(0, 5))
    style.configure("Status.TFrame", padding=0) # Style for status bar frame if needed


    # --- Main Content Frame ---
    # This frame holds the camera and controls, and expands
    main_frame = ttk.Frame(window, padding="15 15 15 15")
    main_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True) # Pack at top, allow expansion
    main_frame.columnconfigure(0, weight=3) # Camera feed column wider
    main_frame.columnconfigure(1, weight=1) # Controls column narrower
    main_frame.rowconfigure(0, weight=1) # Allow row to expand vertically

    # --- Status Bar Frame ---
    # This frame holds the feedback label at the bottom
    status_bar_frame = ttk.Frame(window, relief=tk.SUNKEN, style="Status.TFrame")
    status_bar_frame.pack(side=tk.BOTTOM, fill=tk.X, padx=0, pady=0) # Pack at bottom, fill horizontally

    # --- Feedback Label (Now in Status Bar) ---
    feedback_label = ttk.Label(status_bar_frame, text="", font=feedback_font, anchor="w")
    feedback_label.pack(side=tk.LEFT, padx=5, pady=2) # Pack left within status bar


    # --- Camera Feed Label (Inside main_frame, Left) ---
    camera_label = ttk.Label(main_frame, text="Initializing Camera...", anchor="center", borderwidth=1, relief="sunken")
    camera_label.grid(row=0, column=0, padx=(0, 15), pady=0, sticky="nsew")

    # --- Controls Frame (Inside main_frame, Right) ---
    controls_frame = ttk.Frame(main_frame)
    controls_frame.grid(row=0, column=1, padx=(0, 0), pady=0, sticky="nsew")
    controls_frame.columnconfigure(0, weight=1) # Allow content to expand horizontally

    # Row counter for controls_frame (starts at 0 as feedback is removed)
    controls_row_idx = 0

    # --- Sensor Readings Group ---
    # Reduced pady slightly
    readings_frame = ttk.Labelframe(controls_frame, text=" Sensor Readings ", padding="15 10 15 10")
    readings_frame.grid(row=controls_row_idx, column=0, sticky="new", pady=(0, 10)) # Reduced bottom padding
    controls_row_idx += 1
    readings_frame.columnconfigure(0, weight=0)
    readings_frame.columnconfigure(1, weight=1)
    ttk.Label(readings_frame, text="Magnetism:").grid(row=0, column=0, sticky="w", padx=(0, 10))
    magnetism_label = ttk.Label(readings_frame, text="Init...", style="Readout.TLabel", anchor="e")
    magnetism_label.grid(row=0, column=1, sticky="ew")
    ttk.Label(readings_frame, text="LDC RP:").grid(row=1, column=0, sticky="w", padx=(0, 10), pady=(5,0))
    ldc_label = ttk.Label(readings_frame, text="Init...", style="Readout.TLabel", anchor="e")
    ldc_label.grid(row=1, column=1, sticky="ew", pady=(5,0))

    # --- Data Capture Group ---
    # Reduced pady slightly
    actions_frame = ttk.Labelframe(controls_frame, text=" Data Capture ", padding="15 10 15 15")
    actions_frame.grid(row=controls_row_idx, column=0, sticky="new", pady=(0, 10)) # Reduced bottom padding
    controls_row_idx += 1
    actions_frame.columnconfigure(0, weight=1)
    action_row_idx = 0

    # Target Material Dropdown
    target_title_label = ttk.Label(actions_frame, text="Target Material:")
    target_title_label.grid(row=action_row_idx, column=0, sticky="w", pady=(0, 3))
    action_row_idx += 1
    target_var = tk.StringVar(window)
    default_target = TARGET_OPTIONS[0] if TARGET_OPTIONS else ""
    target_var.set(default_target)
    target_dropdown = ttk.OptionMenu(actions_frame, target_var, default_target, *TARGET_OPTIONS, style="TMenubutton")
    target_dropdown.grid(row=action_row_idx, column=0, sticky="ew", pady=(0, 10))
    action_row_idx += 1

    # Condition Checkboxes
    condition_label = ttk.Label(actions_frame, text="Conditions:")
    condition_label.grid(row=action_row_idx, column=0, sticky="w", pady=(5, 3))
    action_row_idx += 1
    coated_var = tk.IntVar(value=0)
    dilapidated_var = tk.IntVar(value=0)
    degraded_var = tk.IntVar(value=0)
    cb_coated = ttk.Checkbutton(actions_frame, text="Coated", variable=coated_var, onvalue=1, offvalue=0)
    cb_coated.grid(row=action_row_idx, column=0, sticky="w")
    action_row_idx += 1
    cb_dilapidated = ttk.Checkbutton(actions_frame, text="Dilapidated", variable=dilapidated_var, onvalue=1, offvalue=0)
    cb_dilapidated.grid(row=action_row_idx, column=0, sticky="w")
    action_row_idx += 1
    cb_degraded = ttk.Checkbutton(actions_frame, text="Degraded", variable=degraded_var, onvalue=1, offvalue=0)
    cb_degraded.grid(row=action_row_idx, column=0, sticky="w", pady=(0, 10))
    action_row_idx += 1

    # New Object Control
    new_object_var = tk.IntVar(value=1)
    cb_new_object = ttk.Checkbutton(
        actions_frame,
        text="Start New Object Series?",
        variable=new_object_var,
        onvalue=1,
        offvalue=0,
        command=update_next_filename_display
    )
    cb_new_object.grid(row=action_row_idx, column=0, sticky="w", pady=(5, 5))
    action_row_idx += 1

    # Next Filename Display
    next_filename_label = ttk.Label(actions_frame, text="Next File: ...", style="Filename.TLabel")
    next_filename_label.grid(row=action_row_idx, column=0, sticky="w", pady=(0, 10))
    action_row_idx += 1

    # Separator
    separator = ttk.Separator(actions_frame, orient='horizontal')
    separator.grid(row=action_row_idx, column=0, sticky='ew', pady=(5, 10)) # Adjusted padding
    action_row_idx += 1

    # Buttons
    capture_button = ttk.Button(actions_frame, text="Capture & Add Data", command=capture_and_save_data)
    capture_button.grid(row=action_row_idx, column=0, sticky="ew", pady=(0, 5)) # Reduced padding
    action_row_idx += 1
    calibrate_button = ttk.Button(actions_frame, text="Calibrate Sensors", command=calibrate_sensors)
    calibrate_button.grid(row=action_row_idx, column=0, sticky="ew")
    action_row_idx += 1

    # Initialize the filename display after everything is created
    update_next_filename_display()


# ==========================
# === Main Execution =======
# ==========================
def run_application():
    """Sets up state, GUI, starts loops, runs the main loop."""
    global window
    if not initialize_global_state():
        return

    setup_gui() # Creates GUI, including status bar and feedback label

    # Set initial GUI states
    if not camera: camera_label.configure(text="Camera Failed")
    if not hall_sensor: magnetism_label.config(text="N/A")
    if not ldc_initialized: ldc_label.config(text="N/A")
    feedback_label.config(text="Ready.") # Initial status message

    # Start the update loops
    update_camera_feed()
    update_magnetism()
    update_ldc_reading()

    print("Starting Tkinter main loop...")
    window.mainloop()

# --- Cleanup ---
def cleanup_resources():
    """Releases hardware resources."""
    # (Cleanup remains the same as v1.5.0)
    print("Cleaning up resources...")
    if camera and camera.isOpened():
        print("Releasing camera...")
        camera.release()
    cv2.destroyAllWindows()
    if spi:
        print("Closing SPI...")
        spi.close()
    if SPI_ENABLED:
        try:
            print("Cleaning up GPIO...")
            GPIO.cleanup()
        except Exception as e:
            print(f"Note: GPIO cleanup error (maybe already cleaned?): {e}")
    print("Cleanup complete.")

# --- Run ---
if __name__ == '__main__':
    initialize_hardware()
    try:
        run_application()
    except Exception as e:
        print(f"FATAL ERROR in main execution: {e}")
        import traceback
        traceback.print_exc()
    finally:
        cleanup_resources()
