# Combined Code: Camera, Magnetism, Inductance Measurement with Dataset Creation
# Version: v1.3.1 - Checkboxes, Resize, Seq Naming, GUI Fixes, Scope Fix

import tkinter as tk
from tkinter import ttk
from tkinter import font as tkFont
import cv2
from PIL import Image, ImageTk # Pillow for image manipulation
import time
import os
# import uuid # No longer needed for filename
import csv
from datetime import datetime
import statistics
from collections import deque
import re # For parsing existing filenames
import sys # For exiting on critical error

# --- I2C/ADS1115 Imports ---
try:
    import board; import busio
    import adafruit_ads1x15.ads1115 as ADS
    from adafruit_ads1x15.analog_in import AnalogIn
    I2C_ENABLED = True
except ImportError: print("Warning: I2C/ADS1115 libraries not found. Magnetism disabled."); I2C_ENABLED = False

# --- SPI/LDC1101 Imports ---
try:
    import spidev; import RPi.GPIO as GPIO
    SPI_ENABLED = True
except ImportError: print("Warning: SPI/GPIO libraries not found. LDC disabled."); SPI_ENABLED = False

# ==================================
# === Constants and Configuration ===
# ==================================

# --- Accuracy / Stability ---
NUM_SAMPLES_PER_UPDATE = 7
NUM_SAMPLES_CALIBRATION = 10
GUI_UPDATE_INTERVAL_MS = 250
LDC_DISPLAY_BUFFER_SIZE = 5

# --- Image Enhancement / Saving ---
CLAHE_CLIP_LIMIT = 2.0
CLAHE_TILE_GRID_SIZE = (8, 8)
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
IDLE_RP_VALUE = 0 # Needs calibration before delta_rp is meaningful

# --- Dataset / File Saving ---
PROJECT_FOLDER_NAME = "Project_Dataset"
IMAGES_FOLDER_NAME = "images"
METADATA_FILENAME = "metadata.csv"
METADATA_HEADER = [
    'image_filename', 'magnetism_mT', 'ldc_rp', 'delta_rp', 'target_material',
    'is_coated', 'is_dilapidated', 'is_degraded' # New columns
]
try: BASE_PATH = os.path.dirname(os.path.abspath(__file__))
except NameError: BASE_PATH = os.getcwd()
PROJECT_PATH = os.path.join(BASE_PATH, PROJECT_FOLDER_NAME)
IMAGES_PATH = os.path.join(PROJECT_PATH, IMAGES_FOLDER_NAME)
METADATA_PATH = os.path.join(PROJECT_PATH, METADATA_FILENAME)
FILENAME_PREFIX = "data_"
FILENAME_PADDING = 4 # e.g., data_0001.jpg

# --- Target Materials ---
TARGET_OPTIONS = [
    "Copper", "Steel", "Iron", "Aluminum", "Zinc",
    "Brass", "Gold", "Silver", "Bronze", "Others"
]

# --- Global Hardware Objects ---
camera = None; i2c = None; ads = None; hall_sensor = None
spi = None; ldc_initialized = False; clahe_processor = None

# --- Global State ---
RP_DISPLAY_BUFFER = deque(maxlen=LDC_DISPLAY_BUFFER_SIZE)
data_counter = 1 # Default start, updated by initialize_global_state

# --- GUI Globals ---
# Declared global just before use or defined in setup_gui and made global there
window = None; camera_label = None; controls_frame = None; feedback_label = None
magnetism_label = None; ldc_label = None; target_var = None
capture_button = None; calibrate_button = None
# Checkbox variables - declare them here, assign in setup_gui
coated_var = None; dilapidated_var = None; degraded_var = None
# Font Objects
label_font = None; readout_font = None; button_font = None
feedback_font = None; title_font = None; check_font = None

# =========================
# === File/State Utils ====
# =========================
def get_next_data_index(img_dir, prefix, padding):
    max_index = 0; start_index = 1 # Default if no files found
    try:
        if os.path.isdir(img_dir):
            filename_pattern = re.compile(f"^{prefix}(\\d{{{padding}}})\\.jpg$")
            print(f"Scanning {img_dir} for files matching '{prefix}' + {padding} digits...")
            found_files = False
            for filename in os.listdir(img_dir):
                match = filename_pattern.match(filename)
                if match:
                    found_files = True
                    try: index = int(match.group(1)); max_index = max(max_index, index)
                    except ValueError: continue # Ignore non-numeric matches after prefix
            if found_files: return max_index + 1
            else: return start_index # No matching files found, start at 1
        else:
            print(f"Image directory {img_dir} not found. Starting index at {start_index}.")
            return start_index # Directory doesn't exist yet
    except OSError as e: print(f"Warning: Could not read image directory {img_dir}: {e}"); return start_index
    except Exception as e: print(f"Error finding next index: {e}"); return start_index

def initialize_global_state():
    global data_counter
    if not setup_project_directory():
         print("Fatal Error: Could not create project directories. Exiting.")
         return False
    print("Finding next available data index...")
    data_counter = get_next_data_index(IMAGES_PATH, FILENAME_PREFIX, FILENAME_PADDING)
    print(f"Starting data counter at: {data_counter}")
    return True

# =========================
# === Directory Setup =====
# =========================
def setup_project_directory():
    print(f"Ensuring dataset directory exists at: {PROJECT_PATH}")
    try: os.makedirs(PROJECT_PATH, exist_ok=True); os.makedirs(IMAGES_PATH, exist_ok=True); return True
    except OSError as e: print(f"Error creating directories: {e}"); return False

# =========================
# === Hardware Setup ===
# =========================
def initialize_hardware():
    global camera, i2c, ads, hall_sensor, spi, ldc_initialized, clahe_processor
    print("--- Initializing Hardware ---")
    try: camera = cv2.VideoCapture(CAMERA_INDEX)
    except Exception as e: print(f"Error opening camera: {e}"); camera = None
    if camera and not camera.isOpened(): print(f"Error: Could not open camera {CAMERA_INDEX}"); camera = None
    if I2C_ENABLED:
        try: i2c = busio.I2C(board.SCL, board.SDA); ads = ADS.ADS1115(i2c); hall_sensor = AnalogIn(ads, HALL_ADC_CHANNEL); print("ADS1115 Initialized.")
        except Exception as e: print(f"Error initializing I2C/ADS1115: {e}"); hall_sensor = None
    else: print("Skipping I2C/ADS1115 setup.")
    if SPI_ENABLED:
        try:
            GPIO.setwarnings(False); GPIO.setmode(GPIO.BCM); GPIO.setup(CS_PIN, GPIO.OUT); GPIO.output(CS_PIN, GPIO.HIGH); print("GPIO Initialized.")
            spi = spidev.SpiDev(); spi.open(SPI_BUS, SPI_DEVICE); spi.max_speed_hz = SPI_SPEED; spi.mode = SPI_MODE; print(f"SPI Initialized.")
            if initialize_ldc1101(): enable_ldc_rpmode()
            else: print("LDC1101 Initialization Failed.")
        except Exception as e: print(f"Error initializing GPIO/SPI: {e}"); spi = None; ldc_initialized = False
    else: print("Skipping SPI/GPIO/LDC1101 setup.")
    clahe_processor = cv2.createCLAHE(clipLimit=CLAHE_CLIP_LIMIT, tileGridSize=CLAHE_TILE_GRID_SIZE)
    print("--- Hardware Initialization Complete ---")

# =========================
# === LDC1101 Functions ===
# =========================
# (No changes needed in these core LDC functions)
def ldc_write_register(reg_addr, value):
    if not spi: return
    try: GPIO.output(CS_PIN, GPIO.LOW); spi.xfer2([reg_addr & 0x7F, value]); GPIO.output(CS_PIN, GPIO.HIGH)
    except Exception as e: print(f"Error writing LDC 0x{reg_addr:02X}: {e}")
def ldc_read_register(reg_addr):
    if not spi: return 0
    try: GPIO.output(CS_PIN, GPIO.LOW); result = spi.xfer2([reg_addr | 0x80, 0x00]); GPIO.output(CS_PIN, GPIO.HIGH); return result[1]
    except Exception as e: print(f"Error reading LDC 0x{reg_addr:02X}: {e}"); return 0
def initialize_ldc1101():
    global ldc_initialized; ldc_initialized = False; print("Checking LDC1101 Chip ID...")
    if not spi: return False; chip_id = ldc_read_register(CHIP_ID_REG)
    if chip_id != LDC_CHIP_ID: print(f"LDC ID Mismatch: 0x{chip_id:02X}"); return False
    print("Configuring LDC1101..."); ldc_write_register(RP_SET_REG, 0x1B); ldc_write_register(TC1_REG, 0x80); ldc_write_register(TC2_REG, 0x88)
    ldc_write_register(DIG_CONFIG_REG, 0x07); ldc_write_register(ALT_CONFIG_REG, 0x02); ldc_write_register(D_CONF_REG, 0x00); ldc_write_register(INTB_MODE_REG, 0x00)
    ldc_write_register(START_CONFIG_REG, SLEEP_MODE); time.sleep(0.05); print("LDC1101 Configured."); ldc_initialized = True; return True
def enable_ldc_powermode(mode):
    if not spi: return; ldc_write_register(START_CONFIG_REG, mode); time.sleep(0.02)
def enable_ldc_rpmode():
    if not spi or not ldc_initialized: return; print("Enabling LDC RP Mode..."); ldc_write_register(ALT_CONFIG_REG, 0x02); ldc_write_register(D_CONF_REG, 0x00); enable_ldc_powermode(ACTIVE_CONVERSION_MODE); print("LDC RP Mode Active.")
def get_ldc_rpdata():
    if not spi or not ldc_initialized: return None
    try: msb = ldc_read_register(RP_DATA_MSB_REG); lsb = ldc_read_register(RP_DATA_LSB_REG); return (msb << 8) | lsb
    except Exception as e: print(f"Error in get_ldc_rpdata: {e}"); return None

# =========================
# === CSV Handling =========
# =========================
def append_metadata(image_filename, mag_mT, rp_value, delta_rp, target, is_coated, is_dilapidated, is_degraded):
    # (Corrected version from v1.3)
    file_exists = os.path.isfile(METADATA_PATH)
    try:
        with open(METADATA_PATH, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            if not file_exists or os.path.getsize(METADATA_PATH) == 0: writer.writerow(METADATA_HEADER)
            mag_mT_str = f"{mag_mT:.4f}" if mag_mT is not None else "N/A"
            rp_value_str = str(rp_value) if rp_value is not None else "N/A"
            delta_rp_str = str(delta_rp) if delta_rp is not None else "N/A"
            writer.writerow([ image_filename, mag_mT_str, rp_value_str, delta_rp_str, target, is_coated, is_dilapidated, is_degraded ])
        return True
    except IOError as e: print(f"Error writing metadata: {e}"); return False
    except Exception as e: print(f"Unexpected CSV error: {e}"); return False

# ============================
# === Sensor Reading (Avg) ===
# ============================
def get_averaged_hall_voltage(num_samples=NUM_SAMPLES_PER_UPDATE):
    # (Remains the same)
    if not hall_sensor: return None; readings = []
    for _ in range(num_samples):
        try: readings.append(hall_sensor.voltage); time.sleep(0.002)
        except Exception as e: print(f"Hall read error: {e}"); pass
    if not readings: return None; return sum(readings) / len(readings)
def get_averaged_rp_data(num_samples=NUM_SAMPLES_PER_UPDATE):
    # (Remains the same)
    if not ldc_initialized: return None; readings = []
    for _ in range(num_samples): val = get_ldc_rpdata(); time.sleep(0.002); (readings.append(val) if val is not None else None)
    if not readings: return None; return sum(readings) / len(readings)

# ==============================
# === Image Enhancement Func ===
# ==============================
def enhance_image_contrast(frame_rgb):
    # (Remains the same)
    if clahe_processor is None: return frame_rgb
    try: lab = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2LAB); l, a, b = cv2.split(lab); l_clahe = clahe_processor.apply(l); lab_clahe = cv2.merge((l_clahe, a, b)); return cv2.cvtColor(lab_clahe, cv2.COLOR_LAB2RGB)
    except Exception as e: print(f"Error applying CLAHE: {e}"); return frame_rgb

# ======================
# === GUI Functions ===
# ======================
def capture_and_save_data():
    # (Updated for naming, resize, checkboxes)
    global feedback_label, capture_button, window, target_var, IDLE_RP_VALUE
    global coated_var, dilapidated_var, degraded_var, data_counter # Needs checkbox vars and counter

    if not camera: feedback_label.config(text="Camera not available", foreground="#E53935"); return

    capture_button.config(state=tk.DISABLED)
    feedback_label.config(text="Capturing data...", foreground="#FFA726")
    window.update()

    ret, frame = camera.read()
    if not ret: feedback_label.config(text="Failed to capture photo", foreground="#E53935"); capture_button.config(state=tk.NORMAL); return

    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    frame_enhanced_rgb = enhance_image_contrast(frame_rgb)
    img_enhanced = Image.fromarray(frame_enhanced_rgb)

    # Resize image
    try:
        # Use recommended attributes first
        resampling_filter = Image.Resampling.LANCZOS if hasattr(Image, 'Resampling') else Image.LANCZOS
        img_resized_for_save = img_enhanced.resize((SAVE_IMG_WIDTH, SAVE_IMG_HEIGHT), resampling_filter)
    except Exception as e: print(f"Error resizing image: {e}"); feedback_label.config(text=f"Resize Error: {e}", foreground="#E53935"); capture_button.config(state=tk.NORMAL); return

    # Generate filename
    image_filename = f"{FILENAME_PREFIX}{data_counter:0{FILENAME_PADDING}d}.jpg"
    image_save_path = os.path.join(IMAGES_PATH, image_filename)

    # Get Sensor Readings
    avg_voltage = get_averaged_hall_voltage(num_samples=NUM_SAMPLES_CALIBRATION)
    current_mag_mT = None
    if avg_voltage is not None:
        try: adj_v = avg_voltage - IDLE_VOLTAGE; current_mag_mT = adj_v / SENSITIVITY_V_PER_MILLITESLA
        except Exception: current_mag_mT = None
    current_rp_val = get_averaged_rp_data(num_samples=NUM_SAMPLES_CALIBRATION)
    delta_rp = None
    if current_rp_val is not None:
        current_rp_val = int(current_rp_val)
        # Ensure IDLE_RP_VALUE is valid (not 0 or initial state if calibration matters)
        # For now, just check if it's not 0, assuming 0 is unlikely for a real idle value after cal
        if IDLE_RP_VALUE != 0: delta_rp = current_rp_val - IDLE_RP_VALUE

    # Get Target and Checkbox Values
    selected_target = target_var.get()
    is_coated = coated_var.get()
    is_dilapidated = dilapidated_var.get()
    is_degraded = degraded_var.get()

    # Save Image
    save_success = False
    try: img_resized_for_save.save(image_save_path); print(f"Resized image saved: {image_save_path}"); save_success = True
    except Exception as e: print(f"Error saving photo: {e}"); feedback_label.config(text=f"Save Error: {e}", foreground="#E53935"); capture_button.config(state=tk.NORMAL); return

    # Append Metadata
    meta_success = append_metadata(image_filename, current_mag_mT, current_rp_val, delta_rp, selected_target, is_coated, is_dilapidated, is_degraded)
    if meta_success:
        feedback_label.config(text=f"Data Added: {image_filename}", foreground="#66BB6A")
        data_counter += 1 # Increment counter only on full success
    else: feedback_label.config(text="Image saved, metadata FAILED", foreground="#E53935")

    window.after(1500, lambda: capture_button.config(state=tk.NORMAL))
    window.after(3000, lambda: feedback_label.config(text=""))

def calibrate_sensors():
    # (No changes needed here)
    global IDLE_VOLTAGE, IDLE_RP_VALUE, feedback_label, window
    feedback_text = ""; feedback_color = "#29B6F6"
    if hall_sensor:
        voltages = []; feedback_label.config(text="Calibrating Hall...", foreground="#FFA726"); window.update()
        for _ in range(NUM_SAMPLES_CALIBRATION): try: voltages.append(hall_sensor.voltage); time.sleep(0.05) except Exception: pass
        if voltages: IDLE_VOLTAGE = sum(voltages) / len(voltages); feedback_text += f"Hall Idle: {IDLE_VOLTAGE:.4f} V\n"
        else: feedback_text += "Hall Cal Error: No readings\n"; feedback_color = "#FFA726"
    else: feedback_text += "Hall Sensor N/A\n"; feedback_color = "#FFA726"
    if ldc_initialized:
        rp_readings = []; feedback_label.config(text=feedback_text + "Calibrating LDC...", foreground="#FFA726"); window.update()
        for _ in range(NUM_SAMPLES_CALIBRATION): val = get_ldc_rpdata(); time.sleep(0.05); (rp_readings.append(val) if val is not None else None)
        if rp_readings: IDLE_RP_VALUE = int(sum(rp_readings) / len(rp_readings)); feedback_text += f"LDC RP Idle: {IDLE_RP_VALUE}"
        else: feedback_text += "LDC Cal Error: No readings\n"; feedback_color = "#E53935"
    else: feedback_text += "LDC Sensor N/A"; feedback_color = "#E53935" if feedback_color != "#FFA726" else "#FFA726"
    feedback_label.config(text=feedback_text.strip(), foreground=feedback_color)
    window.after(4000, lambda: feedback_label.config(text=""))

def update_camera_feed():
    # (No changes needed here)
    global camera_label, window
    if camera:
        ret, frame = camera.read()
        if ret:
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB); frame_enhanced_rgb = enhance_image_contrast(frame_rgb)
            img = Image.fromarray(frame_enhanced_rgb).resize((DISPLAY_IMG_WIDTH, DISPLAY_IMG_HEIGHT))
            img_tk = ImageTk.PhotoImage(img); camera_label.img_tk = img_tk; camera_label.configure(image=img_tk)
    else:
         if not hasattr(camera_label, 'no_cam_img'): placeholder = Image.new('RGB', (DISPLAY_IMG_WIDTH, DISPLAY_IMG_HEIGHT), color='#BDBDBD'); camera_label.no_cam_img = ImageTk.PhotoImage(placeholder)
         camera_label.configure(image=camera_label.no_cam_img)
    window.after(50, update_camera_feed)

def update_magnetism():
    # (No changes needed here)
    global magnetism_label, window
    avg_voltage = get_averaged_hall_voltage()
    if avg_voltage is not None:
        try: adj_v = avg_voltage - IDLE_VOLTAGE; mag_mT = adj_v / SENSITIVITY_V_PER_MILLITESLA
            if abs(mag_mT) < 1: magnetism_label.config(text=f"{mag_mT * 1000:.2f} µT")
            else: magnetism_label.config(text=f"{mag_mT:.2f} mT")
        except Exception: magnetism_label.config(text="Error")
    else: magnetism_label.config(text="N/A" if not hall_sensor else "Error")
    window.after(GUI_UPDATE_INTERVAL_MS, update_magnetism)

def update_ldc_reading():
    # (No changes needed here, uses moving average buffer)
    global ldc_label, window, RP_DISPLAY_BUFFER
    avg_rp_val = get_averaged_rp_data(); display_rp_text = "N/A"
    if not ldc_initialized: display_rp_text = "N/A"
    elif avg_rp_val is not None:
        RP_DISPLAY_BUFFER.append(avg_rp_val)
        if len(RP_DISPLAY_BUFFER) > 0: moving_avg_rp = sum(RP_DISPLAY_BUFFER) / len(RP_DISPLAY_BUFFER); display_rp_text = f"{int(moving_avg_rp)}"
        else: display_rp_text = "..."
    else: RP_DISPLAY_BUFFER.clear(); display_rp_text = "Error"
    ldc_label.config(text=display_rp_text)
    window.after(GUI_UPDATE_INTERVAL_MS, update_ldc_reading)

# ======================
# === GUI Setup ========
# ======================
def setup_gui():
    # << Added Checkbuttons, fixed feedback label row height, fixed checkbox var scope >>
    global window, camera_label, controls_frame, feedback_label # GUI elements
    global magnetism_label, ldc_label, target_var, capture_button, calibrate_button # GUI elements
    global coated_var, dilapidated_var, degraded_var # Checkbox variables made global
    global label_font, readout_font, button_font, feedback_font, title_font, check_font # Fonts made global

    window = tk.Tk(); window.title("Sensor Data Acquisition Tool v1.3.1"); window.geometry("1100x700")
    style = ttk.Style(); style.theme_use('clam' if 'clam' in style.theme_names() else 'default')
    # Define fonts globally so they can be used for row height calculation
    title_font = tkFont.Font(family="Helvetica", size=14, weight="bold")
    label_font = tkFont.Font(family="Helvetica", size=11)
    readout_font = tkFont.Font(family="Consolas", size=14, weight="bold")
    button_font = tkFont.Font(family="Helvetica", size=11, weight="bold")
    feedback_font = tkFont.Font(family="Helvetica", size=10)
    check_font = tkFont.Font(family="Helvetica", size=10)
    style.configure("TLabel", font=label_font, padding=3); style.configure("TButton", font=button_font, padding=(10, 8))
    style.configure("TMenubutton", font=label_font, padding=5); style.configure("TLabelframe", padding=10)
    style.configure("TLabelframe.Label", font=tkFont.Font(family="Helvetica", size=12, weight="bold"))
    style.configure("Feedback.TLabel", font=feedback_font, padding=5)
    style.configure("Readout.TLabel", font=readout_font, padding=(5, 2))
    style.configure("Unit.TLabel", font=label_font, padding=(0, 2))
    style.configure("TCheckbutton", font=check_font, padding=3)

    main_frame = ttk.Frame(window, padding="15 15 15 15"); main_frame.pack(fill=tk.BOTH, expand=True)
    main_frame.columnconfigure(0, weight=3); main_frame.columnconfigure(1, weight=1); main_frame.rowconfigure(0, weight=1)

    camera_label = ttk.Label(main_frame, text="Initializing Camera...", anchor="center", borderwidth=1, relief="sunken")
    camera_label.grid(row=0, column=0, padx=(0, 15), pady=0, sticky="nsew")

    controls_frame = ttk.Frame(main_frame); controls_frame.grid(row=0, column=1, padx=(0, 0), pady=0, sticky="nsew")
    controls_frame.columnconfigure(0, weight=1)
    row_idx = 0

    # Feedback Area (configure row minsize)
    feedback_label_row = row_idx
    feedback_label = ttk.Label(controls_frame, text="", style="Feedback.TLabel", anchor="w", wraplength=300)
    feedback_label.grid(row=feedback_label_row, column=0, sticky="ew", pady=(0, 10))
    # Set minimum height using font metrics
    feedback_row_minheight = int(feedback_font.metrics('linespace') * 2.5) # Estimate for ~2.5 lines
    controls_frame.rowconfigure(feedback_label_row, minsize=feedback_row_minheight)
    row_idx += 1

    # Sensor Readings Group
    readings_frame = ttk.Labelframe(controls_frame, text=" Sensor Readings ", padding="15 10 15 10")
    readings_frame.grid(row=row_idx, column=0, sticky="new", pady=(0, 15)); row_idx += 1
    readings_frame.columnconfigure(0, weight=0); readings_frame.columnconfigure(1, weight=1)
    ttk.Label(readings_frame, text="Magnetism:").grid(row=0, column=0, sticky="w", padx=(0, 10))
    magnetism_label = ttk.Label(readings_frame, text="Init...", style="Readout.TLabel", anchor="e")
    magnetism_label.grid(row=0, column=1, sticky="ew")
    ttk.Label(readings_frame, text="LDC RP:").grid(row=1, column=0, sticky="w", padx=(0, 10), pady=(5,0))
    ldc_label = ttk.Label(readings_frame, text="Init...", style="Readout.TLabel", anchor="e")
    ldc_label.grid(row=1, column=1, sticky="ew", pady=(5,0))

    # Data Capture Group
    actions_frame = ttk.Labelframe(controls_frame, text=" Data Capture ", padding="15 10 15 15")
    actions_frame.grid(row=row_idx, column=0, sticky="new", pady=(0, 15)); row_idx += 1
    actions_frame.columnconfigure(0, weight=1)
    action_row_idx = 0

    target_title_label = ttk.Label(actions_frame, text="Target Material:"); target_title_label.grid(row=action_row_idx, column=0, sticky="w", pady=(0, 3)); action_row_idx += 1
    target_var = tk.StringVar(window); target_var.set(TARGET_OPTIONS[0])
    target_dropdown = ttk.OptionMenu(actions_frame, target_var, TARGET_OPTIONS[0], *TARGET_OPTIONS, style="TMenubutton")
    target_dropdown.grid(row=action_row_idx, column=0, sticky="ew", pady=(0, 10)); action_row_idx += 1

    # Condition Checkboxes Area
    condition_label = ttk.Label(actions_frame, text="Conditions:")
    condition_label.grid(row=action_row_idx, column=0, sticky="w", pady=(5, 3)); action_row_idx += 1
    # Assign to global vars defined earlier
    coated_var = tk.IntVar(value=0)
    dilapidated_var = tk.IntVar(value=0)
    degraded_var = tk.IntVar(value=0)
    cb_coated = ttk.Checkbutton(actions_frame, text="Coated", variable=coated_var, onvalue=1, offvalue=0)
    cb_coated.grid(row=action_row_idx, column=0, sticky="w"); action_row_idx += 1
    cb_dilapidated = ttk.Checkbutton(actions_frame, text="Dilapidated", variable=dilapidated_var, onvalue=1, offvalue=0)
    cb_dilapidated.grid(row=action_row_idx, column=0, sticky="w"); action_row_idx += 1
    cb_degraded = ttk.Checkbutton(actions_frame, text="Degraded", variable=degraded_var, onvalue=1, offvalue=0)
    cb_degraded.grid(row=action_row_idx, column=0, sticky="w", pady=(0, 10)); action_row_idx += 1

    ttk.Separator(actions_frame, orient='horizontal').grid(row=action_row_idx, column=0, sticky='ew', pady=(0, 15)); action_row_idx += 1
    # Buttons
    capture_button = ttk.Button(actions_frame, text="Capture & Add Data", command=capture_and_save_data)
    capture_button.grid(row=action_row_idx, column=0, sticky="ew", pady=(0, 8)); action_row_idx += 1
    calibrate_button = ttk.Button(actions_frame, text="Calibrate Sensors", command=calibrate_sensors)
    calibrate_button.grid(row=action_row_idx, column=0, sticky="ew"); action_row_idx += 1

# ==========================
# === Main Execution =======
# ==========================
def run_application():
    global window # Make window global for cleanup if setup fails
    # << Call state initialization which includes directory setup >>
    if not initialize_global_state():
        # Optional: Show a Tkinter error message box before exiting
        # try:
        #     root = tk.Tk()
        #     root.withdraw() # Hide the main window
        #     tk.messagebox.showerror("Startup Error", "Failed to create project directories. Exiting.")
        # except Exception:
        #     pass # Ignore if GUI fails here
        sys.exit(1) # Exit if state init fails

    setup_gui()
    if not camera: camera_label.configure(text="Camera Failed")
    if not hall_sensor: magnetism_label.config(text="N/A")
    if not ldc_initialized: ldc_label.config(text="N/A")
    update_camera_feed(); update_magnetism(); update_ldc_reading()
    print("Starting Tkinter main loop...")
    window.mainloop()

# --- Cleanup ---
def cleanup_resources():
    # (No changes needed here)
    print("Cleaning up resources...")
    if camera and camera.isOpened(): print("Releasing camera..."); camera.release()
    cv2.destroyAllWindows()
    if spi: print("Closing SPI..."); spi.close()
    if SPI_ENABLED: # Only cleanup GPIO if it was initialized
        try: print("Cleaning up GPIO..."); GPIO.cleanup()
        except Exception as e: print(f"Note: GPIO cleanup error: {e}")
    print("Cleanup complete.")

# --- Run ---
# <<<< This is the main execution block >>>>
# Ensure indentation is correct here and no stray characters exist.
if __name__ == '__main__':
    # Initialize hardware first
    initialize_hardware()
    # Setup the main application loop with error handling
    try: # <<< START OF TRY BLOCK (Likely around Line 460+)
        run_application() # This function now initializes state and runs GUI
    except Exception as e: # <<< Corresponding Except block
        print(f"--- UNEXPECTED ERROR IN MAIN APPLICATION ---")
        print(f"{type(e).__name__}: {e}")
        import traceback
        traceback.print_exc()
        print(f"--------------------------------------------")
    finally: # <<< Corresponding Finally block
        # This block executes whether an exception occurred or not
        cleanup_resources()
