# Combined Code: Camera, Magnetism, Inductance Measurement with Dataset Creation
# Version: 1.3.4 - Corrected Syntax in calibrate_sensors try block

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
FILENAME_PADDING = 4

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
data_counter = 1

# --- GUI Globals ---
window = None; camera_label = None; controls_frame = None; feedback_label = None
magnetism_label = None; ldc_label = None; target_var = None
capture_button = None; calibrate_button = None
coated_var = None; dilapidated_var = None; degraded_var = None
label_font = None; readout_font = None; button_font = None
feedback_font = None; title_font = None; check_font = None

# =========================
# === File/State Utils ====
# =========================
def get_next_data_index(img_dir, prefix, padding):
    """Finds the next sequential index based on existing files."""
    max_index = 0
    if not os.path.isdir(img_dir):
        print(f"Info: Image directory not found ({img_dir}). Starting index at 1.")
        return 1
    try:
        filename_pattern_str = f"^{prefix}(\\d{{{padding}}})\\.jpg$"
        filename_pattern = re.compile(filename_pattern_str)
        for filename in os.listdir(img_dir):
            match = filename_pattern.match(filename)
            if match:
                try:
                    index = int(match.group(1))
                    max_index = max(max_index, index)
                except ValueError:
                    print(f"Warning: Could not parse index from filename '{filename}'.")
                    continue
    except OSError as e:
        print(f"Warning: Could not read image directory {img_dir}: {e}")
    return max_index + 1

def initialize_global_state():
    """Initializes state variables like the data counter."""
    global data_counter
    if not setup_project_directory():
         print("Fatal Error: Directory setup failed. Cannot initialize state.")
         return False
    print("Finding next available data index...")
    data_counter = get_next_data_index(IMAGES_PATH, FILENAME_PREFIX, FILENAME_PADDING)
    print(f"Starting data counter at: {data_counter}")
    return True

# =========================
# === Directory Setup =====
# =========================
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
def initialize_hardware():
    """Initializes Camera, I2C/ADS1115, SPI/GPIO/LDC1101."""
    global camera, i2c, ads, hall_sensor, spi, ldc_initialized, clahe_processor
    print("--- Initializing Hardware ---")
    # Camera
    try: camera = cv2.VideoCapture(CAMERA_INDEX)
    except Exception as e: print(f"Error opening camera: {e}"); camera = None
    if camera and not camera.isOpened(): print(f"Error: Could not open camera {CAMERA_INDEX}"); camera = None
    # I2C/ADS1115
    if I2C_ENABLED:
        try: i2c = busio.I2C(board.SCL, board.SDA); ads = ADS.ADS1115(i2c); hall_sensor = AnalogIn(ads, HALL_ADC_CHANNEL); print("ADS1115 Initialized.")
        except Exception as e: print(f"Error initializing I2C/ADS1115: {e}"); hall_sensor = None
    else: print("Skipping I2C/ADS1115 setup.")
    # SPI/GPIO/LDC1101
    if SPI_ENABLED:
        try:
            GPIO.setwarnings(False); GPIO.setmode(GPIO.BCM); GPIO.setup(CS_PIN, GPIO.OUT); GPIO.output(CS_PIN, GPIO.HIGH); print("GPIO Initialized.")
            spi = spidev.SpiDev(); spi.open(SPI_BUS, SPI_DEVICE); spi.max_speed_hz = SPI_SPEED; spi.mode = SPI_MODE; print(f"SPI Initialized (Bus={SPI_BUS}, Dev={SPI_DEVICE}).")
            if initialize_ldc1101(): enable_ldc_rpmode()
            else: print("LDC1101 Initialization Failed.")
        except Exception as e: print(f"Error initializing GPIO/SPI: {e}"); spi = None; ldc_initialized = False
    else: print("Skipping SPI/GPIO/LDC1101 setup.")
    # CLAHE
    clahe_processor = cv2.createCLAHE(clipLimit=CLAHE_CLIP_LIMIT, tileGridSize=CLAHE_TILE_GRID_SIZE)
    print("--- Hardware Initialization Complete ---")


# =========================
# === LDC1101 Functions ===
# =========================
def ldc_write_register(reg_addr, value):
    """Writes a byte value to an LDC1101 register via SPI."""
    if not spi: return
    try: GPIO.output(CS_PIN, GPIO.LOW); spi.xfer2([reg_addr & 0x7F, value]); GPIO.output(CS_PIN, GPIO.HIGH)
    except Exception as e: print(f"Error writing LDC 0x{reg_addr:02X}: {e}")

def ldc_read_register(reg_addr):
    """Reads a byte value from an LDC1101 register via SPI."""
    if not spi: return 0
    try: GPIO.output(CS_PIN, GPIO.LOW); result = spi.xfer2([reg_addr | 0x80, 0x00]); GPIO.output(CS_PIN, GPIO.HIGH); return result[1]
    except Exception as e: print(f"Error reading LDC 0x{reg_addr:02X}: {e}"); return 0

def initialize_ldc1101():
    """Initializes the LDC1101 with custom settings for RP mode."""
    global ldc_initialized; ldc_initialized = False
    if not spi: return False
    chip_id = ldc_read_register(CHIP_ID_REG)
    if chip_id != LDC_CHIP_ID: print(f"LDC Chip ID Mismatch: Read 0x{chip_id:02X}"); return False
    print("Configuring LDC1101..."); ldc_write_register(RP_SET_REG, 0x1B); ldc_write_register(TC1_REG, 0x80); ldc_write_register(TC2_REG, 0x88)
    ldc_write_register(DIG_CONFIG_REG, 0x07); ldc_write_register(ALT_CONFIG_REG, 0x02); ldc_write_register(D_CONF_REG, 0x00); ldc_write_register(INTB_MODE_REG, 0x00)
    ldc_write_register(START_CONFIG_REG, SLEEP_MODE); time.sleep(0.05); print("LDC1101 Configured."); ldc_initialized = True; return True

def enable_ldc_powermode(mode):
    """Sets the power mode of the LDC1101."""
    if not spi: return; ldc_write_register(START_CONFIG_REG, mode); time.sleep(0.02)

def enable_ldc_rpmode():
    """Configures and enables RP conversion mode."""
    if not spi or not ldc_initialized: return; print("Enabling LDC RP Mode..."); ldc_write_register(ALT_CONFIG_REG, 0x02); ldc_write_register(D_CONF_REG, 0x00); enable_ldc_powermode(ACTIVE_CONVERSION_MODE); print("LDC RP Mode Active.")

def get_ldc_rpdata():
    """Reads the 16-bit RP data from the LDC1101."""
    if not spi or not ldc_initialized: return None
    try: msb = ldc_read_register(RP_DATA_MSB_REG); lsb = ldc_read_register(RP_DATA_LSB_REG); return (msb << 8) | lsb
    except Exception as e: print(f"Error in get_ldc_rpdata: {e}"); return None

# =========================
# === CSV Handling =========
# =========================
def append_metadata(image_filename, mag_mT, rp_value, delta_rp, target, is_coated, is_dilapidated, is_degraded):
    """Appends a row of data to the metadata CSV file."""
    file_exists = os.path.isfile(METADATA_PATH)
    try:
        with open(METADATA_PATH, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            if not file_exists or os.path.getsize(METADATA_PATH) == 0: writer.writerow(METADATA_HEADER)
            mag_mT_str = f"{mag_mT:.4f}" if mag_mT is not None else "N/A"
            rp_value_str = str(rp_value) if rp_value is not None else "N/A"
            delta_rp_str = str(delta_rp) if delta_rp is not None else "N/A"
            writer.writerow([image_filename, mag_mT_str, rp_value_str, delta_rp_str, target, is_coated, is_dilapidated, is_degraded])
        return True
    except IOError as e: print(f"Error writing metadata: {e}"); return False
    except Exception as e: print(f"Unexpected CSV error: {e}"); return False

# ============================
# === Sensor Reading (Avg) ===
# ============================
def get_averaged_hall_voltage(num_samples=NUM_SAMPLES_PER_UPDATE):
    """Reads the Hall sensor multiple times and returns the average voltage."""
    if not hall_sensor: return None; readings = []
    for _ in range(num_samples):
        try: readings.append(hall_sensor.voltage); time.sleep(0.002)
        except Exception: pass
    if not readings: return None; return sum(readings) / len(readings)

def get_averaged_rp_data(num_samples=NUM_SAMPLES_PER_UPDATE):
    """Reads the LDC RP value multiple times and returns the average."""
    if not ldc_initialized: return None; readings = []
    for _ in range(num_samples):
        val = get_ldc_rpdata(); time.sleep(0.002); (readings.append(val) if val is not None else None)
    if not readings: return None; return sum(readings) / len(readings)

# ==============================
# === Image Enhancement Func ===
# ==============================
def enhance_image_contrast(frame_rgb):
    """Applies CLAHE to enhance contrast, especially for glare reduction."""
    if clahe_processor is None: return frame_rgb
    try:
        lab = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2LAB); l, a, b = cv2.split(lab)
        l_clahe = clahe_processor.apply(l); lab_clahe = cv2.merge((l_clahe, a, b))
        return cv2.cvtColor(lab_clahe, cv2.COLOR_LAB2RGB)
    except Exception as e: print(f"Error applying CLAHE: {e}"); return frame_rgb

# ======================
# === GUI Functions ===
# ======================
def capture_and_save_data():
    """Captures image, reads sensors, gets target, saves image, appends metadata."""
    global feedback_label, capture_button, window, target_var, IDLE_RP_VALUE
    global coated_var, dilapidated_var, degraded_var
    global data_counter

    if not camera: feedback_label.config(text="Camera not available", foreground="#E53935"); return

    capture_button.config(state=tk.DISABLED); feedback_label.config(text="Capturing data...", foreground="#FFA726"); window.update()

    ret, frame = camera.read()
    if not ret: feedback_label.config(text="Failed to capture photo", foreground="#E53935"); capture_button.config(state=tk.NORMAL); return

    # Process Image
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    frame_enhanced_rgb = enhance_image_contrast(frame_rgb)
    img_enhanced = Image.fromarray(frame_enhanced_rgb)
    try: img_resized_for_save = img_enhanced.resize((SAVE_IMG_WIDTH, SAVE_IMG_HEIGHT), Image.Resampling.LANCZOS)
    except AttributeError: img_resized_for_save = img_enhanced.resize((SAVE_IMG_WIDTH, SAVE_IMG_HEIGHT), Image.LANCZOS)
    except Exception as e: print(f"Error resizing image: {e}"); feedback_label.config(text=f"Resize Error: {e}", foreground="#E53935"); capture_button.config(state=tk.NORMAL); return

    # Generate Filename
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
        if IDLE_RP_VALUE != 0: delta_rp = current_rp_val - IDLE_RP_VALUE

    # Get Target and Condition Flags
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
        data_counter += 1 # Increment only on full success
    else:
        feedback_label.config(text="Image saved, metadata FAILED", foreground="#E53935")

    window.after(1500, lambda: capture_button.config(state=tk.NORMAL))
    window.after(3500, lambda: feedback_label.config(text=""))

# --- CORRECTED calibrate_sensors Function ---
def calibrate_sensors():
    """Calibrates idle voltage for Hall sensor and idle RP value for LDC1101."""
    global IDLE_VOLTAGE, IDLE_RP_VALUE, feedback_label, window
    feedback_text = ""
    feedback_color = "#29B6F6" # Blue color

    # Calibrate Hall Sensor
    if hall_sensor:
        voltages = []
        feedback_label.config(text="Calibrating Hall...", foreground="#FFA726")
        window.update()
        # Corrected Try/Except Structure
        for _ in range(NUM_SAMPLES_CALIBRATION):
            try:
                voltages.append(hall_sensor.voltage)
                time.sleep(0.05)
            except Exception as e:
                # print(f"Warning: Hall read error during calib sample: {e}") # Optional
                pass # Ignore read errors during calibration sample gathering
        # End Corrected Structure
        if voltages:
            IDLE_VOLTAGE = sum(voltages) / len(voltages)
            feedback_text += f"Hall Idle: {IDLE_VOLTAGE:.4f} V\n"
        else:
            feedback_text += "Hall Cal Error: No valid readings\n"
            feedback_color = "#FFA726" # Orange
    else:
        feedback_text += "Hall Sensor N/A\n"
        feedback_color = "#FFA726"

    # Calibrate LDC1101
    if ldc_initialized:
        rp_readings = []
        feedback_label.config(text=feedback_text + "Calibrating LDC...", foreground="#FFA726")
        window.update()
        # Rewritten LDC Loop for Clarity
        for _ in range(NUM_SAMPLES_CALIBRATION):
            val = get_ldc_rpdata()
            if val is not None:
                rp_readings.append(val)
            # else: optionally log that a None value was received
            time.sleep(0.05) # Delay regardless of read success
        # End Rewritten Loop
        if rp_readings:
            IDLE_RP_VALUE = int(sum(rp_readings) / len(rp_readings))
            feedback_text += f"LDC RP Idle: {IDLE_RP_VALUE}"
        else:
            feedback_text += "LDC Cal Error: No valid readings\n"
            feedback_color = "#E53935" if feedback_color != "#FFA726" else "#FFA726" # Red or Orange
    else:
        feedback_text += "LDC Sensor N/A"
        feedback_color = "#E53935" if feedback_color != "#FFA726" else "#FFA726" # Red or Orange

    # Update feedback label - Text might wrap if long
    feedback_label.config(text=feedback_text.strip(), foreground=feedback_color)
    # Schedule feedback clear
    window.after(4000, lambda: feedback_label.config(text=""))
# --- End CORRECTED calibrate_sensors Function ---

def update_camera_feed():
    """Updates the camera feed label in the GUI."""
    global camera_label, window
    if camera:
        ret, frame = camera.read()
        if ret:
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            frame_enhanced_rgb = enhance_image_contrast(frame_rgb)
            img = Image.fromarray(frame_enhanced_rgb).resize((DISPLAY_IMG_WIDTH, DISPLAY_IMG_HEIGHT))
            img_tk = ImageTk.PhotoImage(img)
            camera_label.img_tk = img_tk
            camera_label.configure(image=img_tk)
    else:
         if not hasattr(camera_label, 'no_cam_img'):
             placeholder = Image.new('RGB', (DISPLAY_IMG_WIDTH, DISPLAY_IMG_HEIGHT), color = '#BDBDBD')
             camera_label.no_cam_img = ImageTk.PhotoImage(placeholder)
         camera_label.configure(image=camera_label.no_cam_img)
    window.after(50, update_camera_feed)

def update_magnetism():
    """Updates the magnetism reading label in the GUI."""
    global magnetism_label, window
    avg_voltage = get_averaged_hall_voltage()
    if avg_voltage is not None:
        try:
            adj_v = avg_voltage - IDLE_VOLTAGE
            mag_mT = adj_v / SENSITIVITY_V_PER_MILLITESLA
            if abs(mag_mT) < 1: magnetism_label.config(text=f"{mag_mT * 1000:.2f} µT")
            else: magnetism_label.config(text=f"{mag_mT:.2f} mT")
        except Exception: magnetism_label.config(text="Error")
    else: magnetism_label.config(text="N/A" if not hall_sensor else "Error")
    window.after(GUI_UPDATE_INTERVAL_MS, update_magnetism)

def update_ldc_reading():
    """Updates the LDC1101 RP value label using a moving average buffer."""
    global ldc_label, window, RP_DISPLAY_BUFFER
    avg_rp_val = get_averaged_rp_data()
    display_rp_text = "N/A"
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
    """Sets up the Tkinter GUI elements."""
    global window, camera_label, controls_frame, feedback_label, magnetism_label, ldc_label
    global target_var, capture_button, calibrate_button, coated_var, dilapidated_var, degraded_var
    global label_font, readout_font, button_font, feedback_font, title_font, check_font

    window = tk.Tk()
    window.title("Sensor Data Acquisition Tool v1.3.4") # Version update
    window.geometry("1100x700")

    # --- Style & Fonts ---
    style = ttk.Style()
    if 'clam' in style.theme_names(): style.theme_use('clam')
    else: style.theme_use('default')
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

    # --- Main Frame ---
    main_frame = ttk.Frame(window, padding="15 15 15 15"); main_frame.pack(fill=tk.BOTH, expand=True)
    main_frame.columnconfigure(0, weight=3); main_frame.columnconfigure(1, weight=1); main_frame.rowconfigure(0, weight=1)

    # --- Camera Feed Label (Left) ---
    camera_label = ttk.Label(main_frame, text="Initializing Camera...", anchor="center", borderwidth=1, relief="sunken")
    camera_label.grid(row=0, column=0, padx=(0, 15), pady=0, sticky="nsew")

    # --- Controls Frame (Right) ---
    controls_frame = ttk.Frame(main_frame); controls_frame.grid(row=0, column=1, padx=(0, 0), pady=0, sticky="nsew")
    controls_frame.columnconfigure(0, weight=1)
    controls_row_idx = 0

    # --- Feedback Area ---
    # Use fixed height for the label
    feedback_label = ttk.Label(controls_frame, text="", style="Feedback.TLabel",
                               anchor="nw", wraplength=350, height=2) # Fixed height
    feedback_label.grid(row=controls_row_idx, column=0, sticky="ew", pady=(0, 10))
    # No need for rowconfigure minsize as widget height is fixed
    controls_row_idx += 1

    # --- Sensor Readings Group ---
    readings_frame = ttk.Labelframe(controls_frame, text=" Sensor Readings ", padding="15 10 15 10")
    readings_frame.grid(row=controls_row_idx, column=0, sticky="new", pady=(0, 15)); controls_row_idx += 1
    readings_frame.columnconfigure(0, weight=0); readings_frame.columnconfigure(1, weight=1)
    ttk.Label(readings_frame, text="Magnetism:").grid(row=0, column=0, sticky="w", padx=(0, 10))
    magnetism_label = ttk.Label(readings_frame, text="Init...", style="Readout.TLabel", anchor="e")
    magnetism_label.grid(row=0, column=1, sticky="ew")
    ttk.Label(readings_frame, text="LDC RP:").grid(row=1, column=0, sticky="w", padx=(0, 10), pady=(5,0))
    ldc_label = ttk.Label(readings_frame, text="Init...", style="Readout.TLabel", anchor="e")
    ldc_label.grid(row=1, column=1, sticky="ew", pady=(5,0))

    # --- Data Capture Group ---
    actions_frame = ttk.Labelframe(controls_frame, text=" Data Capture ", padding="15 10 15 15")
    actions_frame.grid(row=controls_row_idx, column=0, sticky="new", pady=(0, 15)); controls_row_idx += 1
    actions_frame.columnconfigure(0, weight=1)
    action_row_idx = 0

    # Target Material Dropdown
    target_title_label = ttk.Label(actions_frame, text="Target Material:")
    target_title_label.grid(row=action_row_idx, column=0, sticky="w", pady=(0, 3))
    action_row_idx += 1
    target_var = tk.StringVar(window)
    target_var.set(TARGET_OPTIONS[0])
    target_dropdown = ttk.OptionMenu(actions_frame, target_var, TARGET_OPTIONS[0], *TARGET_OPTIONS, style="TMenubutton")
    target_dropdown.grid(row=action_row_idx, column=0, sticky="ew", pady=(0, 10))
    action_row_idx += 1

    # Condition Checkboxes
    condition_label = ttk.Label(actions_frame, text="Conditions:")
    condition_label.grid(row=action_row_idx, column=0, sticky="w", pady=(5, 3))
    action_row_idx += 1
    coated_var = tk.IntVar(value=0)
    dilapidated_var = tk.IntVar(value=0)
    degraded_var = tk.IntVar(value=0)
    # Create Checkbuttons on separate lines
    cb_coated = ttk.Checkbutton(actions_frame, text="Coated", variable=coated_var, onvalue=1, offvalue=0)
    cb_coated.grid(row=action_row_idx, column=0, sticky="w")
    action_row_idx += 1
    cb_dilapidated = ttk.Checkbutton(actions_frame, text="Dilapidated", variable=dilapidated_var, onvalue=1, offvalue=0)
    cb_dilapidated.grid(row=action_row_idx, column=0, sticky="w")
    action_row_idx += 1
    cb_degraded = ttk.Checkbutton(actions_frame, text="Degraded", variable=degraded_var, onvalue=1, offvalue=0)
    cb_degraded.grid(row=action_row_idx, column=0, sticky="w", pady=(0, 10))
    action_row_idx += 1

    # Separator and Buttons on separate lines
    separator = ttk.Separator(actions_frame, orient='horizontal')
    separator.grid(row=action_row_idx, column=0, sticky='ew', pady=(0, 15))
    action_row_idx += 1
    capture_button = ttk.Button(actions_frame, text="Capture & Add Data", command=capture_and_save_data)
    capture_button.grid(row=action_row_idx, column=0, sticky="ew", pady=(0, 8))
    action_row_idx += 1
    calibrate_button = ttk.Button(actions_frame, text="Calibrate Sensors", command=calibrate_sensors)
    calibrate_button.grid(row=action_row_idx, column=0, sticky="ew")
    action_row_idx += 1

# ==========================
# === Main Execution =======
# ==========================
def run_application():
    """Sets up state, GUI, starts loops, runs the main loop."""
    global window
    if not initialize_global_state(): return
    setup_gui()
    if not camera: camera_label.configure(text="Camera Failed")
    if not hall_sensor: magnetism_label.config(text="N/A")
    if not ldc_initialized: ldc_label.config(text="N/A")
    update_camera_feed(); update_magnetism(); update_ldc_reading()
    print("Starting Tkinter main loop...")
    window.mainloop()

# --- Cleanup ---
def cleanup_resources():
    """Releases hardware resources."""
    print("Cleaning up resources...")
    if camera and camera.isOpened(): print("Releasing camera..."); camera.release()
    cv2.destroyAllWindows()
    if spi: print("Closing SPI..."); spi.close()
    if SPI_ENABLED:
        try: print("Cleaning up GPIO..."); GPIO.cleanup()
        except Exception as e: print(f"Note: GPIO cleanup error: {e}")
    print("Cleanup complete.")

# --- Run ---
if __name__ == '__main__':
    initialize_hardware()
    try: run_application()
    except Exception as e: print(f"FATAL ERROR: {e}"); import traceback; traceback.print_exc()
    finally: cleanup_resources()
