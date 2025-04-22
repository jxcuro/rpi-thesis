# Combined Code: Camera, Magnetism, Inductance Measurement with Dataset Creation
# Version: v1.3 - Checkboxes, GUI Anchor Fix, Stability Enhancements

import tkinter as tk
from tkinter import ttk
from tkinter import font as tkFont
import cv2
from PIL import Image, ImageTk
import time
import os
import uuid
import csv
from datetime import datetime
import statistics
from collections import deque # For moving average buffer

# --- I2C/ADS1115 Imports ---
try:
    import board; import busio; import adafruit_ads1x15.ads1115 as ADS
    from adafruit_ads1x15.analog_in import AnalogIn; I2C_ENABLED = True
except ImportError: print("Warning: I2C/ADS1115 libraries not found. Magnetism disabled."); I2C_ENABLED = False

# --- SPI/LDC1101 Imports ---
try:
    import spidev; import RPi.GPIO as GPIO; SPI_ENABLED = True
except ImportError: print("Warning: SPI/GPIO libraries not found. LDC disabled."); SPI_ENABLED = False

# ==================================
# === Constants and Configuration ===
# ==================================

# --- Accuracy / Stability ---
NUM_SAMPLES_PER_UPDATE = 7
NUM_SAMPLES_CALIBRATION = 10
GUI_UPDATE_INTERVAL_MS = 250
# << Increased buffer sizes for more display smoothing
HALL_DISPLAY_BUFFER_SIZE = 7
LDC_DISPLAY_BUFFER_SIZE = 7

# --- Image Enhancement ---
CLAHE_CLIP_LIMIT = 2.0; CLAHE_TILE_GRID_SIZE = (8, 8)

# --- Camera ---
CAMERA_INDEX = 0; IMG_WIDTH = 640; IMG_HEIGHT = 480

# --- Hall Sensor (ADS1115) ---
HALL_ADC_CHANNEL = ADS.P0 if I2C_ENABLED else None
SENSITIVITY_V_PER_TESLA = 0.0004; SENSITIVITY_V_PER_MILLITESLA = SENSITIVITY_V_PER_TESLA * 1000
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
PROJECT_FOLDER_NAME = "Project_Dataset"; IMAGES_FOLDER_NAME = "images"; METADATA_FILENAME = "metadata.csv"
# << Added condition columns
METADATA_HEADER = [
    'image_path', 'magnetism_mT', 'ldc_rp', 'delta_rp',
    'is_coated', 'is_dilapidated', 'is_degraded', 'target_material'
]
try: BASE_PATH = os.path.dirname(os.path.abspath(__file__))
except NameError: BASE_PATH = os.getcwd()
PROJECT_PATH = os.path.join(BASE_PATH, PROJECT_FOLDER_NAME)
IMAGES_PATH = os.path.join(PROJECT_PATH, IMAGES_FOLDER_NAME)
METADATA_PATH = os.path.join(PROJECT_PATH, METADATA_FILENAME)

# --- Target Materials ---
TARGET_OPTIONS = [
    "Copper", "Steel", "Iron", "Aluminum", "Zinc",
    "Brass", "Gold", "Silver", "Bronze", "Others"
]

# --- Global Hardware Objects ---
camera = None; i2c = None; ads = None; hall_sensor = None
spi = None; ldc_initialized = False; clahe_processor = None

# --- Global State ---
# << Buffers for smoothing displayed values
HALL_DISPLAY_BUFFER = deque(maxlen=HALL_DISPLAY_BUFFER_SIZE)
RP_DISPLAY_BUFFER = deque(maxlen=LDC_DISPLAY_BUFFER_SIZE)

# --- GUI Globals ---
window = None; camera_label = None; controls_frame = None; feedback_label = None
magnetism_label = None; ldc_label = None; target_var = None
capture_button = None; calibrate_button = None
# << Checkbox variables
is_coated_var = None; is_dilapidated_var = None; is_degraded_var = None
# Font Objects
label_font = None; readout_font = None; button_font = None
feedback_font = None; title_font = None; check_font = None

# =========================
# === Directory Setup =====
# =========================
def setup_project_directory():
    # (Remains the same)
    print(f"Ensuring dataset directory exists at: {PROJECT_PATH}")
    try:
        os.makedirs(PROJECT_PATH, exist_ok=True); os.makedirs(IMAGES_PATH, exist_ok=True)
        print("Dataset directories checked/created.")
        return True
    except OSError as e: print(f"Error creating directories: {e}"); return False

# =========================
# === Hardware Setup ===
# =========================
def initialize_hardware():
    # (Remains the same, initializes CLAHE)
    global camera, i2c, ads, hall_sensor, spi, ldc_initialized, clahe_processor
    print("--- Initializing Hardware ---")
    # Camera
    print("Initializing Camera...")
    try: camera = cv2.VideoCapture(CAMERA_INDEX)
    except Exception as e: print(f"Error opening camera: {e}"); camera = None
    if camera and not camera.isOpened(): print(f"Error: Could not open camera {CAMERA_INDEX}"); camera = None

    # I2C/ADS1115
    if I2C_ENABLED:
        print("Initializing I2C and ADS1115...")
        try:
            i2c = busio.I2C(board.SCL, board.SDA); ads = ADS.ADS1115(i2c)
            hall_sensor = AnalogIn(ads, HALL_ADC_CHANNEL); print("ADS1115 Initialized.")
        except Exception as e: print(f"Error initializing I2C/ADS1115: {e}"); hall_sensor = None
    else: print("Skipping I2C/ADS1115 setup.")

    # SPI/GPIO/LDC1101
    if SPI_ENABLED:
        print("Initializing GPIO, SPI...")
        try:
            GPIO.setwarnings(False); GPIO.setmode(GPIO.BCM)
            GPIO.setup(CS_PIN, GPIO.OUT); GPIO.output(CS_PIN, GPIO.HIGH); print("GPIO Initialized.")
            spi = spidev.SpiDev(); spi.open(SPI_BUS, SPI_DEVICE)
            spi.max_speed_hz = SPI_SPEED; spi.mode = SPI_MODE; print(f"SPI Initialized (Bus={SPI_BUS}, Dev={SPI_DEVICE}).")
            if initialize_ldc1101(): enable_ldc_rpmode()
            else: print("LDC1101 Initialization Failed.")
        except Exception as e: print(f"Error initializing GPIO/SPI: {e}"); spi = None; ldc_initialized = False
    else: print("Skipping SPI/GPIO/LDC1101 setup.")

    # Initialize CLAHE processor
    clahe_processor = cv2.createCLAHE(clipLimit=CLAHE_CLIP_LIMIT, tileGridSize=CLAHE_TILE_GRID_SIZE)
    print("--- Hardware Initialization Complete ---")

# =========================
# === LDC1101 Functions ===
# =========================
# (ldc_write_register, ldc_read_register, initialize_ldc1101,
#  enable_ldc_powermode, enable_ldc_rpmode, get_ldc_rpdata remain the same)
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
    print("Checking LDC1101 Chip ID..."); chip_id = ldc_read_register(CHIP_ID_REG)
    print(f"LDC1101 Chip ID Read: 0x{chip_id:02X} (Expected: 0x{LDC_CHIP_ID:02X})")
    if chip_id != LDC_CHIP_ID: return False
    print("Configuring LDC1101 for RP Mode..."); ldc_write_register(RP_SET_REG, 0x1B); ldc_write_register(TC1_REG, 0x80); ldc_write_register(TC2_REG, 0x88)
    ldc_write_register(DIG_CONFIG_REG, 0x07); ldc_write_register(ALT_CONFIG_REG, 0x02); ldc_write_register(D_CONF_REG, 0x00); ldc_write_register(INTB_MODE_REG, 0x00)
    ldc_write_register(START_CONFIG_REG, SLEEP_MODE); time.sleep(0.05); print("LDC1101 Configuration Complete.")
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
# << Modified to include condition flags
def append_metadata(image_rel_path, mag_mT, rp_value, delta_rp,
                    is_coated, is_dilapidated, is_degraded, target):
    file_exists = os.path.isfile(METADATA_PATH)
    try:
        with open(METADATA_PATH, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            if not file_exists or os.path.getsize(METADATA_PATH) == 0:
                writer.writerow(METADATA_HEADER) # Ensure header matches new data

            mag_mT_str = f"{mag_mT:.4f}" if mag_mT is not None else "N/A"
            rp_value_str = str(rp_value) if rp_value is not None else "N/A"
            delta_rp_str = str(delta_rp) if delta_rp is not None else "N/A"
            # Condition flags should be 0 or 1
            coated_str = str(is_coated)
            dilap_str = str(is_dilapidated)
            degrad_str = str(is_degraded)

            writer.writerow([
                image_rel_path, mag_mT_str, rp_value_str, delta_rp_str,
                coated_str, dilap_str, degrad_str, target
            ])
        return True
    except IOError as e: print(f"Error writing metadata: {e}"); return False
    except Exception as e: print(f"Unexpected CSV error: {e}"); return False

# ============================
# === Sensor Reading (Avg) ===
# ============================
def get_averaged_hall_voltage(num_samples=NUM_SAMPLES_PER_UPDATE):
    # << Added simple outlier rejection (remove min/max if enough samples)
    if not hall_sensor: return None
    readings = []
    for _ in range(num_samples):
        try: readings.append(hall_sensor.voltage); time.sleep(0.002)
        except Exception as e: print(f"Hall read error: {e}"); pass
    if not readings: return None

    # Outlier rejection: remove min and max if we have enough samples
    if len(readings) >= 3:
        readings.sort()
        valid_readings = readings[1:-1]
        if not valid_readings: return None # Should not happen if >=3 originally
        return sum(valid_readings) / len(valid_readings)
    elif readings: # Use original readings if less than 3
        return sum(readings) / len(readings)
    else: # No valid readings at all
        return None

def get_averaged_rp_data(num_samples=NUM_SAMPLES_PER_UPDATE):
    # << Added simple outlier rejection
    if not ldc_initialized: return None
    readings = []
    for _ in range(num_samples):
        val = get_ldc_rpdata()
        if val is not None: readings.append(val)
        time.sleep(0.002)
    if not readings: return None

    # Outlier rejection
    if len(readings) >= 3:
        readings.sort()
        valid_readings = readings[1:-1]
        if not valid_readings: return None
        return sum(valid_readings) / len(valid_readings)
    elif readings:
        return sum(readings) / len(readings)
    else:
        return None

# ==============================
# === Image Enhancement Func ===
# ==============================
def enhance_image_contrast(frame_rgb):
    # (Remains the same)
    if clahe_processor is None: return frame_rgb
    try:
        lab = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2LAB); l, a, b = cv2.split(lab)
        l_clahe = clahe_processor.apply(l); lab_clahe = cv2.merge((l_clahe, a, b))
        frame_rgb_enhanced = cv2.cvtColor(lab_clahe, cv2.COLOR_LAB2RGB); return frame_rgb_enhanced
    except Exception as e: print(f"Error applying CLAHE: {e}"); return frame_rgb

# ======================
# === GUI Functions ===
# ======================
def capture_and_save_data():
    # << Reads checkbox states and passes them to append_metadata
    global feedback_label, capture_button, window, target_var, IDLE_RP_VALUE
    # << Need checkbox vars
    global is_coated_var, is_dilapidated_var, is_degraded_var

    if not camera: feedback_label.config(text="Camera not available\n ", foreground="#E53935"); return

    capture_button.config(state=tk.DISABLED)
    feedback_label.config(text="Capturing data...\n ", foreground="#FFA726") # Pad with newline
    window.update()

    ret, frame = camera.read()
    if not ret:
        feedback_label.config(text="Failed to capture photo\n ", foreground="#E53935")
        capture_button.config(state=tk.NORMAL); return

    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    frame_enhanced_rgb = enhance_image_contrast(frame_rgb)
    img_to_save = Image.fromarray(frame_enhanced_rgb)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S"); unique_id = uuid.uuid4().hex[:6]
    image_filename = f"{timestamp}_{unique_id}.jpg"
    image_save_path = os.path.join(IMAGES_PATH, image_filename)
    image_relative_path = os.path.join(IMAGES_FOLDER_NAME, image_filename)

    # Get Averaged Sensor Readings (using more samples)
    avg_voltage = get_averaged_hall_voltage(num_samples=NUM_SAMPLES_CALIBRATION)
    current_mag_mT = None
    if avg_voltage is not None:
        try: adjusted_voltage = avg_voltage - IDLE_VOLTAGE; current_mag_mT = adjusted_voltage / SENSITIVITY_V_PER_MILLITESLA
        except Exception: current_mag_mT = None

    current_rp_val = get_averaged_rp_data(num_samples=NUM_SAMPLES_CALIBRATION)
    delta_rp = None
    if current_rp_val is not None:
        current_rp_val = int(current_rp_val)
        if IDLE_RP_VALUE != 0: # Check if calibrated
             delta_rp = current_rp_val - IDLE_RP_VALUE

    selected_target = target_var.get()
    # << Get checkbox states
    is_coated = is_coated_var.get()
    is_dilapidated = is_dilapidated_var.get()
    is_degraded = is_degraded_var.get()

    # Save Enhanced Image
    save_success = False
    try: img_to_save.save(image_save_path); print(f"Enhanced image saved: {image_save_path}"); save_success = True
    except Exception as e:
        feedback_label.config(text=f"Error saving image\n ", foreground="#E53935")
        print(f"Error saving photo: {e}"); capture_button.config(state=tk.NORMAL); return

    # << Append Metadata including delta_rp and condition flags
    meta_success = append_metadata(
        image_relative_path, current_mag_mT, current_rp_val, delta_rp,
        is_coated, is_dilapidated, is_degraded, selected_target
    )
    if meta_success:
        feedback_label.config(text=f"Data Added: {image_filename}\n ", foreground="#66BB6A") # Pad line
    else:
        feedback_label.config(text="Image saved, metadata FAILED\n ", foreground="#E53935") # Pad line

    window.after(1500, lambda: capture_button.config(state=tk.NORMAL))
    # << Clear feedback, ensure padding
    window.after(3000, lambda: feedback_label.config(text="\n "))

def format_feedback(line1, line2=" "):
    """Ensures feedback text always occupies two lines."""
    return f"{line1}\n{line2}"

def calibrate_sensors():
    # << Uses format_feedback to prevent GUI shift
    global IDLE_VOLTAGE, IDLE_RP_VALUE, feedback_label, window
    line1 = ""; line2 = " "
    feedback_color = "#29B6F6" # Blue

    # Calibrate Hall Sensor
    if hall_sensor:
        voltages = []; feedback_label.config(text=format_feedback("Calibrating Hall...", "(Reading...)"), foreground="#FFA726"); window.update()
        for _ in range(NUM_SAMPLES_CALIBRATION):
            try: voltages.append(hall_sensor.voltage); time.sleep(0.05)
            except Exception: pass
        if voltages: IDLE_VOLTAGE = sum(voltages) / len(voltages); line1 = f"Hall Idle: {IDLE_VOLTAGE:.4f} V"
        else: line1 = "Hall Cal Error: No readings"; feedback_color = "#FFA726"
    else: line1 = "Hall Sensor N/A"; feedback_color = "#FFA726"

    # Calibrate LDC1101
    if ldc_initialized:
        rp_readings = []; feedback_label.config(text=format_feedback(line1, "Calibrating LDC..."), foreground="#FFA726"); window.update()
        for _ in range(NUM_SAMPLES_CALIBRATION):
            val = get_ldc_rpdata()
            if val is not None: rp_readings.append(val)
            time.sleep(0.05)
        if rp_readings: IDLE_RP_VALUE = int(sum(rp_readings) / len(rp_readings)); line2 = f"LDC RP Idle: {IDLE_RP_VALUE}"
        else: line2 = "LDC Cal Error: No readings"; feedback_color = "#E53935"
    else: line2 = "LDC Sensor N/A"; feedback_color = "#E53935" if feedback_color != "#FFA726" else "#FFA726"

    feedback_label.config(text=format_feedback(line1, line2), foreground=feedback_color)
    window.after(4000, lambda: feedback_label.config(text=format_feedback(" ", " "))) # Clear with padding


def update_camera_feed():
    # (Remains the same, uses enhance_image_contrast)
    global camera_label, window
    if camera:
        ret, frame = camera.read()
        if ret:
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB); frame_enhanced_rgb = enhance_image_contrast(frame_rgb)
            img = Image.fromarray(frame_enhanced_rgb).resize((IMG_WIDTH, IMG_HEIGHT)); img_tk = ImageTk.PhotoImage(img)
            camera_label.img_tk = img_tk; camera_label.configure(image=img_tk)
    else:
         if not hasattr(camera_label, 'no_cam_img'):
             placeholder = Image.new('RGB', (IMG_WIDTH, IMG_HEIGHT), color = '#BDBDBD'); camera_label.no_cam_img = ImageTk.PhotoImage(placeholder)
         camera_label.configure(image=camera_label.no_cam_img)
    window.after(50, update_camera_feed)

def update_magnetism():
    # << Uses Hall display buffer
    global magnetism_label, window, HALL_DISPLAY_BUFFER
    avg_voltage = get_averaged_hall_voltage() # Uses NUM_SAMPLES_PER_UPDATE + outlier rejection
    display_mag_text = "N/A"

    if not hall_sensor: display_mag_text = "N/A"
    elif avg_voltage is not None:
        try:
            adjusted_voltage = avg_voltage - IDLE_VOLTAGE; current_mag_mT = adjusted_voltage / SENSITIVITY_V_PER_MILLITESLA
            # Add current calculated value to buffer
            HALL_DISPLAY_BUFFER.append(current_mag_mT)
            # Calculate moving average from buffer
            if len(HALL_DISPLAY_BUFFER) > 0:
                moving_avg_mT = sum(HALL_DISPLAY_BUFFER) / len(HALL_DISPLAY_BUFFER)
                # Format based on magnitude
                if abs(moving_avg_mT) < 1: display_mag_text = f"{moving_avg_mT * 1000:.2f} µT"
                else: display_mag_text = f"{moving_avg_mT:.2f} mT"
            else: display_mag_text = "..." # Buffer filling
        except Exception: display_mag_text = "Error" # Calculation error
    else:
        # Error reading sensor, clear buffer?
        HALL_DISPLAY_BUFFER.clear()
        display_mag_text = "Error"

    magnetism_label.config(text=display_mag_text)
    window.after(GUI_UPDATE_INTERVAL_MS, update_magnetism)

def update_ldc_reading():
    # (Remains same - already uses RP_DISPLAY_BUFFER)
    global ldc_label, window, RP_DISPLAY_BUFFER
    avg_rp_val = get_averaged_rp_data() # Uses NUM_SAMPLES_PER_UPDATE + outlier rejection
    display_rp_text = "N/A"

    if not ldc_initialized: display_rp_text = "N/A"
    elif avg_rp_val is not None:
        RP_DISPLAY_BUFFER.append(avg_rp_val)
        if len(RP_DISPLAY_BUFFER) > 0:
            moving_avg_rp = sum(RP_DISPLAY_BUFFER) / len(RP_DISPLAY_BUFFER)
            display_rp_text = f"{int(moving_avg_rp)}"
        else: display_rp_text = "..."
    else:
        RP_DISPLAY_BUFFER.clear()
        display_rp_text = "Error"

    ldc_label.config(text=display_rp_text)
    window.after(GUI_UPDATE_INTERVAL_MS, update_ldc_reading)


# ======================
# === GUI Setup ========
# ======================
def setup_gui():
    # << Added Checkbuttons and IntVars
    global window, camera_label, controls_frame, feedback_label
    global magnetism_label, ldc_label, target_var, capture_button, calibrate_button
    global is_coated_var, is_dilapidated_var, is_degraded_var # Checkbox vars
    global label_font, readout_font, button_font, feedback_font, title_font, check_font

    window = tk.Tk(); window.title("Sensor Data Acquisition Tool v1.3"); window.geometry("1100x700")
    style = ttk.Style(); style.theme_use('clam' if 'clam' in style.theme_names() else 'default')
    # Fonts
    title_font = tkFont.Font(family="Helvetica", size=14, weight="bold"); label_font = tkFont.Font(family="Helvetica", size=11)
    readout_font = tkFont.Font(family="Consolas", size=14, weight="bold"); button_font = tkFont.Font(family="Helvetica", size=11, weight="bold")
    feedback_font = tkFont.Font(family="Helvetica", size=10); check_font = tkFont.Font(family="Helvetica", size=10) # Font for checkboxes
    # Styles
    style.configure("TLabel", font=label_font, padding=3); style.configure("TButton", font=button_font, padding=(10, 8))
    style.configure("TMenubutton", font=label_font, padding=5); style.configure("TLabelframe", padding=10)
    style.configure("TLabelframe.Label", font=tkFont.Font(family="Helvetica", size=12, weight="bold"))
    # << Ensure feedback label reserves space, e.g., by setting minimum height via row config or fixed height
    style.configure("Feedback.TLabel", font=feedback_font, padding=5) # Anchor W needed?
    style.configure("Readout.TLabel", font=readout_font, padding=(5, 2))
    style.configure("Unit.TLabel", font=label_font, padding=(0, 2))
    style.configure("TCheckbutton", font=check_font) # Style for checkboxes

    # Main Frame
    main_frame = ttk.Frame(window, padding="15 15 15 15"); main_frame.pack(fill=tk.BOTH, expand=True)
    main_frame.columnconfigure(0, weight=3); main_frame.columnconfigure(1, weight=1); main_frame.rowconfigure(0, weight=1)
    # Camera Feed
    camera_label = ttk.Label(main_frame, text="Initializing Camera...", anchor="center", borderwidth=1, relief="sunken")
    camera_label.grid(row=0, column=0, padx=(0, 15), pady=0, sticky="nsew")
    # Controls Frame
    controls_frame = ttk.Frame(main_frame); controls_frame.grid(row=0, column=1, padx=(0, 0), pady=0, sticky="nsew")
    controls_frame.columnconfigure(0, weight=1); row_idx = 0

    # Feedback Label - configure row for min height if text padding isn't enough
    # controls_frame.rowconfigure(row_idx, minsize=35) # Example: minimum height for 2 lines
    feedback_label = ttk.Label(controls_frame, text=format_feedback(" ", " "), style="Feedback.TLabel", anchor="w") # Initial padding
    feedback_label.grid(row=row_idx, column=0, sticky="ew", pady=(0, 10)); row_idx += 1

    # Readings Group
    readings_frame = ttk.Labelframe(controls_frame, text=" Sensor Readings ", padding="15 10 15 10")
    readings_frame.grid(row=row_idx, column=0, sticky="new", pady=(0, 15)); row_idx += 1
    readings_frame.columnconfigure(0, weight=0); readings_frame.columnconfigure(1, weight=1)
    ttk.Label(readings_frame, text="Magnetism:").grid(row=0, column=0, sticky="w", padx=(0, 10))
    magnetism_label = ttk.Label(readings_frame, text="Init...", style="Readout.TLabel", anchor="e") # Anchor East (Right)
    magnetism_label.grid(row=0, column=1, sticky="ew")
    ttk.Label(readings_frame, text="LDC RP:").grid(row=1, column=0, sticky="w", padx=(0, 10), pady=(5,0))
    ldc_label = ttk.Label(readings_frame, text="Init...", style="Readout.TLabel", anchor="e") # Anchor East (Right)
    ldc_label.grid(row=1, column=1, sticky="ew", pady=(5,0))

    # Actions Group
    actions_frame = ttk.Labelframe(controls_frame, text=" Data Capture ", padding="15 10 15 15")
    actions_frame.grid(row=row_idx, column=0, sticky="new", pady=(0, 15)); row_idx += 1
    actions_frame.columnconfigure(0, weight=1)
    action_row = 0 # Internal row counter for actions_frame

    # Target Material
    target_title_label = ttk.Label(actions_frame, text="Target Material:");
    target_title_label.grid(row=action_row, column=0, sticky="w", pady=(0, 3)); action_row += 1
    target_var = tk.StringVar(window); target_var.set(TARGET_OPTIONS[0])
    target_dropdown = ttk.OptionMenu(actions_frame, target_var, TARGET_OPTIONS[0], *TARGET_OPTIONS, style="TMenubutton")
    target_dropdown.grid(row=action_row, column=0, sticky="ew", pady=(0, 10)); action_row += 1

    # Conditions Checkboxes
    conditions_label = ttk.Label(actions_frame, text="Conditions:")
    conditions_label.grid(row=action_row, column=0, sticky="w", pady=(5, 3)); action_row += 1

    # Initialize IntVars for checkboxes
    is_coated_var = tk.IntVar(value=0)
    is_dilapidated_var = tk.IntVar(value=0)
    is_degraded_var = tk.IntVar(value=0)

    # Checkbox widgets - stacked vertically
    coated_check = ttk.Checkbutton(actions_frame, text="Coated", variable=is_coated_var, style="TCheckbutton")
    coated_check.grid(row=action_row, column=0, sticky="w", padx=(10, 0)); action_row += 1 # Indent slightly
    dilap_check = ttk.Checkbutton(actions_frame, text="Dilapidated", variable=is_dilapidated_var, style="TCheckbutton")
    dilap_check.grid(row=action_row, column=0, sticky="w", padx=(10, 0)); action_row += 1
    degrad_check = ttk.Checkbutton(actions_frame, text="Degraded", variable=is_degraded_var, style="TCheckbutton")
    degrad_check.grid(row=action_row, column=0, sticky="w", padx=(10, 0), pady=(0, 10)); action_row += 1 # Add padding below last checkbox

    # Separator
    ttk.Separator(actions_frame, orient='horizontal').grid(row=action_row, column=0, sticky='ew', pady=(5, 15)); action_row += 1

    # Action Buttons
    capture_button = ttk.Button(actions_frame, text="Capture & Add Data", command=capture_and_save_data)
    capture_button.grid(row=action_row, column=0, sticky="ew", pady=(0, 8)); action_row += 1
    calibrate_button = ttk.Button(actions_frame, text="Calibrate Sensors", command=calibrate_sensors)
    calibrate_button.grid(row=action_row, column=0, sticky="ew"); action_row += 1


# ==========================
# === Main Execution =======
# ==========================
def run_application():
    # (Remains the same)
    if not setup_project_directory(): print("Fatal Error: Could not create directories. Exiting."); return
    setup_gui()
    if not camera: camera_label.configure(text="Camera Failed")
    if not hall_sensor: magnetism_label.config(text="N/A")
    if not ldc_initialized: ldc_label.config(text="N/A")
    update_camera_feed(); update_magnetism(); update_ldc_reading()
    print("Starting Tkinter main loop...")
    window.mainloop()

# --- Cleanup ---
def cleanup_resources():
    # (Remains the same)
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
    except Exception as e: print(f"Unexpected error: {e}"); import traceback; traceback.print_exc()
    finally: cleanup_resources()
