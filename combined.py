# Combined Code: Camera, Magnetism, Inductance Measurement with Dataset Creation
# Version: v1.3 - Checkboxes, Image Resize, GUI Anchor Fix

import tkinter as tk
from tkinter import ttk
from tkinter import font as tkFont
import cv2
# << Import Resampling methods from PIL explicitly
from PIL import Image, ImageTk, ImageFilter # Added ImageFilter just in case, explicit Resampling needed
# Check PIL version for Resampling attribute location
try:
    # PIL 9+
    PIL_RESIZE_METHOD = Image.Resampling.LANCZOS
except AttributeError:
    # Older PIL
    PIL_RESIZE_METHOD = Image.LANCZOS

import time
import os
import uuid
import csv
from datetime import datetime
import statistics
from collections import deque

# --- I2C/ADS1115 Imports ---
try:
    import board
    import busio
    import adafruit_ads1x15.ads1115 as ADS
    from adafruit_ads1x15.analog_in import AnalogIn
    I2C_ENABLED = True
except ImportError:
    print("Warning: Could not import I2C/ADS1115 libraries. Magnetism readings disabled.")
    I2C_ENABLED = False

# --- SPI/LDC1101 Imports ---
try:
    import spidev
    import RPi.GPIO as GPIO
    SPI_ENABLED = True
except ImportError:
    print("Warning: Could not import SPI/GPIO libraries. LDC readings disabled.")
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
SAVE_IMG_WIDTH = 224 # << Target width for saved images
SAVE_IMG_HEIGHT = 224 # << Target height for saved images

# --- Camera ---
CAMERA_INDEX = 0
DISPLAY_IMG_WIDTH = 640 # Display size can be different from save size
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
RP_DATA_MSB_REG, RP_DATA_LSB_REG = 0x22, 0x21
CHIP_ID_REG = 0x3F
ACTIVE_CONVERSION_MODE, SLEEP_MODE = 0x00, 0x01

# --- Calibration ---
IDLE_RP_VALUE = 0

# --- Dataset / File Saving ---
PROJECT_FOLDER_NAME = "Project_Dataset"
IMAGES_FOLDER_NAME = "images"
METADATA_FILENAME = "metadata.csv"
# << Added condition columns, moved target to end
METADATA_HEADER = ['image_path', 'magnetism_mT', 'ldc_rp', 'delta_rp',
                   'is_coated', 'is_dilapidated', 'is_degraded', 'target_material']
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
camera = None; i2c = None; ads = None; hall_sensor = None; spi = None
ldc_initialized = False; clahe_processor = None

# --- Global State ---
RP_DISPLAY_BUFFER = deque(maxlen=LDC_DISPLAY_BUFFER_SIZE)

# --- GUI Globals ---
window = None; camera_label = None; controls_frame = None; feedback_label = None
magnetism_label = None; ldc_label = None; target_var = None
capture_button = None; calibrate_button = None
# << IntVar for checkboxes
coated_var = None; dilapidated_var = None; degraded_var = None
# Font Objects
label_font = None; readout_font = None; button_font = None
feedback_font = None; title_font = None

# =========================
# === Directory Setup =====
# =========================
def setup_project_directory():
    # (Function remains the same)
    print(f"Ensuring dataset directory exists at: {PROJECT_PATH}")
    try: os.makedirs(PROJECT_PATH, exist_ok=True); os.makedirs(IMAGES_PATH, exist_ok=True); print("Dataset dirs checked."); return True
    except OSError as e: print(f"Error creating dirs: {e}"); return False

# =========================
# === Hardware Setup ===
# =========================
def initialize_hardware():
    # (Function remains the same)
    global camera, i2c, ads, hall_sensor, spi, ldc_initialized, clahe_processor
    print("--- Initializing Hardware ---")
    print("Init Camera..."); camera = cv2.VideoCapture(CAMERA_INDEX); time.sleep(0.5) # Allow camera init time
    if not camera or not camera.isOpened(): print(f"ERR: Camera {CAMERA_INDEX} failed"); camera = None
    if I2C_ENABLED:
        print("Init I2C/ADS1115..."); try: i2c=busio.I2C(board.SCL, board.SDA); ads=ADS.ADS1115(i2c); hall_sensor=AnalogIn(ads, HALL_ADC_CHANNEL); print("OK.")
        except Exception as e: print(f"ERR: {e}"); hall_sensor = None
    else: print("Skip I2C.")
    if SPI_ENABLED:
        print("Init GPIO/SPI..."); try: GPIO.setwarnings(False); GPIO.setmode(GPIO.BCM); GPIO.setup(CS_PIN, GPIO.OUT); GPIO.output(CS_PIN, GPIO.HIGH); print("GPIO OK."); spi = spidev.SpiDev(); spi.open(SPI_BUS, SPI_DEVICE); spi.max_speed_hz=SPI_SPEED; spi.mode=SPI_MODE; print(f"SPI OK."); ldc_initialized = initialize_ldc1101(); enable_ldc_rpmode() if ldc_initialized else print("LDC Init Failed.")
        except Exception as e: print(f"ERR: {e}"); spi = None; ldc_initialized = False
    else: print("Skip SPI.")
    clahe_processor = cv2.createCLAHE(clipLimit=CLAHE_CLIP_LIMIT, tileGridSize=CLAHE_TILE_GRID_SIZE)
    print("--- Init Complete ---")


# =========================
# === LDC1101 Functions ===
# =========================
# (ldc_write_register, ldc_read_register, initialize_ldc1101,
#  enable_ldc_powermode, enable_ldc_rpmode, get_ldc_rpdata remain the same)
def ldc_write_register(reg_addr, value):
    if not spi: return
    try: GPIO.output(CS_PIN, GPIO.LOW); spi.xfer2([reg_addr & 0x7F, value]); GPIO.output(CS_PIN, GPIO.HIGH)
    except Exception as e: print(f"Err W LDC {reg_addr:02X}: {e}")
def ldc_read_register(reg_addr):
    if not spi: return 0
    try: GPIO.output(CS_PIN, GPIO.LOW); result = spi.xfer2([reg_addr | 0x80, 0x00]); GPIO.output(CS_PIN, GPIO.HIGH); return result[1]
    except Exception as e: print(f"Err R LDC {reg_addr:02X}: {e}"); return 0
def initialize_ldc1101():
    if not spi: return False; print("Check LDC ID...")
    chip_id = ldc_read_register(CHIP_ID_REG); print(f"LDC ID: {chip_id:02X} (Expect: {LDC_CHIP_ID:02X})")
    if chip_id != LDC_CHIP_ID: return False; print("Configure LDC...")
    ldc_write_register(RP_SET_REG, 0x1B); ldc_write_register(TC1_REG, 0x80); ldc_write_register(TC2_REG, 0x88)
    ldc_write_register(DIG_CONFIG_REG, 0x07); ldc_write_register(ALT_CONFIG_REG, 0x02)
    ldc_write_register(D_CONF_REG, 0x00); ldc_write_register(INTB_MODE_REG, 0x00)
    ldc_write_register(START_CONFIG_REG, SLEEP_MODE); time.sleep(0.05); print("LDC OK."); return True
def enable_ldc_powermode(mode):
    if not spi: return; print(f"LDC Mode: {mode}"); ldc_write_register(START_CONFIG_REG, mode); time.sleep(0.02)
def enable_ldc_rpmode():
    if not spi: return; print("LDC RP Mode ON..."); ldc_write_register(ALT_CONFIG_REG, 0x02); ldc_write_register(D_CONF_REG, 0x00); enable_ldc_powermode(ACTIVE_CONVERSION_MODE); print("OK.")
def get_ldc_rpdata():
    if not spi or not ldc_initialized: return None
    try: msb = ldc_read_register(RP_DATA_MSB_REG); lsb = ldc_read_register(RP_DATA_LSB_REG); return (msb << 8) | lsb
    except Exception as e: print(f"Err get LDC RP: {e}"); return None

# =========================
# === CSV Handling =========
# =========================
# << Modified to include checkbox states
def append_metadata(image_rel_path, mag_mT, rp_value, delta_rp,
                    is_coated, is_dilapidated, is_degraded, target):
    file_exists = os.path.isfile(METADATA_PATH)
    try:
        with open(METADATA_PATH, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            if not file_exists or os.path.getsize(METADATA_PATH) == 0:
                writer.writerow(METADATA_HEADER)
            mag_mT_str = f"{mag_mT:.4f}" if mag_mT is not None else "N/A"
            rp_value_str = str(rp_value) if rp_value is not None else "N/A"
            delta_rp_str = str(delta_rp) if delta_rp is not None else "N/A"
            # Checkbox states are already 0 or 1 from IntVar
            writer.writerow([image_rel_path, mag_mT_str, rp_value_str, delta_rp_str,
                             is_coated, is_dilapidated, is_degraded, target])
        return True
    except IOError as e: print(f"Error writing metadata: {e}"); return False
    except Exception as e: print(f"Unexpected CSV error: {e}"); return False

# ============================
# === Sensor Reading (Avg) ===
# ============================
def get_averaged_hall_voltage(num_samples=NUM_SAMPLES_PER_UPDATE):
    # (Function remains the same)
    if not hall_sensor: return None; readings = []
    for _ in range(num_samples):
        try: readings.append(hall_sensor.voltage); time.sleep(0.002)
        except Exception as e: print(f"Hall read error: {e}"); pass
    if not readings: return None; return sum(readings) / len(readings)

def get_averaged_rp_data(num_samples=NUM_SAMPLES_PER_UPDATE):
    # (Function remains the same)
    if not ldc_initialized: return None; readings = []
    for _ in range(num_samples):
        val = get_ldc_rpdata()
        if val is not None: readings.append(val)
        time.sleep(0.002)
    if not readings: return None; return sum(readings) / len(readings)

# ==============================
# === Image Enhancement Func ===
# ==============================
def enhance_image_contrast(frame_rgb):
    # (Function remains the same)
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
    # << Reads checkboxes, resizes saved image
    global feedback_label, capture_button, window, target_var, IDLE_RP_VALUE
    global coated_var, dilapidated_var, degraded_var # Access checkbox vars

    if not camera: feedback_label.config(text="Camera not available", foreground="#E53935"); return

    capture_button.config(state=tk.DISABLED); feedback_label.config(text="Capturing data...", foreground="#FFA726"); window.update()

    ret, frame = camera.read()
    if not ret: feedback_label.config(text="Failed cap photo", foreground="#E53935"); capture_button.config(state=tk.NORMAL); return

    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    frame_enhanced_rgb = enhance_image_contrast(frame_rgb)
    img_to_save = Image.fromarray(frame_enhanced_rgb)

    # << Resize before saving
    img_resized = img_to_save.resize((SAVE_IMG_WIDTH, SAVE_IMG_HEIGHT), PIL_RESIZE_METHOD)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S"); unique_id = uuid.uuid4().hex[:6]
    image_filename = f"{timestamp}_{unique_id}.jpg"
    image_save_path = os.path.join(IMAGES_PATH, image_filename)
    image_relative_path = os.path.join(IMAGES_FOLDER_NAME, image_filename) # Path for CSV

    # Sensor Readings
    avg_voltage = get_averaged_hall_voltage(num_samples=NUM_SAMPLES_CALIBRATION)
    current_mag_mT = None
    if avg_voltage is not None: try: current_mag_mT = (avg_voltage - IDLE_VOLTAGE) / SENSITIVITY_V_PER_MILLITESLA; except Exception: pass
    current_rp_val = get_averaged_rp_data(num_samples=NUM_SAMPLES_CALIBRATION)
    delta_rp = None
    if current_rp_val is not None:
        current_rp_val = int(current_rp_val)
        if IDLE_RP_VALUE is not None: delta_rp = current_rp_val - IDLE_RP_VALUE

    # << Get Checkbox states
    is_coated = coated_var.get()
    is_dilapidated = dilapidated_var.get()
    is_degraded = degraded_var.get()

    selected_target = target_var.get()

    # Save Resized Image
    save_success = False
    try: img_resized.save(image_save_path); print(f"Saved {SAVE_IMG_WIDTH}x{SAVE_IMG_HEIGHT} img: {image_save_path}"); save_success = True
    except Exception as e: feedback_label.config(text=f"ERR save img: {e}", foreground="#E53935"); print(f"ERR save: {e}"); capture_button.config(state=tk.NORMAL); return

    # << Append Metadata including checkbox states
    meta_success = append_metadata(image_relative_path, current_mag_mT, current_rp_val, delta_rp,
                                   is_coated, is_dilapidated, is_degraded, selected_target)

    if meta_success: feedback_label.config(text=f"Data Added: {image_filename}", foreground="#66BB6A") # Green
    else: feedback_label.config(text="Img saved, metadata FAILED", foreground="#E53935")

    window.after(1500, lambda: capture_button.config(state=tk.NORMAL))
    window.after(3000, lambda: feedback_label.config(text=""))


def calibrate_sensors():
    # (Function remains the same)
    global IDLE_VOLTAGE, IDLE_RP_VALUE, feedback_label, window
    feedback_text = ""; feedback_color = "#29B6F6"
    if hall_sensor:
        voltages = []; feedback_label.config(text="Calibrating Hall...", foreground="#FFA726"); window.update()
        for _ in range(NUM_SAMPLES_CALIBRATION): try: voltages.append(hall_sensor.voltage); time.sleep(0.05); except Exception: pass
        if voltages: IDLE_VOLTAGE = sum(voltages) / len(voltages); feedback_text += f"Hall Idle: {IDLE_VOLTAGE:.4f} V\n"
        else: feedback_text += "Hall Cal Err\n"; feedback_color = "#FFA726"
    else: feedback_text += "Hall N/A\n"; feedback_color = "#FFA726"
    if ldc_initialized:
        rp_readings = []; feedback_label.config(text=feedback_text + "Calibrating LDC...", foreground="#FFA726"); window.update()
        for _ in range(NUM_SAMPLES_CALIBRATION): val = get_ldc_rpdata(); time.sleep(0.05); (rp_readings.append(val) if val is not None else None)
        if rp_readings: IDLE_RP_VALUE = int(sum(rp_readings) / len(rp_readings)); feedback_text += f"LDC RP Idle: {IDLE_RP_VALUE}"
        else: feedback_text += "LDC Cal Err\n"; feedback_color = "#E53935"
    else: feedback_text += "LDC N/A"; feedback_color = "#E53935" if feedback_color != "#FFA726" else "#FFA726"
    feedback_label.config(text=feedback_text.strip(), foreground=feedback_color)
    window.after(4000, lambda: feedback_label.config(text=""))

def update_camera_feed():
    # (Displays enhanced image, uses DISPLAY dimensions)
    global camera_label, window
    if camera:
        ret, frame = camera.read()
        if ret:
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            frame_enhanced_rgb = enhance_image_contrast(frame_rgb)
            # Resize for display
            img = Image.fromarray(frame_enhanced_rgb).resize((DISPLAY_IMG_WIDTH, DISPLAY_IMG_HEIGHT), PIL_RESIZE_METHOD)
            img_tk = ImageTk.PhotoImage(img)
            camera_label.img_tk = img_tk; camera_label.configure(image=img_tk)
    else: # Placeholder logic
         if not hasattr(camera_label, 'no_cam_img'):
             placeholder = Image.new('RGB', (DISPLAY_IMG_WIDTH, DISPLAY_IMG_HEIGHT), color = '#BDBDBD')
             camera_label.no_cam_img = ImageTk.PhotoImage(placeholder)
         camera_label.configure(image=camera_label.no_cam_img)
    window.after(50, update_camera_feed) # Keep camera feed responsive


def update_magnetism():
    # (Function remains the same)
    global magnetism_label, window
    avg_voltage = get_averaged_hall_voltage()
    if avg_voltage is not None:
        try:
            adj_v = avg_voltage - IDLE_VOLTAGE; mag_mT = adj_v / SENSITIVITY_V_PER_MILLITESLA
            (magnetism_label.config(text=f"{mag_mT * 1000:.2f} µT") if abs(mag_mT) < 1 else magnetism_label.config(text=f"{mag_mT:.2f} mT"))
        except Exception: magnetism_label.config(text="Error")
    else: magnetism_label.config(text="N/A" if not hall_sensor else "Error")
    window.after(GUI_UPDATE_INTERVAL_MS, update_magnetism)

def update_ldc_reading():
    # (Uses moving average buffer for display)
    global ldc_label, window, RP_DISPLAY_BUFFER
    avg_rp_val = get_averaged_rp_data()
    display_rp_text = "N/A";
    if not ldc_initialized: display_rp_text = "N/A"
    elif avg_rp_val is not None:
        RP_DISPLAY_BUFFER.append(avg_rp_val)
        if len(RP_DISPLAY_BUFFER) > 0: moving_avg_rp = sum(RP_DISPLAY_BUFFER) / len(RP_DISPLAY_BUFFER); display_rp_text = f"{int(moving_avg_rp)}"
        else: display_rp_text = "..."
    else: RP_DISPLAY_BUFFER.clear(); display_rp_text = "Error" # Clear buffer on error
    ldc_label.config(text=display_rp_text)
    window.after(GUI_UPDATE_INTERVAL_MS, update_ldc_reading)


# ======================
# === GUI Setup ========
# ======================
def setup_gui():
    # << Added IntVars, Checkbuttons, fixed feedback row minsize
    global window, camera_label, controls_frame, feedback_label
    global magnetism_label, ldc_label, target_var, capture_button, calibrate_button
    global coated_var, dilapidated_var, degraded_var # Checkbox vars
    global label_font, readout_font, button_font, feedback_font, title_font

    window = tk.Tk(); window.title("Sensor Data Acquisition Tool v1.3"); window.geometry("1100x700")
    style = ttk.Style(); style.theme_use('clam' if 'clam' in style.theme_names() else 'default')
    title_font = tkFont.Font(family="Helvetica", size=14, weight="bold")
    label_font = tkFont.Font(family="Helvetica", size=11)
    readout_font = tkFont.Font(family="Consolas", size=14, weight="bold")
    button_font = tkFont.Font(family="Helvetica", size=11, weight="bold")
    feedback_font = tkFont.Font(family="Helvetica", size=10)
    style.configure("TLabel", font=label_font, padding=3); style.configure("TButton", font=button_font, padding=(10, 8))
    style.configure("TMenubutton", font=label_font, padding=5); style.configure("TLabelframe", padding=10)
    style.configure("TLabelframe.Label", font=tkFont.Font(family="Helvetica", size=12, weight="bold"))
    style.configure("Feedback.TLabel", font=feedback_font, padding=5)
    style.configure("Readout.TLabel", font=readout_font, padding=(5, 2))
    style.configure("Unit.TLabel", font=label_font, padding=(0, 2))
    style.configure("TCheckbutton", font=label_font, padding=(0, 3)) # Style checkboxes

    main_frame = ttk.Frame(window, padding="15 15 15 15"); main_frame.pack(fill=tk.BOTH, expand=True)
    main_frame.columnconfigure(0, weight=3); main_frame.columnconfigure(1, weight=1); main_frame.rowconfigure(0, weight=1)

    camera_label = ttk.Label(main_frame, text="Init Cam...", anchor="center", borderwidth=1, relief="sunken")
    camera_label.grid(row=0, column=0, padx=(0, 15), pady=0, sticky="nsew")

    controls_frame = ttk.Frame(main_frame); controls_frame.grid(row=0, column=1, padx=(0, 0), pady=0, sticky="nsew")
    controls_frame.columnconfigure(0, weight=1); row_idx = 0

    # << Configure feedback row height
    controls_frame.rowconfigure(row_idx, minsize=45) # Min height for feedback (adjust as needed)
    feedback_label = ttk.Label(controls_frame, text="", style="Feedback.TLabel", anchor="nw", justify="left", wraplength=250) # Anchor NW, wrap
    feedback_label.grid(row=row_idx, column=0, sticky="new", pady=(0, 5)); row_idx += 1 # Fill cell

    # Readings Group
    readings_frame = ttk.Labelframe(controls_frame, text=" Sensor Readings ", padding="15 10 15 10")
    readings_frame.grid(row=row_idx, column=0, sticky="new", pady=(0, 15)); row_idx += 1
    readings_frame.columnconfigure(0, weight=0); readings_frame.columnconfigure(1, weight=1)
    ttk.Label(readings_frame, text="Magnetism:").grid(row=0, column=0, sticky="w", padx=(0, 10))
    magnetism_label = ttk.Label(readings_frame, text="Init...", style="Readout.TLabel", anchor="e")
    magnetism_label.grid(row=0, column=1, sticky="ew")
    ttk.Label(readings_frame, text="LDC RP:").grid(row=1, column=0, sticky="w", padx=(0, 10), pady=(5,0))
    ldc_label = ttk.Label(readings_frame, text="Init...", style="Readout.TLabel", anchor="e")
    ldc_label.grid(row=1, column=1, sticky="ew", pady=(5,0))

    # Capture Group
    actions_frame = ttk.Labelframe(controls_frame, text=" Data Capture ", padding="15 10 15 15")
    actions_frame.grid(row=row_idx, column=0, sticky="new", pady=(0, 15)); row_idx += 1
    actions_frame.columnconfigure(0, weight=1)

    # Target Material Dropdown
    action_row = 0
    target_title_label = ttk.Label(actions_frame, text="Target Material:")
    target_title_label.grid(row=action_row, column=0, sticky="w", pady=(0, 3)); action_row += 1
    target_var = tk.StringVar(window); target_var.set(TARGET_OPTIONS[0])
    target_dropdown = ttk.OptionMenu(actions_frame, target_var, TARGET_OPTIONS[0], *TARGET_OPTIONS, style="TMenubutton")
    target_dropdown.grid(row=action_row, column=0, sticky="ew", pady=(0, 10)); action_row += 1

    # << Condition Checkboxes Frame
    condition_frame = ttk.Frame(actions_frame) # No label frame needed inside
    condition_frame.grid(row=action_row, column=0, sticky="ew", pady=(0, 10)); action_row += 1
    condition_frame.columnconfigure(0, weight=1) # Make checkboxes align left

    ttk.Label(condition_frame, text="Conditions:").grid(row=0, column=0, sticky="w", pady=(0, 5)) # Optional title for checkboxes
    coated_var = tk.IntVar()
    dilapidated_var = tk.IntVar()
    degraded_var = tk.IntVar()
    cb_coated = ttk.Checkbutton(condition_frame, text="Coated", variable=coated_var)
    cb_dilapidated = ttk.Checkbutton(condition_frame, text="Dilapidated", variable=dilapidated_var)
    cb_degraded = ttk.Checkbutton(condition_frame, text="Degraded", variable=degraded_var)
    cb_coated.grid(row=1, column=0, sticky="w")
    cb_dilapidated.grid(row=2, column=0, sticky="w")
    cb_degraded.grid(row=3, column=0, sticky="w")

    # Separator & Buttons
    ttk.Separator(actions_frame, orient='horizontal').grid(row=action_row, column=0, sticky='ew', pady=(5, 15)); action_row += 1
    capture_button = ttk.Button(actions_frame, text="Capture & Add Data", command=capture_and_save_data)
    capture_button.grid(row=action_row, column=0, sticky="ew", pady=(0, 8)); action_row += 1
    calibrate_button = ttk.Button(actions_frame, text="Calibrate Sensors", command=calibrate_sensors)
    calibrate_button.grid(row=action_row, column=0, sticky="ew"); action_row += 1


# ==========================
# === Main Execution =======
# ==========================
def run_application():
    # (Remains the same)
    if not setup_project_directory(): print("Fatal Error: Could not create dirs. Exiting."); return
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
    if SPI_ENABLED: try: print("Cleaning up GPIO..."); GPIO.cleanup(); except Exception as e: print(f"Note: GPIO cleanup error: {e}")
    print("Cleanup complete.")

# --- Run ---
if __name__ == '__main__':
    initialize_hardware()
    try: run_application()
    except Exception as e: print(f"Unexpected error: {e}"); import traceback; traceback.print_exc()
    finally: cleanup_resources()
