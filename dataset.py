# CODE 1 Modified - Integrating LDC fixes from CODE 2
# Combined Code: Camera, Magnetism, Inductance Measurement with Dataset Creation
# Version: 1.6.1_mod1 - Integrated LDC fixes (SPI speed, R/W delays)

import tkinter as tk
from tkinter import ttk
from tkinter import font as tkFont
from tkinter import messagebox
import cv2
from PIL import Image, ImageTk
import time
import os
import csv
from datetime import datetime
import statistics
from collections import deque
import re

# --- I2C/ADS1115 Imports ---
# (Imports remain the same)
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
# (Imports remain the same)
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

# Accuracy/Stability/Speed
NUM_SAMPLES_PER_UPDATE = 3
NUM_SAMPLES_CALIBRATION = 5
GUI_UPDATE_INTERVAL_MS = 250
LDC_DISPLAY_BUFFER_SIZE = 5
MAGNETISM_FILTER_ALPHA = 0.2  # Smoothing factor for EMA filter (0 < alpha <= 1). Smaller = more smoothing.

# Image Saving
SAVE_IMG_WIDTH = 224
SAVE_IMG_HEIGHT = 224

# Camera
CAMERA_INDEX = 0
DISPLAY_IMG_WIDTH = 640
DISPLAY_IMG_HEIGHT = 480

# Hall Sensor (ADS1115)
HALL_ADC_CHANNEL = ADS.P0 if I2C_ENABLED else None
SENSITIVITY_V_PER_TESLA = 0.0004
SENSITIVITY_V_PER_MILLITESLA = SENSITIVITY_V_PER_TESLA * 1000
IDLE_VOLTAGE = 1.7348

# Inductive Sensor (LDC1101)
# <<< MODIFIED: Changed SPI Speed to match working CODE 2 >>>
SPI_BUS = 0; SPI_DEVICE = 0; SPI_SPEED = 50000; SPI_MODE = 0b00 # 50kHz
CS_PIN = 8; LDC_CHIP_ID = 0xD4
START_CONFIG_REG, RP_SET_REG, TC1_REG, TC2_REG, DIG_CONFIG_REG = 0x0B, 0x01, 0x02, 0x03, 0x04
ALT_CONFIG_REG, D_CONF_REG, INTB_MODE_REG = 0x05, 0x0C, 0x0A
RP_DATA_MSB_REG, RP_DATA_LSB_REG = 0x22, 0x21; CHIP_ID_REG = 0x3F
ACTIVE_CONVERSION_MODE, SLEEP_MODE = 0x00, 0x01


# Calibration
IDLE_RP_VALUE = 0

# Dataset / File Saving
# (Constants remain the same)
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


# Target Materials
TARGET_OPTIONS = ["Copper", "Steel", "Aluminum", "Others"]

# Global Hardware Objects
camera = None; i2c = None; ads = None; hall_sensor = None
spi = None; ldc_initialized = False

# Global State
RP_DISPLAY_BUFFER = deque(maxlen=LDC_DISPLAY_BUFFER_SIZE)
object_counter = 0
shot_counter = 0
previous_filtered_mag_mT = None # State for the EMA filter

# GUI Globals
# (Globals remain the same)
window = None; camera_label = None; controls_frame = None
magnetism_label = None; ldc_label = None; target_var = None
capture_button = None; calibrate_button = None
coated_var = None; dilapidated_var = None; degraded_var = None
new_object_var = None
next_filename_label = None
label_font = None; readout_font = None; button_font = None
title_font = None; check_font = None
filename_display_font = None

# =========================
# === File/State Utils ====
# =========================
# (Functions remain the same)
def get_last_object_index(img_dir, prefix):
    max_obj_index = 0
    if not os.path.isdir(img_dir): return 0
    try:
        filename_pattern = re.compile(f"^{prefix}(\\d+)_(\\d+)\\.jpg$")
        for filename in os.listdir(img_dir):
            match = filename_pattern.match(filename)
            if match:
                try: max_obj_index = max(max_obj_index, int(match.group(1)))
                except ValueError: continue
    except OSError as e: print(f"Warning: Could not read image directory {img_dir}: {e}")
    print(f"Determined last used object index: {max_obj_index}")
    return max_obj_index

def initialize_global_state():
    global object_counter, shot_counter, previous_filtered_mag_mT # Add filter state reset
    if not setup_project_directory(): return False
    object_counter = get_last_object_index(IMAGES_PATH, FILENAME_PREFIX)
    shot_counter = 0
    previous_filtered_mag_mT = None # Reset filter state on init
    print(f"Starting with Object Counter = {object_counter} (Next new object will be {object_counter + 1})")
    return True

def setup_project_directory():
    try: os.makedirs(PROJECT_PATH, exist_ok=True); os.makedirs(IMAGES_PATH, exist_ok=True); return True
    except OSError as e: print(f"Error creating directories: {e}"); return False

# =========================
# === Hardware Setup ===
# =========================
# (Function remains the same)
def initialize_hardware():
    global camera, i2c, ads, hall_sensor, spi, ldc_initialized
    print("--- Initializing Hardware ---")
    try:
        camera = cv2.VideoCapture(CAMERA_INDEX)
        if camera and not camera.isOpened(): raise ValueError("Could not open camera")
    except Exception as e: print(f"Error opening camera {CAMERA_INDEX}: {e}"); camera = None
    if I2C_ENABLED:
        try:
            i2c = busio.I2C(board.SCL, board.SDA); ads = ADS.ADS1115(i2c)
            hall_sensor = AnalogIn(ads, HALL_ADC_CHANNEL); print("ADS1115 Initialized.")
        except Exception as e: print(f"Error initializing I2C/ADS1115: {e}"); hall_sensor = None
    else: print("Skipping I2C/ADS1115 setup.")
    if SPI_ENABLED:
        try:
            GPIO.setwarnings(False); GPIO.setmode(GPIO.BCM); GPIO.setup(CS_PIN, GPIO.OUT); GPIO.output(CS_PIN, GPIO.HIGH); print("GPIO Initialized.")
            spi = spidev.SpiDev(); spi.open(SPI_BUS, SPI_DEVICE); spi.max_speed_hz = SPI_SPEED; spi.mode = SPI_MODE; print(f"SPI Initialized (Bus={SPI_BUS}, Dev={SPI_DEVICE}, Speed={SPI_SPEED}Hz).") # <<< Added Speed log
            if initialize_ldc1101(): enable_ldc_rpmode()
            else: print("LDC1101 Initialization Failed.")
        except Exception as e: print(f"Error initializing GPIO/SPI: {e}"); spi = None; ldc_initialized = False
    else: print("Skipping SPI/GPIO/LDC1101 setup.")
    print("--- Hardware Initialization Complete ---")

# =========================
# === LDC1101 Functions ===
# =========================

# <<< MODIFIED: Replaced with CODE 2's write function (includes delays) >>>
def ldc_write_register(reg_addr, value):
    if not spi: return
    try:
        GPIO.output(CS_PIN, GPIO.LOW)
        time.sleep(0.01) # Reduced delay slightly from 0.1 to potentially improve speed
        spi.xfer2([reg_addr & 0x7F, value]) # Send write command (MSB = 0)
        time.sleep(0.01) # Reduced delay slightly from 0.1
        GPIO.output(CS_PIN, GPIO.HIGH)
    except Exception as e:
        print(f"Error writing LDC 0x{reg_addr:02X}: {e}")
        # Consider adding GPIO.output(CS_PIN, GPIO.HIGH) in a finally block if errors are frequent

# <<< MODIFIED: Replaced with CODE 2's read function (includes delays) >>>
def ldc_read_register(reg_addr):
    if not spi: return 0
    result = [0, 0] # Default value in case of error
    try:
        GPIO.output(CS_PIN, GPIO.LOW)
        time.sleep(0.01) # Reduced delay slightly from 0.1
        result = spi.xfer2([reg_addr | 0x80, 0x00]) # Send read command (MSB = 1)
        time.sleep(0.01) # Reduced delay slightly from 0.1
        GPIO.output(CS_PIN, GPIO.HIGH)
        return result[1] # Return data from the register
    except Exception as e:
        print(f"Error reading LDC 0x{reg_addr:02X}: {e}")
        # Consider adding GPIO.output(CS_PIN, GPIO.HIGH) in a finally block if errors are frequent
        return 0 # Return 0 on error

# <<< MODIFIED: Added delay at the end of initialization >>>
def initialize_ldc1101():
    global ldc_initialized; ldc_initialized = False;
    if not spi: return False
    try:
        chip_id = ldc_read_register(CHIP_ID_REG)
        if chip_id != LDC_CHIP_ID:
            print(f"LDC Mismatch: Read 0x{chip_id:02X}, Expected 0x{LDC_CHIP_ID:02X}")
            return False
        print("Configuring LDC1101...")
        # Using standard RP mode config from original CODE 1
        ldc_write_register(RP_SET_REG, 0x07)
        ldc_write_register(TC1_REG, 0x90)
        ldc_write_register(TC2_REG, 0xA0)
        ldc_write_register(DIG_CONFIG_REG, 0x03)
        ldc_write_register(ALT_CONFIG_REG, 0x00) # Keep 0x00 for standard RP
        ldc_write_register(D_CONF_REG, 0x00)     # Keep 0x00 for standard RP
        ldc_write_register(INTB_MODE_REG, 0x00)
        ldc_write_register(START_CONFIG_REG, SLEEP_MODE) # Go to sleep after config
        time.sleep(0.1) # <<< Added delay based on CODE 2's init
        print("LDC1101 Configured and in Sleep Mode.")
        ldc_initialized = True
        return True
    except Exception as e:
        print(f"Exception during LDC1101 initialization: {e}")
        ldc_initialized = False
        return False

# <<< MODIFIED: Increased delay in power mode function slightly >>>
def enable_ldc_powermode(mode):
    if not spi or not ldc_initialized: return; # <<< Added check for initialized
    mode_str = "ACTIVE" if mode == ACTIVE_CONVERSION_MODE else "SLEEP" if mode == SLEEP_MODE else "SHUTDOWN"
    print(f"Setting LDC Power Mode to: {mode_str} (0x{mode:02X})")
    ldc_write_register(START_CONFIG_REG, mode)
    time.sleep(0.05) # <<< Increased delay slightly

# <<< MODIFIED: Added more debug prints and ensured checks >>>
def enable_ldc_rpmode():
    if not spi or not ldc_initialized:
        print("Cannot enable RP mode: SPI or LDC not initialized.")
        return;
    print("Enabling LDC RP Mode (ALT=0x00, D_CONF=0x00)...")
    # Ensure correct registers for standard RP mode are set before activating
    ldc_write_register(ALT_CONFIG_REG, 0x00)
    ldc_write_register(D_CONF_REG, 0x00)
    enable_ldc_powermode(ACTIVE_CONVERSION_MODE)
    print("LDC RP Mode potentially Active (verify readings).")

def get_ldc_rpdata():
    if not spi or not ldc_initialized: return None
    try:
        # Reading MSB then LSB is correct
        msb = ldc_read_register(RP_DATA_MSB_REG)
        lsb = ldc_read_register(RP_DATA_LSB_REG)
        rp_value = (msb << 8) | lsb
        # print(f"Read RP: MSB=0x{msb:02X}, LSB=0x{lsb:02X}, Value={rp_value}") # Optional debug
        return rp_value
    except Exception as e:
        print(f"Error in get_ldc_rpdata: {e}")
        return None

# =========================
# === CSV Handling =========
# =========================
# (Function remains the same)
def append_metadata(image_filename, mag_mT, rp_value, delta_rp, target, is_coated, is_dilapidated, is_degraded):
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
# (Functions remain the same)
def get_averaged_hall_voltage(num_samples=NUM_SAMPLES_PER_UPDATE):
    if not hall_sensor: return None
    readings = []
    for _ in range(num_samples):
        try:
            if hall_sensor: readings.append(hall_sensor.voltage)
            time.sleep(0.01) # Small delay between samples
        except Exception as e:
            print(f"Warning: Error reading Hall sensor voltage: {e}")
            return None # Abort averaging if one read fails
    return sum(readings) / len(readings) if readings else None

def get_averaged_rp_data(num_samples=NUM_SAMPLES_PER_UPDATE):
    if not ldc_initialized: return None
    readings = []
    for _ in range(num_samples):
        rp_val = get_ldc_rpdata()
        if rp_val is not None:
             readings.append(rp_val)
        # No sleep here, get_ldc_rpdata now has internal delays
    valid_readings = [r for r in readings if r is not None] # Ensure None isn't averaged
    return sum(valid_readings) / len(valid_readings) if valid_readings else None


# ======================
# === GUI Functions ===
# ======================

# (update_next_filename_display remains the same)
def update_next_filename_display():
    global next_filename_label, new_object_var, object_counter, shot_counter
    if not next_filename_label or not new_object_var: return
    try:
        is_new = new_object_var.get()
        display_obj = object_counter + 1 if is_new == 1 else (object_counter if object_counter > 0 else 1)
        display_shot = 1 if is_new == 1 else shot_counter + 1
        next_fn = f"{FILENAME_PREFIX}{display_obj}_{display_shot}.jpg"
        next_filename_label.config(text=f"Next: {next_fn}")
    except tk.TclError: next_filename_label.config(text="Next: ...")
    except Exception as e: print(f"Error updating filename display: {e}"); next_filename_label.config(text="Next: Error")

# (capture_and_save_data remains the same - uses unfiltered data for saving)
def capture_and_save_data():
    global capture_button, window, target_var, IDLE_RP_VALUE
    global coated_var, dilapidated_var, degraded_var
    global object_counter, shot_counter, new_object_var
    if not camera: messagebox.showerror("Camera Error", "Camera not available."); return
    capture_button.config(state=tk.DISABLED)
    ret, frame = camera.read()
    if not ret: messagebox.showerror("Capture Error", "Failed to capture photo."); capture_button.config(state=tk.NORMAL); return
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB); img_raw = Image.fromarray(frame_rgb)
    try:
        img_resized_for_save = img_raw.resize((SAVE_IMG_WIDTH, SAVE_IMG_HEIGHT), Image.Resampling.LANCZOS)
    except AttributeError: img_resized_for_save = img_raw.resize((SAVE_IMG_WIDTH, SAVE_IMG_HEIGHT), Image.LANCZOS)
    except Exception as e: messagebox.showerror("Resize Error", f"Error resizing image: {e}"); capture_button.config(state=tk.NORMAL); return

    is_new_object = new_object_var.get()
    if is_new_object == 1: object_counter += 1; shot_counter = 1
    else:
        if object_counter == 0: object_counter = 1 # Ensure first object is 1
        shot_counter += 1
    image_filename = f"{FILENAME_PREFIX}{object_counter}_{shot_counter}.jpg"
    image_save_path = os.path.join(IMAGES_PATH, image_filename)

    # --- Get UNFILTERED sensor data for saving (using more samples like calibration) ---
    avg_voltage = get_averaged_hall_voltage(num_samples=NUM_SAMPLES_CALIBRATION)
    current_mag_mT = None
    if avg_voltage is not None:
        try: current_mag_mT = (avg_voltage - IDLE_VOLTAGE) / SENSITIVITY_V_PER_MILLITESLA
        except Exception: current_mag_mT = None
    current_rp_val = get_averaged_rp_data(num_samples=NUM_SAMPLES_CALIBRATION)
    delta_rp = None
    if current_rp_val is not None:
        current_rp_val = int(current_rp_val) # Store integer average
        if IDLE_RP_VALUE != 0: delta_rp = current_rp_val - IDLE_RP_VALUE
    # --- End sensor data for saving ---

    selected_target = target_var.get(); is_coated = coated_var.get(); is_dilapidated = dilapidated_var.get(); is_degraded = degraded_var.get()
    try:
        img_resized_for_save.save(image_save_path); print(f"Image saved: {image_save_path}")
    except Exception as e: messagebox.showerror("Save Error", f"Error saving photo:\n{e}"); capture_button.config(state=tk.NORMAL); update_next_filename_display(); return

    meta_success = append_metadata(image_filename, current_mag_mT, current_rp_val, delta_rp, selected_target, is_coated, is_dilapidated, is_degraded)
    if meta_success:
        messagebox.showinfo("Success", f"Data Added: {image_filename}"); new_object_var.set(0)
    else: messagebox.showwarning("Metadata Error", f"Image '{image_filename}' saved,\nbut failed to write metadata.")
    update_next_filename_display()
    window.after(500, lambda: capture_button.config(state=tk.NORMAL))

# (calibrate_sensors remains the same)
def calibrate_sensors():
    global IDLE_VOLTAGE, IDLE_RP_VALUE, window, previous_filtered_mag_mT # Access filter state
    hall_results = "Hall Sensor N/A"; hall_error = False
    if hall_sensor:
        voltages = [v for v in (hall_sensor.voltage if hall_sensor else None for _ in range(NUM_SAMPLES_CALIBRATION)) if v is not None and time.sleep(0.05) is None] # Compact reading with check
        if voltages: IDLE_VOLTAGE = sum(voltages) / len(voltages); hall_results = f"Hall Idle: {IDLE_VOLTAGE:.4f} V"
        else: hall_results = "Hall Cal Error: No readings"; hall_error = True
    ldc_results = "LDC Sensor N/A"; ldc_error = False
    if ldc_initialized:
        # Use the averaging function directly for calibration
        rp_cal_value = get_averaged_rp_data(num_samples=NUM_SAMPLES_CALIBRATION)
        if rp_cal_value is not None:
            IDLE_RP_VALUE = int(rp_cal_value)
            ldc_results = f"LDC Idle: {IDLE_RP_VALUE}"
        else: ldc_results = "LDC Cal Error: No readings"; ldc_error = True

    # Reset the EMA filter state after calibration
    previous_filtered_mag_mT = None
    print("Magnetism filter state reset due to calibration.")
    print(f"Calibration Results: {hall_results}, {ldc_results}") # Log results

    final_message = f"{hall_results}\n{ldc_results}"
    if hall_error or ldc_error: messagebox.showwarning("Calibration Warning", final_message)
    elif not hall_sensor and not ldc_initialized: messagebox.showerror("Calibration Error", final_message)
    else: messagebox.showinfo("Calibration Complete", final_message)


# (update_camera_feed remains the same)
def update_camera_feed():
    global camera_label, window
    if camera:
        ret, frame = camera.read()
        if ret:
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(frame_rgb).resize((DISPLAY_IMG_WIDTH, DISPLAY_IMG_HEIGHT))
            img_tk = ImageTk.PhotoImage(img)
            camera_label.img_tk = img_tk; camera_label.configure(image=img_tk)
        else: # Handle frame read error
             print("Warning: Failed to read frame from camera.")
             # Optionally display a static error image or clear the label
    else:
        if not hasattr(camera_label, 'no_cam_img'):
            placeholder = Image.new('RGB', (DISPLAY_IMG_WIDTH, DISPLAY_IMG_HEIGHT), color='#BDBDBD')
            camera_label.no_cam_img = ImageTk.PhotoImage(placeholder)
            # Ensure the label is configured only once or if image needs update
            if camera_label.cget("image") != str(camera_label.no_cam_img):
                 camera_label.configure(image=camera_label.no_cam_img, text="") # Clear text if any
    if window: window.after(50, update_camera_feed) # Schedule next update


# (update_magnetism remains the same - uses EMA filter)
def update_magnetism():
    """Updates the magnetism reading label in the GUI using an EMA filter."""
    global magnetism_label, window, previous_filtered_mag_mT # Need access to filter state

    avg_voltage = get_averaged_hall_voltage() # Gets average of NUM_SAMPLES_PER_UPDATE
    display_text = "N/A"
    raw_mag_mT = None # Store the unfiltered value

    if hall_sensor:
        if avg_voltage is not None:
            try:
                # Calculate the raw (averaged) magnetism for this interval
                raw_mag_mT = (avg_voltage - IDLE_VOLTAGE) / SENSITIVITY_V_PER_MILLITESLA

                # --- EMA Filter ---
                if previous_filtered_mag_mT is None:
                    # Initialize filter state with the first valid reading
                    filtered_mag_mT = raw_mag_mT
                else:
                    # Apply EMA formula
                    filtered_mag_mT = (MAGNETISM_FILTER_ALPHA * raw_mag_mT) + \
                                      ((1 - MAGNETISM_FILTER_ALPHA) * previous_filtered_mag_mT)
                # Update the state for the next iteration
                previous_filtered_mag_mT = filtered_mag_mT
                # --- End EMA Filter ---

                # Format the *filtered* value for display
                unit, value = ("mT", filtered_mag_mT) if abs(filtered_mag_mT) >= 1 else ("µT", filtered_mag_mT * 1000)
                display_text = f"{value:.2f} {unit}"

            except Exception as e:
                # Handle calculation errors
                print(f"Error calculating magnetism: {e}") # Log error
                display_text = "Error"
                previous_filtered_mag_mT = None # Reset filter on error
        else:
            # Handle sensor read errors (avg_voltage is None)
            display_text = "Error"
            previous_filtered_mag_mT = None # Reset filter on error

    # Update the GUI label
    if magnetism_label: magnetism_label.config(text=display_text)

    # Schedule next update
    if window:
        window.after(GUI_UPDATE_INTERVAL_MS, update_magnetism)

# (update_ldc_reading remains the same logic, but uses updated get_averaged_rp_data)
def update_ldc_reading():
    global ldc_label, window, RP_DISPLAY_BUFFER
    avg_rp_val = get_averaged_rp_data() # Gets average of NUM_SAMPLES_PER_UPDATE
    display_rp_text = "N/A"

    if ldc_initialized:
        if avg_rp_val is not None:
            RP_DISPLAY_BUFFER.append(avg_rp_val)
            # Calculate average of the display buffer for smoother display
            if RP_DISPLAY_BUFFER:
                display_rp_text = f"{int(sum(RP_DISPLAY_BUFFER) / len(RP_DISPLAY_BUFFER))}"
            else:
                display_rp_text = "..." # Buffer empty
        else:
            # Handle sensor read errors (avg_rp_val is None)
            RP_DISPLAY_BUFFER.clear() # Clear buffer on error
            display_rp_text = "Error"

    # Update the GUI label
    if ldc_label: ldc_label.config(text=display_rp_text)

    # Schedule next update
    if window:
        window.after(GUI_UPDATE_INTERVAL_MS, update_ldc_reading)


# ======================
# === GUI Setup ========
# ======================
# (Function remains the same)
def setup_gui():
    global window, camera_label, controls_frame, magnetism_label, ldc_label
    global target_var, capture_button, calibrate_button, coated_var, dilapidated_var, degraded_var
    global new_object_var, next_filename_label
    global label_font, readout_font, button_font, title_font, check_font, filename_display_font

    window = tk.Tk(); window.title("Sensor Data Acquisition Tool v1.6.1_mod1"); window.geometry("1100x680") # Updated title
    style = ttk.Style(); style.theme_use('clam' if 'clam' in style.theme_names() else 'default')
    title_font=tkFont.Font(family="Helvetica",size=14,weight="bold"); label_font=tkFont.Font(family="Helvetica",size=11); readout_font=tkFont.Font(family="Consolas",size=14,weight="bold"); button_font=tkFont.Font(family="Helvetica",size=10,weight="bold"); check_font=tkFont.Font(family="Helvetica",size=10); filename_display_font=tkFont.Font(family="Consolas",size=9)
    style.configure("TLabel",font=label_font,padding=2); style.configure("TButton",font=button_font,padding=(8,5)); style.configure("TMenubutton",font=label_font,padding=4); style.configure("TLabelframe",padding=8); style.configure("TLabelframe.Label",font=tkFont.Font(family="Helvetica",size=12,weight="bold")); style.configure("Readout.TLabel",font=readout_font,padding=(5,1)); style.configure("TCheckbutton",font=check_font,padding=2); style.configure("Filename.TLabel",font=filename_display_font,padding=(0,2))

    main_frame = ttk.Frame(window, padding="10 10 10 10"); main_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True); main_frame.columnconfigure(0, weight=3); main_frame.columnconfigure(1, weight=1); main_frame.rowconfigure(0, weight=1)
    camera_label = ttk.Label(main_frame, text="Init Cam...", anchor="center", borderwidth=1, relief="sunken"); camera_label.grid(row=0, column=0, padx=(0, 10), pady=0, sticky="nsew")
    controls_frame = ttk.Frame(main_frame); controls_frame.grid(row=0, column=1, padx=(0, 0), pady=0, sticky="nsew"); controls_frame.columnconfigure(0, weight=1); controls_row_idx = 0

    readings_frame = ttk.Labelframe(controls_frame, text=" Readings ", padding="10 5 10 5"); readings_frame.grid(row=controls_row_idx, column=0, sticky="new", pady=(0, 5)); controls_row_idx += 1; readings_frame.columnconfigure(0, weight=0); readings_frame.columnconfigure(1, weight=1)
    ttk.Label(readings_frame, text="Magnetism:").grid(row=0, column=0, sticky="w", padx=(0, 5)); magnetism_label = ttk.Label(readings_frame, text="Init...", style="Readout.TLabel", anchor="e"); magnetism_label.grid(row=0, column=1, sticky="ew")
    ttk.Label(readings_frame, text="LDC RP:").grid(row=1, column=0, sticky="w", padx=(0, 5), pady=(2,0)); ldc_label = ttk.Label(readings_frame, text="Init...", style="Readout.TLabel", anchor="e"); ldc_label.grid(row=1, column=1, sticky="ew", pady=(2,0))

    actions_frame = ttk.Labelframe(controls_frame, text=" Capture ", padding="10 5 10 10"); actions_frame.grid(row=controls_row_idx, column=0, sticky="new", pady=(0, 5)); controls_row_idx += 1; actions_frame.columnconfigure(0, weight=1); actions_frame.columnconfigure(1, weight=1); action_row_idx = 0
    ttk.Label(actions_frame, text="Target Material:").grid(row=action_row_idx, column=0, columnspan=2, sticky="w", pady=(0, 1)); action_row_idx += 1 # Span label
    target_var = tk.StringVar(window); default_target = TARGET_OPTIONS[0] if TARGET_OPTIONS else ""; target_var.set(default_target); target_dropdown = ttk.OptionMenu(actions_frame, target_var, default_target, *TARGET_OPTIONS, style="TMenubutton"); target_dropdown.grid(row=action_row_idx, column=0, columnspan=2, sticky="ew", pady=(0, 5)); action_row_idx += 1
    ttk.Label(actions_frame, text="Conditions:").grid(row=action_row_idx, column=0, columnspan=2, sticky="w", pady=(5, 1)); action_row_idx += 1
    coated_var=tk.IntVar(value=0); dilapidated_var=tk.IntVar(value=0); degraded_var=tk.IntVar(value=0)
    ttk.Checkbutton(actions_frame, text="Coated", variable=coated_var, onvalue=1, offvalue=0).grid(row=action_row_idx, column=0, columnspan=2, sticky="w", pady=0); action_row_idx += 1
    ttk.Checkbutton(actions_frame, text="Dilapidated", variable=dilapidated_var, onvalue=1, offvalue=0).grid(row=action_row_idx, column=0, columnspan=2, sticky="w", pady=0); action_row_idx += 1
    ttk.Checkbutton(actions_frame, text="Degraded", variable=degraded_var, onvalue=1, offvalue=0).grid(row=action_row_idx, column=0, columnspan=2, sticky="w", pady=(0, 5)); action_row_idx += 1
    new_object_var = tk.IntVar(value=1); ttk.Checkbutton(actions_frame, text="New Object?", variable=new_object_var, onvalue=1, offvalue=0, command=update_next_filename_display).grid(row=action_row_idx, column=0, sticky="w", pady=(5, 5))
    next_filename_label = ttk.Label(actions_frame, text="Next: ...", style="Filename.TLabel", anchor='w'); next_filename_label.grid(row=action_row_idx, column=1, sticky="ew", pady=(5, 5), padx=(5, 0)); action_row_idx += 1
    capture_button = ttk.Button(actions_frame, text="Capture & Add", command=capture_and_save_data); capture_button.grid(row=action_row_idx, column=0, sticky="ew", pady=(5, 5), padx=(0, 2))
    calibrate_button = ttk.Button(actions_frame, text="Calibrate", command=calibrate_sensors); calibrate_button.grid(row=action_row_idx, column=1, sticky="ew", pady=(5, 5), padx=(2, 0)); action_row_idx += 1
    update_next_filename_display() # Initialize


# ==========================
# === Main Execution =======
# ==========================
# (Function remains the same)
def run_application():
    global window
    if not initialize_global_state(): return
    setup_gui()
    if not camera: camera_label.configure(text="Camera Failed")
    if not hall_sensor: magnetism_label.config(text="N/A")
    if not ldc_initialized: ldc_label.config(text="N/A")
    # Start the update loops
    update_camera_feed(); update_magnetism(); update_ldc_reading()
    print("Starting Tkinter main loop...")
    window.mainloop()


# --- Cleanup ---
# (Function remains the same)
def cleanup_resources():
    print("Cleaning up resources...")
    if camera and camera.isOpened(): camera.release()
    cv2.destroyAllWindows()
    if spi:
        try:
            # Try putting LDC to sleep before closing SPI
            if ldc_initialized:
                 print("Putting LDC to sleep...")
                 ldc_write_register(START_CONFIG_REG, SLEEP_MODE)
                 time.sleep(0.1)
        except Exception as e: print(f"Note: Error putting LDC to sleep: {e}")
        spi.close()
        print("SPI closed.")
    if SPI_ENABLED:
        try:
            GPIO.cleanup()
            print("GPIO cleaned up.")
        except Exception as e: print(f"Note: GPIO cleanup error: {e}")
    print("Cleanup complete.")

# --- Run ---
# (Remains the same)
if __name__ == '__main__':
    initialize_hardware()
    try:
        run_application()
    except Exception as e:
        print(f"FATAL ERROR in main execution: {e}")
        try: messagebox.showerror("Fatal Error", f"An unrecoverable error occurred:\n{e}")
        except Exception: pass # Avoid error if GUI isn't up
        import traceback; traceback.print_exc()
    finally:
        cleanup_resources()
