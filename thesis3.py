# CODE 3.0.22 - AI Metal Classifier (Gated Automation - Stable Base)
# Description: This version is based on the original v3.0.18 to ensure stability.
#              - Implements the gated automatic classification from v3.0.21.
#              - On startup, waits for a LOW->HIGH signal on GPIO 23 to classify.
#              - While GPIO 23 is LOW, it continuously auto-calibrates.
#              - After classifying, it enters a PAUSED state, ignoring new triggers.
#              - Clicking 'Classify Another' RE-ARMS the system for the next trigger.
# Version: 3.0.22 - Reverted to v3.0.18 as a base to fix camera/GPIO issues.
#                  - Re-applied only the necessary logic for gated automation.
#                  - Fixed a bug in cv2.cvtColor call.
# FIXED:       Camera feed issue by reverting to original stable code.
# FIXED:       Potential instability in GPIO signaling by reverting to original code.

import tkinter as tk
from tkinter import ttk
from tkinter import font as tkFont
from tkinter import messagebox
import cv2 # OpenCV for camera access
from PIL import Image, ImageTk, ImageDraw, ImageFont # Added ImageDraw, ImageFont
import time
import os
import statistics
from collections import deque
import numpy as np
import math
import warnings # To potentially suppress warnings later if needed
import traceback # For more detailed error logging

# --- AI Imports ---
try:
    from tflite_runtime.interpreter import Interpreter
except ImportError:
    try:
        from tensorflow.lite.python.interpreter import Interpreter
    except ImportError:
        print("ERROR: TensorFlow Lite Runtime is not installed.")
        print("Please install it (e.g., 'pip install tflite-runtime' or follow official Pi instructions)")
        exit()

try:
    import joblib
except ImportError:
    print("ERROR: Joblib is not installed.")
    print("Please install it: pip install joblib")
    exit()

# --- I2C/ADS1115 Imports (for Hall Sensor/Magnetism) ---
I2C_ENABLED = False 
try:
    import board      
    import busio      
    import adafruit_ads1x15.ads1115 as ADS 
    from adafruit_ads1x15.analog_in import AnalogIn 
    I2C_ENABLED = True
    print("I2C/ADS1115 libraries imported successfully.")
except ImportError:
    print("Warning: I2C/ADS1115 libraries (board, busio, adafruit-circuitpython-ads1x15) not found.")
    print("Magnetism readings will be disabled.")
except NotImplementedError:
    print("Warning: I2C not supported on this platform according to Blinka. Magnetism readings disabled.")
except Exception as e:
    print(f"Warning: Error importing I2C/ADS1115 libraries: {e}. Magnetism readings disabled.")

# --- SPI/LDC1101 & RPi.GPIO Imports ---
SPI_ENABLED = False 
RPi_GPIO_AVAILABLE = False 
try:
    import spidev   
    SPI_ENABLED = True
    print("SPI library (spidev) imported successfully.")
except ImportError:
    print("Warning: SPI library (spidev) not found. LDC readings will be disabled.")

try:
    import RPi.GPIO as GPIO 
    RPi_GPIO_AVAILABLE = True
    print("RPi.GPIO library imported successfully (needed for LDC CS, Sorting, and Automation Control).")
except ImportError:
    print("Warning: RPi.GPIO library not found. LDC CS control, Sorting, and Automation Control will be disabled.")
except RuntimeError:
    print("Warning: RPi.GPIO library likely requires root privileges (sudo). LDC CS, Sorting, and Automation Control may fail.")
except Exception as e:
    print(f"Warning: Error importing RPi.GPIO library: {e}. LDC CS control, Sorting, and Automation Control disabled.")


# --- Sorting GPIO Configuration ---
SORTING_GPIO_ENABLED = False 
SORTING_DATA_PIN_LSB = 16
SORTING_DATA_PIN_MID = 6
SORTING_DATA_READY_PIN = 26

# --- NEW: Automation GPIO Configuration (Replaces old calibration trigger) ---
CONTROL_PIN = 23 # BCM Pin for automation control
CONTROL_PIN_SETUP_OK = False   # Tracks if the control pin was set up
CONTROL_CHECK_INTERVAL_MS = 50 # How often to check the control pin


# ==================================
# === Constants and Configuration ===
# ==================================
NUM_SAMPLES_PER_UPDATE = 3
NUM_SAMPLES_CALIBRATION = 15
GUI_UPDATE_INTERVAL_MS = 100
CAMERA_UPDATE_INTERVAL_MS = 50
LDC_DISPLAY_BUFFER_SIZE = 5
MAGNETISM_FILTER_ALPHA = 0.08

CAMERA_INDEX = 0
DISPLAY_IMG_WIDTH = 640
DISPLAY_IMG_HEIGHT = 480
RESULT_IMG_DISPLAY_WIDTH = 280

try:
    BASE_PATH = os.path.dirname(os.path.abspath(__file__))
except NameError:
    BASE_PATH = os.getcwd()

MODEL_FILENAME = "material_classifier_model.tflite"
LABELS_FILENAME = "material_labels.txt"
SCALER_FILENAME = "numerical_scaler.joblib"
MODEL_PATH = os.path.join(BASE_PATH, MODEL_FILENAME)
LABELS_PATH = os.path.join(BASE_PATH, LABELS_FILENAME)
SCALER_PATH = os.path.join(BASE_PATH, SCALER_FILENAME)
TESTING_FOLDER_NAME = "testing"

AI_IMG_WIDTH = 224
AI_IMG_HEIGHT = 224

HALL_ADC_CHANNEL = ADS.P0 if I2C_ENABLED else None
SENSITIVITY_V_PER_TESLA = 0.0004
SENSITIVITY_V_PER_MILLITESLA = SENSITIVITY_V_PER_TESLA * 1000
IDLE_VOLTAGE = 0.0

SPI_BUS = 0
SPI_DEVICE = 0
SPI_SPEED = 500000
SPI_MODE = 0b00
CS_PIN = 8

LDC_CHIP_ID = 0xD4
START_CONFIG_REG, RP_SET_REG, TC1_REG, TC2_REG, DIG_CONFIG_REG, ALT_CONFIG_REG, \
D_CONF_REG, INTB_MODE_REG, RP_DATA_MSB_REG, RP_DATA_LSB_REG, CHIP_ID_REG = \
0x0B, 0x01, 0x02, 0x03, 0x04, 0x05, 0x0C, 0x0A, 0x22, 0x21, 0x3F
ACTIVE_CONVERSION_MODE, SLEEP_MODE = 0x00, 0x01
IDLE_RP_VALUE = 0

# ============================
# === Global Objects/State ===
# ============================
camera = None
i2c = None
ads = None
hall_sensor = None
spi = None
ldc_initialized = False

interpreter = None
input_details = None
output_details = None
loaded_labels = []
numerical_scaler = None

RP_DISPLAY_BUFFER = deque(maxlen=LDC_DISPLAY_BUFFER_SIZE)
previous_filtered_mag_mT = None
g_last_live_magnetism_mT = 0.0

# --- GUI Globals ---
window = None
main_frame = None
live_view_frame = None
results_view_frame = None
label_font, readout_font, button_font, title_font, result_title_font, result_value_font, pred_font = (None,) * 7
# MODIFIED: Removed buttons for automation
lv_camera_label, lv_magnetism_label, lv_ldc_label, lv_save_checkbox = (None,) * 4
rv_image_label, rv_prediction_label, rv_confidence_label, rv_magnetism_label, rv_ldc_label, rv_classify_another_button = (None,) * 6
placeholder_img_tk = None
save_output_var = None 

# --- NEW: State for Gated GPIO Automation ---
g_accepting_triggers = True
g_previous_control_state = None
g_last_calibration_time = 0

# =========================
# === Hardware Setup ===
# =========================
def initialize_hardware():
    global camera, i2c, ads, hall_sensor, spi, ldc_initialized, CS_PIN
    global SORTING_GPIO_ENABLED, RPi_GPIO_AVAILABLE
    global CONTROL_PIN_SETUP_OK, CONTROL_PIN

    print("\n--- Initializing Hardware ---")

    # --- Camera Initialization ---
    print(f"Attempting to open camera at index {CAMERA_INDEX}...")
    try:
        camera = cv2.VideoCapture(CAMERA_INDEX)
        time.sleep(0.5)
        if not camera or not camera.isOpened():
            raise ValueError(f"Could not open camera at index {CAMERA_INDEX}.")
        print(f"Camera {CAMERA_INDEX} opened successfully.")
    except Exception as e:
        print(f"ERROR: Failed to open camera {CAMERA_INDEX}: {e}")
        camera = None

    # --- I2C/ADS1115 Initialization ---
    if I2C_ENABLED:
        print("Initializing I2C and ADS1115...")
        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            ads = ADS.ADS1115(i2c)
            if HALL_ADC_CHANNEL is not None:
                hall_sensor = AnalogIn(ads, HALL_ADC_CHANNEL)
            else:
                hall_sensor = None
        except Exception as e:
            print(f"ERROR: Initializing I2C/ADS1115 failed: {e}")
            i2c = ads = hall_sensor = None
    else:
        print("Skipping I2C/ADS1115 setup (libraries not found or disabled).")

    # --- GPIO general setup (BCM mode) ---
    gpio_bcm_mode_set = False
    if RPi_GPIO_AVAILABLE:
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            gpio_bcm_mode_set = True
            print("GPIO BCM mode set successfully.")
        except Exception as e:
            print(f"ERROR: GPIO.setmode(GPIO.BCM) failed: {e}")
    else:
        print("RPi.GPIO library not available. Skipping all GPIO-dependent setups.")

    # --- SPI/LDC1101 Initialization ---
    if SPI_ENABLED and gpio_bcm_mode_set:
        print("Initializing SPI and LDC1101 (CS pin setup)...")
        try:
            GPIO.setup(CS_PIN, GPIO.OUT, initial=GPIO.HIGH)
            spi = spidev.SpiDev()
            spi.open(SPI_BUS, SPI_DEVICE)
            spi.max_speed_hz = SPI_SPEED
            spi.mode = SPI_MODE
            if initialize_ldc1101():
                enable_ldc_rpmode()
            else:
                ldc_initialized = False
        except Exception as e:
            print(f"ERROR: An error occurred during SPI/LDC initialization: {e}")
            if spi: spi.close()
            spi = None
            ldc_initialized = False
    else:
        print("Skipping SPI/LDC1101 setup.")

    # --- Sorting GPIO Pin Initialization ---
    if gpio_bcm_mode_set:
        print("Attempting to initialize GPIO pins for Sorting Mechanism...")
        try:
            GPIO.setup(SORTING_DATA_PIN_LSB, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(SORTING_DATA_PIN_MID, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(SORTING_DATA_READY_PIN, GPIO.OUT, initial=GPIO.LOW)
            SORTING_GPIO_ENABLED = True
        except Exception as e:
            print(f"ERROR: Failed to set up sorting GPIO pins: {e}. Sorting is DISABLED.")
            SORTING_GPIO_ENABLED = False
    else:
        print("Skipping Sorting GPIO setup. Sorting is DISABLED.")
        SORTING_GPIO_ENABLED = False

    # --- NEW: Automation Control Pin Initialization ---
    if gpio_bcm_mode_set:
        print(f"Attempting to initialize GPIO pin {CONTROL_PIN} for Automation Control...")
        try:
            GPIO.setup(CONTROL_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
            CONTROL_PIN_SETUP_OK = True
            print(f"Automation Control Pin {CONTROL_PIN} set as INPUT with PULL-DOWN.")
        except Exception as e:
            print(f"ERROR: Failed to set up Automation Control Pin {CONTROL_PIN}: {e}")
            CONTROL_PIN_SETUP_OK = False
    else:
        print(f"Skipping Automation Control Pin {CONTROL_PIN} setup.")
        CONTROL_PIN_SETUP_OK = False

    # --- Create Testing Folder ---
    try:
        testing_path = os.path.join(BASE_PATH, TESTING_FOLDER_NAME)
        os.makedirs(testing_path, exist_ok=True)
    except Exception as e:
        print(f"ERROR: Could not create testing folder: {e}")

    print("--- Hardware Initialization Complete ---")

# =========================
# === AI Model Setup ======
# =========================
def initialize_ai(): # Original Function
    global interpreter, input_details, output_details, loaded_labels, numerical_scaler
    print("\n--- Initializing AI Components ---")
    ai_ready = True
    try:
        with open(LABELS_PATH, 'r') as f: loaded_labels = [line.strip() for line in f.readlines()]
        if not loaded_labels: raise ValueError("Labels file is empty.")
    except Exception as e: print(f"ERROR: Reading labels '{LABELS_FILENAME}': {e}"); ai_ready = False

    if ai_ready:
        try:
            numerical_scaler = joblib.load(SCALER_PATH)
            if not hasattr(numerical_scaler, 'transform'): raise TypeError("Loaded scaler invalid.")
        except Exception as e: print(f"ERROR: Loading scaler '{SCALER_FILENAME}': {e}"); ai_ready = False

    if ai_ready:
        try:
            interpreter = Interpreter(model_path=MODEL_PATH)
            interpreter.allocate_tensors()
            input_details = interpreter.get_input_details()
            output_details = interpreter.get_output_details()
            if len(input_details) != 2: print(f"ERROR: Model inputs != 2."); ai_ready = False
            if output_details and output_details[0]['shape'][-1] != len(loaded_labels):
                print(f"ERROR: Model output size != labels ({len(loaded_labels)})."); ai_ready = False
        except Exception as e: print(f"ERROR: Loading TFLite model '{MODEL_FILENAME}': {e}"); traceback.print_exc(); ai_ready = False

    if not ai_ready:
        interpreter = input_details = output_details = numerical_scaler = None
    else: print("--- AI Initialization Complete ---")
    return ai_ready

# =========================
# === LDC1101 Functions ===
# =========================
def ldc_write_register(reg_addr, value): # Original Function
    if not spi or not RPi_GPIO_AVAILABLE: return False
    try:
        GPIO.output(CS_PIN, GPIO.LOW)
        spi.xfer2([reg_addr & 0x7F, value])
        GPIO.output(CS_PIN, GPIO.HIGH)
        return True
    except Exception as e:
        return False

def ldc_read_register(reg_addr): # Original Function
    if not spi or not RPi_GPIO_AVAILABLE: return None
    try:
        GPIO.output(CS_PIN, GPIO.LOW)
        result = spi.xfer2([reg_addr | 0x80, 0x00])
        GPIO.output(CS_PIN, GPIO.HIGH)
        return result[1]
    except Exception:
        return None

def initialize_ldc1101(): # Original Function
    global ldc_initialized
    ldc_initialized = False
    if not spi: return False
    try:
        chip_id = ldc_read_register(CHIP_ID_REG)
        if chip_id is None or chip_id != LDC_CHIP_ID: return False
        regs_to_write = { RP_SET_REG: 0x07, TC1_REG: 0x90, TC2_REG: 0xA0, DIG_CONFIG_REG: 0x03,
                          ALT_CONFIG_REG: 0x00, D_CONF_REG: 0x00, INTB_MODE_REG: 0x00 }
        for reg, val in regs_to_write.items():
            if not ldc_write_register(reg, val): return False
        if not ldc_write_register(START_CONFIG_REG, SLEEP_MODE): return False
        time.sleep(0.02)
        ldc_initialized = True
        return True
    except Exception: ldc_initialized = False; return False

def enable_ldc_powermode(mode): # Original Function
    if not spi or not ldc_initialized: return False
    if ldc_write_register(START_CONFIG_REG, mode): time.sleep(0.01); return True
    return False

def enable_ldc_rpmode(): # Original Function
    if not spi or not ldc_initialized: return False
    try:
        if not ldc_write_register(ALT_CONFIG_REG, 0x00): return False
        if not ldc_write_register(D_CONF_REG, 0x00): return False
        return enable_ldc_powermode(ACTIVE_CONVERSION_MODE)
    except Exception: return False

def get_ldc_rpdata(): # Original Function
    if not spi or not ldc_initialized: return None
    try:
        msb = ldc_read_register(RP_DATA_MSB_REG)
        lsb = ldc_read_register(RP_DATA_LSB_REG)
        if msb is None or lsb is None: return None
        return (msb << 8) | lsb
    except Exception: return None

# ============================
# === Sensor Reading (Avg) ===
# ============================
def get_averaged_hall_voltage(num_samples=NUM_SAMPLES_PER_UPDATE): # Original Function
    if not hall_sensor: return None
    readings = []
    for _ in range(num_samples):
        try: readings.append(hall_sensor.voltage)
        except Exception: return None
    return statistics.mean(readings) if readings else None

def get_averaged_rp_data(num_samples=NUM_SAMPLES_PER_UPDATE): # Original Function
    if not ldc_initialized: return None
    readings = []
    for _ in range(num_samples):
        rp_value = get_ldc_rpdata()
        if rp_value is not None: readings.append(rp_value)
    return statistics.mean(readings) if readings else None

# ==========================
# === AI Processing ========
# ==========================
def preprocess_input(image_pil, mag_mT, ldc_rp_raw): # Original Function
    global numerical_scaler, input_details
    try:
        img_resized = image_pil.resize((AI_IMG_WIDTH, AI_IMG_HEIGHT), Image.Resampling.LANCZOS)
        image_np = np.array(img_resized.convert('RGB'), dtype=np.float32) / 255.0
        image_input = np.expand_dims(image_np, axis=0)
        numerical_features = np.array([[float(mag_mT or 0.0), float(ldc_rp_raw or 0.0)]], dtype=np.float32)
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", UserWarning)
            scaled_numerical_features = numerical_scaler.transform(numerical_features)
        
        image_input_index = input_details[0]['index']
        numerical_input_index = input_details[1]['index']
        return { image_input_index: image_input, numerical_input_index: scaled_numerical_features }
    except Exception: return None

def run_inference(model_inputs): # Original Function
    global interpreter, output_details
    try:
        for index, data in model_inputs.items(): interpreter.set_tensor(index, data)
        interpreter.invoke()
        return interpreter.get_tensor(output_details[0]['index'])
    except Exception: return None

def postprocess_output(output_data): # Original Function
    global loaded_labels
    try:
        probabilities = output_data[0]
        predicted_index = np.argmax(probabilities)
        return loaded_labels[predicted_index], float(probabilities[predicted_index])
    except Exception: return "Error", 0.0

# ==================================
# === Sorting Signal Functions ===
# ==================================
def send_sorting_signal(material_label): # Original Function
    if not SORTING_GPIO_ENABLED or not RPi_GPIO_AVAILABLE: return
    mid_val, lsb_val = GPIO.LOW, GPIO.LOW
    if material_label == "Aluminum": lsb_val = GPIO.HIGH
    elif material_label == "Copper": mid_val = GPIO.HIGH
    elif material_label == "Steel": mid_val, lsb_val = GPIO.HIGH, GPIO.HIGH
    try:
        GPIO.output(SORTING_DATA_READY_PIN, GPIO.LOW)
        time.sleep(0.01)
        GPIO.output(SORTING_DATA_PIN_MID, mid_val)
        GPIO.output(SORTING_DATA_PIN_LSB, lsb_val)
        time.sleep(0.01)
        GPIO.output(SORTING_DATA_READY_PIN, GPIO.HIGH)
        time.sleep(0.05)
        GPIO.output(SORTING_DATA_READY_PIN, GPIO.LOW)
        time.sleep(0.01)
        GPIO.output(SORTING_DATA_PIN_MID, GPIO.LOW)
        GPIO.output(SORTING_DATA_PIN_LSB, GPIO.LOW)
    except Exception as e:
        print(f"ERROR: Failed to send sorting signal via GPIO: {e}")

# ==============================
# === View Switching Logic ===
# ==============================
# MODIFIED: Re-arms the trigger system
def calibrate_and_show_live_view():
    """Re-arms the trigger, runs a calibration, and shows the live view."""
    global g_accepting_triggers
    print("\n--- 'Classify Another' clicked: Re-arming GPIO trigger ---")
    g_accepting_triggers = True # Re-arm the system
    calibrate_sensors(is_manual_call=True) 
    show_live_view()

def show_live_view(): # MODIFIED to remove button logic
    global live_view_frame, results_view_frame
    if results_view_frame and results_view_frame.winfo_ismapped():
        results_view_frame.pack_forget()
    if live_view_frame and not live_view_frame.winfo_ismapped():
        live_view_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

def show_results_view(): # Original Function
    global live_view_frame, results_view_frame
    if live_view_frame and live_view_frame.winfo_ismapped():
        live_view_frame.pack_forget()
    if results_view_frame and not results_view_frame.winfo_ismapped():
        results_view_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

# ============================
# === Screenshot Function ===
# ============================
def save_result_screenshot(image_pil, prediction, confidence, mag_text, ldc_text): # Original Function
    testing_folder = os.path.join(BASE_PATH, TESTING_FOLDER_NAME)
    try: os.makedirs(testing_folder, exist_ok=True)
    except Exception: return
    i = 1
    while True:
        filename = os.path.join(testing_folder, f"data_{i}.png")
        if not os.path.exists(filename): break
        i += 1
        if i > 9999: return

    try:
        # Simplified composite image creation from original for brevity
        IMG_WIDTH, IMG_HEIGHT, BG_COLOR, TEXT_COLOR, MARGIN = 400, 600, "white", "black", 20
        ss_img = Image.new('RGB', (IMG_WIDTH, IMG_HEIGHT), BG_COLOR)
        draw = ImageDraw.Draw(ss_img)
        try:
            font_title = ImageFont.truetype("DejaVuSans-Bold.ttf", 20)
            font_text = ImageFont.truetype("DejaVuSans.ttf", 16)
        except IOError:
            font_title = ImageFont.load_default()
            font_text = ImageFont.load_default()
        
        y_pos = MARGIN
        draw.text((MARGIN, y_pos), "Classification Result", fill=TEXT_COLOR, font=font_title); y_pos += 40
        
        # Simplified paste operation
        img_disp = image_pil.resize((280, 210), Image.Resampling.LANCZOS)
        ss_img.paste(img_disp, (60, y_pos)); y_pos += 230

        draw.text((MARGIN, y_pos), f"Material: {prediction}", fill=TEXT_COLOR, font=font_title); y_pos += 30
        draw.text((MARGIN, y_pos), f"Confidence: {confidence:.1%}", fill=TEXT_COLOR, font=font_text); y_pos += 30
        draw.text((MARGIN, y_pos), f"Magnetism: {mag_text}", fill=TEXT_COLOR, font=font_text); y_pos += 30
        draw.text((MARGIN, y_pos), f"LDC Reading: {ldc_text}", fill=TEXT_COLOR, font=font_text)
        
        ss_img.save(filename)
    except Exception as e:
        print(f"ERROR: Failed to save screenshot: {e}")

# ======================
# === GUI Functions ===
# ======================
def create_placeholder_image(width, height, color='#E0E0E0', text="No Image"): # Original Function
    try:
        pil_img = Image.new('RGB', (width, height), color)
        return ImageTk.PhotoImage(pil_img)
    except Exception: return None

def clear_results_display(): # Original Function
    global rv_image_label, rv_prediction_label, rv_confidence_label, rv_magnetism_label, rv_ldc_label, placeholder_img_tk
    if rv_image_label and placeholder_img_tk: rv_image_label.config(image=placeholder_img_tk)
    default_text = "---"
    if rv_prediction_label: rv_prediction_label.config(text=default_text)
    if rv_confidence_label: rv_confidence_label.config(text=default_text)
    if rv_magnetism_label: rv_magnetism_label.config(text=default_text)
    if rv_ldc_label: rv_ldc_label.config(text=default_text)

# MODIFIED: Disarms the trigger system
def capture_and_classify():
    global window, camera, IDLE_VOLTAGE, IDLE_RP_VALUE, interpreter, save_output_var
    global rv_image_label, rv_prediction_label, rv_confidence_label, rv_magnetism_label, rv_ldc_label
    global g_last_live_magnetism_mT, g_accepting_triggers

    g_accepting_triggers = False # Disarm the system
    print("\n" + "="*10 + " Automatic Classification Triggered (System Paused) " + "="*10)

    if not interpreter or not camera or not camera.isOpened():
        messagebox.showerror("Error", "AI/Camera not ready.")
        show_live_view()
        return

    ret, frame = camera.read()
    if not ret or frame is None:
        messagebox.showerror("Capture Error", "Failed to capture image.")
        show_live_view()
        return
    img_captured_pil = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))

    current_mag_mT = g_last_live_magnetism_mT
    mag_display_text = "N/A"
    if current_mag_mT is not None:
        mag_display_text = f"{current_mag_mT:+.2f}mT" if abs(current_mag_mT) >= 0.1 else f"{current_mag_mT*1000:+.1f}µT"

    avg_rp_val = get_averaged_rp_data(num_samples=NUM_SAMPLES_CALIBRATION)
    current_rp_raw, ldc_display_text = None, "N/A"
    if avg_rp_val is not None:
        current_rp_raw = avg_rp_val
        delta_rp = int(round(avg_rp_val)) - IDLE_RP_VALUE
        ldc_display_text = f"{int(round(avg_rp_val))} (Δ{delta_rp:+,})"

    model_inputs = preprocess_input(img_captured_pil, current_mag_mT, current_rp_raw)
    output_data = run_inference(model_inputs)
    predicted_label, confidence = postprocess_output(output_data)

    if save_output_var and save_output_var.get() == 1:
        save_result_screenshot(img_captured_pil, predicted_label, confidence, mag_display_text, ldc_display_text)
    
    send_sorting_signal(predicted_label)

    # Update Results Display
    img_disp = img_captured_pil.resize((RESULT_IMG_DISPLAY_WIDTH, int(RESULT_IMG_DISPLAY_WIDTH*0.75)), Image.Resampling.LANCZOS)
    img_tk = ImageTk.PhotoImage(img_disp)
    rv_image_label.img_tk = img_tk
    rv_image_label.config(image=img_tk)
    rv_prediction_label.config(text=f"{predicted_label}")
    rv_confidence_label.config(text=f"{confidence:.1%}")
    rv_magnetism_label.config(text=mag_display_text)
    rv_ldc_label.config(text=ldc_display_text)
    
    show_results_view()

# MODIFIED: Made silent unless called manually
def calibrate_sensors(is_manual_call=False):
    global IDLE_VOLTAGE, IDLE_RP_VALUE, window, previous_filtered_mag_mT
    
    if is_manual_call:
        print("\n" + "="*10 + " Manual Sensor Calibration " + "="*10)
        messagebox.showinfo("Calibration", "Starting sensor calibration.\nPlease ensure NO metal object is near the sensors.")
    
    avg_v = get_averaged_hall_voltage(num_samples=NUM_SAMPLES_CALIBRATION)
    if avg_v is not None: IDLE_VOLTAGE = avg_v
    
    avg_rp = get_averaged_rp_data(num_samples=NUM_SAMPLES_CALIBRATION)
    if avg_rp is not None: IDLE_RP_VALUE = int(round(avg_rp))
    
    previous_filtered_mag_mT = None
    
    if is_manual_call:
        messagebox.showinfo("Calibration Complete", f"Calibration finished.\nHall Idle: {IDLE_VOLTAGE:.4f}V\nLDC Idle: {IDLE_RP_VALUE}")
        print("--- Calibration Complete ---")

def update_camera_feed(): # Original Function
    global lv_camera_label, window, camera
    if not window or not window.winfo_exists(): return
    img_tk = None
    if camera and camera.isOpened():
        ret, frame = camera.read()
        if ret and frame is not None:
            try:
                img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                img_pil = Image.fromarray(img_rgb)
                img_pil.thumbnail((DISPLAY_IMG_WIDTH, DISPLAY_IMG_HEIGHT), Image.Resampling.NEAREST)
                img_tk = ImageTk.PhotoImage(img_pil)
            except Exception: pass
    if lv_camera_label:
        if img_tk: lv_camera_label.img_tk = img_tk; lv_camera_label.configure(image=img_tk, text="")
    if window and window.winfo_exists(): window.after(CAMERA_UPDATE_INTERVAL_MS, update_camera_feed)

def update_magnetism(): # Original Function
    global lv_magnetism_label, window, previous_filtered_mag_mT, IDLE_VOLTAGE
    global g_last_live_magnetism_mT
    if not window or not window.winfo_exists(): return
    display_text = "N/A"
    if hall_sensor:
        avg_v = get_averaged_hall_voltage(num_samples=NUM_SAMPLES_PER_UPDATE)
        if avg_v is not None:
            raw_mT = (avg_v - IDLE_VOLTAGE) / SENSITIVITY_V_PER_MILLITESLA
            if previous_filtered_mag_mT is None: previous_filtered_mag_mT = raw_mT
            filt_mT = (MAGNETISM_FILTER_ALPHA * raw_mT) + ((1-MAGNETISM_FILTER_ALPHA)*previous_filtered_mag_mT)
            g_last_live_magnetism_mT = filt_mT
            previous_filtered_mag_mT = filt_mT
            display_text = f"{filt_mT:+.2f}mT" if abs(filt_mT) >= 0.1 else f"{filt_mT*1000:+.1f}µT"
    if lv_magnetism_label: lv_magnetism_label.config(text=display_text)
    if window and window.winfo_exists(): window.after(GUI_UPDATE_INTERVAL_MS, update_magnetism)

def update_ldc_reading(): # Original Function
    global lv_ldc_label, window, RP_DISPLAY_BUFFER, IDLE_RP_VALUE
    if not window or not window.winfo_exists(): return
    display_text = "N/A"
    if ldc_initialized:
        avg_rp = get_averaged_rp_data(num_samples=NUM_SAMPLES_PER_UPDATE)
        if avg_rp is not None:
            RP_DISPLAY_BUFFER.append(avg_rp)
            cur_rp = int(round(statistics.mean(RP_DISPLAY_BUFFER)))
            delta = cur_rp - IDLE_RP_VALUE
            display_text = f"{cur_rp}(Δ{delta:+,})"
    if lv_ldc_label: lv_ldc_label.config(text=display_text)
    if window and window.winfo_exists(): window.after(GUI_UPDATE_INTERVAL_MS, update_ldc_reading)

# --- NEW: Gated Automation Loop ---
def manage_automation_flow():
    global window, g_previous_control_state, g_last_calibration_time, g_accepting_triggers
    global CONTROL_PIN, CONTROL_PIN_SETUP_OK, RPi_GPIO_AVAILABLE

    if not window or not window.winfo_exists(): return
    if not CONTROL_PIN_SETUP_OK or not RPi_GPIO_AVAILABLE:
        if window.winfo_exists():
            window.after(CONTROL_CHECK_INTERVAL_MS, manage_automation_flow)
        return
    
    try:
        current_state = GPIO.input(CONTROL_PIN)
        if g_previous_control_state is None: g_previous_control_state = current_state

        if g_accepting_triggers and current_state == GPIO.HIGH and g_previous_control_state == GPIO.LOW:
            print(f"AUTOMATION: Armed and rising edge detected. Triggering classification...")
            window.after(10, capture_and_classify) # Short delay before capture
        
        elif current_state == GPIO.LOW:
            current_time = time.time()
            if (current_time - g_last_calibration_time) >= 0.5:
                calibrate_sensors(is_manual_call=False)
                g_last_calibration_time = current_time

        g_previous_control_state = current_state
    except Exception as e:
        print(f"ERROR in automation loop: {e}")

    if window.winfo_exists():
        window.after(CONTROL_CHECK_INTERVAL_MS, manage_automation_flow)

# ======================
# === GUI Setup ========
# ======================
def setup_gui(): # Modified for automation
    global window, main_frame, placeholder_img_tk, live_view_frame, results_view_frame
    global lv_camera_label, lv_magnetism_label, lv_ldc_label, lv_save_checkbox
    global rv_image_label, rv_prediction_label, rv_confidence_label, rv_magnetism_label, rv_ldc_label, rv_classify_another_button
    global label_font, readout_font, button_font, title_font, result_title_font, result_value_font, pred_font, save_output_var

    window = tk.Tk()
    window.title("AI Metal Classifier v3.0.22 (Gated Automation)")
    window.geometry("800x600")
    # Simplified GUI setup from original for brevity
    main_frame = ttk.Frame(window, padding="5"); main_frame.pack(fill=tk.BOTH, expand=True)
    live_view_frame = ttk.Frame(main_frame);
    lv_camera_label = ttk.Label(live_view_frame, text="Initializing Camera...", anchor="center")
    lv_camera_label.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
    
    controls_frame = ttk.Frame(live_view_frame, padding="10")
    lv_magnetism_label = ttk.Label(controls_frame, text="Mag: Init...")
    lv_magnetism_label.pack(pady=5)
    lv_ldc_label = ttk.Label(controls_frame, text="LDC: Init...")
    lv_ldc_label.pack(pady=5)
    
    # Status Label instead of buttons
    status_label = ttk.Label(controls_frame, text="AUTO-CONTROL ACTIVE", foreground="green", font=("TkDefaultFont", 10, "bold"))
    status_label.pack(pady=20)
    
    save_output_var = tk.IntVar(value=0)
    lv_save_checkbox = ttk.Checkbutton(controls_frame, text="Save Result Screenshot", variable=save_output_var)
    lv_save_checkbox.pack(pady=10)
    controls_frame.pack(side=tk.RIGHT, fill=tk.Y)
    
    results_view_frame = ttk.Frame(main_frame, padding="10")
    placeholder_img_tk = create_placeholder_image(RESULT_IMG_DISPLAY_WIDTH, int(RESULT_IMG_DISPLAY_WIDTH*0.75))
    rv_image_label = ttk.Label(results_view_frame, image=placeholder_img_tk)
    rv_image_label.pack(pady=10)
    rv_prediction_label = ttk.Label(results_view_frame, text="---")
    rv_prediction_label.pack(pady=2)
    rv_confidence_label = ttk.Label(results_view_frame, text="---")
    rv_confidence_label.pack(pady=2)
    rv_magnetism_label = ttk.Label(results_view_frame, text="---")
    rv_magnetism_label.pack(pady=2)
    rv_ldc_label = ttk.Label(results_view_frame, text="---")
    rv_ldc_label.pack(pady=2)
    rv_classify_another_button = ttk.Button(results_view_frame, text="<< Classify Another", command=calibrate_and_show_live_view)
    rv_classify_another_button.pack(pady=20)

    clear_results_display()
    show_live_view()

# ==========================
# === Main Execution =======
# ==========================
def run_application():
    setup_gui()
    update_camera_feed()
    update_magnetism()
    update_ldc_reading()
    manage_automation_flow() # NEW automation loop
    window.protocol("WM_DELETE_WINDOW", on_closing)
    window.mainloop()

def on_closing():
    global window
    if messagebox.askokcancel("Quit", "Do you want to quit?"):
        if window: window.quit()

# ==========================
# === Cleanup Resources ====
# ==========================
def cleanup_resources():
    print("\n--- Cleaning up resources ---")
    if camera and camera.isOpened(): camera.release()
    if spi:
        if ldc_initialized: enable_ldc_powermode(SLEEP_MODE)
        spi.close()
    if RPi_GPIO_AVAILABLE and GPIO.getmode() is not None:
        GPIO.cleanup()
    print("--- Cleanup complete ---")

# ==========================
# === Main Entry Point =====
# ==========================
if __name__ == '__main__':
    hw_init_attempted = False
    try:
        initialize_hardware(); hw_init_attempted = True
        initialize_ai()
        run_application()
    except KeyboardInterrupt: print("\nKeyboard interrupt detected.")
    except Exception as e: traceback.print_exc()
    finally:
        if 'window' in globals() and window:
            try: window.destroy()
            except Exception: pass
        if hw_init_attempted: cleanup_resources()
        print("\nApplication finished.")
