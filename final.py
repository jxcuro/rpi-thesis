# CODE 3.0.12 - AI Metal Classifier GUI with Results Page and Sorting Output
# Description: Displays live sensor data and camera feed.
#              Captures image and sensor readings, classifies metal using a TFLite model,
#              displays the results on a dedicated page, and sends a sorting signal via GPIO.
# Version: 3.0.12 - Modified send_sorting_signal to reset data pins to LOW after each transmission
#                  for a cleaner signal protocol, potentially helping receiver hardware.
#                  Enhanced GPIO initialization prints for sorting.
# FIXED:       Potential mismatch between sensor data processing and scaler expectation.
# DEBUG:       Enhanced prints in capture_and_classify, preprocess_input, run_inference, postprocess_output.

import tkinter as tk
from tkinter import ttk
from tkinter import font as tkFont
from tkinter import messagebox
import cv2 # OpenCV for camera access
from PIL import Image, ImageTk
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
        exit()

try:
    import joblib
except ImportError:
    print("ERROR: Joblib is not installed.")
    exit()

# --- I2C/ADS1115 Imports ---
I2C_ENABLED = False
try:
    import board
    import busio
    import adafruit_ads1x15.ads1115 as ADS
    from adafruit_ads1x15.analog_in import AnalogIn
    I2C_ENABLED = True
    print("I2C/ADS1115 libraries imported successfully.")
except Exception as e:
    print(f"Warning: I2C/ADS1115 libraries import failed: {e}. Magnetism readings disabled.")

# --- SPI/LDC1101 & RPi.GPIO Imports ---
SPI_ENABLED = False # For spidev library itself
RPi_GPIO_AVAILABLE = False # For RPi.GPIO library itself
try:
    import spidev
    SPI_ENABLED = True
    print("SPI library (spidev) imported successfully.")
except ImportError:
    print("Warning: SPI library (spidev) not found. LDC readings will be disabled.")

try:
    import RPi.GPIO as GPIO
    RPi_GPIO_AVAILABLE = True
    print("RPi.GPIO library imported successfully.")
except ImportError:
    print("Warning: RPi.GPIO library not found. LDC CS control and Sorting will be disabled.")
except RuntimeError:
    print("Warning: RPi.GPIO library likely requires root privileges (sudo). LDC CS and Sorting may fail.")
    # RPi_GPIO_AVAILABLE might still be true if imported but runtime error on load
    # We'll rely on subsequent GPIO calls to fail if permissions are the issue.


# --- Sorting GPIO Configuration ---
SORTING_GPIO_ENABLED = False # Set True if RPi.GPIO is available AND setup succeeds
SORTING_DATA_PIN_LSB = 16 # BCM Pin
SORTING_DATA_PIN_MID = 6  # BCM Pin
SORTING_DATA_READY_PIN = 26 # BCM Pin

# ==================================
# === Constants and Configuration ===
# ==================================
NUM_SAMPLES_PER_UPDATE = 3
NUM_SAMPLES_CALIBRATION = 15
GUI_UPDATE_INTERVAL_MS = 100
CAMERA_UPDATE_INTERVAL_MS = 50
LDC_DISPLAY_BUFFER_SIZE = 5
MAGNETISM_FILTER_ALPHA = 0.2

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
CS_PIN = 8 # BCM Pin for LDC Chip Select

LDC_CHIP_ID = 0xD4
START_CONFIG_REG, RP_SET_REG, TC1_REG, TC2_REG, DIG_CONFIG_REG, ALT_CONFIG_REG, \
D_CONF_REG, INTB_MODE_REG, RP_DATA_MSB_REG, RP_DATA_LSB_REG, CHIP_ID_REG = \
0x0B, 0x01, 0x02, 0x03, 0x04, 0x05, 0x0C, 0x0A, 0x22, 0x21, 0x3F
ACTIVE_CONVERSION_MODE, SLEEP_MODE = 0x00, 0x01
IDLE_RP_VALUE = 0

# Global Objects
camera = i2c = ads = hall_sensor = spi = None
ldc_initialized = False
interpreter = input_details = output_details = numerical_scaler = None
loaded_labels = []
RP_DISPLAY_BUFFER = deque(maxlen=LDC_DISPLAY_BUFFER_SIZE)
previous_filtered_mag_mT = None
window = main_frame = live_view_frame = results_view_frame = None
label_font, readout_font, button_font, title_font, result_title_font, result_value_font = (None,) * 6
lv_camera_label, lv_magnetism_label, lv_ldc_label, lv_classify_button, lv_calibrate_button = (None,) * 5
rv_image_label, rv_prediction_label, rv_confidence_label, rv_magnetism_label, rv_ldc_label, rv_classify_another_button = (None,) * 6
placeholder_img_tk = None

# =========================
# === Hardware Setup ===
# =========================
def initialize_hardware():
    global camera, i2c, ads, hall_sensor, spi, ldc_initialized, CS_PIN
    global SORTING_GPIO_ENABLED, RPi_GPIO_AVAILABLE

    print("\n--- Initializing Hardware ---")

    # Camera
    try:
        camera = cv2.VideoCapture(CAMERA_INDEX); time.sleep(0.5)
        if not camera or not camera.isOpened(): raise ValueError("Cam open failed.")
        print(f"Camera {CAMERA_INDEX} opened.")
    except Exception as e: print(f"ERROR: Camera init: {e}"); camera = None

    # I2C/ADS1115
    if I2C_ENABLED:
        try:
            i2c = busio.I2C(board.SCL, board.SDA); ads = ADS.ADS1115(i2c)
            if HALL_ADC_CHANNEL: hall_sensor = AnalogIn(ads, HALL_ADC_CHANNEL); print("ADS1115/Hall initialized.")
            else: print("Warning: HALL_ADC_CHANNEL not set."); hall_sensor = None
        except Exception as e: print(f"ERROR: I2C/ADS1115 init: {e}"); i2c = ads = hall_sensor = None
    else: print("Skipping I2C/ADS1115 setup.")

    # GPIO general setup (BCM mode)
    gpio_bcm_mode_set = False
    if RPi_GPIO_AVAILABLE:
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            gpio_bcm_mode_set = True
            print("GPIO BCM mode set.")
        except Exception as e: print(f"ERROR: GPIO.setmode(BCM) failed: {e}")
    else: print("RPi.GPIO not available, skipping GPIO setups.")

    # SPI/LDC1101 (needs SPI_ENABLED and working GPIO for CS)
    if SPI_ENABLED and gpio_bcm_mode_set:
        try:
            GPIO.setup(CS_PIN, GPIO.OUT, initial=GPIO.HIGH)
            print(f"LDC CS Pin {CS_PIN} set as OUTPUT HIGH.")
            spi = spidev.SpiDev(); spi.open(SPI_BUS, SPI_DEVICE)
            spi.max_speed_hz = SPI_SPEED; spi.mode = SPI_MODE
            print(f"SPI for LDC initialized.")
            if initialize_ldc1101(): enable_ldc_rpmode(); print("LDC1101 initialized and RP+L mode enabled.")
            else: print("ERROR: LDC1101 Low-level Init Failed."); ldc_initialized = False
        except Exception as e:
            print(f"ERROR: SPI/LDC init: {e}")
            if spi: spi.close(); spi = None
            ldc_initialized = False
    elif SPI_ENABLED and not gpio_bcm_mode_set:
        print("Skipping LDC setup (spidev present, but GPIO for CS failed).")
    else: print("Skipping SPI/LDC setup (spidev lib not found or GPIO BCM mode failed).")

    # Sorting GPIO Pin Initialization
    if gpio_bcm_mode_set: # Only if RPi.GPIO is available and BCM mode was set
        print("Attempting to initialize GPIO pins for Sorting Mechanism...")
        try:
            GPIO.setup(SORTING_DATA_PIN_LSB, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(SORTING_DATA_PIN_MID, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(SORTING_DATA_READY_PIN, GPIO.OUT, initial=GPIO.LOW)
            SORTING_GPIO_ENABLED = True
            print(f"Sorting GPIO pins ({SORTING_DATA_PIN_LSB}, {SORTING_DATA_PIN_MID}, {SORTING_DATA_READY_PIN}) set as OUTPUT LOW. Sorting is ENABLED.")
        except Exception as e:
            print(f"ERROR: Failed to set up sorting GPIO pins: {e}. Sorting is DISABLED.")
            SORTING_GPIO_ENABLED = False
    else:
        print("Skipping Sorting GPIO setup (RPi.GPIO not available or BCM mode failed). Sorting is DISABLED.")
        SORTING_GPIO_ENABLED = False
    print("--- Hardware Initialization Complete ---")

# =========================
# === AI Model Setup ======
# =========================
def initialize_ai():
    global interpreter, input_details, output_details, loaded_labels, numerical_scaler
    print("\n--- Initializing AI Components ---")
    ai_ready = True
    try:
        with open(LABELS_PATH, 'r') as f: loaded_labels = [line.strip() for line in f.readlines()]
        if not loaded_labels: raise ValueError("Labels file empty.")
        print(f"Loaded {len(loaded_labels)} labels.")
    except Exception as e: print(f"ERROR: Reading labels '{LABELS_FILENAME}': {e}"); ai_ready = False

    if ai_ready:
        try:
            numerical_scaler = joblib.load(SCALER_PATH)
            if not hasattr(numerical_scaler, 'transform'): raise TypeError("Scaler invalid.")
            expected_features = 2 # Mag, LDC_RP
            if hasattr(numerical_scaler, 'n_features_in_') and numerical_scaler.n_features_in_ != expected_features:
                print(f"ERROR: Scaler/script feature mismatch ({numerical_scaler.n_features_in_} vs {expected_features})."); ai_ready = False
        except Exception as e: print(f"ERROR: Loading scaler '{SCALER_FILENAME}': {e}"); ai_ready = False
    if ai_ready:
        try:
            interpreter = Interpreter(model_path=MODEL_PATH); interpreter.allocate_tensors()
            input_details = interpreter.get_input_details(); output_details = interpreter.get_output_details()
            print("TFLite model loaded.")
            if len(input_details) != 2: print("ERROR: Model inputs != 2."); ai_ready = False
            if output_details and output_details[0]['shape'][-1] != len(loaded_labels): print("ERROR: Model output size != labels."); ai_ready = False
        except Exception as e: print(f"ERROR: Loading model '{MODEL_FILENAME}': {e}"); ai_ready = False
    if not ai_ready: print("--- AI Initialization Failed ---"); interpreter = numerical_scaler = None
    else: print("--- AI Initialization Complete ---")
    return ai_ready

# =========================
# === LDC1101 Functions ===
# =========================
def ldc_write_register(reg_addr, value): # Assumes CS_PIN, spi, and GPIO are globally available and initialized
    if not spi or not RPi_GPIO_AVAILABLE : return False
    try: GPIO.output(CS_PIN, GPIO.LOW); spi.xfer2([reg_addr & 0x7F, value]); GPIO.output(CS_PIN, GPIO.HIGH); return True
    except Exception as e: print(f"Warn: LDC write (0x{reg_addr:02X}): {e}"); try: GPIO.output(CS_PIN, GPIO.HIGH); except: pass; return False

def ldc_read_register(reg_addr):
    if not spi or not RPi_GPIO_AVAILABLE : return None
    try: GPIO.output(CS_PIN, GPIO.LOW); result = spi.xfer2([reg_addr | 0x80, 0x00]); GPIO.output(CS_PIN, GPIO.HIGH); return result[1]
    except Exception as e: print(f"Warn: LDC read (0x{reg_addr:02X}): {e}"); try: GPIO.output(CS_PIN, GPIO.HIGH); except: pass; return None

def initialize_ldc1101(): # Low-level LDC init
    global ldc_initialized; ldc_initialized = False
    if not spi: return False
    try:
        chip_id = ldc_read_register(CHIP_ID_REG)
        if chip_id is None or chip_id != LDC_CHIP_ID: print(f"ERR: LDC ID fail (Read:{chip_id})"); return False
        print(f"LDC1101 Chip ID OK (0x{chip_id:02X}).")
        # Simplified: Use your required register settings
        regs_to_write = {RP_SET_REG: 0x07, TC1_REG: 0x90, TC2_REG: 0xA0, DIG_CONFIG_REG: 0x03,
                         ALT_CONFIG_REG: 0x00, D_CONF_REG: 0x00, INTB_MODE_REG: 0x00}
        for reg, val in regs_to_write.items():
            if not ldc_write_register(reg, val): print(f"ERR: LDC write failed reg {reg:02X}"); return False
        if not ldc_write_register(START_CONFIG_REG, SLEEP_MODE): return False; time.sleep(0.02)
        print("LDC1101 Config OK."); ldc_initialized = True; return True
    except Exception as e: print(f"ERR: LDC Init Exception: {e}"); return False

def enable_ldc_powermode(mode):
    if not spi or not ldc_initialized: return False
    if ldc_write_register(START_CONFIG_REG, mode): time.sleep(0.01); return True
    print(f"Warn: Failed to set LDC power mode."); return False

def enable_ldc_rpmode():
    if not spi or not ldc_initialized: return False
    try:
        if not ldc_write_register(ALT_CONFIG_REG, 0x00): return False # Example for RP+L
        if not ldc_write_register(D_CONF_REG, 0x00): return False   # Example for RP+L
        if enable_ldc_powermode(ACTIVE_CONVERSION_MODE): return True
        print("Failed to set LDC to Active for RP+L."); return False
    except Exception as e: print(f"Warn: LDC RP mode enable fail: {e}"); return False

def get_ldc_rpdata():
    if not spi or not ldc_initialized: return None
    try:
        msb = ldc_read_register(RP_DATA_MSB_REG); lsb = ldc_read_register(RP_DATA_LSB_REG)
        if msb is None or lsb is None: return None
        return (msb << 8) | lsb
    except Exception as e: print(f"Warn: LDC RP data read exception: {e}"); return None

# ============================
# === Sensor Reading (Avg) ===
# ============================
def get_averaged_hall_voltage(num_samples=NUM_SAMPLES_PER_UPDATE):
    if not hall_sensor: return None
    readings = []
    for _ in range(num_samples):
        try: readings.append(hall_sensor.voltage)
        except Exception as e: print(f"Warn: Hall read: {e}"); return None
    return statistics.mean(readings) if readings else None

def get_averaged_rp_data(num_samples=NUM_SAMPLES_PER_UPDATE):
    if not ldc_initialized: return None
    readings = [val for val in (get_ldc_rpdata() for _ in range(num_samples)) if val is not None]
    return statistics.mean(readings) if readings else None

# ==========================
# === AI Processing ========
# ==========================
def preprocess_input(image_pil, mag_mT, ldc_rp_raw): # Same as before
    global numerical_scaler, input_details, interpreter
    if interpreter is None or numerical_scaler is None: print("ERR: AI/Scaler not init."); return None
    try:
        img_resized = image_pil.resize((AI_IMG_WIDTH, AI_IMG_HEIGHT), Image.Resampling.LANCZOS)
        image_np = np.array(img_resized.convert('RGB'), dtype=np.float32) / 255.0
        image_input = np.expand_dims(image_np, axis=0)
    except Exception as e: print(f"ERR: Image preprocess: {e}"); return None

    mag_mT_val = float(mag_mT) if mag_mT is not None else 0.0
    ldc_rp_raw_val = float(ldc_rp_raw) if ldc_rp_raw is not None else 0.0
    numerical_features = np.array([[mag_mT_val, ldc_rp_raw_val]], dtype=np.float32)
    try:
        with warnings.catch_warnings():
            warnings.filterwarnings("ignore", message="X does not have valid feature names.*", category=UserWarning)
            scaled_numerical_features = numerical_scaler.transform(numerical_features)
    except Exception as e: print(f"ERR: Scaling numerical: {e}"); return None

    image_idx, numerical_idx = -1, -1; img_dtype, num_dtype = None, None
    for detail in input_details:
        shape = detail['shape']
        if len(shape) == 4 and shape[1:3] == (AI_IMG_HEIGHT, AI_IMG_WIDTH): image_idx, img_dtype = detail['index'], detail['dtype']
        elif len(shape) == 2: numerical_idx, num_dtype = detail['index'], detail['dtype']
    if image_idx == -1 or numerical_idx == -1: print("ERR: Cannot ID model inputs."); return None
    if input_details[numerical_idx]['shape'][1] != scaled_numerical_features.shape[1]: print("ERR: Numerical feature count mismatch."); return None

    final_img_input = image_input.astype(img_dtype)
    if img_dtype == np.uint8: final_img_input = (image_input * 255.0).astype(np.uint8) # Rescale for UINT8
    return {image_idx: final_img_input, numerical_idx: scaled_numerical_features.astype(num_dtype)}

def run_inference(model_inputs): # Same as before
    global interpreter, output_details
    if interpreter is None or model_inputs is None: print("ERR: Interpreter/inputs not ready."); return None
    try:
        for idx, data in model_inputs.items(): interpreter.set_tensor(idx, data)
        interpreter.invoke()
        return interpreter.get_tensor(output_details[0]['index'])
    except Exception as e: print(f"ERR: Inference: {e}"); traceback.print_exc(); return None

def postprocess_output(output_data): # Same as before
    global loaded_labels
    if output_data is None or not loaded_labels: print("ERR: No output/labels for postproc."); return "Error", 0.0
    try:
        probs = output_data[0] if len(output_data.shape) == 2 else output_data
        if len(probs) != len(loaded_labels): print("ERR: Probs vs Labels mismatch."); return "Label Mismatch", 0.0
        idx = np.argmax(probs)
        return loaded_labels[idx], float(probs[idx])
    except Exception as e: print(f"ERR: Postprocess: {e}"); return "Post Err", 0.0

# ==================================
# === Sorting Signal Functions ===
# ==================================
def send_sorting_signal(material_label):
    if not SORTING_GPIO_ENABLED:
        print("Sorting Signal: GPIO for sorting not enabled or RPi.GPIO not available. Skipping send.")
        return
    # This check is redundant if SORTING_GPIO_ENABLED implies RPi_GPIO_AVAILABLE, but good for safety
    if not RPi_GPIO_AVAILABLE:
        print("Sorting Signal: RPi.GPIO library not available. Cannot send signal.")
        return

    print(f"\n--- Sending Sorting Signal for: {material_label} ---")
    mid_val, lsb_val = GPIO.LOW, GPIO.LOW # Default to "Others" (00)
    signal_desc = "Others (00)"

    if material_label == "Aluminum": mid_val, lsb_val, signal_desc = GPIO.LOW, GPIO.HIGH, "Aluminum (01)"
    elif material_label == "Copper": mid_val, lsb_val, signal_desc = GPIO.HIGH, GPIO.LOW, "Copper (10)"
    elif material_label == "Steel": mid_val, lsb_val, signal_desc = GPIO.HIGH, GPIO.HIGH, "Steel (11)"

    try:
        GPIO.output(SORTING_DATA_READY_PIN, GPIO.LOW) # Ensure ready is low
        time.sleep(0.01)
        GPIO.output(SORTING_DATA_PIN_MID, mid_val)
        GPIO.output(SORTING_DATA_PIN_LSB, lsb_val)
        print(f"Set GPIO Pins: MID={mid_val}, LSB={lsb_val} for {signal_desc}")
        time.sleep(0.01) # Settle

        GPIO.output(SORTING_DATA_READY_PIN, GPIO.HIGH) # Pulse ready HIGH
        print(f"Pulsed {SORTING_DATA_READY_PIN} HIGH (Data Ready)")
        time.sleep(0.05) # Hold time for receiver
        GPIO.output(SORTING_DATA_READY_PIN, GPIO.LOW) # Pulse ready LOW
        print(f"Set {SORTING_DATA_READY_PIN} LOW (Data Transmitted)")

        # Reset data pins to LOW after transmission
        time.sleep(0.01) # Brief pause
        GPIO.output(SORTING_DATA_PIN_MID, GPIO.LOW)
        GPIO.output(SORTING_DATA_PIN_LSB, GPIO.LOW)
        print(f"Data pins ({SORTING_DATA_PIN_MID}, {SORTING_DATA_PIN_LSB}) reset to LOW after signal.")
        print(f"--- Sorting signal {signal_desc} sent ---")
    except Exception as e:
        print(f"ERROR: Failed to send sorting signal via GPIO: {e}")
        try: # Attempt to leave pins in a known low state on error
            if RPi_GPIO_AVAILABLE:
                GPIO.output(SORTING_DATA_READY_PIN, GPIO.LOW)
                GPIO.output(SORTING_DATA_PIN_MID, GPIO.LOW)
                GPIO.output(SORTING_DATA_PIN_LSB, GPIO.LOW)
                print("Ensured sorting pins are LOW after error.")
        except Exception as e_cleanup: print(f"Warn: Could not reset pins after error: {e_cleanup}")

# ==============================
# === View Switching Logic ===
# ==============================
def show_live_view(): # Mostly same
    global live_view_frame, results_view_frame, lv_classify_button, interpreter
    if results_view_frame and results_view_frame.winfo_ismapped(): results_view_frame.pack_forget()
    if live_view_frame and not live_view_frame.winfo_ismapped():
        live_view_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
    if lv_classify_button: lv_classify_button.config(state=tk.NORMAL if interpreter else tk.DISABLED)

def show_results_view(): # Same
    global live_view_frame, results_view_frame
    if live_view_frame and live_view_frame.winfo_ismapped(): live_view_frame.pack_forget()
    if results_view_frame and not results_view_frame.winfo_ismapped():
        results_view_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

# ======================
# === GUI Functions ===
# ======================
def create_placeholder_image(width, height, color='#E0E0E0', text="No Image"): # Same
    try: return ImageTk.PhotoImage(Image.new('RGB', (width, height), color))
    except: return None

def clear_results_display(): # Same
    global rv_image_label, rv_prediction_label, rv_confidence_label, rv_magnetism_label, rv_ldc_label, placeholder_img_tk
    if rv_image_label: rv_image_label.config(image=placeholder_img_tk or '', text="No Image" if not placeholder_img_tk else ""); rv_image_label.img_tk = placeholder_img_tk
    for label_widget in [rv_prediction_label, rv_confidence_label, rv_magnetism_label, rv_ldc_label]:
        if label_widget: label_widget.config(text="---")

def capture_and_classify(): # Calls new send_sorting_signal
    global lv_classify_button, window, camera, IDLE_VOLTAGE, IDLE_RP_VALUE, interpreter
    global rv_image_label, rv_prediction_label, rv_confidence_label, rv_magnetism_label, rv_ldc_label

    print("\n" + "="*10 + " Capture & Classify Triggered " + "="*10)
    if not interpreter: messagebox.showerror("Error", "AI Model not init."); return
    if not camera or not camera.isOpened(): messagebox.showerror("Error", "Camera not avail."); return

    if lv_classify_button: lv_classify_button.config(state=tk.DISABLED); window.update_idletasks()
    ret, frame = camera.read()
    if not ret or frame is None: messagebox.showerror("Capture Error", "Failed image capture."); show_live_view(); return
    try: img_cap_pil = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    except Exception as e: messagebox.showerror("Image Error", f"Img proc fail: {e}"); show_live_view(); return

    avg_volt = get_averaged_hall_voltage(NUM_SAMPLES_CALIBRATION)
    cur_mag_mT, mag_disp_txt, sens_warn = None, "N/A", False
    if avg_volt is not None:
        try:
            if abs(SENSITIVITY_V_PER_MILLITESLA) < 1e-9: raise ZeroDivisionError("Sens zero")
            cur_mag_mT = (avg_volt - IDLE_VOLTAGE) / SENSITIVITY_V_PER_MILLITESLA
            mag_disp_txt = f"{cur_mag_mT:+.3f} mT" + (" (NoCal)" if IDLE_VOLTAGE==0 else "")
        except Exception: mag_disp_txt="CalcErr"; sens_warn=True
    else: mag_disp_txt="ReadErr"; sens_warn=True

    avg_rp = get_averaged_rp_data(NUM_SAMPLES_CALIBRATION)
    cur_rp_raw, ldc_disp_txt = None, "N/A"
    if avg_rp is not None:
        cur_rp_raw = avg_rp; cur_rp_int = int(round(avg_rp))
        delta = cur_rp_int - IDLE_RP_VALUE
        ldc_disp_txt = f"{cur_rp_int}" + (f"(Δ{delta:+,})" if IDLE_RP_VALUE!=0 else "(NoCal)")
    else: ldc_disp_txt="ReadErr"; sens_warn=True
    if sens_warn: print("WARN: Sensor issues may affect classification.")

    model_inputs = preprocess_input(img_cap_pil, cur_mag_mT, cur_rp_raw)
    if model_inputs is None: messagebox.showerror("AI Error", "Preprocessing failed."); show_live_view(); return
    output_data = run_inference(model_inputs)
    if output_data is None: messagebox.showerror("AI Error", "Inference failed."); show_live_view(); return
    pred_label, conf = postprocess_output(output_data)
    print(f"--- Result: '{pred_label}', Conf={conf:.1%} ---")

    send_sorting_signal(pred_label) # INTEGRATED CALL

    if rv_image_label: # Update results GUI
        try:
            aspect = img_cap_pil.height / img_cap_pil.width if img_cap_pil.width > 0 else 0.75
            h = int(RESULT_IMG_DISPLAY_WIDTH * aspect) if aspect > 0 else int(RESULT_IMG_DISPLAY_WIDTH*0.75)
            img_disp = img_cap_pil.resize((RESULT_IMG_DISPLAY_WIDTH, max(1,h)), Image.Resampling.LANCZOS)
            img_tk = ImageTk.PhotoImage(img_disp); rv_image_label.img_tk = img_tk
            rv_image_label.config(image=img_tk, text="")
        except Exception as e: rv_image_label.config(image=placeholder_img_tk or '', text="ImgErr")
    if rv_prediction_label: rv_prediction_label.config(text=f"{pred_label}")
    if rv_confidence_label: rv_confidence_label.config(text=f"{conf:.1%}")
    if rv_magnetism_label: rv_magnetism_label.config(text=mag_disp_txt)
    if rv_ldc_label: rv_ldc_label.config(text=ldc_disp_txt)
    show_results_view()
    print("="*10 + " Capture & Classify Complete " + "="*10 + "\n")

def calibrate_sensors(): # Mostly same
    global IDLE_VOLTAGE, IDLE_RP_VALUE, window, previous_filtered_mag_mT, hall_sensor, ldc_initialized
    hall_avail, ldc_avail = hall_sensor is not None, ldc_initialized
    if not hall_avail and not ldc_avail: messagebox.showwarning("Calibration", "No sensors available."); return
    instr = "Ensure NO metal near sensors.\n\n" + ("- Hall idle voltage.\n" if hall_avail else "") + ("- LDC idle RP.\n" if ldc_avail else "")
    if not messagebox.askokcancel("Calibration", instr + "\nOK to start?"): return

    if lv_calibrate_button: lv_calibrate_button.config(state=tk.DISABLED)
    if lv_classify_button: lv_classify_button.config(state=tk.DISABLED); window.update_idletasks()
    hall_res, ldc_res = "Hall:N/A", "LDC:N/A"; hall_ok, ldc_ok = False, False
    if hall_avail:
        avg_v = get_averaged_hall_voltage(NUM_SAMPLES_CALIBRATION)
        if avg_v is not None: IDLE_VOLTAGE=avg_v; hall_res=f"Hall Idle:{IDLE_VOLTAGE:.4f}V"; hall_ok=True
        else: IDLE_VOLTAGE=0.0; hall_res="Hall:ReadErr!"
    if ldc_avail:
        avg_rp = get_averaged_rp_data(NUM_SAMPLES_CALIBRATION)
        if avg_rp is not None: IDLE_RP_VALUE=int(round(avg_rp)); ldc_res=f"LDC Idle RP:{IDLE_RP_VALUE}"; ldc_ok=True
        else: IDLE_RP_VALUE=0; ldc_res="LDC:ReadErr!"
    previous_filtered_mag_mT = None
    if lv_calibrate_button: lv_calibrate_button.config(state=tk.NORMAL)
    if lv_classify_button: lv_classify_button.config(state=tk.NORMAL if interpreter else tk.DISABLED)
    msg = f"Calibration Results:\n\n{hall_res}\n{ldc_res}"
    if (hall_avail and not hall_ok) or (ldc_avail and not ldc_ok): messagebox.showwarning("Calibration Warning", msg)
    else: messagebox.showinfo("Calibration Complete", msg)

# Live update loops (update_camera_feed, update_magnetism, update_ldc_reading) are mostly unchanged
def update_camera_feed(): # Mostly same
    global lv_camera_label, window, camera
    if not window or not window.winfo_exists(): return
    img_tk = None
    if camera and camera.isOpened():
        ret, frame = camera.read()
        if ret and frame is not None:
            try:
                img_pil = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
                img_pil.thumbnail((DISPLAY_IMG_WIDTH, DISPLAY_IMG_HEIGHT), Image.Resampling.NEAREST)
                img_tk = ImageTk.PhotoImage(img_pil)
            except: pass # Error processing frame
    if lv_camera_label:
        if img_tk: lv_camera_label.img_tk = img_tk; lv_camera_label.configure(image=img_tk, text="")
        else: # Placeholder logic (simplified)
            if not hasattr(lv_camera_label, 'no_cam_img'): lv_camera_label.no_cam_img = create_placeholder_image(DISPLAY_IMG_WIDTH//2, DISPLAY_IMG_HEIGHT//2)
            if lv_camera_label.no_cam_img and str(lv_camera_label.cget("image")) != str(lv_camera_label.no_cam_img):
                 lv_camera_label.configure(image=lv_camera_label.no_cam_img, text=""); lv_camera_label.img_tk = lv_camera_label.no_cam_img
    if window and window.winfo_exists(): window.after(CAMERA_UPDATE_INTERVAL_MS, update_camera_feed)

def update_magnetism(): # Mostly same
    global lv_magnetism_label, window, previous_filtered_mag_mT, IDLE_VOLTAGE, hall_sensor
    if not window or not window.winfo_exists(): return
    display_text = "N/A"
    if hall_sensor:
        avg_v = get_averaged_hall_voltage(NUM_SAMPLES_PER_UPDATE)
        if avg_v is not None:
            try:
                if abs(SENSITIVITY_V_PER_MILLITESLA) < 1e-9: raise ZeroDivisionError("Sens0")
                raw_mT = (avg_v - IDLE_VOLTAGE) / SENSITIVITY_V_PER_MILLITESLA
                filt_mT = (MAGNETISM_FILTER_ALPHA * raw_mT) + ((1-MAGNETISM_FILTER_ALPHA)*(previous_filtered_mag_mT or raw_mT))
                previous_filtered_mag_mT = filt_mT
                display_text = f"{filt_mT*1000:+.1f}µT" if abs(filt_mT)<0.1 else f"{filt_mT:+.2f}mT"
                if IDLE_VOLTAGE == 0.0: display_text += "(NoCal)"
            except: display_text = "CalcErr"; previous_filtered_mag_mT=None
        else: display_text = "ReadErr"; previous_filtered_mag_mT=None
    if lv_magnetism_label and lv_magnetism_label.cget("text") != display_text: lv_magnetism_label.config(text=display_text)
    if window and window.winfo_exists(): window.after(GUI_UPDATE_INTERVAL_MS, update_magnetism)

def update_ldc_reading(): # Mostly same
    global lv_ldc_label, window, RP_DISPLAY_BUFFER, IDLE_RP_VALUE, ldc_initialized
    if not window or not window.winfo_exists(): return
    display_text = "N/A"
    if ldc_initialized:
        avg_rp = get_averaged_rp_data(NUM_SAMPLES_PER_UPDATE)
        if avg_rp is not None:
            RP_DISPLAY_BUFFER.append(avg_rp)
            if RP_DISPLAY_BUFFER:
                cur_rp = int(round(statistics.mean(RP_DISPLAY_BUFFER)))
                delta = cur_rp - IDLE_RP_VALUE
                display_text = f"{cur_rp}" + (f"(Δ{delta:+,})" if IDLE_RP_VALUE!=0 else "(NoCal)")
            else: display_text = "Buffer..."
        else: display_text = "ReadErr"
    if lv_ldc_label and lv_ldc_label.cget("text") != display_text: lv_ldc_label.config(text=display_text)
    if window and window.winfo_exists(): window.after(GUI_UPDATE_INTERVAL_MS, update_ldc_reading)

# ======================
# === GUI Setup ========
# ======================
def setup_gui(): # Structurally same, minor font fallback detail
    global window, main_frame, placeholder_img_tk, live_view_frame, results_view_frame
    global lv_camera_label, lv_magnetism_label, lv_ldc_label, lv_classify_button, lv_calibrate_button
    global rv_image_label, rv_prediction_label, rv_confidence_label, rv_magnetism_label, rv_ldc_label, rv_classify_another_button
    global label_font, readout_font, button_font, title_font, result_title_font, result_value_font

    window = tk.Tk(); window.title("AI Metal Classifier v3.0.12"); window.geometry("800x600")
    style = ttk.Style(); style.theme_use('clam' if 'clam' in style.theme_names() else 'default')
    try: # Define fonts
        font_family = "DejaVu Sans"
        title_font = tkFont.Font(family=font_family, size=16, weight="bold"); label_font = tkFont.Font(family=font_family, size=10)
        readout_font = tkFont.Font(family=font_family+" Mono", size=12, weight="bold"); button_font = tkFont.Font(family=font_family, size=10, weight="bold")
        result_title_font = tkFont.Font(family=font_family, size=11, weight="bold"); result_value_font = tkFont.Font(family=font_family+" Mono", size=12, weight="bold")
        pred_font = tkFont.Font(family=font_family, size=16, weight="bold")
    except tk.TclError: # Fallback if Dejavu Sans is not available
        title_font, label_font, readout_font, button_font, result_title_font, result_value_font, pred_font = \
        "TkHeadingFont", "TkTextFont", "TkFixedFont", "TkDefaultFont", "TkDefaultFont", "TkFixedFont", "TkHeadingFont"
        print("Warn: DejaVu fonts not found, using Tkinter defaults.")

    style.configure("TLabel", font=label_font); style.configure("TButton", font=button_font)
    style.configure("Readout.TLabel", font=readout_font, foreground="#0000AA")
    style.configure("ResultValue.TLabel", font=result_value_font, foreground="#0000AA")
    style.configure("Prediction.TLabel", font=pred_font, foreground="#AA0000")

    main_frame = ttk.Frame(window, padding=5); main_frame.pack(fill=tk.BOTH, expand=True); main_frame.rowconfigure(0, weight=1); main_frame.columnconfigure(0, weight=1)
    # Live View (structure same)
    live_view_frame = ttk.Frame(main_frame, padding=5); live_view_frame.columnconfigure(0, weight=3); live_view_frame.columnconfigure(1, weight=1); live_view_frame.rowconfigure(0, weight=1)
    lv_camera_label = ttk.Label(live_view_frame, text="Init Cam...", relief="sunken", background="#CCC"); lv_camera_label.grid(row=0, column=0, sticky="nsew", padx=(0,5))
    lv_controls = ttk.Frame(live_view_frame); lv_controls.grid(row=0, column=1, sticky="nsew", padx=(5,0)); lv_controls.columnconfigure(0, weight=1)
    lv_readings = ttk.Labelframe(lv_controls, text=" Live Readings ", padding=5); lv_readings.grid(row=0, column=0, sticky="new", pady=(0,10)); lv_readings.columnconfigure(1, weight=1)
    ttk.Label(lv_readings, text="Magnetism:").grid(row=0, column=0, sticky="w"); lv_magnetism_label = ttk.Label(lv_readings, text="Init...", style="Readout.TLabel"); lv_magnetism_label.grid(row=0, column=1, sticky="ew")
    ttk.Label(lv_readings, text="LDC (Delta):").grid(row=1, column=0, sticky="w"); lv_ldc_label = ttk.Label(lv_readings, text="Init...", style="Readout.TLabel"); lv_ldc_label.grid(row=1, column=1, sticky="ew")
    lv_actions = ttk.Labelframe(lv_controls, text=" Actions ", padding=5); lv_actions.grid(row=1, column=0, sticky="new"); lv_actions.columnconfigure(0, weight=1)
    lv_classify_button = ttk.Button(lv_actions, text="Capture & Classify", command=capture_and_classify); lv_classify_button.grid(row=0, column=0, sticky="ew", pady=4)
    lv_calibrate_button = ttk.Button(lv_actions, text="Calibrate Sensors", command=calibrate_sensors); lv_calibrate_button.grid(row=1, column=0, sticky="ew", pady=4)
    # Results View (structure same)
    results_view_frame = ttk.Frame(main_frame, padding=10); results_view_frame.rowconfigure(1, weight=1); results_view_frame.columnconfigure(1, weight=1)
    rv_content = ttk.Frame(results_view_frame); rv_content.grid(row=1, column=1)
    ttk.Label(rv_content, text="Classification Result", font=title_font).grid(row=0, columnspan=2, pady=(5,15))
    ph_h = int(RESULT_IMG_DISPLAY_WIDTH*0.75); placeholder_img_tk = create_placeholder_image(RESULT_IMG_DISPLAY_WIDTH,ph_h)
    rv_image_label = ttk.Label(rv_content, relief="sunken"); rv_image_label.config(image=placeholder_img_tk if placeholder_img_tk else '', text="Img Area" if not placeholder_img_tk else ""); rv_image_label.img_tk=placeholder_img_tk
    rv_image_label.grid(row=1, columnspan=2, pady=(0,15))
    rv_details = ttk.Frame(rv_content); rv_details.grid(row=2, columnspan=2, pady=(0,15)); rv_details.columnconfigure(1, weight=1); r=0
    ttk.Label(rv_details, text="Material:", font=result_title_font).grid(row=r, column=0, sticky="w"); rv_prediction_label = ttk.Label(rv_details, text="---", style="Prediction.TLabel"); rv_prediction_label.grid(row=r, column=1, sticky="ew", padx=5); r+=1
    ttk.Label(rv_details, text="Confidence:", font=result_title_font).grid(row=r, column=0, sticky="w"); rv_confidence_label = ttk.Label(rv_details, text="---", style="ResultValue.TLabel"); rv_confidence_label.grid(row=r, column=1, sticky="ew", padx=5); r+=1
    ttk.Separator(rv_details, orient='horizontal').grid(row=r, columnspan=2, sticky='ew', pady=8); r+=1
    ttk.Label(rv_details, text="Sensor Values:", font=result_title_font).grid(row=r, columnspan=2, sticky="w", pady=(0,3)); r+=1
    ttk.Label(rv_details, text=" Magnetism:", font=result_title_font).grid(row=r, column=0, sticky="w", padx=(5,0)); rv_magnetism_label = ttk.Label(rv_details, text="---", style="ResultValue.TLabel"); rv_magnetism_label.grid(row=r, column=1, sticky="ew", padx=5); r+=1
    ttk.Label(rv_details, text=" LDC Reading:", font=result_title_font).grid(row=r, column=0, sticky="w", padx=(5,0)); rv_ldc_label = ttk.Label(rv_details, text="---", style="ResultValue.TLabel"); rv_ldc_label.grid(row=r, column=1, sticky="ew", padx=5)
    rv_classify_another_button = ttk.Button(rv_content, text="<< Classify Another", command=show_live_view); rv_classify_another_button.grid(row=3, columnspan=2, pady=(15,5))

    clear_results_display(); show_live_view()

# ==========================
# === Main Execution =======
# ==========================
def run_application(): # Mostly same
    global window, lv_classify_button, interpreter, camera, hall_sensor, ldc_initialized
    try: setup_gui()
    except Exception as e: print(f"FATAL: GUI setup: {e}"); traceback.print_exc(); return

    if not camera and lv_camera_label: lv_camera_label.configure(text="CamFail", image='')
    if not hall_sensor and lv_magnetism_label: lv_magnetism_label.config(text="N/A")
    if not ldc_initialized and lv_ldc_label: lv_ldc_label.config(text="N/A")
    if not interpreter and lv_classify_button: lv_classify_button.config(state=tk.DISABLED, text="Classify(AI Fail)")
    if not (hall_sensor or ldc_initialized) and lv_calibrate_button: lv_calibrate_button.config(state=tk.DISABLED, text="Calib(NoSens)")

    update_camera_feed(); update_magnetism(); update_ldc_reading()
    print("Starting Tkinter main loop...")
    try: window.protocol("WM_DELETE_WINDOW", on_closing); window.mainloop()
    except Exception as e: print(f"ERR: Tkinter main loop: {e}")

def on_closing(): # Same
    global window
    if messagebox.askokcancel("Quit", "Quit AI Metal Classifier?"):
        if window: window.destroy() # Let finally block handle resource cleanup

# ==========================
# === Cleanup Resources ====
# ==========================
def cleanup_resources():
    print("\n--- Cleaning up resources ---")
    global camera, spi, ldc_initialized, CS_PIN, RPi_GPIO_AVAILABLE, SORTING_GPIO_ENABLED

    if camera and camera.isOpened():
        try: camera.release(); print("Camera released.")
        except Exception as e: print(f"Warn: Cam release: {e}")
    if spi: # LDC SPI
        try:
            if ldc_initialized and RPi_GPIO_AVAILABLE and CS_PIN is not None:
                print("Putting LDC1101 to sleep...")
                try: ldc_write_register(START_CONFIG_REG, SLEEP_MODE); print("LDC sleep cmd sent.")
                except Exception as ldc_e: print(f"Note: LDC sleep err: {ldc_e}")
        finally:
            try: spi.close(); print("LDC SPI closed.")
            except Exception as e: print(f"Warn: LDC SPI close: {e}")

    if RPi_GPIO_AVAILABLE and GPIO.getmode() is not None: # Check if GPIO was used (mode set)
        # This covers GPIO used for LDC CS and/or Sorting Pins
        print("Cleaning up GPIO...")
        try: GPIO.cleanup(); print("GPIO cleaned up.")
        except RuntimeError as e: print(f"Note: GPIO cleanup runtime (already clean?): {e}")
        except Exception as e: print(f"Warn: GPIO cleanup: {e}")
    else: print("Note: GPIO not used or mode not set, skipping GPIO.cleanup().")
    print("--- Cleanup complete ---")

if __name__ == '__main__':
    print("="*30 + "\n Starting AI Metal Classifier (RPi Combined) \n" + "="*30)
    hw_init_attempted = False
    try:
        initialize_hardware(); hw_init_attempted = True
        initialize_ai()
        run_application()
    except KeyboardInterrupt: print("\nKeyboard interrupt. Exiting.")
    except Exception as e:
        print("\n" + "="*30 + f"\nFATAL ERROR in main: {e}\n" + "="*30); traceback.print_exc()
        if 'window' in globals() and window and window.winfo_exists():
            try: messagebox.showerror("Fatal Error", f"Unrecoverable error:\n\n{e}")
            except: pass # GUI might be unusable
    finally:
        # Ensure window is destroyed if it exists, to stop Tkinter loops before cleanup
        if 'window' in globals() and window:
            try:
                if window.winfo_exists(): window.destroy()
                print("Tkinter window destroyed in finally block.")
            except tk.TclError: print("Note: Tkinter window already destroyed or not fully initialized.")
            except Exception as e: print(f"Warn: Error destroying Tkinter window in finally: {e}")

        if hw_init_attempted: cleanup_resources()
        else: print("Skipping resource cleanup as hardware initialization was not fully attempted.")
        print("\nApplication finished.\n" + "="*30)
