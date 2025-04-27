# CODE 2 - AI Metal Classification Integration
# Version: 2.0.0 - Integrates TFLite model, displays results
# Based on: 1.6.1_mod2

import tkinter as tk
from tkinter import ttk
from tkinter import font as tkFont
from tkinter import messagebox
import cv2
from PIL import Image, ImageTk
import time
import os
# import csv # No longer needed for metadata saving
from datetime import datetime
import statistics
from collections import deque
import re
import numpy as np # For AI data processing

# --- AI/ML Imports ---
try:
    import tflite_runtime.interpreter as tflite
    print("TensorFlow Lite Runtime found.")
    TFLITE_ENABLED = True
except ImportError:
    print("Warning: tflite_runtime not found. AI Classification disabled.")
    print("Install using: pip install tflite-runtime")
    TFLITE_ENABLED = False

try:
    import joblib
    print("Joblib found.")
    JOBLIB_ENABLED = True
except ImportError:
    print("Warning: joblib not found. Numerical scaling disabled.")
    print("Install using: pip install joblib")
    JOBLIB_ENABLED = False


# --- I2C/ADS1115 Imports ---
# (Keep as is)
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
# (Keep as is)
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

# --- NEW: AI Configuration ---
try: BASE_PATH = os.path.dirname(os.path.abspath(__file__))
except NameError: BASE_PATH = os.getcwd() # Fallback for interactive use

MODEL_FILENAME = "material_classifier_model.tflite"
LABELS_FILENAME = "material_labels.txt"
SCALER_FILENAME = "numerical_scaler.joblib"

MODEL_PATH = os.path.join(BASE_PATH, MODEL_FILENAME)
LABELS_PATH = os.path.join(BASE_PATH, LABELS_FILENAME)
SCALER_PATH = os.path.join(BASE_PATH, SCALER_FILENAME)

# --- IMPORTANT: Define Model Input Specs ---
# Modify these based on your actual model!
MODEL_INPUT_IMG_WIDTH = 224 # Example width
MODEL_INPUT_IMG_HEIGHT = 224 # Example height
MODEL_EXPECTS_FLOAT_IMG = True # True if model expects float (e.g., 0-1), False if uint8 (0-255)

# --- IMPORTANT: Define order of numerical features expected by the scaler/model ---
# Modify this list to match the exact order used during training!
# Use None for a feature if it wasn't used in training or unavailable.
NUMERICAL_FEATURES_ORDER = ['magnetism_mT', 'ldc_rp', 'delta_rp']

# Accuracy/Stability/Speed
NUM_SAMPLES_PER_UPDATE = 3
NUM_SAMPLES_CALIBRATION = 5 # Use more samples for calibration/capture
GUI_UPDATE_INTERVAL_MS = 100 # Faster GUI refresh rate
CAMERA_UPDATE_INTERVAL_MS = 30 # Faster camera refresh target
LDC_DISPLAY_BUFFER_SIZE = 5
MAGNETISM_FILTER_ALPHA = 0.2 # EMA filter for smoother display

# Image Display/Capture (Display uses different size than model input)
DISPLAY_IMG_WIDTH = 640
DISPLAY_IMG_HEIGHT = 480
# SAVE_IMG_WIDTH/HEIGHT not needed for saving, but keep for potential model input resizing reference
SAVE_IMG_WIDTH = 224 # Reference for model input if needed
SAVE_IMG_HEIGHT = 224 # Reference for model input if needed

# Camera
CAMERA_INDEX = 0

# Hall Sensor (ADS1115)
HALL_ADC_CHANNEL = ADS.P0 if I2C_ENABLED else None
SENSITIVITY_V_PER_TESLA = 0.0004
SENSITIVITY_V_PER_MILLITESLA = SENSITIVITY_V_PER_TESLA * 1000
IDLE_VOLTAGE = 1.7348 # Will be recalibrated

# Inductive Sensor (LDC1101)
SPI_BUS = 0; SPI_DEVICE = 0; SPI_SPEED = 500000; SPI_MODE = 0b00
CS_PIN = 8; LDC_CHIP_ID = 0xD4
START_CONFIG_REG, RP_SET_REG, TC1_REG, TC2_REG, DIG_CONFIG_REG = 0x0B, 0x01, 0x02, 0x03, 0x04
ALT_CONFIG_REG, D_CONF_REG, INTB_MODE_REG = 0x05, 0x0C, 0x0A
RP_DATA_MSB_REG, RP_DATA_LSB_REG = 0x22, 0x21; CHIP_ID_REG = 0x3F
ACTIVE_CONVERSION_MODE, SLEEP_MODE = 0x00, 0x01

# Calibration
IDLE_RP_VALUE = 0 # Will be recalibrated

# --- Removed Dataset / File Saving Constants ---

# Global Hardware Objects
camera = None; i2c = None; ads = None; hall_sensor = None
spi = None; ldc_initialized = False

# Global AI Objects
interpreter = None
input_details = None
output_details = None
labels = None
scaler = None
ai_ready = False

# Global State
RP_DISPLAY_BUFFER = deque(maxlen=LDC_DISPLAY_BUFFER_SIZE)
previous_filtered_mag_mT = None
last_captured_image_for_display = None # To show on results page
last_classification_results = None # To show on results page

# GUI Globals
window = None
main_frame = None # Frame for live view + classify button
results_frame = None # Frame for showing classification results
camera_label = None # On main_frame
controls_frame = None # On main_frame
magnetism_label = None # On main_frame
ldc_label = None # On main_frame
classify_button = None # On main_frame
calibrate_button = None # On main_frame

# Results Page Widgets
results_image_label = None
results_text_label = None
back_button = None

# Fonts (Keep as is)
label_font = None; readout_font = None; button_font = None
title_font = None; check_font = None; results_font = None # Added for results


# =========================
# === Hardware Setup ====
# =========================
# (Function remains largely the same, relies on updated constants/functions)
def initialize_hardware():
    global camera, i2c, ads, hall_sensor, spi, ldc_initialized
    print("--- Initializing Hardware ---")
    # Camera setup (no changes needed)
    try:
        camera = cv2.VideoCapture(CAMERA_INDEX)
        if camera and not camera.isOpened(): raise ValueError("Could not open camera")
        else: print(f"Camera {CAMERA_INDEX} opened.")
    except Exception as e: print(f"Error opening camera {CAMERA_INDEX}: {e}"); camera = None

    # I2C/ADS1115 setup (no changes needed)
    if I2C_ENABLED:
        try:
            i2c = busio.I2C(board.SCL, board.SDA); ads = ADS.ADS1115(i2c)
            hall_sensor = AnalogIn(ads, HALL_ADC_CHANNEL); print("ADS1115 Initialized.")
        except Exception as e: print(f"Error initializing I2C/ADS1115: {e}"); hall_sensor = None
    else: print("Skipping I2C/ADS1115 setup.")

    # SPI/LDC1101 setup (no changes needed)
    if SPI_ENABLED:
        try:
            GPIO.setwarnings(False); GPIO.setmode(GPIO.BCM); GPIO.setup(CS_PIN, GPIO.OUT); GPIO.output(CS_PIN, GPIO.HIGH); print("GPIO Initialized.")
            spi = spidev.SpiDev(); spi.open(SPI_BUS, SPI_DEVICE); spi.max_speed_hz = SPI_SPEED; spi.mode = SPI_MODE; print(f"SPI Initialized (Bus={SPI_BUS}, Dev={SPI_DEVICE}, Speed={SPI_SPEED}Hz).")
            if initialize_ldc1101(): enable_ldc_rpmode()
            else: print("LDC1101 Initialization Failed.")
        except Exception as e: print(f"Error initializing GPIO/SPI: {e}"); spi = None; ldc_initialized = False
    else: print("Skipping SPI/GPIO/LDC1101 setup.")
    print("--- Hardware Initialization Complete ---")

# =========================
# === LDC1101 Functions ===
# =========================
# (Keep ldc_write_register, ldc_read_register, initialize_ldc1101,
# enable_ldc_powermode, enable_ldc_rpmode, get_ldc_rpdata as they were
# in the provided CODE 1 - they were already optimized)

def ldc_write_register(reg_addr, value):
    if not spi: return
    try:
        GPIO.output(CS_PIN, GPIO.LOW)
        spi.xfer2([reg_addr & 0x7F, value])
        GPIO.output(CS_PIN, GPIO.HIGH)
    except Exception as e:
        print(f"Error writing LDC 0x{reg_addr:02X}: {e}")
        try: GPIO.output(CS_PIN, GPIO.HIGH)
        except: pass

def ldc_read_register(reg_addr):
    if not spi: return 0
    result = [0, 0]
    try:
        GPIO.output(CS_PIN, GPIO.LOW)
        result = spi.xfer2([reg_addr | 0x80, 0x00])
        GPIO.output(CS_PIN, GPIO.HIGH)
        return result[1]
    except Exception as e:
        print(f"Error reading LDC 0x{reg_addr:02X}: {e}")
        try: GPIO.output(CS_PIN, GPIO.HIGH)
        except: pass
        return 0

def initialize_ldc1101():
    global ldc_initialized; ldc_initialized = False;
    if not spi: return False
    try:
        chip_id = ldc_read_register(CHIP_ID_REG)
        if chip_id != LDC_CHIP_ID:
            print(f"LDC Mismatch: Read 0x{chip_id:02X}, Expected 0x{LDC_CHIP_ID:02X}")
            return False
        print("Configuring LDC1101...")
        ldc_write_register(RP_SET_REG, 0x07)
        ldc_write_register(TC1_REG, 0x90)
        ldc_write_register(TC2_REG, 0xA0)
        ldc_write_register(DIG_CONFIG_REG, 0x03)
        ldc_write_register(ALT_CONFIG_REG, 0x00)
        ldc_write_register(D_CONF_REG, 0x00)
        ldc_write_register(INTB_MODE_REG, 0x00)
        ldc_write_register(START_CONFIG_REG, SLEEP_MODE)
        time.sleep(0.01)
        print("LDC1101 Configured and in Sleep Mode.")
        ldc_initialized = True
        return True
    except Exception as e:
        print(f"Exception during LDC1101 initialization: {e}")
        ldc_initialized = False
        return False

def enable_ldc_powermode(mode):
    if not spi or not ldc_initialized: return;
    ldc_write_register(START_CONFIG_REG, mode)
    time.sleep(0.01)

def enable_ldc_rpmode():
    if not spi or not ldc_initialized: return;
    ldc_write_register(ALT_CONFIG_REG, 0x00)
    ldc_write_register(D_CONF_REG, 0x00)
    enable_ldc_powermode(ACTIVE_CONVERSION_MODE)

def get_ldc_rpdata():
    if not spi or not ldc_initialized: return None
    try:
        msb = ldc_read_register(RP_DATA_MSB_REG)
        lsb = ldc_read_register(RP_DATA_LSB_REG)
        return (msb << 8) | lsb
    except Exception as e:
        print(f"Error in get_ldc_rpdata: {e}")
        return None

# ============================
# === Sensor Reading (Avg) ===
# ============================
# (Keep get_averaged_hall_voltage and get_averaged_rp_data as they were)
def get_averaged_hall_voltage(num_samples=NUM_SAMPLES_PER_UPDATE):
    if not hall_sensor: return None
    readings = []
    for _ in range(num_samples):
        try:
            if hall_sensor: readings.append(hall_sensor.voltage)
        except Exception as e:
            print(f"Warning: Error reading Hall sensor voltage: {e}")
            return None # Be conservative
    return sum(readings) / len(readings) if readings else None

def get_averaged_rp_data(num_samples=NUM_SAMPLES_PER_UPDATE):
    if not ldc_initialized: return None
    readings = [get_ldc_rpdata() for _ in range(num_samples)]
    valid_readings = [r for r in readings if r is not None]
    return sum(valid_readings) / len(valid_readings) if valid_readings else None


# =========================
# === AI Setup & Helpers ==
# =========================

def load_labels(path):
    """Loads labels from file (strip numbers)."""
    try:
        with open(path, 'r') as f:
            # Assumes format "0 Label0", "1 Label1", etc.
            return [line.strip().split(' ', 1)[1] for line in f.readlines()]
    except FileNotFoundError:
        print(f"ERROR: Labels file not found at {path}")
        return None
    except Exception as e:
        print(f"ERROR: Could not read labels file {path}: {e}")
        return None

def initialize_ai_model():
    """Loads the TFLite model, scaler, and labels."""
    global interpreter, input_details, output_details, labels, scaler, ai_ready
    ai_ready = False # Reset status

    if not TFLITE_ENABLED or not JOBLIB_ENABLED:
        print("AI Disabled due to missing libraries (tflite_runtime or joblib).")
        messagebox.showerror("AI Error", "AI components (tflite_runtime, joblib) not installed.")
        return False

    print("--- Initializing AI Components ---")
    # Load Labels
    labels = load_labels(LABELS_PATH)
    if not labels:
        messagebox.showerror("AI Init Error", f"Failed to load labels from:\n{LABELS_PATH}")
        return False
    print(f"Labels loaded: {labels}")

    # Load Scaler
    try:
        scaler = joblib.load(SCALER_PATH)
        print(f"Scaler loaded from {SCALER_PATH}")
        if not hasattr(scaler, 'transform'):
             raise ValueError("Loaded object is not a valid scaler (missing 'transform' method).")
    except FileNotFoundError:
        print(f"ERROR: Scaler file not found at {SCALER_PATH}")
        messagebox.showerror("AI Init Error", f"Scaler file not found:\n{SCALER_PATH}")
        return False
    except Exception as e:
        print(f"ERROR: Failed to load scaler from {SCALER_PATH}: {e}")
        messagebox.showerror("AI Init Error", f"Error loading scaler:\n{e}")
        return False

    # Load TFLite Model
    try:
        interpreter = tflite.Interpreter(model_path=MODEL_PATH)
        interpreter.allocate_tensors()
        input_details = interpreter.get_input_details()
        output_details = interpreter.get_output_details()
        print(f"Model loaded from {MODEL_PATH}")
        # Basic validation (check number of inputs/outputs if needed)
        print(f"Model Inputs: {len(input_details)}, Model Outputs: {len(output_details)}")
        # Example: Check input tensor details (optional but good practice)
        # print("Input Details:", input_details)
        # print("Output Details:", output_details)

    except FileNotFoundError:
        print(f"ERROR: Model file not found at {MODEL_PATH}")
        messagebox.showerror("AI Init Error", f"Model file not found:\n{MODEL_PATH}")
        return False
    except Exception as e:
        print(f"ERROR: Failed to load TFLite model {MODEL_PATH}: {e}")
        messagebox.showerror("AI Init Error", f"Error loading TFLite model:\n{e}")
        return False

    print("--- AI Initialization Complete ---")
    ai_ready = True
    return True

def preprocess_image(img_rgb):
    """Prepares PIL Image for the TFLite model."""
    img_resized = img_rgb.resize((MODEL_INPUT_IMG_WIDTH, MODEL_INPUT_IMG_HEIGHT), Image.Resampling.LANCZOS)
    input_data = np.array(img_resized, dtype=np.float32 if MODEL_EXPECTS_FLOAT_IMG else np.uint8)

    if MODEL_EXPECTS_FLOAT_IMG:
        input_data = input_data / 255.0 # Normalize to [0, 1]

    # Add batch dimension
    input_data = np.expand_dims(input_data, axis=0)
    return input_data

def preprocess_numerical(sensor_data_dict):
    """Prepares numerical data using the loaded scaler."""
    # Create feature vector in the correct order, handling missing values
    feature_vector = []
    for feature_name in NUMERICAL_FEATURES_ORDER:
        value = sensor_data_dict.get(feature_name)
        # Handle missing values (e.g., sensor disabled or read error)
        # Use 0 or another sensible default *before* scaling.
        # Training data should ideally handle missing values consistently.
        if value is None:
            print(f"Warning: Numerical feature '{feature_name}' is None. Using 0.")
            value = 0.0
        feature_vector.append(value)

    if not feature_vector:
        print("Warning: No numerical features to process.")
        return None # Or return an array of zeros if the model expects it

    # Convert to NumPy array (shape (1, num_features))
    numerical_data = np.array([feature_vector], dtype=np.float32)

    # Apply the scaler
    try:
        scaled_data = scaler.transform(numerical_data)
        return scaled_data
    except Exception as e:
        print(f"Error applying scaler: {e}")
        print(f"Input shape to scaler: {numerical_data.shape}")
        if hasattr(scaler, 'n_features_in_'):
             print(f"Scaler expected features: {scaler.n_features_in_}")
        return None


# ======================
# === GUI Functions ===
# ======================

def switch_to_main_page():
    """Hides results page and shows the main page."""
    if results_frame: results_frame.grid_remove()
    if main_frame: main_frame.grid(row=0, column=0, sticky="nsew")
    # Re-enable classify button if it was disabled
    if classify_button: classify_button.config(state=tk.NORMAL)
    if calibrate_button: calibrate_button.config(state=tk.NORMAL)
    # Ensure sensor updates are visible
    update_magnetism()
    update_ldc_reading()
    update_camera_feed()


def switch_to_results_page():
    """Hides main page and shows the results page."""
    if main_frame: main_frame.grid_remove()
    if results_frame: results_frame.grid(row=0, column=0, sticky="nsew")
    display_classification_results() # Update results display


def display_classification_results():
    """Updates the widgets on the results page."""
    global last_captured_image_for_display, last_classification_results, results_image_label, results_text_label

    if not results_frame or not last_classification_results:
        print("Warning: Cannot display results - frame or data missing.")
        return

    # Display Image
    if last_captured_image_for_display and results_image_label:
        try:
            # Resize for display on results page (can be different from live feed size)
            img_for_results = last_captured_image_for_display.resize((320, 240), Image.Resampling.LANCZOS)
            img_tk = ImageTk.PhotoImage(img_for_results)
            results_image_label.img_tk = img_tk # Keep reference
            results_image_label.config(image=img_tk)
        except Exception as e:
            print(f"Error displaying result image: {e}")
            results_image_label.config(image='', text='Image Error')
    elif results_image_label:
         results_image_label.config(image='', text='No Image')


    # Display Text Results
    if results_text_label:
        try:
            res = last_classification_results
            pred_label = res['prediction']
            pred_conf = res['confidence'] * 100 # As percentage
            mag_val = res['magnetism_mT']
            rp_val = res['ldc_rp']
            delta_rp_val = res['delta_rp']
            all_probs = res['probabilities'] # Dict {label: prob}

            mag_str = f"{mag_val:.2f} mT" if mag_val is not None else "N/A"
            rp_str = f"{rp_val}" if rp_val is not None else "N/A"
            delta_rp_str = f"{delta_rp_val}" if delta_rp_val is not None else "N/A"

            results_text = f"Prediction: {pred_label} ({pred_conf:.1f}%)\n\n"
            results_text += f"Magnetism: {mag_str}\n"
            results_text += f"LDC RP: {rp_str}\n"
            results_text += f"Delta RP: {delta_rp_str}\n\n"
            results_text += "Probabilities:\n"
            # Sort probabilities descending for display
            sorted_probs = sorted(all_probs.items(), key=lambda item: item[1], reverse=True)
            for label, prob in sorted_probs:
                 results_text += f"  - {label}: {prob*100:.1f}%\n"

            results_text_label.config(text=results_text)

        except Exception as e:
            print(f"Error formatting results text: {e}")
            results_text_label.config(text="Error displaying results.")


def classify_material():
    """Captures image, reads sensors, runs AI, and displays results."""
    global classify_button, calibrate_button, window, ai_ready
    global last_captured_image_for_display, last_classification_results

    if not ai_ready:
        messagebox.showerror("AI Error", "AI Model is not initialized or ready.")
        return
    if not camera:
        messagebox.showerror("Camera Error", "Camera not available.")
        return

    # Disable buttons during processing
    classify_button.config(state=tk.DISABLED)
    calibrate_button.config(state=tk.DISABLED)
    window.update_idletasks() # Force GUI update

    # 1. Capture Image
    ret, frame = camera.read()
    if not ret:
        messagebox.showerror("Capture Error", "Failed to capture photo.")
        classify_button.config(state=tk.NORMAL)
        calibrate_button.config(state=tk.NORMAL)
        return

    # Process image (BGR to RGB, create PIL Image)
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    img_raw = Image.fromarray(frame_rgb)
    last_captured_image_for_display = img_raw.copy() # Save for results page display

    # 2. Get Sensor Data (use more samples for accuracy)
    print("Reading sensor data for classification...")
    avg_voltage = get_averaged_hall_voltage(num_samples=NUM_SAMPLES_CALIBRATION)
    current_mag_mT = None
    if avg_voltage is not None:
        try: current_mag_mT = (avg_voltage - IDLE_VOLTAGE) / SENSITIVITY_V_PER_MILLITESLA
        except ZeroDivisionError: current_mag_mT = None # Avoid crash if sensitivity is zero
        except Exception as e: print(f"Mag calculation error: {e}"); current_mag_mT = None

    current_rp_val = get_averaged_rp_data(num_samples=NUM_SAMPLES_CALIBRATION)
    delta_rp = None
    if current_rp_val is not None:
        current_rp_val = int(current_rp_val) # Use integer average
        if IDLE_RP_VALUE != 0: delta_rp = current_rp_val - IDLE_RP_VALUE
    print(f"Sensor Readings: Mag={current_mag_mT}, RP={current_rp_val}, DeltaRP={delta_rp}")

    # Store raw sensor readings for the results display
    raw_sensor_data = {
        'magnetism_mT': current_mag_mT,
        'ldc_rp': current_rp_val,
        'delta_rp': delta_rp
    }

    # 3. Preprocess Data for AI
    try:
        image_input_data = preprocess_image(img_raw)
        numerical_input_data = preprocess_numerical(raw_sensor_data)

        # Check if preprocessing returned valid data
        if image_input_data is None: raise ValueError("Image preprocessing failed.")
        # Handle case where numerical data might be optional or fail
        if numerical_input_data is None and len(input_details) > 1:
             # If model expects numerical data but preprocessing failed
             raise ValueError("Numerical data preprocessing failed, but model expects it.")

    except Exception as e:
        messagebox.showerror("Preprocessing Error", f"Failed to prepare data for AI:\n{e}")
        classify_button.config(state=tk.NORMAL)
        calibrate_button.config(state=tk.NORMAL)
        return

    # 4. Run Inference
    try:
        print("Running AI inference...")
        # --- Assuming multiple inputs: index 0 for image, index 1 for numerical ---
        # --- Adjust indices if your model expects a different order ---
        if len(input_details) == 1: # Model expects only image
             print("Model expects only Image input.")
             interpreter.set_tensor(input_details[0]['index'], image_input_data)
        elif len(input_details) == 2 : # Model expects image and numerical
             print("Model expects Image and Numerical inputs.")
             # Check tensor shapes and types if errors occur here
             print(f"Setting Image Tensor (Index {input_details[0]['index']}): Shape={image_input_data.shape}, DType={image_input_data.dtype}")
             interpreter.set_tensor(input_details[0]['index'], image_input_data)

             # Ensure numerical data is present if expected
             if numerical_input_data is None:
                 raise ValueError("Model expects numerical input, but it's missing/failed preprocessing.")
             print(f"Setting Numerical Tensor (Index {input_details[1]['index']}): Shape={numerical_input_data.shape}, DType={numerical_input_data.dtype}")
             interpreter.set_tensor(input_details[1]['index'], numerical_input_data)
        else:
             raise ValueError(f"Unsupported number of model inputs: {len(input_details)}. Expected 1 or 2.")

        start_time = time.perf_counter()
        interpreter.invoke()
        stop_time = time.perf_counter()
        print(f"Inference time: {(stop_time - start_time) * 1000:.2f} ms")

        # --- Assuming single output tensor with probabilities ---
        output_data = interpreter.get_tensor(output_details[0]['index'])
        probabilities = output_data[0] # Remove batch dimension

        # 5. Postprocess Results
        prediction_index = np.argmax(probabilities)
        prediction_label = labels[prediction_index]
        prediction_confidence = float(probabilities[prediction_index]) # Ensure it's python float

        # Store results
        all_probabilities_dict = {labels[i]: float(probabilities[i]) for i in range(len(labels))}
        last_classification_results = {
            "prediction": prediction_label,
            "confidence": prediction_confidence,
            "probabilities": all_probabilities_dict,
            **raw_sensor_data # Add the sensor readings
        }
        print(f"Prediction: {prediction_label} (Confidence: {prediction_confidence:.3f})")

        # 6. Switch to Results Page
        switch_to_results_page()

    except Exception as e:
        print(f"ERROR during AI Inference or Postprocessing: {e}")
        import traceback
        traceback.print_exc() # Print detailed traceback for debugging
        messagebox.showerror("AI Inference Error", f"An error occurred during classification:\n{e}")
        # Re-enable buttons after error
        classify_button.config(state=tk.NORMAL)
        calibrate_button.config(state=tk.NORMAL)


# Calibrate Sensors (Keep as is, but ensure buttons are re-enabled correctly)
def calibrate_sensors():
    global IDLE_VOLTAGE, IDLE_RP_VALUE, window, previous_filtered_mag_mT
    global classify_button, calibrate_button # Ensure access to buttons
    print("Starting Calibration...")
    # Temporarily disable buttons during calibration
    calibrate_button.config(state=tk.DISABLED)
    classify_button.config(state=tk.DISABLED) # Also disable classify button
    window.update_idletasks()

    hall_results = "Hall Sensor N/A"; hall_error = False
    if hall_sensor:
        avg_v = get_averaged_hall_voltage(num_samples=NUM_SAMPLES_CALIBRATION)
        if avg_v is not None:
            IDLE_VOLTAGE = avg_v
            hall_results = f"Hall Idle: {IDLE_VOLTAGE:.4f} V"
        else:
            hall_results = "Hall Cal Error: No readings"; hall_error = True; IDLE_VOLTAGE = 0
    ldc_results = "LDC Sensor N/A"; ldc_error = False
    if ldc_initialized:
        avg_rp = get_averaged_rp_data(num_samples=NUM_SAMPLES_CALIBRATION)
        if avg_rp is not None:
            IDLE_RP_VALUE = int(avg_rp)
            ldc_results = f"LDC Idle: {IDLE_RP_VALUE}"
        else:
            ldc_results = "LDC Cal Error: No readings"; ldc_error = True; IDLE_RP_VALUE = 0

    previous_filtered_mag_mT = None # Reset EMA filter
    print("Magnetism filter state reset due to calibration.")
    print(f"Calibration Results: {hall_results}, {ldc_results}")

    # Re-enable buttons
    calibrate_button.config(state=tk.NORMAL)
    classify_button.config(state=tk.NORMAL) # Re-enable classify too

    # Show results
    final_message = f"{hall_results}\n{ldc_results}"
    if hall_error or ldc_error: messagebox.showwarning("Calibration Warning", final_message)
    elif not hall_sensor and not ldc_initialized: messagebox.showerror("Calibration Error", "Neither sensor available.")
    else: messagebox.showinfo("Calibration Complete", final_message)


# Camera Feed Update (Keep as is)
def update_camera_feed():
    global camera_label, window
    img_tk = None
    if camera and camera.isOpened():
        ret, frame = camera.read()
        if ret:
            try:
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                img = Image.fromarray(frame_rgb).resize((DISPLAY_IMG_WIDTH, DISPLAY_IMG_HEIGHT), Image.Resampling.NEAREST)
                img_tk = ImageTk.PhotoImage(img)
            except Exception as e: print(f"Error processing camera frame: {e}")
        else: print("Warning: Failed to read frame from camera.")

    if img_tk:
        camera_label.img_tk = img_tk
        camera_label.configure(image=img_tk, text="")
    else:
        # Placeholder logic (keep as is)
        if not hasattr(camera_label, 'no_cam_img'):
            try:
                placeholder = Image.new('RGB', (DISPLAY_IMG_WIDTH, DISPLAY_IMG_HEIGHT), color='#BDBDBD')
                camera_label.no_cam_img = ImageTk.PhotoImage(placeholder)
            except Exception as e:
                print(f"Error creating placeholder image: {e}")
                camera_label.no_cam_img = None
        if camera_label.no_cam_img and camera_label.cget("image") != str(camera_label.no_cam_img):
            camera_label.configure(image=camera_label.no_cam_img, text="No Camera Feed")

    # Schedule next update ONLY if the main frame is visible
    if window and main_frame and main_frame.winfo_ismapped():
         window.after(CAMERA_UPDATE_INTERVAL_MS, update_camera_feed)


# Magnetism Update (Keep as is, but schedule conditionally)
def update_magnetism():
    global magnetism_label, window, previous_filtered_mag_mT
    avg_voltage = get_averaged_hall_voltage()
    display_text = "N/A"
    if hall_sensor:
        if avg_voltage is not None:
            try:
                raw_mag_mT = (avg_voltage - IDLE_VOLTAGE) / SENSITIVITY_V_PER_MILLITESLA
                # EMA Filter
                if previous_filtered_mag_mT is None: filtered_mag_mT = raw_mag_mT
                else: filtered_mag_mT = (MAGNETISM_FILTER_ALPHA * raw_mag_mT) + ((1 - MAGNETISM_FILTER_ALPHA) * previous_filtered_mag_mT)
                previous_filtered_mag_mT = filtered_mag_mT
                # Unit selection
                unit, value = ("mT", filtered_mag_mT) if abs(filtered_mag_mT) >= 1 else ("µT", filtered_mag_mT * 1000)
                display_text = f"{value:.2f} {unit}"
            except ZeroDivisionError: display_text = "Cal Error?"; previous_filtered_mag_mT = None
            except Exception as e: print(f"Error calc mag: {e}"); display_text = "Error"; previous_filtered_mag_mT = None
        else: display_text = "Read Err"; previous_filtered_mag_mT = None

    if magnetism_label: magnetism_label.config(text=display_text)
    # Schedule next update ONLY if the main frame is visible
    if window and main_frame and main_frame.winfo_ismapped():
         window.after(GUI_UPDATE_INTERVAL_MS, update_magnetism)

# LDC Update (Keep as is, but schedule conditionally)
def update_ldc_reading():
    global ldc_label, window, RP_DISPLAY_BUFFER
    avg_rp_val = get_averaged_rp_data()
    display_rp_text = "N/A"
    if ldc_initialized:
        if avg_rp_val is not None:
            RP_DISPLAY_BUFFER.append(avg_rp_val)
            if RP_DISPLAY_BUFFER:
                buffer_avg = sum(RP_DISPLAY_BUFFER) / len(RP_DISPLAY_BUFFER)
                display_rp_text = f"{int(buffer_avg)}"
            else: display_rp_text = "..."
        else: RP_DISPLAY_BUFFER.clear(); display_rp_text = "Read Err"

    if ldc_label: ldc_label.config(text=display_rp_text)
    # Schedule next update ONLY if the main frame is visible
    if window and main_frame and main_frame.winfo_ismapped():
         window.after(GUI_UPDATE_INTERVAL_MS, update_ldc_reading)


# ======================
# === GUI Setup ========
# ======================
def setup_gui():
    global window, main_frame, results_frame
    global camera_label, controls_frame, magnetism_label, ldc_label
    global classify_button, calibrate_button
    global results_image_label, results_text_label, back_button
    global label_font, readout_font, button_font, title_font, check_font, results_font

    window = tk.Tk(); window.title("AI Metal Classifier v2.0.0"); window.geometry("1000x650") # Adjusted size maybe
    style = ttk.Style(); style.theme_use('clam' if 'clam' in style.theme_names() else 'default')

    # Fonts
    title_font=tkFont.Font(family="Helvetica",size=14,weight="bold"); label_font=tkFont.Font(family="Helvetica",size=11); readout_font=tkFont.Font(family="Consolas",size=14,weight="bold"); button_font=tkFont.Font(family="Helvetica",size=11,weight="bold"); check_font=tkFont.Font(family="Helvetica",size=10); results_font=tkFont.Font(family="Consolas", size=11) # Font for results text

    # Styles
    style.configure("TLabel",font=label_font,padding=2); style.configure("TButton",font=button_font,padding=(10,6)); style.configure("TMenubutton",font=label_font,padding=4); style.configure("TLabelframe",padding=8); style.configure("TLabelframe.Label",font=tkFont.Font(family="Helvetica",size=12,weight="bold")); style.configure("Readout.TLabel",font=readout_font,padding=(5,1)); style.configure("TCheckbutton",font=check_font,padding=2); style.configure("Results.TLabel", font=results_font, padding=5, anchor=tk.NW, justify=tk.LEFT); style.configure("Big.TButton", font=tkFont.Font(family="Helvetica",size=14,weight="bold"), padding=(15,10)) # Bigger classify button

    # Main container to hold the switchable frames
    container = ttk.Frame(window, padding="5 5 5 5"); container.pack(side=tk.TOP, fill=tk.BOTH, expand=True); container.grid_rowconfigure(0, weight=1); container.grid_columnconfigure(0, weight=1)

    # --- Main Page Frame ---
    main_frame = ttk.Frame(container, padding="10 10 10 10"); main_frame.grid(row=0, column=0, sticky="nsew"); # Starts visible
    main_frame.columnconfigure(0, weight=3); main_frame.columnconfigure(1, weight=1); main_frame.rowconfigure(0, weight=1)

    camera_label = ttk.Label(main_frame, text="Initializing Camera...", anchor="center", borderwidth=1, relief="sunken"); camera_label.grid(row=0, column=0, padx=(0, 10), pady=0, sticky="nsew")

    controls_frame = ttk.Frame(main_frame); controls_frame.grid(row=0, column=1, padx=(0, 0), pady=0, sticky="nsew"); controls_frame.columnconfigure(0, weight=1); controls_row_idx = 0

    readings_frame = ttk.Labelframe(controls_frame, text=" Live Readings ", padding="10 5 10 5"); readings_frame.grid(row=controls_row_idx, column=0, sticky="new", pady=(0, 15)); controls_row_idx += 1; readings_frame.columnconfigure(0, weight=0); readings_frame.columnconfigure(1, weight=1)
    ttk.Label(readings_frame, text="Magnetism:").grid(row=0, column=0, sticky="w", padx=(0, 5)); magnetism_label = ttk.Label(readings_frame, text="Init...", style="Readout.TLabel", anchor="e"); magnetism_label.grid(row=0, column=1, sticky="ew")
    ttk.Label(readings_frame, text="LDC RP:").grid(row=1, column=0, sticky="w", padx=(0, 5), pady=(5,0)); ldc_label = ttk.Label(readings_frame, text="Init...", style="Readout.TLabel", anchor="e"); ldc_label.grid(row=1, column=1, sticky="ew", pady=(5,0))

    actions_frame = ttk.Labelframe(controls_frame, text=" Actions ", padding="10 10 10 10"); actions_frame.grid(row=controls_row_idx, column=0, sticky="new", pady=(0, 5)); controls_row_idx += 1; actions_frame.columnconfigure(0, weight=1);

    classify_button = ttk.Button(actions_frame, text="Classify Material", command=classify_material, style="Big.TButton"); classify_button.grid(row=0, column=0, sticky="ew", pady=(5, 10))
    calibrate_button = ttk.Button(actions_frame, text="Calibrate Sensors", command=calibrate_sensors); calibrate_button.grid(row=1, column=0, sticky="ew", pady=(5, 5))


    # --- Results Page Frame ---
    results_frame = ttk.Frame(container, padding="15 15 15 15"); results_frame.grid(row=0, column=0, sticky="nsew"); # Starts hidden
    results_frame.columnconfigure(0, weight=1); results_frame.columnconfigure(1, weight=1); results_frame.rowconfigure(0, weight=1); results_frame.rowconfigure(1, weight=0); # Image+Text row expands

    results_image_label = ttk.Label(results_frame, text="Result Image", anchor="center", borderwidth=1, relief="sunken"); results_image_label.grid(row=0, column=0, padx=(0, 10), pady=(0,10), sticky="nsew")
    results_text_label = ttk.Label(results_frame, text="Classification Results...", style="Results.TLabel", anchor="nw"); results_text_label.grid(row=0, column=1, padx=(5, 0), pady=(0,10), sticky="nsew")

    back_button = ttk.Button(results_frame, text="<< Back to Live View", command=switch_to_main_page); back_button.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(10, 0))

    results_frame.grid_remove() # Hide results frame initially


# ==========================
# === Main Execution =======
# ==========================
def run_application():
    global window
    setup_gui() # Setup GUI elements first

    # Set initial text before starting loops
    if not camera: camera_label.configure(text="Camera Failed", image='')
    if not hall_sensor: magnetism_label.config(text="N/A")
    if not ldc_initialized: ldc_label.config(text="N/A")
    if not ai_ready: # Disable classify button if AI init failed
        classify_button.config(state=tk.DISABLED)
        messagebox.showwarning("AI Warning", "AI failed to initialize. Classification disabled.")

    # Start the update loops (they will self-regulate based on visibility)
    print("Starting update loops...")
    update_camera_feed(); update_magnetism(); update_ldc_reading()

    print("Starting Tkinter main loop...")
    window.mainloop() # Blocks here until window is closed


# --- Cleanup ---
# (Keep cleanup_resources as is)
def cleanup_resources():
    print("Cleaning up resources...")
    if camera and camera.isOpened():
        camera.release()
        print("Camera released.")
    cv2.destroyAllWindows()

    if spi:
        try:
            if ldc_initialized:
                print("Putting LDC to sleep...")
                ldc_write_register(START_CONFIG_REG, SLEEP_MODE)
                time.sleep(0.05)
        except Exception as e: print(f"Note: Error putting LDC to sleep: {e}")
        finally:
            spi.close()
            print("SPI closed.")

    if SPI_ENABLED:
        try:
            GPIO.cleanup()
            print("GPIO cleaned up.")
        except Exception as e: print(f"Note: GPIO cleanup error: {e}")

    print("Cleanup complete.")

# --- Run ---
if __name__ == '__main__':
    # Initialize hardware first
    initialize_hardware()

    # Initialize AI components
    # Do this *after* hardware, but *before* starting the GUI loops
    # as the GUI setup might check ai_ready status
    initialize_ai_model()

    try:
        run_application()
    except Exception as e:
        print(f"FATAL ERROR in main execution: {e}")
        try: messagebox.showerror("Fatal Error", f"An unrecoverable error occurred:\n{e}")
        except Exception: pass
        import traceback; traceback.print_exc()
    finally:
        cleanup_resources()
