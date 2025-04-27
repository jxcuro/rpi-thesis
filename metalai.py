# CODE 3 - AI Metal Classifier GUI with Results Page
# Description: Displays live sensor data and camera feed.
#              Captures image and sensor readings, classifies metal using a TFLite model,
#              and displays the results on a dedicated page.
# Version: 3.0.0 - Added results page, dynamic input index detection, NumPy<2 compatibility assumed.

import tkinter as tk
from tkinter import ttk
from tkinter import font as tkFont
from tkinter import messagebox
import cv2
from PIL import Image, ImageTk
import time
import os
import statistics
from collections import deque
import numpy as np
import math
import warnings # To potentially suppress warnings later if needed

# --- AI Imports ---
try:
    from tflite_runtime.interpreter import Interpreter
except ImportError:
    try:
        from tensorflow.lite.python.interpreter import Interpreter
    except ImportError:
        print("ERROR: TensorFlow Lite Runtime is not installed.")
        print("Please install it: pip install tflite-runtime")
        exit()

try:
    import joblib
except ImportError:
    print("ERROR: Joblib is not installed.")
    print("Please install it: pip install joblib")
    exit()

# --- I2C/ADS1115 Imports ---
try:
    import board
    import busio
    import adafruit_ads1x15.ads1115 as ADS
    from adafruit_ads1x15.analog_in import AnalogIn
    I2C_ENABLED = True
except ImportError:
    print("Warning: I2C/ADS1115 libraries not found. Magnetism readings disabled.")
    I2C_ENABLED = False

# --- SPI/LDC1101 Imports ---
try:
    import spidev
    import RPi.GPIO as GPIO
    SPI_ENABLED = True
except ImportError:
    print("Warning: SPI/GPIO libraries not found. LDC readings disabled.")
    SPI_ENABLED = False

# ==================================
# === Constants and Configuration ===
# ==================================

# Accuracy/Stability/Speed
NUM_SAMPLES_PER_UPDATE = 3
NUM_SAMPLES_CALIBRATION = 10 # Increased for more stable calibration
GUI_UPDATE_INTERVAL_MS = 100
CAMERA_UPDATE_INTERVAL_MS = 40 # Adjusted slightly
LDC_DISPLAY_BUFFER_SIZE = 5
MAGNETISM_FILTER_ALPHA = 0.2

# Camera
CAMERA_INDEX = 0
DISPLAY_IMG_WIDTH = 640
DISPLAY_IMG_HEIGHT = 480
RESULT_IMG_DISPLAY_WIDTH = 280 # Size for image on results page

# --- AI Model Configuration ---
try: BASE_PATH = os.path.dirname(os.path.abspath(__file__))
except NameError: BASE_PATH = os.getcwd()
MODEL_FILENAME = "material_classifier_model.tflite"
LABELS_FILENAME = "material_labels.txt"
SCALER_FILENAME = "numerical_scaler.joblib"
MODEL_PATH = os.path.join(BASE_PATH, MODEL_FILENAME)
LABELS_PATH = os.path.join(BASE_PATH, LABELS_FILENAME)
SCALER_PATH = os.path.join(BASE_PATH, SCALER_FILENAME)

AI_IMG_WIDTH = 224 # Expected model input width
AI_IMG_HEIGHT = 224 # Expected model input height

# Hall Sensor (ADS1115)
HALL_ADC_CHANNEL = ADS.P0 if I2C_ENABLED else None
SENSITIVITY_V_PER_TESLA = 0.0004
SENSITIVITY_V_PER_MILLITESLA = SENSITIVITY_V_PER_TESLA * 1000
IDLE_VOLTAGE = 1.7348 # Default, recalibrated

# Inductive Sensor (LDC1101)
SPI_BUS = 0; SPI_DEVICE = 0; SPI_SPEED = 500000; SPI_MODE = 0b00
CS_PIN = 8; LDC_CHIP_ID = 0xD4
START_CONFIG_REG, RP_SET_REG, TC1_REG, TC2_REG, DIG_CONFIG_REG = 0x0B, 0x01, 0x02, 0x03, 0x04
ALT_CONFIG_REG, D_CONF_REG, INTB_MODE_REG = 0x05, 0x0C, 0x0A
RP_DATA_MSB_REG, RP_DATA_LSB_REG = 0x22, 0x21; CHIP_ID_REG = 0x3F
ACTIVE_CONVERSION_MODE, SLEEP_MODE = 0x00, 0x01

# Calibration
IDLE_RP_VALUE = 0 # Default, recalibrated

# Global Hardware Objects
camera = None; i2c = None; ads = None; hall_sensor = None
spi = None; ldc_initialized = False

# Global AI Objects
interpreter = None
input_details = None
output_details = None
loaded_labels = []
numerical_scaler = None

# Global State
RP_DISPLAY_BUFFER = deque(maxlen=LDC_DISPLAY_BUFFER_SIZE)
previous_filtered_mag_mT = None

# --- GUI Globals ---
window = None
main_frame = None # Top-level frame inside window
live_view_frame = None
results_view_frame = None

# Fonts
label_font = None; readout_font = None; button_font = None
title_font = None; result_title_font = None; result_font = None; result_value_font = None

# Live View Widgets
lv_camera_label = None
lv_magnetism_label = None
lv_ldc_label = None
lv_classify_button = None
lv_calibrate_button = None

# Results View Widgets
rv_image_label = None
rv_prediction_label = None
rv_confidence_label = None
rv_magnetism_label = None
rv_ldc_label = None
rv_classify_another_button = None

# Placeholder image for results view initialization
placeholder_img_tk = None


# =========================
# === Hardware Setup ===
# =========================
# (Identical to CODE 2)
def initialize_hardware():
    global camera, i2c, ads, hall_sensor, spi, ldc_initialized
    print("--- Initializing Hardware ---")
    # Initialize Camera
    try:
        camera = cv2.VideoCapture(CAMERA_INDEX)
        if camera and not camera.isOpened(): raise ValueError("Could not open camera")
        else: print(f"Camera {CAMERA_INDEX} opened.")
    except Exception as e: print(f"Error opening camera {CAMERA_INDEX}: {e}"); camera = None

    # Initialize I2C / ADS1115 (Magnetism)
    if I2C_ENABLED:
        try:
            i2c = busio.I2C(board.SCL, board.SDA); ads = ADS.ADS1115(i2c)
            hall_sensor = AnalogIn(ads, HALL_ADC_CHANNEL); print("ADS1115 Initialized.")
        except Exception as e: print(f"Error initializing I2C/ADS1115: {e}"); hall_sensor = None
    else: print("Skipping I2C/ADS1115 setup.")

    # Initialize SPI / LDC1101 (Inductance)
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
# === AI Model Setup ======
# =========================
# (Identical to CODE 2, ensures NumPy < 2.0 is compatible with loaded tflite-runtime)
def initialize_ai():
    global interpreter, input_details, output_details, loaded_labels, numerical_scaler
    print("--- Initializing AI Components ---")
    # Load Labels
    try:
        with open(LABELS_PATH, 'r') as f:
            loaded_labels = [line.strip() for line in f.readlines()]
        if not loaded_labels: raise ValueError("Labels file is empty.")
        print(f"Loaded {len(loaded_labels)} labels: {loaded_labels}")
    except FileNotFoundError:
        print(f"ERROR: Labels file not found at {LABELS_PATH}")
        return False
    except Exception as e:
        print(f"ERROR: Could not read labels file {LABELS_PATH}: {e}")
        return False

    # Load Numerical Scaler
    try:
        numerical_scaler = joblib.load(SCALER_PATH)
        print(f"Loaded numerical scaler from {SCALER_PATH}")
        if not hasattr(numerical_scaler, 'transform'):
             raise TypeError("Loaded scaler object does not have a 'transform' method.")
    except FileNotFoundError:
        print(f"ERROR: Scaler file not found at {SCALER_PATH}")
        return False
    except Exception as e:
        print(f"ERROR: Could not load scaler from {SCALER_PATH}: {e}")
        return False

    # Load TFLite Model and Allocate Tensors
    try:
        interpreter = Interpreter(model_path=MODEL_PATH)
        interpreter.allocate_tensors()
        input_details = interpreter.get_input_details()
        output_details = interpreter.get_output_details()
        print(f"Loaded TFLite model from {MODEL_PATH}")
        # print("Model Input Details:", input_details) # Uncomment for debugging shapes/types
        # print("Model Output Details:", output_details)

        output_shape = output_details[0]['shape']
        if output_shape[-1] != len(loaded_labels):
             print(f"WARNING: Model output size ({output_shape[-1]}) does not match number of labels ({len(loaded_labels)}).")

    except FileNotFoundError:
        print(f"ERROR: Model file not found at {MODEL_PATH}")
        return False
    except Exception as e:
        print(f"ERROR: Failed to load TFLite model or allocate tensors: {e}")
        import traceback
        traceback.print_exc() # Print detailed TFLite error
        return False

    print("--- AI Initialization Complete ---")
    return True

# =========================
# === LDC1101 Functions ===
# =========================
# (Identical to CODE 2)
def ldc_write_register(reg_addr, value):
    if not spi: return
    try:
        GPIO.output(CS_PIN, GPIO.LOW)
        spi.xfer2([reg_addr & 0x7F, value]) # Write command MSB=0
        GPIO.output(CS_PIN, GPIO.HIGH)
    except Exception as e:
        # print(f"Warning: Error writing LDC 0x{reg_addr:02X}: {e}") # Less verbose
        try: GPIO.output(CS_PIN, GPIO.HIGH)
        except: pass

def ldc_read_register(reg_addr):
    if not spi: return 0
    result = [0, 0]
    try:
        GPIO.output(CS_PIN, GPIO.LOW)
        result = spi.xfer2([reg_addr | 0x80, 0x00]) # Read command MSB=1
        GPIO.output(CS_PIN, GPIO.HIGH)
        return result[1]
    except Exception as e:
        # print(f"Warning: Error reading LDC 0x{reg_addr:02X}: {e}") # Less verbose
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
        time.sleep(0.01) # Minimal settling time
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
    time.sleep(0.01) # Reduced delay

def enable_ldc_rpmode():
    if not spi or not ldc_initialized:
        print("Cannot enable RP mode: SPI or LDC not initialized.")
        return;
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
        # print(f"Warning: Error in get_ldc_rpdata: {e}") # Less verbose
        return None

# ============================
# === Sensor Reading (Avg) ===
# ============================
# (Identical to CODE 2)
def get_averaged_hall_voltage(num_samples=NUM_SAMPLES_PER_UPDATE):
    if not hall_sensor: return None
    readings = []
    for _ in range(num_samples):
        try:
            if hall_sensor: readings.append(hall_sensor.voltage)
        except Exception as e:
            # print(f"Warning: Error reading Hall sensor voltage: {e}") # Less verbose
            return None
    return sum(readings) / len(readings) if readings else None

def get_averaged_rp_data(num_samples=NUM_SAMPLES_PER_UPDATE):
    if not ldc_initialized: return None
    readings = [get_ldc_rpdata() for _ in range(num_samples)]
    valid_readings = [r for r in readings if r is not None]
    return sum(valid_readings) / len(valid_readings) if valid_readings else None

# ==========================
# === AI Processing ========
# ==========================

# --- CORRECTED preprocess_input Function ---
def preprocess_input(image_pil, mag_mT, ldc_rp_delta):
    """Prepares image and sensor data for the TFLite model."""
    global numerical_scaler, input_details # Need input_details here

    # Make sure model details and scaler are loaded
    if numerical_scaler is None or input_details is None:
        print("ERROR: Numerical scaler or model input details not loaded.")
        return None

    # --- Image Preprocessing ---
    try:
        img_resized = image_pil.resize((AI_IMG_WIDTH, AI_IMG_HEIGHT), Image.Resampling.LANCZOS)
        img_rgb = img_resized.convert('RGB')
        image_np = np.array(img_rgb, dtype=np.float32)
        image_np /= 255.0 # Normalize to [0, 1] - Adjust if your model expects different
        image_input = np.expand_dims(image_np, axis=0) # Shape: (1, H, W, 3) -> 4 dims
    except Exception as e:
        print(f"ERROR: Failed during image preprocessing: {e}")
        return None

    # --- Numerical Feature Preprocessing ---
    mag_mT_val = mag_mT if mag_mT is not None else 0.0
    ldc_rp_delta_val = ldc_rp_delta if ldc_rp_delta is not None else 0.0
    numerical_features = np.array([[mag_mT_val, ldc_rp_delta_val]], dtype=np.float32)

    try:
        # Suppress the specific UserWarning about feature names if desired
        # with warnings.catch_warnings():
        #     warnings.filterwarnings("ignore", message="X does not have valid feature names.*StandardScaler was fitted with feature names", category=UserWarning)
        scaled_numerical_features = numerical_scaler.transform(numerical_features) # Shape: (1, 2) -> 2 dims
    except Exception as e:
        print(f"ERROR: Failed to scale numerical features: {e}")
        scaled_numerical_features = np.zeros_like(numerical_features) # Default to zeros if scaling fails


    # --- Determine Correct Input Indices Dynamically ---
    image_input_index = -1
    numerical_input_index = -1
    num_features_expected = 2 # Should match the scaler/training

    # print("Debug: Model Input Details:", input_details) # Uncomment for detailed debugging

    # Iterate through the model's input details provided by TFLite interpreter
    for detail in input_details:
        shape = detail['shape']
        index = detail['index']
        # print(f"Debug: Checking input index {index}, shape {shape}") # Uncomment for detailed debugging

        # Identify image input (typically 4 dimensions: Batch, Height, Width, Channels)
        if len(shape) == 4 and shape[1] == AI_IMG_HEIGHT and shape[2] == AI_IMG_WIDTH and shape[3] == 3:
            # print(f"  -> Identified as Image Input") # Uncomment for detailed debugging
            if image_input_index != -1: print("Warning: Found multiple potential image inputs!")
            image_input_index = index
        # Identify numerical input (typically 2 dimensions: Batch, NumFeatures)
        elif len(shape) == 2 and shape[1] == num_features_expected:
             # print(f"  -> Identified as Numerical Input") # Uncomment for detailed debugging
             if numerical_input_index != -1: print("Warning: Found multiple potential numerical inputs!")
             numerical_input_index = index
        # else: print(f"  -> Not identified as primary image or numerical input based on shape.") # Uncomment


    # Check if we successfully found both indices via primary method
    if image_input_index == -1 or numerical_input_index == -1:
         print("Warning: Could not reliably determine input tensor indices from shape details.")
         # Try identifying based only on number of dimensions as a fallback (less safe)
         if len(input_details) == 2:
              print("Attempting fallback identification based only on number of dimensions...")
              try:
                  input_0_dims = len(input_details[0]['shape'])
                  input_1_dims = len(input_details[1]['shape'])
                  input_0_index = input_details[0]['index']
                  input_1_index = input_details[1]['index']

                  if input_0_dims == 4 and input_1_dims == 2:
                       image_input_index = input_0_index
                       numerical_input_index = input_1_index
                       print(f"Fallback: Assumed index {input_0_index} is image (4D), index {input_1_index} is numerical (2D).")
                  elif input_0_dims == 2 and input_1_dims == 4:
                       numerical_input_index = input_0_index
                       image_input_index = input_1_index
                       print(f"Fallback: Assumed index {input_0_index} is numerical (2D), index {input_1_index} is image (4D).")
                  else:
                       print("Fallback failed: Input dimensions don't match expected 4D/2D pattern.")
                       return None
              except Exception as e:
                    print(f"Fallback failed with exception: {e}")
                    return None
         else:
             print("Cannot use fallback, model does not have exactly 2 inputs.")
             return None

         # Check again if fallback succeeded
         if image_input_index == -1 or numerical_input_index == -1:
              print("Fallback identification failed.")
              return None

    # print(f"Debug: Final Assignment -> Image Input Index: {image_input_index}, Numerical Input Index: {numerical_input_index}") # Uncomment

    # --- Prepare Input Data Dictionary with CORRECT indices ---
    try:
        expected_image_dtype = next(d['dtype'] for d in input_details if d['index'] == image_input_index)
        expected_num_dtype = next(d['dtype'] for d in input_details if d['index'] == numerical_input_index)

        model_inputs = {
            image_input_index: image_input.astype(expected_image_dtype),
            numerical_input_index: scaled_numerical_features.astype(expected_num_dtype)
        }
    except Exception as e:
        print(f"ERROR: Failed setting input tensor data types: {e}")
        return None

    return model_inputs


# (run_inference and postprocess_output are identical to CODE 2)
def run_inference(model_inputs):
    """Runs inference using the loaded TFLite model."""
    global interpreter, input_details, output_details

    if interpreter is None or model_inputs is None:
        print("ERROR: Interpreter not loaded or input data missing.")
        return None

    try:
        # Set the input tensors using the model_inputs dictionary
        for index, data in model_inputs.items():
             # Find expected dtype for this index
             expected_dtype = next(d['dtype'] for d in input_details if d['index'] == index)
             # Check if cast is needed (safer for TFLite)
             if data.dtype != expected_dtype:
                  # print(f"Warning: Casting input tensor {index} from {data.dtype} to {expected_dtype}.") # Uncomment if needed
                  data = data.astype(expected_dtype)
             interpreter.set_tensor(index, data)

        # Run inference
        interpreter.invoke()

        # Get the output tensor
        output_data = interpreter.get_tensor(output_details[0]['index'])
        return output_data
    except Exception as e:
        print(f"ERROR: Failed during model inference: {e}")
        # import traceback # Uncomment for full traceback if needed
        # traceback.print_exc()
        return None


def postprocess_output(output_data):
    """Interprets the model's output to get prediction and confidence."""
    global loaded_labels

    if output_data is None or not loaded_labels:
        return "Error", 0.0

    try:
        # Assuming output_data is a 1D array of probabilities or logits for each class
        probabilities = output_data[0] # Example: [[prob1, prob2, prob3, prob4]]

        # Apply softmax if model output is logits (optional, depends on your model)
        # def softmax(x):
        #     e_x = np.exp(x - np.max(x))
        #     return e_x / e_x.sum(axis=0)
        # probabilities = softmax(output_data[0])

        predicted_index = np.argmax(probabilities)
        predicted_label = loaded_labels[predicted_index]
        confidence = float(probabilities[predicted_index]) # Convert numpy float to python float
        return predicted_label, confidence
    except IndexError:
        print(f"ERROR: Output data shape unexpected: {output_data.shape}")
        return "Shape Err", 0.0
    except Exception as e:
        print(f"ERROR: Failed during postprocessing: {e}")
        return "Post Err", 0.0


# ==============================
# === View Switching Logic ===
# ==============================

def show_live_view():
    """Hides results view and shows the live camera/sensor view."""
    global live_view_frame, results_view_frame, lv_classify_button
    if results_view_frame:
        results_view_frame.pack_forget() # Hide results
    if live_view_frame:
        live_view_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True) # Show live view
    # Ensure classify button is enabled when returning
    if lv_classify_button:
        lv_classify_button.config(state=tk.NORMAL)
    # print("Debug: Switched to Live View") # Uncomment if needed


def show_results_view():
    """Hides live view and shows the classification results view."""
    global live_view_frame, results_view_frame
    if live_view_frame:
        live_view_frame.pack_forget() # Hide live view
    if results_view_frame:
        results_view_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True) # Show results
    # print("Debug: Switched to Results View") # Uncomment if needed


# ======================
# === GUI Functions ===
# ======================

def clear_results_display():
    """Clears the classification results display widgets to placeholders."""
    global rv_image_label, rv_prediction_label, rv_confidence_label
    global rv_magnetism_label, rv_ldc_label, placeholder_img_tk

    if rv_image_label:
        # Use the globally created placeholder
        if placeholder_img_tk:
            rv_image_label.img_tk = placeholder_img_tk # Keep reference
            rv_image_label.config(image=placeholder_img_tk, text="")
        else:
             rv_image_label.config(image='', text="No Image") # Fallback
    if rv_prediction_label: rv_prediction_label.config(text="...")
    if rv_confidence_label: rv_confidence_label.config(text="...")
    if rv_magnetism_label: rv_magnetism_label.config(text="...")
    if rv_ldc_label: rv_ldc_label.config(text="...")


def capture_and_classify():
    """Captures image and sensor data, runs AI classification, displays results on results page."""
    global lv_classify_button, window, camera, IDLE_VOLTAGE, IDLE_RP_VALUE
    global rv_image_label, rv_prediction_label, rv_confidence_label
    global rv_magnetism_label, rv_ldc_label
    global interpreter # Check if AI is ready

    if not interpreter:
         messagebox.showerror("AI Error", "AI Model is not initialized. Cannot classify.")
         return
    if not camera:
        messagebox.showerror("Camera Error", "Camera not available.")
        return

    # --- Disable Button ---
    lv_classify_button.config(state=tk.DISABLED)
    # Don't clear results here, do it when switching back or initially
    window.update_idletasks() # Force GUI update

    # --- Capture Image ---
    print("Capturing image...")
    ret, frame = camera.read()
    if not ret:
        messagebox.showerror("Capture Error", "Failed to capture image.")
        lv_classify_button.config(state=tk.NORMAL) # Re-enable on failure
        return
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    img_captured_pil = Image.fromarray(frame_rgb)
    print("Image captured.")

    # --- Get Sensor Data ---
    print("Reading sensors...")
    # Use calibration sample count for better accuracy during capture
    avg_voltage = get_averaged_hall_voltage(num_samples=NUM_SAMPLES_CALIBRATION)
    current_mag_mT = None
    mag_display_text = "N/A"
    if avg_voltage is not None:
        try:
            calibrated_idle_v = IDLE_VOLTAGE if IDLE_VOLTAGE != 0 else avg_voltage
            current_mag_mT = (avg_voltage - calibrated_idle_v) / SENSITIVITY_V_PER_MILLITESLA
            mag_display_text = f"{current_mag_mT:+.3f} mT" # Add sign
        except ZeroDivisionError: mag_display_text = "Cal Error?"
        except Exception: mag_display_text = "Calc Error"
        if IDLE_VOLTAGE == 0: mag_display_text += " (No Cal)" # Indicate if not calibrated

    current_rp_val_avg = get_averaged_rp_data(num_samples=NUM_SAMPLES_CALIBRATION)
    current_rp_val = None
    delta_rp = None
    ldc_display_text = "N/A"
    if current_rp_val_avg is not None:
        current_rp_val = int(current_rp_val_avg)
        if IDLE_RP_VALUE != 0:
            delta_rp = current_rp_val - IDLE_RP_VALUE
            ldc_display_text = f"{current_rp_val} (Δ{delta_rp:+,})" # Add sign
        else:
            ldc_display_text = f"{current_rp_val} (No Cal)"
    else:
        ldc_display_text = "Read Error"
    print(f"Sensor readings: Mag={mag_display_text}, LDC={ldc_display_text}")

    # --- Preprocess Data for AI ---
    print("Preprocessing data...")
    model_inputs = preprocess_input(img_captured_pil, current_mag_mT, delta_rp)
    if model_inputs is None:
        messagebox.showerror("AI Error", "Failed to preprocess data for the model.")
        lv_classify_button.config(state=tk.NORMAL) # Re-enable on failure
        return

    # --- Run AI Inference ---
    print("Running inference...")
    output_data = run_inference(model_inputs)
    if output_data is None:
        messagebox.showerror("AI Error", "Failed during model inference.")
        lv_classify_button.config(state=tk.NORMAL) # Re-enable on failure
        return

    # --- Postprocess Results ---
    predicted_label, confidence = postprocess_output(output_data)
    print(f"Inference complete: Prediction={predicted_label}, Confidence={confidence:.1%}")

    # --- Update Results View Widgets ---
    print("Updating results display...")
    # Update captured image
    try:
        # Resize for display on results page
        img_disp_width = RESULT_IMG_DISPLAY_WIDTH
        w, h = img_captured_pil.size
        aspect_ratio = h / w
        img_disp_height = int(img_disp_width * aspect_ratio)
        img_for_display = img_captured_pil.resize((img_disp_width, img_disp_height), Image.Resampling.LANCZOS)
        img_tk = ImageTk.PhotoImage(img_for_display)

        if rv_image_label:
            rv_image_label.img_tk = img_tk # Keep reference!
            rv_image_label.config(image=img_tk)
        else: print("Warning: rv_image_label not found")
    except Exception as e:
        print(f"Error resizing/displaying result image: {e}")
        if rv_image_label: rv_image_label.config(image='', text="Img Error")

    # Update text results
    if rv_prediction_label: rv_prediction_label.config(text=f"{predicted_label}")
    else: print("Warning: rv_prediction_label not found")
    if rv_confidence_label: rv_confidence_label.config(text=f"{confidence:.1%}")
    else: print("Warning: rv_confidence_label not found")
    if rv_magnetism_label: rv_magnetism_label.config(text=mag_display_text)
    else: print("Warning: rv_magnetism_label not found")
    if rv_ldc_label: rv_ldc_label.config(text=ldc_display_text)
    else: print("Warning: rv_ldc_label not found")

    # --- Switch View ---
    show_results_view()
    # NOTE: lv_classify_button remains disabled until user clicks "Classify Another" which calls show_live_view()


def calibrate_sensors():
    """Calibrates the idle voltage for Hall sensor and idle RP for LDC."""
    global IDLE_VOLTAGE, IDLE_RP_VALUE, window, previous_filtered_mag_mT
    global lv_calibrate_button, lv_classify_button # Reference correct buttons
    print("Starting Calibration...")

    # Temporarily disable buttons in live view
    if lv_calibrate_button: lv_calibrate_button.config(state=tk.DISABLED)
    if lv_classify_button: lv_classify_button.config(state=tk.DISABLED)
    window.update_idletasks() # Ensure buttons appear disabled

    hall_results = "Hall Sensor N/A"; hall_error = False
    if hall_sensor:
        avg_v = get_averaged_hall_voltage(num_samples=NUM_SAMPLES_CALIBRATION)
        if avg_v is not None:
            IDLE_VOLTAGE = avg_v
            hall_results = f"Hall Idle: {IDLE_VOLTAGE:.4f} V"
        else:
            hall_results = "Hall Cal Error: No readings"; hall_error = True
            IDLE_VOLTAGE = 0 # Reset on error
    ldc_results = "LDC Sensor N/A"; ldc_error = False
    if ldc_initialized:
        avg_rp = get_averaged_rp_data(num_samples=NUM_SAMPLES_CALIBRATION)
        if avg_rp is not None:
            IDLE_RP_VALUE = int(avg_rp)
            ldc_results = f"LDC Idle: {IDLE_RP_VALUE}"
        else:
            ldc_results = "LDC Cal Error: No readings"; ldc_error = True
            IDLE_RP_VALUE = 0 # Reset on error

    # Reset the EMA filter state after calibration
    previous_filtered_mag_mT = None
    print("Magnetism filter state reset due to calibration.")
    print(f"Calibration Results: {hall_results}, {ldc_results}")

    # Re-enable buttons in live view
    if lv_calibrate_button: lv_calibrate_button.config(state=tk.NORMAL)
    if lv_classify_button: lv_classify_button.config(state=tk.NORMAL)

    # Show results popup
    final_message = f"{hall_results}\n{ldc_results}"
    if hall_error or ldc_error: messagebox.showwarning("Calibration Warning", final_message)
    elif not hall_sensor and not ldc_initialized: messagebox.showerror("Calibration Error", "Neither sensor available.")
    else: messagebox.showinfo("Calibration Complete", final_message)


def update_camera_feed():
    """Updates the live camera feed in the GUI (Live View)."""
    global lv_camera_label, window

    img_tk = None
    if camera and camera.isOpened():
        ret, frame = camera.read()
        if ret:
            try:
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                img = Image.fromarray(frame_rgb).resize((DISPLAY_IMG_WIDTH, DISPLAY_IMG_HEIGHT), Image.Resampling.NEAREST) # Fastest resize
                img_tk = ImageTk.PhotoImage(img)
            except Exception as e:
                print(f"Error processing camera frame: {e}")
        # else: print("Warning: Failed camera read") # Can be noisy

    if lv_camera_label: # Check if label exists
        if img_tk:
            lv_camera_label.img_tk = img_tk # Anchor update to label
            lv_camera_label.configure(image=img_tk, text="")
        else: # If camera failed or frame processing failed
            if not hasattr(lv_camera_label, 'no_cam_img'): # Create placeholder only once
                try:
                    placeholder = Image.new('RGB', (DISPLAY_IMG_WIDTH, DISPLAY_IMG_HEIGHT), color='#BDBDBD')
                    lv_camera_label.no_cam_img = ImageTk.PhotoImage(placeholder)
                except Exception as e:
                    print(f"Error creating placeholder image: {e}")
                    lv_camera_label.no_cam_img = None

            # Configure placeholder only if it exists and is different
            if lv_camera_label.no_cam_img and lv_camera_label.cget("image") != str(lv_camera_label.no_cam_img):
                 lv_camera_label.configure(image=lv_camera_label.no_cam_img, text="No Camera Feed")

    # Schedule next update only if window still exists
    if window and window.winfo_exists():
        window.after(CAMERA_UPDATE_INTERVAL_MS, update_camera_feed)


def update_magnetism():
    """Updates the live magnetism reading in the GUI (Live View)."""
    global lv_magnetism_label, window, previous_filtered_mag_mT, IDLE_VOLTAGE

    avg_voltage = get_averaged_hall_voltage()
    display_text = "N/A"

    if hall_sensor:
        if avg_voltage is not None:
            try:
                idle_v = IDLE_VOLTAGE if IDLE_VOLTAGE != 0 else avg_voltage
                raw_mag_mT = (avg_voltage - idle_v) / SENSITIVITY_V_PER_MILLITESLA

                # --- EMA Filter ---
                if previous_filtered_mag_mT is None: filtered_mag_mT = raw_mag_mT
                else: filtered_mag_mT = (MAGNETISM_FILTER_ALPHA * raw_mag_mT) + ((1 - MAGNETISM_FILTER_ALPHA) * previous_filtered_mag_mT)
                previous_filtered_mag_mT = filtered_mag_mT
                # --- End EMA Filter ---

                unit, value = ("mT", filtered_mag_mT) if abs(filtered_mag_mT) >= 0.1 else ("µT", filtered_mag_mT * 1000)
                sign = "+" if value > 1e-9 else "" # Add '+' for positive non-zero
                display_text = f"{sign}{value:.2f} {unit}"
                if IDLE_VOLTAGE == 0: display_text += " (No Cal)"

            except ZeroDivisionError: display_text = "Cal Error?"; previous_filtered_mag_mT = None
            except Exception as e: display_text = "Error"; previous_filtered_mag_mT = None
        else: display_text = "Read Err"; previous_filtered_mag_mT = None

    if lv_magnetism_label: lv_magnetism_label.config(text=display_text)
    if window and window.winfo_exists(): window.after(GUI_UPDATE_INTERVAL_MS, update_magnetism)


def update_ldc_reading():
    """Updates the live LDC RP reading in the GUI (Live View)."""
    global lv_ldc_label, window, RP_DISPLAY_BUFFER, IDLE_RP_VALUE

    avg_rp_val = get_averaged_rp_data()
    display_rp_text = "N/A"

    if ldc_initialized:
        if avg_rp_val is not None:
            RP_DISPLAY_BUFFER.append(avg_rp_val)
            if RP_DISPLAY_BUFFER:
                buffer_avg = sum(RP_DISPLAY_BUFFER) / len(RP_DISPLAY_BUFFER)
                current_rp_int = int(buffer_avg)
                if IDLE_RP_VALUE != 0:
                    delta = current_rp_int - IDLE_RP_VALUE
                    display_rp_text = f"{current_rp_int} (Δ{delta:+,})" # Add sign
                else:
                    display_rp_text = f"{current_rp_int} (No Cal)"
            else: display_rp_text = "..." # Buffer empty
        else: RP_DISPLAY_BUFFER.clear(); display_rp_text = "Read Err"

    if lv_ldc_label: lv_ldc_label.config(text=display_rp_text)
    if window and window.winfo_exists(): window.after(GUI_UPDATE_INTERVAL_MS, update_ldc_reading)

# ======================
# === GUI Setup ========
# ======================
def setup_gui():
    global window, main_frame, placeholder_img_tk
    global live_view_frame, results_view_frame
    global lv_camera_label, lv_magnetism_label, lv_ldc_label
    global lv_classify_button, lv_calibrate_button
    global rv_image_label, rv_prediction_label, rv_confidence_label
    global rv_magnetism_label, rv_ldc_label, rv_classify_another_button
    global label_font, readout_font, button_font, title_font
    global result_title_font, result_font, result_value_font

    window = tk.Tk(); window.title("AI Metal Classifier v3.0.0"); window.geometry("1000x650") # Adjusted size slightly
    style = ttk.Style(); style.theme_use('clam' if 'clam' in style.theme_names() else 'default')

    # Define fonts
    title_font=tkFont.Font(family="Helvetica", size=16, weight="bold")
    label_font=tkFont.Font(family="Helvetica", size=11)
    readout_font=tkFont.Font(family="Consolas", size=14, weight="bold")
    button_font=tkFont.Font(family="Helvetica", size=11, weight="bold")
    result_title_font=tkFont.Font(family="Helvetica", size=12, weight="bold") # For "Material:", "Confidence:" etc.
    result_value_font=tkFont.Font(family="Consolas", size=14, weight="bold") # For the actual result value

    # Configure styles
    style.configure("TLabel",font=label_font,padding=2)
    style.configure("TButton",font=button_font,padding=(10, 6))
    style.configure("TLabelframe",padding=8)
    style.configure("TLabelframe.Label",font=tkFont.Font(family="Helvetica",size=12,weight="bold"))
    style.configure("Readout.TLabel",font=readout_font,padding=(5,1))
    style.configure("ResultTitle.TLabel",font=label_font, padding=(5,2)) # Use standard label font for titles
    style.configure("ResultValue.TLabel",font=result_value_font, padding=(5,1), anchor=tk.E) # Right align results

    # --- Main container frame ---
    main_frame = ttk.Frame(window, padding="5 5 5 5")
    main_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
    main_frame.rowconfigure(0, weight=1)
    main_frame.columnconfigure(0, weight=1)

    # ==================================
    # === Create Live View Frame ===
    # ==================================
    live_view_frame = ttk.Frame(main_frame, padding="5 5 5 5")
    # Don't grid/pack live_view_frame yet, show_live_view will do it.
    live_view_frame.columnconfigure(0, weight=3) # Camera feed area (larger)
    live_view_frame.columnconfigure(1, weight=1) # Controls area (smaller)
    live_view_frame.rowconfigure(0, weight=1) # Allow vertical expansion

    # --- Live View: Camera Feed ---
    lv_camera_label = ttk.Label(live_view_frame, text="Initializing Camera...", anchor="center", borderwidth=1, relief="sunken")
    lv_camera_label.grid(row=0, column=0, padx=(0, 10), pady=0, sticky="nsew")

    # --- Live View: Controls Column ---
    lv_controls_frame = ttk.Frame(live_view_frame)
    lv_controls_frame.grid(row=0, column=1, padx=(0, 0), pady=0, sticky="nsew")
    lv_controls_frame.columnconfigure(0, weight=1) # Allow controls to expand width
    lv_controls_row_idx = 0

    # --- Live View: Readings Frame ---
    lv_readings_frame = ttk.Labelframe(lv_controls_frame, text=" Live Readings ", padding="10 5 10 5")
    lv_readings_frame.grid(row=lv_controls_row_idx, column=0, sticky="new", pady=(0, 10))
    lv_controls_row_idx += 1
    lv_readings_frame.columnconfigure(0, weight=0) # Label column fixed width
    lv_readings_frame.columnconfigure(1, weight=1) # Value column expands
    ttk.Label(lv_readings_frame, text="Magnetism:").grid(row=0, column=0, sticky="w", padx=(0, 10))
    lv_magnetism_label = ttk.Label(lv_readings_frame, text="Init...", style="Readout.TLabel", anchor="e")
    lv_magnetism_label.grid(row=0, column=1, sticky="ew")
    ttk.Label(lv_readings_frame, text="LDC RP (Δ):").grid(row=1, column=0, sticky="w", padx=(0, 10), pady=(3,0))
    lv_ldc_label = ttk.Label(lv_readings_frame, text="Init...", style="Readout.TLabel", anchor="e")
    lv_ldc_label.grid(row=1, column=1, sticky="ew", pady=(3,0))

    # --- Live View: Actions Frame ---
    lv_actions_frame = ttk.Labelframe(lv_controls_frame, text=" Actions ", padding="10 5 10 10")
    lv_actions_frame.grid(row=lv_controls_row_idx, column=0, sticky="new", pady=(0, 10))
    lv_controls_row_idx += 1
    lv_actions_frame.columnconfigure(0, weight=1) # Single column, button expands
    # Classify Button
    lv_classify_button = ttk.Button(lv_actions_frame, text="Capture & Classify", command=capture_and_classify)
    lv_classify_button.grid(row=0, column=0, sticky="ew", pady=(5, 5))
    # Calibrate Button
    lv_calibrate_button = ttk.Button(lv_actions_frame, text="Calibrate Sensors", command=calibrate_sensors)
    lv_calibrate_button.grid(row=1, column=0, sticky="ew", pady=(5, 5))

    # ===================================
    # === Create Results View Frame ===
    # ===================================
    results_view_frame = ttk.Frame(main_frame, padding="10 10 10 10")
    # Don't grid/pack results_view_frame yet.
    results_view_frame.columnconfigure(0, weight=1) # Center content using a single column

    # --- Results View: Inner frame for content ---
    rv_content_frame = ttk.Frame(results_view_frame)
    rv_content_frame.grid(row=0, column=0, sticky="n") # Place inner frame at top-center

    ttk.Label(rv_content_frame, text="Classification Result", font=title_font).grid(row=0, column=0, columnspan=2, pady=(5, 15))

    # Placeholder for the results image
    try:
        placeholder = Image.new('RGB', (RESULT_IMG_DISPLAY_WIDTH, int(RESULT_IMG_DISPLAY_WIDTH * 0.75)), color='#E0E0E0')
        placeholder_img_tk = ImageTk.PhotoImage(placeholder)
    except Exception as e:
        print(f"Error creating placeholder results image: {e}")
        placeholder_img_tk = None

    rv_image_label = ttk.Label(rv_content_frame, anchor="center", borderwidth=1, relief="sunken", image=placeholder_img_tk)
    rv_image_label.grid(row=1, column=0, columnspan=2, pady=(0, 15))

    # --- Results View: Details Grid ---
    rv_details_frame = ttk.Frame(rv_content_frame)
    rv_details_frame.grid(row=2, column=0, columnspan=2, pady=(0, 15))
    rv_details_frame.columnconfigure(0, weight=0, pad=5) # Labels column
    rv_details_frame.columnconfigure(1, weight=1, pad=5) # Values column
    res_row = 0

    ttk.Label(rv_details_frame, text="Material:", style="ResultTitle.TLabel", anchor='w').grid(row=res_row, column=0, sticky="w")
    rv_prediction_label = ttk.Label(rv_details_frame, text="...", style="ResultValue.TLabel", anchor='e')
    rv_prediction_label.grid(row=res_row, column=1, sticky="ew"); res_row += 1

    ttk.Label(rv_details_frame, text="Confidence:", style="ResultTitle.TLabel", anchor='w').grid(row=res_row, column=0, sticky="w")
    rv_confidence_label = ttk.Label(rv_details_frame, text="...", style="ResultValue.TLabel", anchor='e')
    rv_confidence_label.grid(row=res_row, column=1, sticky="ew"); res_row += 1

    # Separator
    ttk.Separator(rv_details_frame, orient='horizontal').grid(row=res_row, column=0, columnspan=2, sticky='ew', pady=8); res_row += 1

    ttk.Label(rv_details_frame, text="Magnetism Used:", style="ResultTitle.TLabel", anchor='w').grid(row=res_row, column=0, sticky="w")
    rv_magnetism_label = ttk.Label(rv_details_frame, text="...", style="ResultValue.TLabel", anchor='e')
    rv_magnetism_label.grid(row=res_row, column=1, sticky="ew"); res_row += 1

    ttk.Label(rv_details_frame, text="LDC RP (Δ) Used:", style="ResultTitle.TLabel", anchor='w').grid(row=res_row, column=0, sticky="w")
    rv_ldc_label = ttk.Label(rv_details_frame, text="...", style="ResultValue.TLabel", anchor='e')
    rv_ldc_label.grid(row=res_row, column=1, sticky="ew"); res_row += 1

    # --- Results View: Classify Another Button ---
    rv_classify_another_button = ttk.Button(rv_content_frame, text="<< Classify Another", command=show_live_view)
    rv_classify_another_button.grid(row=3, column=0, columnspan=2, pady=(10, 5))


    # --- Initial GUI State ---
    clear_results_display() # Set initial text/image for results widgets
    show_live_view() # Show the live view frame first

# ==========================
# === Main Execution =======
# ==========================
def run_application():
    global window
    setup_gui() # Setup all GUI elements first

    # Set initial text for live view before starting loops
    if not camera: lv_camera_label.configure(text="Camera Failed", image='')
    if not hall_sensor: lv_magnetism_label.config(text="N/A")
    if not ldc_initialized: lv_ldc_label.config(text="N/A")
    if not interpreter: # Disable classify if AI failed
        if lv_classify_button: lv_classify_button.config(state=tk.DISABLED)
        messagebox.showwarning("AI Init Failed", "AI components failed to initialize. Classification disabled.")

    # Start the update loops for the live view
    print("Starting update loops...")
    update_camera_feed()
    update_magnetism()
    update_ldc_reading()

    print("Starting Tkinter main loop...")
    window.mainloop() # Blocks here until window is closed

# --- Cleanup ---
# (Identical to CODE 2)
def cleanup_resources():
    print("Cleaning up resources...")
    # Release camera first
    if camera and camera.isOpened():
        camera.release()
        print("Camera released.")
    cv2.destroyAllWindows()

    # Close SPI and put LDC to sleep
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

    # Cleanup GPIO
    if SPI_ENABLED:
        try:
            GPIO.cleanup()
            print("GPIO cleaned up.")
        except Exception as e: print(f"Note: GPIO cleanup error: {e}")

    print("Cleanup complete.")

# --- Run ---
if __name__ == '__main__':
    hardware_ok = False
    ai_ok = False
    try:
        initialize_hardware()
        hardware_ok = True # Assume okay if no immediate fatal error
        ai_ok = initialize_ai() # Initialize AI components
        run_application()
    except KeyboardInterrupt:
         print("Keyboard interrupt detected. Exiting.")
    except Exception as e:
        print(f"FATAL ERROR in main execution: {e}")
        try:
            if window: messagebox.showerror("Fatal Error", f"An unrecoverable error occurred:\n{e}")
        except Exception: pass # Avoid error if GUI isn't up
        import traceback; traceback.print_exc()
    finally:
        # Only cleanup hardware if it was potentially initialized
        if hardware_ok:
             cleanup_resources()
