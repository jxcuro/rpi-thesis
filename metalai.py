# CODE 2 - AI Metal Classifier GUI
# Description: Displays live sensor data and camera feed.
#              Captures image and sensor readings, classifies metal using a TFLite model,
#              and displays the results.
# Version: 2.0.0 - Based on CODE 1 (v1.6.1_mod2), integrated AI classification.

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

# --- AI Imports ---
try:
    # Use tflite_runtime for Raspberry Pi
    from tflite_runtime.interpreter import Interpreter
    # If tflite_runtime fails, try the full tensorflow (less common on Pi)
except ImportError:
    try:
        from tensorflow.lite.python.interpreter import Interpreter
    except ImportError:
        print("ERROR: TensorFlow Lite Runtime is not installed.")
        print("Please install it: pip install tflite-runtime")
        exit() # Exit if no TFLite interpreter is found

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
NUM_SAMPLES_CALIBRATION = 5
GUI_UPDATE_INTERVAL_MS = 100 # Faster GUI refresh rate
CAMERA_UPDATE_INTERVAL_MS = 30 # Faster camera refresh target
LDC_DISPLAY_BUFFER_SIZE = 5
MAGNETISM_FILTER_ALPHA = 0.2

# Camera
CAMERA_INDEX = 0
DISPLAY_IMG_WIDTH = 640
DISPLAY_IMG_HEIGHT = 480
RESULT_IMG_DISPLAY_WIDTH = 200 # Size to display the captured image in results

# --- AI Model Configuration ---
# <<<*** IMPORTANT: Set these paths correctly ***>>>
try: BASE_PATH = os.path.dirname(os.path.abspath(__file__))
except NameError: BASE_PATH = os.getcwd() # Fallback if __file__ not defined
MODEL_FILENAME = "material_classifier_model.tflite"
LABELS_FILENAME = "material_labels.txt"
SCALER_FILENAME = "numerical_scaler.joblib"
MODEL_PATH = os.path.join(BASE_PATH, MODEL_FILENAME)
LABELS_PATH = os.path.join(BASE_PATH, LABELS_FILENAME)
SCALER_PATH = os.path.join(BASE_PATH, SCALER_FILENAME)

# Expected input size for the AI model's image input
AI_IMG_WIDTH = 224
AI_IMG_HEIGHT = 224

# Hall Sensor (ADS1115)
HALL_ADC_CHANNEL = ADS.P0 if I2C_ENABLED else None
SENSITIVITY_V_PER_TESLA = 0.0004
SENSITIVITY_V_PER_MILLITESLA = SENSITIVITY_V_PER_TESLA * 1000
IDLE_VOLTAGE = 1.7348 # Will be recalibrated

# Inductive Sensor (LDC1101)
SPI_BUS = 0; SPI_DEVICE = 0; SPI_SPEED = 500000; SPI_MODE = 0b00 # 500kHz
CS_PIN = 8; LDC_CHIP_ID = 0xD4
START_CONFIG_REG, RP_SET_REG, TC1_REG, TC2_REG, DIG_CONFIG_REG = 0x0B, 0x01, 0x02, 0x03, 0x04
ALT_CONFIG_REG, D_CONF_REG, INTB_MODE_REG = 0x05, 0x0C, 0x0A
RP_DATA_MSB_REG, RP_DATA_LSB_REG = 0x22, 0x21; CHIP_ID_REG = 0x3F
ACTIVE_CONVERSION_MODE, SLEEP_MODE = 0x00, 0x01

# Calibration
IDLE_RP_VALUE = 0 # Will be recalibrated

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

# GUI Globals
window = None; camera_label = None; controls_frame = None
magnetism_label = None; ldc_label = None
classify_button = None; calibrate_button = None
label_font = None; readout_font = None; button_font = None
title_font = None; check_font = None; result_title_font = None; result_font = None
# Result display widgets
results_frame = None
result_image_label = None
prediction_label = None
confidence_label = None
result_magnetism_label = None
result_ldc_label = None


# =========================
# === Hardware Setup ===
# =========================
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
        # Check if it has the expected 'transform' method
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
        # Optional: Print model input/output details for verification
        # print("Model Input Details:", input_details)
        # print("Model Output Details:", output_details)

        # Basic check: Does the number of labels match the model's output size?
        output_shape = output_details[0]['shape']
        if output_shape[-1] != len(loaded_labels):
             print(f"WARNING: Model output size ({output_shape[-1]}) does not match number of labels ({len(loaded_labels)}).")

    except FileNotFoundError:
        print(f"ERROR: Model file not found at {MODEL_PATH}")
        return False
    except Exception as e:
        print(f"ERROR: Failed to load TFLite model or allocate tensors: {e}")
        # Print detailed traceback for TFLite errors
        import traceback
        traceback.print_exc()
        return False

    print("--- AI Initialization Complete ---")
    return True

# =========================
# === LDC1101 Functions ===
# =========================
# (Mostly unchanged from CODE 1, using faster interactions)
def ldc_write_register(reg_addr, value):
    if not spi: return
    try:
        GPIO.output(CS_PIN, GPIO.LOW)
        spi.xfer2([reg_addr & 0x7F, value]) # Write command MSB=0
        GPIO.output(CS_PIN, GPIO.HIGH)
    except Exception as e:
        print(f"Error writing LDC 0x{reg_addr:02X}: {e}")
        try: GPIO.output(CS_PIN, GPIO.HIGH) # Ensure CS high on error
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
        print(f"Error reading LDC 0x{reg_addr:02X}: {e}")
        try: GPIO.output(CS_PIN, GPIO.HIGH) # Ensure CS high on error
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
        print(f"Error in get_ldc_rpdata: {e}")
        return None

# ============================
# === Sensor Reading (Avg) ===
# ============================
def get_averaged_hall_voltage(num_samples=NUM_SAMPLES_PER_UPDATE):
    if not hall_sensor: return None
    readings = []
    for _ in range(num_samples):
        try:
            if hall_sensor: readings.append(hall_sensor.voltage)
            # time.sleep(0.001) # Minimal delay if ADS needs settling, depends on data rate
        except Exception as e:
            print(f"Warning: Error reading Hall sensor voltage: {e}")
            return None # Return None if any read fails
    return sum(readings) / len(readings) if readings else None

def get_averaged_rp_data(num_samples=NUM_SAMPLES_PER_UPDATE):
    if not ldc_initialized: return None
    readings = [get_ldc_rpdata() for _ in range(num_samples)]
    valid_readings = [r for r in readings if r is not None]
    return sum(valid_readings) / len(valid_readings) if valid_readings else None

# ==========================
# === AI Processing ========
# ==========================

def preprocess_input(image_pil, mag_mT, ldc_rp_delta):
    """Prepares image and sensor data for the TFLite model."""
    global numerical_scaler, input_details

    if numerical_scaler is None:
        print("ERROR: Numerical scaler not loaded.")
        return None # Or handle appropriately

    # --- Image Preprocessing ---
    # Resize to the model's expected input size
    img_resized = image_pil.resize((AI_IMG_WIDTH, AI_IMG_HEIGHT), Image.Resampling.LANCZOS)
    # Convert to numpy array, ensure RGB
    img_rgb = img_resized.convert('RGB')
    image_np = np.array(img_rgb, dtype=np.float32)
    # Normalize pixel values (e.g., to [0, 1] - *ADJUST IF YOUR MODEL EXPECTS DIFFERENT NORMALIZATION*)
    image_np /= 255.0
    # Add batch dimension
    image_input = np.expand_dims(image_np, axis=0)

    # --- Numerical Feature Preprocessing ---
    # Handle potential None values (e.g., if sensor failed or wasn't calibrated)
    # Use 0.0 as a placeholder. Scaling should handle this if trained similarly.
    mag_mT_val = mag_mT if mag_mT is not None else 0.0
    ldc_rp_delta_val = ldc_rp_delta if ldc_rp_delta is not None else 0.0

    # Create numpy array for numerical features
    numerical_features = np.array([[mag_mT_val, ldc_rp_delta_val]], dtype=np.float32)

    # Scale numerical features using the loaded scaler
    try:
        scaled_numerical_features = numerical_scaler.transform(numerical_features)
    except Exception as e:
        print(f"ERROR: Failed to scale numerical features: {e}")
        # Optionally, provide default scaled values or raise error
        scaled_numerical_features = np.zeros_like(numerical_features) # Default to zeros if scaling fails

    # --- Combine Inputs (if needed by model) ---
    # This depends heavily on how your TFLite model was constructed.
    # Option 1: Model has multiple inputs (common)
    #   - Identify the input tensor index for the image and numerical data
    #   - Return them separately or in a dictionary/list
    # Option 2: Model expects a single, concatenated input
    #   - Flatten and concatenate image_input and scaled_numerical_features
    #   - This is less common for mixed data types unless handled carefully during training

    # ***ASSUMPTION: Model has TWO inputs. Adjust based on your input_details***
    # Find the indices based on tensor names or expected shapes/types if names aren't reliable.
    # This is a placeholder - *YOU MUST VERIFY/ADJUST THIS* based on `input_details`
    image_input_index = input_details[0]['index'] # Assuming first input is image
    numerical_input_index = input_details[1]['index'] # Assuming second is numerical

    # Prepare input data as a dictionary mapping index to data
    model_inputs = {
        image_input_index: image_input,
        numerical_input_index: scaled_numerical_features
    }
    # Alternatively, if using set_tensor:
    # return image_input, scaled_numerical_features

    return model_inputs # Return the dictionary of inputs


def run_inference(model_inputs):
    """Runs inference using the loaded TFLite model."""
    global interpreter, input_details, output_details

    if interpreter is None or model_inputs is None:
        print("ERROR: Interpreter not loaded or input data missing.")
        return None

    try:
        # Set the input tensors
        # Assumes model_inputs is the dictionary from preprocess_input
        for index, data in model_inputs.items():
             # Check data type compatibility
             expected_dtype = next(d['dtype'] for d in input_details if d['index'] == index)
             if data.dtype != expected_dtype:
                  print(f"Warning: Input data type mismatch for tensor {index}. Got {data.dtype}, expected {expected_dtype}. Attempting cast.")
                  data = data.astype(expected_dtype)
             interpreter.set_tensor(index, data)

        # Run inference
        interpreter.invoke()

        # Get the output tensor
        output_data = interpreter.get_tensor(output_details[0]['index'])
        return output_data
    except Exception as e:
        print(f"ERROR: Failed during model inference: {e}")
        import traceback
        traceback.print_exc()
        return None


def postprocess_output(output_data):
    """Interprets the model's output to get prediction and confidence."""
    global loaded_labels

    if output_data is None or not loaded_labels:
        return "Error", 0.0

    # Assuming output_data is a 1D array of probabilities or logits
    # If logits, apply softmax (optional, depends on model output)
    # probabilities = softmax(output_data[0]) # Example if output needs softmax
    probabilities = output_data[0] # Assume direct probabilities

    # Get the index of the highest probability
    predicted_index = np.argmax(probabilities)

    # Get the corresponding label
    predicted_label = loaded_labels[predicted_index]

    # Get the confidence score
    confidence = float(probabilities[predicted_index])

    return predicted_label, confidence

# Helper function for softmax (if needed)
# def softmax(x):
#     e_x = np.exp(x - np.max(x)) # Subtract max for numerical stability
#     return e_x / e_x.sum(axis=0)


# ======================
# === GUI Functions ===
# ======================

def clear_results_display():
    """Clears the classification results area."""
    global result_image_label, prediction_label, confidence_label
    global result_magnetism_label, result_ldc_label

    if result_image_label:
        placeholder = Image.new('RGB', (RESULT_IMG_DISPLAY_WIDTH, int(RESULT_IMG_DISPLAY_WIDTH * (3/4))), color='#E0E0E0')
        ph_img = ImageTk.PhotoImage(placeholder)
        result_image_label.img_tk = ph_img # Keep reference
        result_image_label.config(image=ph_img, text="")
    if prediction_label: prediction_label.config(text="...")
    if confidence_label: confidence_label.config(text="...")
    if result_magnetism_label: result_magnetism_label.config(text="...")
    if result_ldc_label: result_ldc_label.config(text="...")


def capture_and_classify():
    """Captures image and sensor data, runs AI classification, and displays results."""
    global classify_button, window, camera, IDLE_VOLTAGE, IDLE_RP_VALUE
    global result_image_label, prediction_label, confidence_label
    global result_magnetism_label, result_ldc_label
    global interpreter # Check if AI is ready

    if not interpreter:
         messagebox.showerror("AI Error", "AI Model is not initialized. Cannot classify.")
         return
    if not camera:
        messagebox.showerror("Camera Error", "Camera not available.")
        return

    # --- Disable Button ---
    classify_button.config(state=tk.DISABLED)
    clear_results_display() # Clear previous results
    window.update_idletasks() # Force GUI update

    # --- Capture Image ---
    ret, frame = camera.read()
    if not ret:
        messagebox.showerror("Capture Error", "Failed to capture image.")
        classify_button.config(state=tk.NORMAL)
        return
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    img_captured_pil = Image.fromarray(frame_rgb)

    # --- Get Sensor Data ---
    # Use calibration sample count for better accuracy during capture
    avg_voltage = get_averaged_hall_voltage(num_samples=NUM_SAMPLES_CALIBRATION)
    current_mag_mT = None
    mag_display_text = "N/A"
    if avg_voltage is not None:
        try:
            # Use calibrated idle voltage if available
            calibrated_idle_v = IDLE_VOLTAGE if IDLE_VOLTAGE != 0 else avg_voltage # Use current if not calibrated
            current_mag_mT = (avg_voltage - calibrated_idle_v) / SENSITIVITY_V_PER_MILLITESLA
            mag_display_text = f"{current_mag_mT:.3f} mT"
        except ZeroDivisionError:
            mag_display_text = "Cal Error?"
        except Exception:
             mag_display_text = "Calc Error"

    current_rp_val_avg = get_averaged_rp_data(num_samples=NUM_SAMPLES_CALIBRATION)
    current_rp_val = None
    delta_rp = None
    ldc_display_text = "N/A"
    if current_rp_val_avg is not None:
        current_rp_val = int(current_rp_val_avg)
        if IDLE_RP_VALUE != 0:
            delta_rp = current_rp_val - IDLE_RP_VALUE
            ldc_display_text = f"{current_rp_val} (Δ{delta_rp})"
        else:
            ldc_display_text = f"{current_rp_val} (No Cal)"
    else:
        ldc_display_text = "Read Error" # Sensor read failed

    # --- Preprocess Data for AI ---
    # Note: Pass mag_mT and delta_rp to preprocessing
    model_inputs = preprocess_input(img_captured_pil, current_mag_mT, delta_rp)

    if model_inputs is None:
        messagebox.showerror("AI Error", "Failed to preprocess data for the model.")
        classify_button.config(state=tk.NORMAL)
        return

    # --- Run AI Inference ---
    output_data = run_inference(model_inputs)

    if output_data is None:
        messagebox.showerror("AI Error", "Failed during model inference.")
        classify_button.config(state=tk.NORMAL)
        return

    # --- Postprocess Results ---
    predicted_label, confidence = postprocess_output(output_data)

    # --- Display Results ---
    # Display captured image
    try:
        img_for_display = img_captured_pil.resize((RESULT_IMG_DISPLAY_WIDTH, int(RESULT_IMG_DISPLAY_WIDTH * (img_captured_pil.height / img_captured_pil.width))), Image.Resampling.LANCZOS)
        img_tk = ImageTk.PhotoImage(img_for_display)
        result_image_label.img_tk = img_tk # Keep reference!
        result_image_label.config(image=img_tk)
    except Exception as e:
        print(f"Error resizing/displaying result image: {e}")
        result_image_label.config(text="Img Error")

    # Display text results
    prediction_label.config(text=f"{predicted_label}")
    confidence_label.config(text=f"{confidence:.1%}")
    result_magnetism_label.config(text=mag_display_text)
    result_ldc_label.config(text=ldc_display_text)

    print(f"Classification: {predicted_label} ({confidence:.1%}), Mag: {mag_display_text}, LDC: {ldc_display_text}")

    # --- Re-enable Button ---
    # Add a small delay if needed, otherwise enable immediately
    window.after(100, lambda: classify_button.config(state=tk.NORMAL))


def calibrate_sensors():
    """Calibrates the idle voltage for Hall sensor and idle RP for LDC."""
    global IDLE_VOLTAGE, IDLE_RP_VALUE, window, previous_filtered_mag_mT
    print("Starting Calibration...")
    # Temporarily disable buttons
    calibrate_button.config(state=tk.DISABLED)
    classify_button.config(state=tk.DISABLED)
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

    # Re-enable buttons
    calibrate_button.config(state=tk.NORMAL)
    classify_button.config(state=tk.NORMAL)

    # Show results
    final_message = f"{hall_results}\n{ldc_results}"
    if hall_error or ldc_error: messagebox.showwarning("Calibration Warning", final_message)
    elif not hall_sensor and not ldc_initialized: messagebox.showerror("Calibration Error", "Neither sensor available for calibration.")
    else: messagebox.showinfo("Calibration Complete", final_message)
    # Clear results display after calibration as readings might change relative to idle
    clear_results_display()


def update_camera_feed():
    """Updates the live camera feed in the GUI."""
    global camera_label, window

    img_tk = None
    if camera and camera.isOpened():
        ret, frame = camera.read()
        if ret:
            try:
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                # Use Pillow for resizing - NEAREST is fastest if quality is acceptable
                img = Image.fromarray(frame_rgb).resize((DISPLAY_IMG_WIDTH, DISPLAY_IMG_HEIGHT), Image.Resampling.NEAREST)
                img_tk = ImageTk.PhotoImage(img)
            except Exception as e:
                print(f"Error processing camera frame: {e}")
        # else: print("Warning: Failed to read frame from camera.") # Can be noisy

    if img_tk:
        camera_label.img_tk = img_tk # Anchor update to label
        camera_label.configure(image=img_tk, text="")
    else: # If camera failed or frame processing failed
        if not hasattr(camera_label, 'no_cam_img'): # Create placeholder only once
            try:
                placeholder = Image.new('RGB', (DISPLAY_IMG_WIDTH, DISPLAY_IMG_HEIGHT), color='#BDBDBD')
                camera_label.no_cam_img = ImageTk.PhotoImage(placeholder)
            except Exception as e:
                print(f"Error creating placeholder image: {e}")
                camera_label.no_cam_img = None

        # Configure placeholder only if it exists and is different
        if camera_label.no_cam_img and camera_label.cget("image") != str(camera_label.no_cam_img):
             camera_label.configure(image=camera_label.no_cam_img, text="No Camera Feed")

    if window: window.after(CAMERA_UPDATE_INTERVAL_MS, update_camera_feed) # Schedule next update


def update_magnetism():
    """Updates the live magnetism reading in the GUI."""
    global magnetism_label, window, previous_filtered_mag_mT, IDLE_VOLTAGE

    avg_voltage = get_averaged_hall_voltage()
    display_text = "N/A"

    if hall_sensor:
        if avg_voltage is not None:
            try:
                # Use calibrated idle voltage if available, otherwise raw voltage difference is less meaningful
                idle_v = IDLE_VOLTAGE if IDLE_VOLTAGE != 0 else avg_voltage
                raw_mag_mT = (avg_voltage - idle_v) / SENSITIVITY_V_PER_MILLITESLA

                # --- EMA Filter for smooth display ---
                if previous_filtered_mag_mT is None:
                    filtered_mag_mT = raw_mag_mT
                else:
                    filtered_mag_mT = (MAGNETISM_FILTER_ALPHA * raw_mag_mT) + \
                                      ((1 - MAGNETISM_FILTER_ALPHA) * previous_filtered_mag_mT)
                previous_filtered_mag_mT = filtered_mag_mT
                # --- End EMA Filter ---

                unit, value = ("mT", filtered_mag_mT) if abs(filtered_mag_mT) >= 1 else ("µT", filtered_mag_mT * 1000)
                # Show + sign for positive non-zero values
                sign = "+" if value > 1e-9 else "" # Add '+' for positive non-zero
                display_text = f"{sign}{value:.2f} {unit}"
                if IDLE_VOLTAGE == 0: display_text += " (No Cal)"

            except ZeroDivisionError:
                display_text = "Cal Error?"
                previous_filtered_mag_mT = None
            except Exception as e:
                print(f"Error calculating magnetism: {e}")
                display_text = "Error"
                previous_filtered_mag_mT = None
        else:
            display_text = "Read Err" # More specific than "Error"
            previous_filtered_mag_mT = None

    if magnetism_label: magnetism_label.config(text=display_text)
    if window: window.after(GUI_UPDATE_INTERVAL_MS, update_magnetism)


def update_ldc_reading():
    """Updates the live LDC RP reading in the GUI."""
    global ldc_label, window, RP_DISPLAY_BUFFER, IDLE_RP_VALUE

    avg_rp_val = get_averaged_rp_data()
    display_rp_text = "N/A"

    if ldc_initialized:
        if avg_rp_val is not None:
            RP_DISPLAY_BUFFER.append(avg_rp_val)
            if RP_DISPLAY_BUFFER:
                # Use buffer average for display smoothness
                buffer_avg = sum(RP_DISPLAY_BUFFER) / len(RP_DISPLAY_BUFFER)
                current_rp_int = int(buffer_avg)
                # Display delta if calibrated
                if IDLE_RP_VALUE != 0:
                    delta = current_rp_int - IDLE_RP_VALUE
                    sign = "+" if delta > 0 else ""
                    display_rp_text = f"{current_rp_int} (Δ{sign}{delta})"
                else:
                    display_rp_text = f"{current_rp_int} (No Cal)"
            else:
                display_rp_text = "..." # Buffer empty
        else:
            RP_DISPLAY_BUFFER.clear() # Clear buffer on read error
            display_rp_text = "Read Err"

    if ldc_label: ldc_label.config(text=display_rp_text)
    if window: window.after(GUI_UPDATE_INTERVAL_MS, update_ldc_reading)

# ======================
# === GUI Setup ========
# ======================
def setup_gui():
    global window, camera_label, controls_frame, magnetism_label, ldc_label
    global classify_button, calibrate_button
    global label_font, readout_font, button_font, title_font, check_font
    global results_frame, result_image_label, prediction_label, confidence_label
    global result_magnetism_label, result_ldc_label, result_title_font, result_font

    window = tk.Tk(); window.title("AI Metal Classifier v2.0.0"); window.geometry("1150x720") # Adjusted size
    style = ttk.Style(); style.theme_use('clam' if 'clam' in style.theme_names() else 'default')

    # Define fonts
    title_font=tkFont.Font(family="Helvetica", size=14, weight="bold")
    label_font=tkFont.Font(family="Helvetica", size=11)
    readout_font=tkFont.Font(family="Consolas", size=14, weight="bold")
    button_font=tkFont.Font(family="Helvetica", size=11, weight="bold")
    result_title_font=tkFont.Font(family="Helvetica", size=12, weight="bold")
    result_font=tkFont.Font(family="Consolas", size=12, weight="bold")

    # Configure styles
    style.configure("TLabel",font=label_font,padding=2)
    style.configure("TButton",font=button_font,padding=(10, 6))
    style.configure("TLabelframe",padding=8)
    style.configure("TLabelframe.Label",font=tkFont.Font(family="Helvetica",size=12,weight="bold"))
    style.configure("Readout.TLabel",font=readout_font,padding=(5,1))
    style.configure("ResultTitle.TLabel",font=result_title_font,padding=(5,1))
    style.configure("ResultValue.TLabel",font=result_font, padding=(5,1), anchor=tk.E) # Right align results

    # --- Main Layout ---
    main_frame = ttk.Frame(window, padding="10 10 10 10")
    main_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
    main_frame.columnconfigure(0, weight=3) # Camera feed area
    main_frame.columnconfigure(1, weight=1) # Controls area
    main_frame.rowconfigure(0, weight=1)

    # --- Camera Feed ---
    camera_label = ttk.Label(main_frame, text="Initializing Camera...", anchor="center", borderwidth=1, relief="sunken")
    camera_label.grid(row=0, column=0, padx=(0, 10), pady=0, sticky="nsew")

    # --- Controls Column ---
    controls_frame = ttk.Frame(main_frame)
    controls_frame.grid(row=0, column=1, padx=(0, 0), pady=0, sticky="nsew")
    controls_frame.columnconfigure(0, weight=1)
    controls_row_idx = 0

    # --- Live Readings Frame ---
    readings_frame = ttk.Labelframe(controls_frame, text=" Live Readings ", padding="10 5 10 5")
    readings_frame.grid(row=controls_row_idx, column=0, sticky="new", pady=(0, 10))
    controls_row_idx += 1
    readings_frame.columnconfigure(0, weight=0) # Label column
    readings_frame.columnconfigure(1, weight=1) # Value column
    ttk.Label(readings_frame, text="Magnetism:").grid(row=0, column=0, sticky="w", padx=(0, 10))
    magnetism_label = ttk.Label(readings_frame, text="Init...", style="Readout.TLabel", anchor="e")
    magnetism_label.grid(row=0, column=1, sticky="ew")
    ttk.Label(readings_frame, text="LDC RP:").grid(row=1, column=0, sticky="w", padx=(0, 10), pady=(3,0))
    ldc_label = ttk.Label(readings_frame, text="Init...", style="Readout.TLabel", anchor="e")
    ldc_label.grid(row=1, column=1, sticky="ew", pady=(3,0))

    # --- Actions Frame ---
    actions_frame = ttk.Labelframe(controls_frame, text=" Actions ", padding="10 5 10 10")
    actions_frame.grid(row=controls_row_idx, column=0, sticky="new", pady=(0, 10))
    controls_row_idx += 1
    actions_frame.columnconfigure(0, weight=1)
    actions_frame.columnconfigure(1, weight=1)
    # Classify Button (spans both columns)
    classify_button = ttk.Button(actions_frame, text="Capture & Classify", command=capture_and_classify)
    classify_button.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(5, 5))
    # Calibrate Button
    calibrate_button = ttk.Button(actions_frame, text="Calibrate Sensors", command=calibrate_sensors)
    calibrate_button.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(5, 5))

    # --- Classification Results Frame ---
    results_frame = ttk.Labelframe(controls_frame, text=" Classification Result ", padding="10 5 10 10")
    results_frame.grid(row=controls_row_idx, column=0, sticky="new", pady=(0, 5))
    controls_row_idx += 1
    results_frame.columnconfigure(0, weight=1) # Left column for image/labels
    results_frame.columnconfigure(1, weight=2) # Right column for values

    res_row = 0
    # Captured Image Placeholder
    result_image_label = ttk.Label(results_frame, text=" ", anchor="center", borderwidth=1, relief="sunken")
    result_image_label.grid(row=res_row, column=0, columnspan=2, pady=(5, 10), sticky="n")
    res_row += 1

    # Prediction
    ttk.Label(results_frame, text="Material:", style="ResultTitle.TLabel").grid(row=res_row, column=0, sticky="w")
    prediction_label = ttk.Label(results_frame, text="...", style="ResultValue.TLabel", anchor="e")
    prediction_label.grid(row=res_row, column=1, sticky="ew")
    res_row += 1

    # Confidence
    ttk.Label(results_frame, text="Confidence:", style="ResultTitle.TLabel").grid(row=res_row, column=0, sticky="w")
    confidence_label = ttk.Label(results_frame, text="...", style="ResultValue.TLabel", anchor="e")
    confidence_label.grid(row=res_row, column=1, sticky="ew")
    res_row += 1

    # Separator
    ttk.Separator(results_frame, orient='horizontal').grid(row=res_row, column=0, columnspan=2, sticky='ew', pady=5)
    res_row += 1

    # Sensor values used for this prediction
    ttk.Label(results_frame, text="Magnetism:", style="ResultTitle.TLabel").grid(row=res_row, column=0, sticky="w")
    result_magnetism_label = ttk.Label(results_frame, text="...", style="ResultValue.TLabel", anchor="e")
    result_magnetism_label.grid(row=res_row, column=1, sticky="ew")
    res_row += 1

    ttk.Label(results_frame, text="LDC RP (Δ):", style="ResultTitle.TLabel").grid(row=res_row, column=0, sticky="w")
    result_ldc_label = ttk.Label(results_frame, text="...", style="ResultValue.TLabel", anchor="e")
    result_ldc_label.grid(row=res_row, column=1, sticky="ew")
    res_row += 1

    # Initialize results display to empty/placeholder state
    clear_results_display()

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
    if not interpreter: # Disable classify if AI failed
        classify_button.config(state=tk.DISABLED)
        messagebox.showwarning("AI Init Failed", "AI components failed to initialize. Classification disabled.")

    # Start the update loops
    print("Starting update loops...")
    update_camera_feed()
    update_magnetism()
    update_ldc_reading()

    print("Starting Tkinter main loop...")
    window.mainloop() # Blocks here until window is closed

# --- Cleanup ---
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
        # If AI init failed, we can still run, but classification will be disabled.
        run_application()
    except Exception as e:
        print(f"FATAL ERROR in main execution: {e}")
        try:
            # Try showing error in messagebox if GUI might be up
            if window: # Check if window object exists
                 messagebox.showerror("Fatal Error", f"An unrecoverable error occurred:\n{e}")
        except Exception: pass # Avoid error if GUI isn't up
        import traceback; traceback.print_exc()
    finally:
        # Only cleanup hardware if it was potentially initialized
        if hardware_ok:
             cleanup_resources()
