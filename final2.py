# CODE 3.2.0 - AI Metal Classifier GUI with Pulse Communication
# Description: Uses pulse-based communication with Arduino.
#              - RPI GPIO 16 (Data): Sends 1-4 pulses for classification.
#              - RPI GPIO 6 (Remove): Sends 1 pulse to remove oldest from Arduino queue.
#              - RPI GPIO 26 (Ready): HIGH during RPI->Arduino pulse transmission.
#              - RPI GPIO 5 (Confirm): Receives 1-4 pulses from Arduino upon item sort.
#              - RPI GPIO 21 (Enable): Enables classification process.
# Version: 3.2.0 - Implemented pulse sending/receiving. Added RPI-side pulse listening
#              using interrupts and threading.Timer for debouncing/grouping.
#              Updated GUI and queue logic, including "Request Oldest Removal" button.

import tkinter as tk
from tkinter import ttk, Listbox, Scrollbar
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
import RPi.GPIO as GPIO # Ensure RPi.GPIO is used
import threading
from queue import Queue as ThreadQueue # For thread-safe communication from ISR to GUI

# --- AI Imports ---
try:
    # Preferred import for dedicated TFLite runtime package
    from tflite_runtime.interpreter import Interpreter
except ImportError:
    try:
        # Fallback for full TensorFlow package (less common on Pi for inference)
        from tensorflow.lite.python.interpreter import Interpreter
    except ImportError:
        print("ERROR: TensorFlow Lite Runtime is not installed.")
        print("Please install it (e.g., 'pip install tflite-runtime' or follow official Pi instructions)")
        exit()

# Try importing joblib for loading the scaler
try:
    import joblib
except ImportError:
    print("ERROR: Joblib is not installed.")
    print("Please install it: pip install joblib")
    exit()

# --- I2C/ADS1115 Imports (for Hall Sensor/Magnetism) ---
I2C_ENABLED = False # Default to False, set True if libraries import successfully
try:
    import board      # Adafruit Blinka library for hardware pins
    import busio      # For I2C communication
    import adafruit_ads1x15.ads1115 as ADS # ADS1115 library
    from adafruit_ads1x15.analog_in import AnalogIn # Helper for reading analog pins
    I2C_ENABLED = True
    print("I2C/ADS1115 libraries imported successfully.")
except ImportError:
    print("Warning: I2C/ADS1115 libraries (board, busio, adafruit-circuitpython-ads1x15) not found.")
    print("Magnetism readings will be disabled.")
except Exception as e:
    print(f"Warning: Error importing I2C/ADS1115 libraries: {e}. Magnetism readings disabled.")


# --- SPI/LDC1101 Imports ---
SPI_ENABLED = False # For spidev library itself
try:
    import spidev   # For SPI communication
    SPI_ENABLED = True
    print("SPI library (spidev) imported successfully.")
except ImportError:
    print("Warning: SPI library (spidev) not found. LDC readings will be disabled.")


# --- RPi.GPIO Setup ---
RPi_GPIO_AVAILABLE = False
try:
    # GPIO.setmode(GPIO.BCM) # Moved to initialize_hardware to avoid issues if script is re-run
    # GPIO.setwarnings(False)
    RPi_GPIO_AVAILABLE = True
    print("RPi.GPIO library imported.")
except ImportError:
    print("Warning: RPi.GPIO library not found. All GPIO functionality will be disabled.")
except Exception as e:
    print(f"Warning: Error importing RPi.GPIO: {e}. All GPIO functionality will be disabled.")


# --- GPIO Configuration ---
DATA_PIN = 16    # OUT: Classification Pulses (to Arduino Pin 8)
REMOVE_PIN = 6     # OUT: Remove Oldest Queue Item Pulse (to Arduino Pin 9)
READY_PIN = 26     # OUT: Data Ready Signal (to Arduino Pin 10)
CONFIRM_PIN = 5    # IN:  Arduino Confirmation Pulses (from Arduino Pin 13)
ENABLE_CLASSIFICATION_PIN = 21 # IN: Enable Classify Button

SORTING_GPIO_ENABLED = False # True if GPIO setup for communication is successful
ENABLE_PIN_SETUP_OK = False  # True if the enable pin (21) is set up

# --- Pulse Timing (CRITICAL - TUNE THESE VALUES) ---
PULSE_DURATION_S = 0.015  # Duration of a single pulse (e.g., 15ms)
PULSE_GAP_S = 0.040      # Gap between pulses in a sequence (e.g., 40ms)
# Timeout for grouping incoming pulses from Arduino. If no new pulse arrives within this time,
# the current group is considered complete.
PULSE_GROUP_TIMEOUT_S = 0.250 # e.g., 250ms

# ==================================
# === Constants and Configuration ===
# ==================================
NUM_SAMPLES_PER_UPDATE = 3
NUM_SAMPLES_CALIBRATION = 15
GUI_UPDATE_INTERVAL_MS = 100
CAMERA_UPDATE_INTERVAL_MS = 50
LDC_DISPLAY_BUFFER_SIZE = 5
MAGNETISM_FILTER_ALPHA = 0.2
CHECK_ENABLE_INTERVAL_MS = 250


CAMERA_INDEX = 0
DISPLAY_IMG_WIDTH = 640
DISPLAY_IMG_HEIGHT = 480
RESULT_IMG_DISPLAY_WIDTH = 280

try:
    BASE_PATH = os.path.dirname(os.path.abspath(__file__))
except NameError:
    BASE_PATH = os.getcwd()
    print(f"Warning: __file__ not defined, using current working directory as base path: {BASE_PATH}")

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

# --- GUI Globals ---
window = None
main_frame = None
live_view_frame = None
results_view_frame = None
label_font, readout_font, button_font, title_font, result_title_font, result_value_font, pred_font = (None,) * 7
lv_camera_label, lv_magnetism_label, lv_ldc_label, lv_classify_button, lv_calibrate_button = (None,) * 5
rv_image_label, rv_prediction_label, rv_confidence_label, rv_magnetism_label, rv_ldc_label, rv_classify_another_button = (None,) * 6
placeholder_img_tk = None

# --- Queue Globals ---
lv_queue_listbox = None
lv_remove_queue_button = None
rpi_queue = deque() # Local queue stores tuples: (material_label, original_pulse_count_sent_to_arduino)

classification_pending_enable = False # True if waiting for GPIO signal to re-enable classification

# --- Pulse Reception Globals ---
g_incoming_pulse_count = 0      # Counts pulses received from Arduino on CONFIRM_PIN
g_incoming_pulse_timer = None   # threading.Timer to group incoming pulses
g_pulse_reception_lock = threading.Lock() # To protect g_incoming_pulse_count
g_gui_event_queue = ThreadQueue() # Thread-safe queue to pass data from ISR context to GUI thread

# --- Material to Pulse Mapping ---
MATERIAL_TO_PULSE = {"Aluminum": 1, "Copper": 2, "Steel": 3, "Others": 4}
PULSE_TO_MATERIAL = {v: k for k, v in MATERIAL_TO_PULSE.items()}


# =========================
# === Hardware Setup ===
# =========================
def initialize_hardware():
    global camera, i2c, ads, hall_sensor, spi, ldc_initialized, CS_PIN
    global SORTING_GPIO_ENABLED, RPi_GPIO_AVAILABLE, ENABLE_PIN_SETUP_OK

    print("\n--- Initializing Hardware ---")

    # --- Camera Initialization ---
    print(f"Attempting to open camera at index {CAMERA_INDEX}...")
    try:
        camera = cv2.VideoCapture(CAMERA_INDEX)
        time.sleep(0.5) # Allow camera to initialize
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
                print(f"ADS1115 initialized. Hall sensor assigned to channel {HALL_ADC_CHANNEL}.")
            else:
                print("Warning: HALL_ADC_CHANNEL not defined, cannot create Hall sensor input.")
                hall_sensor = None
        except Exception as e:
            print(f"ERROR: Initializing I2C/ADS1115 failed: {e}")
            i2c = ads = hall_sensor = None
    else:
        print("Skipping I2C/ADS1115 setup (libraries not found or disabled).")

    # --- GPIO general setup (BCM mode) ---
    if RPi_GPIO_AVAILABLE:
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            print("GPIO BCM mode set successfully.")
        except Exception as e:
            print(f"ERROR: GPIO.setmode(GPIO.BCM) failed: {e}")
            RPi_GPIO_AVAILABLE = False # Critical failure
    else:
        print("RPi.GPIO library not available. Skipping all GPIO-dependent setups.")


    # --- SPI/LDC1101 Initialization ---
    if SPI_ENABLED and RPi_GPIO_AVAILABLE: # RPi.GPIO needed for CS pin
        print("Initializing SPI and LDC1101 (CS pin setup)...")
        try:
            GPIO.setup(CS_PIN, GPIO.OUT, initial=GPIO.HIGH)
            print(f"LDC CS Pin {CS_PIN} set as OUTPUT HIGH.")
            spi = spidev.SpiDev()
            spi.open(SPI_BUS, SPI_DEVICE)
            spi.max_speed_hz = SPI_SPEED
            spi.mode = SPI_MODE
            print(f"SPI initialized for LDC (Bus={SPI_BUS}, Device={SPI_DEVICE}).")
            if initialize_ldc1101(): # Low-level LDC init
                enable_ldc_rpmode() # Put into active RP+L mode
                print("LDC1101 initialized and RP+L mode enabled.")
            else:
                print("ERROR: LDC1101 Low-level Initialization Failed.")
                ldc_initialized = False
        except Exception as e:
            print(f"ERROR: An error occurred during SPI/LDC initialization: {e}")
            if spi: spi.close()
            spi = None
            ldc_initialized = False
    else:
        print("Skipping SPI/LDC1101 setup (SPI disabled or RPi.GPIO failed).")
        spi = None
        ldc_initialized = False


    # --- Pulse Communication & Enable GPIO Pin Initialization ---
    if RPi_GPIO_AVAILABLE:
        print("Initializing GPIO pins for Pulse Communication & Enable Signal...")
        try:
            GPIO.setup(DATA_PIN, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(REMOVE_PIN, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(READY_PIN, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(CONFIRM_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
            GPIO.setup(ENABLE_CLASSIFICATION_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

            # Setup Interrupt for Confirmation Pin (pulses from Arduino)
            # Bouncetime helps to filter out electrical noise on the pin.
            GPIO.add_event_detect(CONFIRM_PIN, GPIO.RISING,
                                  callback=confirm_pulse_isr,
                                  bouncetime=int(PULSE_DURATION_S * 1000 / 2) ) # Bounce based on half pulse duration
            print(f"Interrupt setup on CONFIRM_PIN (GPIO {CONFIRM_PIN}) for Arduino pulses.")

            SORTING_GPIO_ENABLED = True
            ENABLE_PIN_SETUP_OK = True # For the enable classification pin
            print("Pulse communication GPIO pins set. Communication is ENABLED.")
            print(f"Enable Classification Pin {ENABLE_CLASSIFICATION_PIN} set as INPUT with PULL-DOWN.")
        except Exception as e:
            print(f"ERROR: Failed to set up Pulse communication or Enable GPIO pins: {e}")
            traceback.print_exc()
            SORTING_GPIO_ENABLED = False
            ENABLE_PIN_SETUP_OK = False
    else:
        print("Skipping Pulse Communication & Enable GPIO setup (RPi.GPIO not available).")
        SORTING_GPIO_ENABLED = False
        ENABLE_PIN_SETUP_OK = False

    print("--- Hardware Initialization Complete ---")

# =========================
# === AI Model Setup ====== (Unchanged from previous versions)
# =========================
def initialize_ai():
    global interpreter, input_details, output_details, loaded_labels, numerical_scaler
    print("\n--- Initializing AI Components ---")
    ai_ready = True
    # Load Labels
    print(f"Loading labels from: {LABELS_PATH}")
    try:
        with open(LABELS_PATH, 'r') as f: loaded_labels = [line.strip() for line in f.readlines()]
        if not loaded_labels: raise ValueError("Labels file is empty.")
        print(f"Loaded {len(loaded_labels)} labels: {loaded_labels}")
    except Exception as e: print(f"ERROR: Reading labels '{LABELS_FILENAME}': {e}"); ai_ready = False

    # Load Scaler
    if ai_ready:
        print(f"Loading numerical scaler from: {SCALER_PATH}")
        try:
            numerical_scaler = joblib.load(SCALER_PATH)
            if not hasattr(numerical_scaler, 'transform') or not callable(numerical_scaler.transform):
                raise TypeError("Loaded scaler is invalid (no transform method).")
            # Basic check, assumes scaler was trained on 2 features (mag, ldc)
            expected_features = 2
            if hasattr(numerical_scaler, 'n_features_in_') and numerical_scaler.n_features_in_ != expected_features:
                print(f"ERROR: Scaler expects {numerical_scaler.n_features_in_} features, data has {expected_features}.")
                ai_ready = False
            else:
                 print(f"Numerical scaler loaded. Expects {getattr(numerical_scaler, 'n_features_in_', 'unknown')} features.")
        except Exception as e: print(f"ERROR: Loading scaler '{SCALER_FILENAME}': {e}"); ai_ready = False

    # Load TFLite Model
    if ai_ready:
        print(f"Loading TFLite model from: {MODEL_PATH}")
        try:
            interpreter = Interpreter(model_path=MODEL_PATH)
            interpreter.allocate_tensors()
            input_details = interpreter.get_input_details()
            output_details = interpreter.get_output_details()
            print("TFLite model loaded and tensors allocated.")
            # Validate model inputs/outputs against expectations
            if len(input_details) != 2: # Expecting image and numerical data
                print(f"ERROR: Model expects {len(input_details)} inputs, but code is set for 2."); ai_ready = False
            if output_details and output_details[0]['shape'][-1] != len(loaded_labels):
                print(f"ERROR: Model output size ({output_details[0]['shape'][-1]}) != labels count ({len(loaded_labels)})."); ai_ready = False
        except Exception as e: print(f"ERROR: Loading TFLite model '{MODEL_FILENAME}': {e}"); traceback.print_exc(); ai_ready = False

    if not ai_ready:
        print("--- AI Initialization Failed ---")
        interpreter = input_details = output_details = numerical_scaler = None # Clear globals
    else:
        print("--- AI Initialization Complete ---")
    return ai_ready

# =========================
# === LDC1101 Functions === (Unchanged from previous versions)
# =========================
def ldc_write_register(reg_addr, value):
    if not spi or not RPi_GPIO_AVAILABLE: return False
    success = False
    try:
        GPIO.output(CS_PIN, GPIO.LOW)
        spi.xfer2([reg_addr & 0x7F, value]) # MSB=0 for write
        GPIO.output(CS_PIN, GPIO.HIGH)
        success = True
    except Exception as e:
        print(f"Warning: LDC write error (Reg 0x{reg_addr:02X}): {e}")
        try:
            if RPi_GPIO_AVAILABLE: GPIO.output(CS_PIN, GPIO.HIGH) # Ensure CS is high
        except Exception: pass # Ignore errors during cleanup attempt
    return success

def ldc_read_register(reg_addr):
    if not spi or not RPi_GPIO_AVAILABLE: return None
    read_value = None
    try:
        GPIO.output(CS_PIN, GPIO.LOW)
        # Send register address with MSB=1 for read, then a dummy byte to clock data out
        result = spi.xfer2([reg_addr | 0x80, 0x00])
        GPIO.output(CS_PIN, GPIO.HIGH)
        read_value = result[1] # Second byte in result is the read value
    except Exception as e:
        print(f"Warning: LDC read error (Reg 0x{reg_addr:02X}): {e}")
        try:
            if RPi_GPIO_AVAILABLE: GPIO.output(CS_PIN, GPIO.HIGH)
        except Exception: pass
    return read_value

def initialize_ldc1101():
    global ldc_initialized
    ldc_initialized = False
    if not spi: print("Cannot initialize LDC1101: SPI not available."); return False
    print("Initializing LDC1101...")
    try:
        chip_id = ldc_read_register(CHIP_ID_REG)
        if chip_id is None: print("ERROR: Failed to read LDC Chip ID (read returned None)."); return False
        if chip_id != LDC_CHIP_ID: print(f"ERROR: LDC Chip ID mismatch! Expected 0x{LDC_CHIP_ID:02X}, Read:0x{chip_id:02X}"); return False
        print(f"LDC1101 Chip ID verified (0x{chip_id:02X}).")
        # Configure LDC registers for RP+L mode (example values, consult datasheet)
        regs_to_write = {
            RP_SET_REG: 0x07,      # RP_MAX, RP_MIN settings
            TC1_REG: 0x90,         # Timing constant 1
            TC2_REG: 0xA0,         # Timing constant 2
            DIG_CONFIG_REG: 0x03,  # Digital configuration
            ALT_CONFIG_REG: 0x00,  # Alternate configuration (RP+L mode)
            D_CONF_REG: 0x00,      # Data configuration
            INTB_MODE_REG: 0x00    # Interrupt mode (disabled)
        }
        for reg, val in regs_to_write.items():
            if not ldc_write_register(reg, val): print(f"ERROR: LDC write to reg 0x{reg:02X} failed."); return False
        # Put into sleep mode initially
        if not ldc_write_register(START_CONFIG_REG, SLEEP_MODE): return False
        time.sleep(0.02) # Allow settings to take effect
        print("LDC1101 Configuration successful.")
        ldc_initialized = True
        return True
    except Exception as e: print(f"ERROR: Exception during LDC1101 Initialization: {e}"); ldc_initialized = False; return False

def enable_ldc_powermode(mode):
    if not spi or not ldc_initialized: return False
    if ldc_write_register(START_CONFIG_REG, mode):
        time.sleep(0.01) # Allow mode change
        return True
    else:
        print(f"Warning: Failed to set LDC power mode register to 0x{mode:02X}.")
        return False

def enable_ldc_rpmode():
    if not spi or not ldc_initialized: print("Warning: Cannot enable LDC RP mode (SPI/LDC not ready)."); return False
    print("Enabling LDC RP+L Mode...")
    try:
        # Ensure correct configuration for RP+L mode (some might be defaults)
        if not ldc_write_register(ALT_CONFIG_REG, 0x00): return False # RP+L mode
        if not ldc_write_register(D_CONF_REG, 0x00): return False # Ensure data is RP data
        # Activate
        if enable_ldc_powermode(ACTIVE_CONVERSION_MODE):
            print("LDC RP+L Mode Enabled and Active.")
            return True
        else:
            print("Failed to set LDC to Active mode for RP+L.")
            return False
    except Exception as e: print(f"Warning: Failed to enable LDC RP mode: {e}"); return False

def get_ldc_rpdata():
    if not spi or not ldc_initialized: return None
    try:
        msb = ldc_read_register(RP_DATA_MSB_REG)
        lsb = ldc_read_register(RP_DATA_LSB_REG)
        if msb is None or lsb is None: return None # Read error
        # RP data is 16-bit
        return (msb << 8) | lsb
    except Exception as e: print(f"Warning: Exception while reading LDC RP data: {e}"); return None

# ============================
# === Sensor Reading (Avg) === (Unchanged from previous versions)
# ============================
def get_averaged_hall_voltage(num_samples=NUM_SAMPLES_PER_UPDATE):
    if not hall_sensor: return None
    readings = []
    for _ in range(num_samples):
        try: readings.append(hall_sensor.voltage)
        except Exception as e: print(f"Warning: Error reading Hall sensor: {e}. Aborting average."); return None
    return statistics.mean(readings) if readings else None

def get_averaged_rp_data(num_samples=NUM_SAMPLES_PER_UPDATE):
    if not ldc_initialized: return None
    readings = []
    for _ in range(num_samples):
        rp_value = get_ldc_rpdata()
        if rp_value is not None: readings.append(rp_value)
    return statistics.mean(readings) if readings else None

# ==========================
# === AI Processing ======== (Unchanged from previous versions)
# ==========================
def preprocess_input(image_pil, mag_mT, ldc_rp_raw):
    global numerical_scaler, input_details, interpreter
    # print("\n--- Preprocessing Input for AI ---")
    if interpreter is None or input_details is None or numerical_scaler is None:
        print("ERROR: AI Model/Scaler not initialized for preprocessing."); return None
    # Image Preprocessing
    try:
        img_resized = image_pil.resize((AI_IMG_WIDTH, AI_IMG_HEIGHT), Image.Resampling.LANCZOS)
        image_np = np.array(img_resized.convert('RGB'), dtype=np.float32)
        image_np /= 255.0 # Normalize to [0,1]
        image_input = np.expand_dims(image_np, axis=0) # Add batch dimension
        # print(f"Image preprocessed. Shape: {image_input.shape}")
    except Exception as e: print(f"ERROR: Image preprocessing failed: {e}"); return None

    # Numerical Features Preprocessing
    mag_mT_val = float(mag_mT) if mag_mT is not None else 0.0
    ldc_rp_raw_val = float(ldc_rp_raw) if ldc_rp_raw is not None else 0.0
    numerical_features = np.array([[mag_mT_val, ldc_rp_raw_val]], dtype=np.float32)
    # print(f"DEBUG Preprocess: Raw numerical features: {numerical_features}")
    try:
        with warnings.catch_warnings(): # Suppress UserWarning about feature names if scaler is from scikit-learn
            warnings.filterwarnings("ignore", message="X does not have valid feature names.*", category=UserWarning)
            scaled_numerical_features = numerical_scaler.transform(numerical_features)
        # print(f"DEBUG Preprocess: Scaled numerical features: {scaled_numerical_features}")
    except Exception as e: print(f"ERROR: Scaling numerical features failed: {e}"); return None

    # Prepare model inputs dictionary
    image_input_index, numerical_input_index = -1, -1
    image_input_dtype, numerical_input_dtype = None, None

    # Identify input tensor indices and dtypes based on shape (more robust than assuming order)
    # This assumes image input is 4D [batch, H, W, C] and numerical is 2D [batch, features]
    for detail in input_details:
        shape = detail['shape']
        if len(shape) == 4 and shape[1] == AI_IMG_HEIGHT and shape[2] == AI_IMG_WIDTH: # Image tensor
            image_input_index, image_input_dtype = detail['index'], detail['dtype']
        elif len(shape) == 2: # Numerical features tensor
            numerical_input_index, numerical_input_dtype = detail['index'], detail['dtype']
            # Sanity check number of numerical features
            if shape[1] != scaled_numerical_features.shape[1]:
                print(f"ERROR: Model expects {shape[1]} numerical features, but got {scaled_numerical_features.shape[1]}."); return None

    if image_input_index == -1 or numerical_input_index == -1:
        print("ERROR: Failed to identify image and/or numerical input tensors in the model."); return None

    # Cast to model's expected dtypes
    final_image_input = image_input.astype(image_input_dtype)
    # If model expects UINT8 for image, denormalize (model specific)
    if image_input_dtype == np.uint8 and np.max(image_input) <= 1.0: # Check if it was normalized
        final_image_input = (image_input * 255.0).astype(np.uint8)
        # print("DEBUG Preprocess: Image input rescaled to UINT8 [0-255] for model.")

    model_inputs = {
        image_input_index: final_image_input,
        numerical_input_index: scaled_numerical_features.astype(numerical_input_dtype)
    }
    # print("--- Preprocessing Complete ---")
    return model_inputs

def run_inference(model_inputs):
    global interpreter, output_details
    # print("\n--- Running AI Inference ---")
    if interpreter is None or model_inputs is None: print("ERROR: Interpreter/inputs not ready for inference."); return None
    try:
        # Set input tensors
        for index, data in model_inputs.items():
            interpreter.set_tensor(index, data)
        interpreter.invoke() # Run inference
        output_data = interpreter.get_tensor(output_details[0]['index']) # Get output tensor
        # print(f"DEBUG Inference: Raw output data shape: {output_data.shape}, values (first 5): {output_data[0][:5]}")
        # print("--- Inference Complete ---")
        return output_data
    except Exception as e: print(f"ERROR: Inference failed: {e}"); traceback.print_exc(); return None

def postprocess_output(output_data):
    global loaded_labels
    # print("\n--- Postprocessing AI Output ---")
    if output_data is None or not loaded_labels: print("ERROR: No output/labels for postprocessing."); return "Error", 0.0
    try:
        # Output is usually a 2D array [batch_size, num_classes]
        if len(output_data.shape) == 2 and output_data.shape[0] == 1:
            probabilities = output_data[0] # Get the probabilities for the first (and only) batch item
        elif len(output_data.shape) == 1: # If output is already 1D
            probabilities = output_data
        else:
            print(f"ERROR: Unexpected output data shape {output_data.shape}."); return "Shape Err", 0.0

        if len(probabilities) != len(loaded_labels):
            print(f"ERROR: Probabilities count ({len(probabilities)}) != labels count ({len(loaded_labels)})."); return "Label Mismatch", 0.0

        predicted_index = np.argmax(probabilities)
        confidence = float(probabilities[predicted_index])
        predicted_label = loaded_labels[predicted_index]
        # print(f"Final Prediction: '{predicted_label}', Confidence: {confidence:.4f}")
        # print("--- Postprocessing Complete ---")
        return predicted_label, confidence
    except Exception as e: print(f"ERROR: Postprocessing failed: {e}"); return "Post Err", 0.0


# =========================
# === Pulse Functions =====
# =========================
def _send_single_pulse(pin, duration_s):
    """Helper to send one pulse."""
    try:
        GPIO.output(pin, GPIO.HIGH)
        time.sleep(duration_s)
        GPIO.output(pin, GPIO.LOW)
    except RuntimeError as e: # Handle cases where GPIO might not be set up (e.g. during shutdown)
        print(f"GPIO runtime error sending pulse on pin {pin}: {e}")
    except Exception as e:
        print(f"Error sending single pulse on pin {pin}: {e}")
        try: GPIO.output(pin, GPIO.LOW) # Ensure pin is low on other errors
        except: pass


def send_pulses_to_arduino(pin, count, pulse_duration_s, pulse_gap_s):
    """Sends a specific number of pulses on a GPIO pin to Arduino."""
    if not RPi_GPIO_AVAILABLE or not SORTING_GPIO_ENABLED:
        print(f"Skipping send_pulses on pin {pin}: GPIO not available/enabled.")
        return
    print(f"Sending {count} pulse(s) on GPIO {pin}...")
    for i in range(count):
        _send_single_pulse(pin, pulse_duration_s)
        if i < count - 1: # If not the last pulse, add a gap
            time.sleep(pulse_gap_s)
    print(f"Pulses sent on GPIO {pin}.")

def add_to_arduino_queue_signal(material_label):
    """Sends classification pulses to Arduino via DATA_PIN, bracketed by READY_PIN."""
    if not RPi_GPIO_AVAILABLE or not SORTING_GPIO_ENABLED: return
    pulse_count = MATERIAL_TO_PULSE.get(material_label, MATERIAL_TO_PULSE["Others"])
    print(f"\n--- Sending ADD Signal to Arduino ({material_label} = {pulse_count} pulses) ---")
    try:
        GPIO.output(READY_PIN, GPIO.HIGH) # Signal Arduino: "Pulses coming on DATA_PIN"
        time.sleep(0.01) # Small delay for Arduino to see READY_PIN HIGH
        send_pulses_to_arduino(DATA_PIN, pulse_count, PULSE_DURATION_S, PULSE_GAP_S)
        time.sleep(0.01) # Hold data low briefly after last pulse
        GPIO.output(READY_PIN, GPIO.LOW)  # Signal Arduino: "Pulse transmission finished"
        print("--- ADD Signal Sent to Arduino ---")
    except Exception as e:
        print(f"ERROR: Failed to send ADD signal to Arduino: {e}")
        traceback.print_exc()
        try: GPIO.output(READY_PIN, GPIO.LOW) # Ensure READY_PIN is low on error
        except: pass

def request_oldest_removal_from_arduino_signal():
    """Sends a single pulse on REMOVE_PIN to tell Arduino to remove oldest queue item."""
    if not RPi_GPIO_AVAILABLE or not SORTING_GPIO_ENABLED: return
    print(f"\n--- Sending REMOVE OLDEST Signal to Arduino ---")
    # A single pulse on REMOVE_PIN is the signal. No READY_PIN needed for this.
    send_pulses_to_arduino(REMOVE_PIN, 1, PULSE_DURATION_S, PULSE_GAP_S)
    print("--- REMOVE OLDEST Signal Sent to Arduino ---")

def process_arduino_confirmation_pulses():
    """Called by the timer when no new pulses arrive from Arduino.
    Processes the counted pulses and updates the GUI via g_gui_event_queue."""
    global g_incoming_pulse_count, g_pulse_reception_lock, g_gui_event_queue
    with g_pulse_reception_lock:
        if g_incoming_pulse_count > 0:
            print(f"Arduino confirmation pulse group finished. Total pulses = {g_incoming_pulse_count}")
            # Put the count into the thread-safe queue for the GUI thread to process
            g_gui_event_queue.put(g_incoming_pulse_count)
            g_incoming_pulse_count = 0 # Reset for the next group of pulses
        # else:
            # print("Pulse group timer expired, but no pulses were counted.")

def confirm_pulse_isr(channel):
    """Interrupt Service Routine for CONFIRM_PIN. Counts pulses from Arduino."""
    global g_incoming_pulse_count, g_incoming_pulse_timer, g_pulse_reception_lock
    # This function runs in a separate thread context, so be careful with shared resources.
    with g_pulse_reception_lock:
        g_incoming_pulse_count += 1
        # print(f"Pulse detected from Arduino on GPIO {channel}! Current count in group: {g_incoming_pulse_count}") # Can be noisy
        # If a timer is already running (meaning we're in a pulse group), cancel it
        if g_incoming_pulse_timer is not None and g_incoming_pulse_timer.is_alive():
            g_incoming_pulse_timer.cancel()
        # Start/Restart the timer. If more pulses arrive within PULSE_GROUP_TIMEOUT_S,
        # the timer will be cancelled and restarted. If not, process_arduino_confirmation_pulses will run.
        g_incoming_pulse_timer = threading.Timer(PULSE_GROUP_TIMEOUT_S, process_arduino_confirmation_pulses)
        g_incoming_pulse_timer.start()

def check_gui_event_queue_for_updates():
    """Periodically checks the thread-safe g_gui_event_queue for messages
    from the ISR context and updates the RPI's local queue/GUI accordingly."""
    global g_gui_event_queue, window
    try:
        while not g_gui_event_queue.empty():
            received_pulse_count = g_gui_event_queue.get_nowait() # Non-blocking get
            material_confirmed = PULSE_TO_MATERIAL.get(received_pulse_count, "Unknown Material")
            print(f"GUI Event: Arduino confirmed {received_pulse_count} pulses ({material_confirmed}). Removing from RPI queue.")
            # Now, remove the corresponding item from the RPI's local queue
            remove_from_rpi_queue_on_confirmation(received_pulse_count)
    except Exception as e:
        print(f"Error processing GUI event queue: {e}")
        traceback.print_exc()
    finally:
        # Reschedule itself to run again if the window still exists
        if window and window.winfo_exists():
            window.after(200, check_gui_event_queue_for_updates) # Check every 200ms

# ==============================
# === View Switching Logic === (Unchanged from previous versions)
# ==============================
def show_live_view():
    global live_view_frame, results_view_frame
    if results_view_frame and results_view_frame.winfo_ismapped(): results_view_frame.pack_forget()
    if live_view_frame and not live_view_frame.winfo_ismapped(): live_view_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
    update_classify_button_state()

def show_results_view():
    global live_view_frame, results_view_frame
    if live_view_frame and live_view_frame.winfo_ismapped(): live_view_frame.pack_forget()
    if results_view_frame and not results_view_frame.winfo_ismapped(): results_view_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

# ======================
# === Queue Functions ===
# ======================
def update_queue_display():
    """Updates the Listbox with the current RPI queue."""
    global lv_queue_listbox, rpi_queue
    if not lv_queue_listbox: return
    lv_queue_listbox.delete(0, tk.END)
    for i, (item_label, pulse_count) in enumerate(rpi_queue):
        lv_queue_listbox.insert(tk.END, f"{i+1}: {item_label} ({pulse_count}p)")

def add_to_rpi_queue(material_label):
    """Adds an item to the RPI's local queue and updates display."""
    global rpi_queue
    pulse_count_for_material = MATERIAL_TO_PULSE.get(material_label, MATERIAL_TO_PULSE["Others"])
    rpi_queue.append((material_label, pulse_count_for_material)) # Store as tuple
    update_queue_display()
    update_remove_queue_button_state() # Enable remove button if queue has items
    print(f"Added '{material_label}' (sent as {pulse_count_for_material} pulses) to RPI queue. Current: {list(rpi_queue)}")

def remove_from_rpi_queue_on_confirmation(confirmed_pulse_count):
    """Removes the *first* item from the RPI queue that matches the confirmed_pulse_count."""
    global rpi_queue
    removed = False
    for i, (item_label, pulse_count_in_queue) in enumerate(rpi_queue):
        if pulse_count_in_queue == confirmed_pulse_count:
            del rpi_queue[i]
            removed = True
            print(f"Confirmed & Removed '{item_label}' (originally {pulse_count_in_queue} pulses) from RPI queue.")
            break # Remove only the first matching one
    if not removed:
        print(f"Warning: Arduino confirmed {confirmed_pulse_count} pulses, but no matching item found in RPI queue.")
    update_queue_display()
    update_remove_queue_button_state()

def ui_request_oldest_removal():
    """Handles the 'Request Oldest Removal' button click.
    Sends a signal to Arduino and optimistically removes the oldest from RPI's local GUI queue."""
    global rpi_queue
    if not rpi_queue:
        messagebox.showinfo("Queue Info", "The RPI's local queue is already empty.")
        return

    # Confirmation dialog before sending the signal
    if messagebox.askokcancel("Confirm Arduino Queue Removal",
                              "Send signal to Arduino to remove its OLDEST item?\n\n"
                              "This will also remove the oldest item from THIS RPI display queue."):
        print("User requested removal of oldest item. Sending signal to Arduino...")
        request_oldest_removal_from_arduino_signal() # Send the pulse signal to Arduino

        # Optimistically remove the oldest item from the RPI's local queue
        try:
            removed_item_label, removed_pulse_count = rpi_queue.popleft()
            print(f"Optimistically removed '{removed_item_label}' (was {removed_pulse_count}p) from RPI queue display.")
            update_queue_display()
            update_remove_queue_button_state()
        except IndexError:
            # Should not happen if the initial check passed, but good to have
            print("Warning: RPI queue was empty when trying to popleft after confirmation.")
            messagebox.showwarning("Queue Warning", "RPI queue was already empty.")
    else:
        print("User cancelled oldest item removal request.")


def update_remove_queue_button_state():
    """Enables/Disables the 'Request Oldest Removal' button based on RPI queue state."""
    global lv_remove_queue_button, rpi_queue
    if lv_remove_queue_button:
        state = tk.NORMAL if rpi_queue else tk.DISABLED
        lv_remove_queue_button.config(state=state)

# ======================
# === GUI Functions ===
# ======================
def create_placeholder_image(width, height, color='#E0E0E0', text="No Image"): # Unchanged
    try: img = Image.new('RGB', (width, height), color); return ImageTk.PhotoImage(img)
    except Exception as e: print(f"Warn: Placeholder fail: {e}"); return None

def clear_results_display(): # Unchanged
    global rv_image_label, rv_prediction_label, rv_confidence_label, rv_magnetism_label, rv_ldc_label, placeholder_img_tk
    if rv_image_label and placeholder_img_tk:
         rv_image_label.config(image=placeholder_img_tk, text=""); rv_image_label.img_tk = placeholder_img_tk
    elif rv_image_label:
         rv_image_label.config(image='', text="No Image"); rv_image_label.img_tk = None

    for label in [rv_prediction_label, rv_confidence_label, rv_magnetism_label, rv_ldc_label]:
        if label: label.config(text="---")

def capture_and_classify():
    global lv_classify_button, window, camera, IDLE_VOLTAGE, IDLE_RP_VALUE, interpreter
    global rv_image_label, rv_prediction_label, rv_confidence_label, rv_magnetism_label, rv_ldc_label
    global classification_pending_enable

    print("\n" + "="*10 + " Capture & Classify Triggered " + "="*10)
    if not interpreter:
        messagebox.showerror("Error", "AI Model is not initialized. Cannot classify.")
        print("Classification aborted: AI not ready."); return
    if not camera or not camera.isOpened():
        messagebox.showerror("Error", "Camera is not available. Cannot capture image.")
        print("Classification aborted: Camera not ready."); return

    # Disable button immediately
    if lv_classify_button: lv_classify_button.config(state=tk.DISABLED, text="Processing...")
    window.update_idletasks()

    ret, frame = camera.read()
    if not ret or frame is None:
        messagebox.showerror("Capture Error", "Failed to capture image from camera.")
        print("ERROR: Failed to read frame from camera."); show_live_view(); return
    try:
        img_captured_pil = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    except Exception as e:
        messagebox.showerror("Image Error", f"Failed to process captured image: {e}")
        print(f"ERROR: Failed converting captured frame to PIL Image: {e}"); show_live_view(); return

    # Sensor reading
    print(f"Reading sensors for classification ({NUM_SAMPLES_CALIBRATION} samples)...")
    avg_voltage = get_averaged_hall_voltage(num_samples=NUM_SAMPLES_CALIBRATION)
    current_mag_mT, mag_display_text, sensor_warning = None, "N/A", False
    if avg_voltage is not None:
        try:
            if abs(SENSITIVITY_V_PER_MILLITESLA) < 1e-9: raise ZeroDivisionError("Hall sens zero.")
            current_mag_mT = (avg_voltage - IDLE_VOLTAGE) / SENSITIVITY_V_PER_MILLITESLA
            mag_display_text = f"{current_mag_mT:+.3f} mT"
            if IDLE_VOLTAGE == 0.0: mag_display_text += " (NoCal)"
        except Exception as e_calc: mag_display_text = "CalcErr"; print(f"Warn: Mag calc: {e_calc}"); sensor_warning = True
    else: mag_display_text = "ReadErr"; print("ERROR: Hall read fail."); sensor_warning = True

    avg_rp_val = get_averaged_rp_data(num_samples=NUM_SAMPLES_CALIBRATION)
    current_rp_raw, ldc_display_text = None, "N/A"
    if avg_rp_val is not None:
        current_rp_raw = avg_rp_val; current_rp_int = int(round(avg_rp_val))
        delta_rp_display = current_rp_int - IDLE_RP_VALUE
        ldc_display_text = f"{current_rp_int}"
        if IDLE_RP_VALUE != 0: ldc_display_text += f" (Δ{delta_rp_display:+,})"
        else: ldc_display_text += " (NoCal)"
    else: ldc_display_text = "ReadErr"; print("ERROR: LDC read fail."); sensor_warning = True
    if sensor_warning: print("WARNING: Sensor issues may affect classification accuracy.")

    model_inputs = preprocess_input(img_captured_pil, current_mag_mT, current_rp_raw)
    if model_inputs is None:
        messagebox.showerror("AI Error", "Data preprocessing failed for AI model."); print("ERROR: Preprocessing abort."); show_live_view(); return

    output_data = run_inference(model_inputs)
    if output_data is None:
        messagebox.showerror("AI Error", "AI model inference failed."); print("ERROR: Inference abort."); show_live_view(); return

    predicted_label, confidence = postprocess_output(output_data)
    print(f"--- Classification Result: Prediction='{predicted_label}', Confidence={confidence:.1%} ---")

    # --- Add to RPI Queue and Send Pulse Signal to Arduino ---
    add_to_rpi_queue(predicted_label)
    add_to_arduino_queue_signal(predicted_label)

    # Update Results Display
    if rv_image_label:
        try:
            w, h_img = img_captured_pil.size; aspect = h_img/w if w>0 else 0.75
            display_h = int(RESULT_IMG_DISPLAY_WIDTH * aspect) if aspect > 0 else int(RESULT_IMG_DISPLAY_WIDTH * 0.75)
            img_disp = img_captured_pil.resize((RESULT_IMG_DISPLAY_WIDTH, max(1, display_h)), Image.Resampling.LANCZOS)
            img_tk = ImageTk.PhotoImage(img_disp); rv_image_label.img_tk = img_tk
            rv_image_label.config(image=img_tk, text="")
        except Exception as e:
            print(f"ERROR: Results image update: {e}")
            if placeholder_img_tk: rv_image_label.config(image=placeholder_img_tk, text="ImgErr"); rv_image_label.img_tk = placeholder_img_tk
            else: rv_image_label.config(image='', text="ImgErr"); rv_image_label.img_tk = None
    if rv_prediction_label: rv_prediction_label.config(text=f"{predicted_label}")
    if rv_confidence_label: rv_confidence_label.config(text=f"{confidence:.1%}")
    if rv_magnetism_label: rv_magnetism_label.config(text=mag_display_text)
    if rv_ldc_label: rv_ldc_label.config(text=ldc_display_text)

    show_results_view()
    print("="*10 + " Capture & Classify Complete " + "="*10 + "\n")

    classification_pending_enable = True
    update_classify_button_state() # Set to "Waiting for Enable..."
    print("Classification complete. Waiting for enable signal on GPIO", ENABLE_CLASSIFICATION_PIN)


def calibrate_sensors(): # (Unchanged logic, but uses update_classify_button_state)
    global IDLE_VOLTAGE, IDLE_RP_VALUE, window, previous_filtered_mag_mT
    global lv_calibrate_button, lv_classify_button, hall_sensor, ldc_initialized

    print("\n" + "="*10 + " Sensor Calibration Triggered " + "="*10)
    hall_avail, ldc_avail = hall_sensor is not None, ldc_initialized
    if not hall_avail and not ldc_avail:
        messagebox.showwarning("Calibration", "Neither Hall nor LDC sensor is available."); print("Aborted: No sensors."); return
    instr = "Ensure NO metal object is near sensors.\n\n"
    if hall_avail: instr += "- Hall sensor idle voltage will be measured.\n"
    if ldc_avail: instr += "- LDC sensor idle RP value will be measured.\n"
    instr += "\nClick OK to start calibration."
    if not messagebox.askokcancel("Calibration Instructions", instr): print("Calibration cancelled by user."); return

    if lv_calibrate_button: lv_calibrate_button.config(state=tk.DISABLED)
    if lv_classify_button: lv_classify_button.config(state=tk.DISABLED) # Also disable classify
    window.update_idletasks()

    hall_res, ldc_res = "Hall Sensor: N/A", "LDC Sensor: N/A"; hall_ok, ldc_ok = False, False
    if hall_avail:
        avg_v = get_averaged_hall_voltage(num_samples=NUM_SAMPLES_CALIBRATION)
        if avg_v is not None: IDLE_VOLTAGE = avg_v; hall_res = f"Hall Idle: {IDLE_VOLTAGE:.4f} V"; hall_ok = True
        else: IDLE_VOLTAGE = 0.0; hall_res = "Hall Sensor: Read Error!"
        print(hall_res)
    if ldc_avail:
        avg_rp = get_averaged_rp_data(num_samples=NUM_SAMPLES_CALIBRATION)
        if avg_rp is not None: IDLE_RP_VALUE = int(round(avg_rp)); ldc_res = f"LDC Idle RP: {IDLE_RP_VALUE}"; ldc_ok = True
        else: IDLE_RP_VALUE = 0; ldc_res = "LDC Sensor: Read Error!"
        print(ldc_res)

    previous_filtered_mag_mT = None # Reset filtered magnetism after calibration
    if lv_calibrate_button: lv_calibrate_button.config(state=tk.NORMAL)
    update_classify_button_state() # Restore classify button state considering AI/Enable state

    msg = f"Calibration Results:\n\n{hall_res}\n{ldc_res}"
    print("--- Calibration Complete ---")
    if (hall_avail and not hall_ok) or (ldc_avail and not ldc_ok): messagebox.showwarning("Calibration Warning", msg)
    else: messagebox.showinfo("Calibration Complete", msg)
    print("="*10 + " Sensor Calibration Finished " + "="*10 + "\n")


def update_camera_feed(): # Unchanged
    global lv_camera_label, window, camera, placeholder_img_tk
    if not window or not window.winfo_exists(): return
    img_tk_local = None # Use local var to avoid race conditions with global placeholder
    if camera and camera.isOpened():
        ret, frame = camera.read()
        if ret and frame is not None:
            try:
                img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                img_pil = Image.fromarray(img_rgb)
                img_pil.thumbnail((DISPLAY_IMG_WIDTH, DISPLAY_IMG_HEIGHT), Image.Resampling.NEAREST)
                img_tk_local = ImageTk.PhotoImage(img_pil)
            except Exception as e: print(f"Camera feed processing error: {e}") # pass
    if lv_camera_label:
        if img_tk_local:
            lv_camera_label.img_tk = img_tk_local # Keep reference
            lv_camera_label.configure(image=img_tk_local, text="")
        else: # No image from camera
            current_img = lv_camera_label.cget("image")
            # Update to placeholder only if not already showing it or if placeholder exists
            if placeholder_img_tk and str(current_img) != str(placeholder_img_tk):
                lv_camera_label.configure(image=placeholder_img_tk, text="No Feed")
                lv_camera_label.img_tk = placeholder_img_tk
            elif not placeholder_img_tk and lv_camera_label.cget("text") != "Camera Failed":
                 lv_camera_label.configure(image='', text="Camera Failed")
                 lv_camera_label.img_tk = None
    if window and window.winfo_exists(): window.after(CAMERA_UPDATE_INTERVAL_MS, update_camera_feed)

def update_magnetism(): # Unchanged
    global lv_magnetism_label, window, previous_filtered_mag_mT, IDLE_VOLTAGE, hall_sensor
    if not window or not window.winfo_exists(): return
    display_text = "N/A"
    if hall_sensor:
        avg_v = get_averaged_hall_voltage(num_samples=NUM_SAMPLES_PER_UPDATE)
        if avg_v is not None:
            try:
                if abs(SENSITIVITY_V_PER_MILLITESLA) < 1e-9: raise ZeroDivisionError("Sens0")
                raw_mT = (avg_v - IDLE_VOLTAGE) / SENSITIVITY_V_PER_MILLITESLA
                if previous_filtered_mag_mT is None: previous_filtered_mag_mT = raw_mT # Initialize filter
                # Simple Exponential Moving Average (EMA) filter
                filt_mT = (MAGNETISM_FILTER_ALPHA * raw_mT) + ((1-MAGNETISM_FILTER_ALPHA)*previous_filtered_mag_mT)
                previous_filtered_mag_mT = filt_mT
                # Display in µT if small, else mT
                if abs(filt_mT) < 0.1: display_text = f"{filt_mT*1000:+.1f}µT"
                else: display_text = f"{filt_mT:+.2f}mT"
                if IDLE_VOLTAGE == 0.0: display_text += "(NoCal)" # Indicate if not calibrated
            except Exception: display_text = "CalcErr"; previous_filtered_mag_mT=None
        else: display_text = "ReadErr"; previous_filtered_mag_mT=None
    if lv_magnetism_label and lv_magnetism_label.cget("text") != display_text: lv_magnetism_label.config(text=display_text)
    if window and window.winfo_exists(): window.after(GUI_UPDATE_INTERVAL_MS, update_magnetism)

def update_ldc_reading(): # Unchanged
    global lv_ldc_label, window, RP_DISPLAY_BUFFER, IDLE_RP_VALUE, ldc_initialized
    if not window or not window.winfo_exists(): return
    display_text = "N/A"
    if ldc_initialized:
        avg_rp = get_averaged_rp_data(num_samples=NUM_SAMPLES_PER_UPDATE)
        if avg_rp is not None:
            RP_DISPLAY_BUFFER.append(avg_rp)
            if RP_DISPLAY_BUFFER: # Ensure buffer has data
                # Average readings in buffer for smoother display
                cur_rp = int(round(statistics.mean(RP_DISPLAY_BUFFER)))
                delta = cur_rp - IDLE_RP_VALUE
                display_text = f"{cur_rp}"
                if IDLE_RP_VALUE != 0: display_text += f"(Δ{delta:+,})"
                else: display_text += "(NoCal)"
            else: display_text = "Buffer..." # Waiting for buffer to fill
        else: display_text = "ReadErr"
    if lv_ldc_label and lv_ldc_label.cget("text") != display_text: lv_ldc_label.config(text=display_text)
    if window and window.winfo_exists(): window.after(GUI_UPDATE_INTERVAL_MS, update_ldc_reading)

def update_classify_button_state():
    """Helper to set the classify button state correctly based on AI readiness and enable signal."""
    global lv_classify_button, classification_pending_enable, interpreter
    if not lv_classify_button: return

    if classification_pending_enable:
        lv_classify_button.config(state=tk.DISABLED, text="Waiting for Enable...")
    elif interpreter: # AI is ready
        lv_classify_button.config(state=tk.NORMAL, text="Capture & Classify")
    else: # AI not ready
        lv_classify_button.config(state=tk.DISABLED, text="Classify (AI Failed)")

def check_enable_signal():
    """Periodically checks the ENABLE_CLASSIFICATION_PIN."""
    global window, lv_classify_button, classification_pending_enable, ENABLE_PIN_SETUP_OK, RPi_GPIO_AVAILABLE

    if not window or not window.winfo_exists(): return

    if classification_pending_enable and ENABLE_PIN_SETUP_OK and RPi_GPIO_AVAILABLE:
        try:
            if GPIO.input(ENABLE_CLASSIFICATION_PIN) == GPIO.HIGH:
                print(f"Enable signal DETECTED on GPIO {ENABLE_CLASSIFICATION_PIN}!")
                classification_pending_enable = False
                update_classify_button_state() # Update button state based on AI readiness
        except Exception as e:
            print(f"Error reading Enable Classification Pin {ENABLE_CLASSIFICATION_PIN}: {e}")
            # Optionally disable further checks or handle error. For now, it will keep trying.

    # Always reschedule the check if the window exists
    if window.winfo_exists():
        window.after(CHECK_ENABLE_INTERVAL_MS, check_enable_signal)


# ======================
# === GUI Setup ========
# ======================
def setup_gui():
    global window, main_frame, placeholder_img_tk, live_view_frame, results_view_frame
    global lv_camera_label, lv_magnetism_label, lv_ldc_label, lv_classify_button, lv_calibrate_button
    global rv_image_label, rv_prediction_label, rv_confidence_label, rv_magnetism_label, rv_ldc_label, rv_classify_another_button
    global label_font, readout_font, button_font, title_font, result_title_font, result_value_font, pred_font
    global lv_queue_listbox, lv_remove_queue_button

    print("Setting up GUI...")
    window = tk.Tk()
    window.title("AI Metal Classifier v3.2.0 (Pulse Comm)")
    window.geometry("800x600") # Adjust as needed
    style = ttk.Style()
    available_themes = style.theme_names(); style.theme_use('clam' if 'clam' in available_themes else 'default')

    try: # Font setup
        font_family = "DejaVu Sans" # Or "Arial", "Helvetica", "Calibri"
        title_font = tkFont.Font(family=font_family, size=16, weight="bold")
        label_font = tkFont.Font(family=font_family, size=10)
        readout_font = tkFont.Font(family=font_family+" Mono", size=12, weight="bold") # Monospaced for sensor values
        button_font = tkFont.Font(family=font_family, size=10, weight="bold")
        result_title_font = tkFont.Font(family=font_family, size=11, weight="bold")
        result_value_font = tkFont.Font(family=font_family+" Mono", size=12, weight="bold")
        pred_font = tkFont.Font(family=font_family, size=16, weight="bold")
    except tk.TclError: # Fallback if preferred fonts aren't found
        print("Warning: DejaVu fonts not found, using Tkinter default fonts.")
        title_font=tkFont.nametofont("TkHeadingFont"); label_font=tkFont.nametofont("TkTextFont")
        readout_font=tkFont.nametofont("TkFixedFont"); button_font=tkFont.nametofont("TkDefaultFont")
        result_title_font=tkFont.nametofont("TkDefaultFont"); result_value_font=tkFont.nametofont("TkFixedFont")
        pred_font=tkFont.nametofont("TkHeadingFont")

    # Configure styles for ttk widgets
    style.configure("TLabel", font=label_font, padding=2)
    style.configure("TButton", font=button_font, padding=(8,5))
    style.configure("Readout.TLabel", font=readout_font, foreground="#0000AA") # Blue for readouts
    style.configure("ResultValue.TLabel", font=result_value_font, foreground="#0000AA")
    style.configure("Prediction.TLabel", font=pred_font, foreground="#AA0000") # Red for prediction

    main_frame = ttk.Frame(window, padding="5 5 5 5"); main_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
    main_frame.rowconfigure(0, weight=1); main_frame.columnconfigure(0, weight=1) # Make main_frame expandable

    # --- Live View Frame ---
    live_view_frame = ttk.Frame(main_frame, padding="5 5 5 5")
    live_view_frame.columnconfigure(0, weight=3) # Camera feed gets more space
    live_view_frame.columnconfigure(1, weight=1) # Controls column
    live_view_frame.rowconfigure(0, weight=1)    # Allow camera feed row to expand

    lv_camera_label = ttk.Label(live_view_frame, text="Initializing Camera...", anchor="center", borderwidth=1, relief="sunken", background="#CCCCCC")
    lv_camera_label.grid(row=0, column=0, padx=(0, 5), pady=0, sticky="nsew")

    lv_controls_frame = ttk.Frame(live_view_frame)
    lv_controls_frame.grid(row=0, column=1, sticky="nsew", padx=(5,0))
    lv_controls_frame.columnconfigure(0, weight=1) # Allow content within controls to expand

    # Live Readings
    lv_readings_frame = ttk.Labelframe(lv_controls_frame, text=" Live Readings ", padding="8 4 8 4")
    lv_readings_frame.grid(row=0, column=0, sticky="new", pady=(0, 10))
    lv_readings_frame.columnconfigure(1, weight=1) # Allow readout values to expand
    ttk.Label(lv_readings_frame, text="Magnetism:").grid(row=0, column=0, sticky="w", padx=(0, 8))
    lv_magnetism_label = ttk.Label(lv_readings_frame, text="Init...", style="Readout.TLabel"); lv_magnetism_label.grid(row=0, column=1, sticky="ew")
    ttk.Label(lv_readings_frame, text="LDC (Delta):").grid(row=1, column=0, sticky="w", padx=(0, 8), pady=(2,0))
    lv_ldc_label = ttk.Label(lv_readings_frame, text="Init...", style="Readout.TLabel"); lv_ldc_label.grid(row=1, column=1, sticky="ew", pady=(2,0))

    # Actions
    lv_actions_frame = ttk.Labelframe(lv_controls_frame, text=" Actions ", padding="8 4 8 8")
    lv_actions_frame.grid(row=1, column=0, sticky="new", pady=(0,10))
    lv_actions_frame.columnconfigure(0, weight=1) # Allow buttons to expand
    lv_classify_button = ttk.Button(lv_actions_frame, text="Capture & Classify", command=capture_and_classify)
    lv_classify_button.grid(row=0, column=0, sticky="ew", pady=(4,4))
    lv_calibrate_button = ttk.Button(lv_actions_frame, text="Calibrate Sensors", command=calibrate_sensors)
    lv_calibrate_button.grid(row=1, column=0, sticky="ew", pady=(4,4))

    # Sorter Queue Display
    lv_queue_frame = ttk.Labelframe(lv_controls_frame, text=" Sorter Queue (RPI View) ", padding="8 4 8 8")
    lv_queue_frame.grid(row=2, column=0, sticky="nsew", pady=(0,10)) # Allow this frame to expand vertically
    lv_queue_frame.columnconfigure(0, weight=1) # Listbox expands horizontally
    lv_queue_frame.rowconfigure(0, weight=1)    # Listbox expands vertically
    lv_queue_listbox = Listbox(lv_queue_frame, height=6, font=label_font, relief="sunken", borderwidth=1)
    lv_queue_listbox.grid(row=0, column=0, sticky="nsew", pady=(0, 5))
    # Scrollbar for the queue listbox
    queue_scrollbar = Scrollbar(lv_queue_frame, orient="vertical", command=lv_queue_listbox.yview)
    queue_scrollbar.grid(row=0, column=1, sticky="ns", pady=(0,5))
    lv_queue_listbox.config(yscrollcommand=queue_scrollbar.set)
    # Button to request removal of oldest item from Arduino's queue
    lv_remove_queue_button = ttk.Button(lv_queue_frame, text="Request Arduino Remove Oldest", command=ui_request_oldest_removal, state=tk.DISABLED)
    lv_remove_queue_button.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(4,0))
    lv_controls_frame.rowconfigure(2, weight=1) # Allow queue frame to take available vertical space

    # --- Results View Frame --- (Centered content)
    results_view_frame = ttk.Frame(main_frame, padding="10 10 10 10")
    results_view_frame.rowconfigure(0, weight=1); results_view_frame.rowconfigure(1, weight=0); results_view_frame.rowconfigure(2, weight=1) # Center row 1
    results_view_frame.columnconfigure(0, weight=1); results_view_frame.columnconfigure(1, weight=0); results_view_frame.columnconfigure(2, weight=1) # Center col 1

    rv_content_frame = ttk.Frame(results_view_frame) # This frame will hold all results content
    rv_content_frame.grid(row=1, column=1, sticky="") # Place in the center cell

    ttk.Label(rv_content_frame, text="Classification Result", font=title_font).grid(row=0, column=0, columnspan=2, pady=(5, 15))
    placeholder_h = int(RESULT_IMG_DISPLAY_WIDTH * 0.75)
    placeholder_img_tk = create_placeholder_image(RESULT_IMG_DISPLAY_WIDTH, placeholder_h) # Create placeholder once
    rv_image_label = ttk.Label(rv_content_frame, anchor="center", borderwidth=1, relief="sunken")
    if placeholder_img_tk: rv_image_label.config(image=placeholder_img_tk); rv_image_label.img_tk = placeholder_img_tk
    else: rv_image_label.config(text="Image Area", width=30, height=15) # Fallback if placeholder fails
    rv_image_label.grid(row=1, column=0, columnspan=2, pady=(0, 15))

    rv_details_frame = ttk.Frame(rv_content_frame)
    rv_details_frame.grid(row=2, column=0, columnspan=2, pady=(0,15))
    rv_details_frame.columnconfigure(1, weight=1); res_row = 0 # For easy row incrementing
    ttk.Label(rv_details_frame, text="Material:", font=result_title_font).grid(row=res_row, column=0, sticky="w", padx=(0,5))
    rv_prediction_label = ttk.Label(rv_details_frame, text="---", style="Prediction.TLabel"); rv_prediction_label.grid(row=res_row, column=1, sticky="ew", padx=5); res_row += 1
    ttk.Label(rv_details_frame, text="Confidence:", font=result_title_font).grid(row=res_row, column=0, sticky="w", padx=(0,5), pady=(3,0))
    rv_confidence_label = ttk.Label(rv_details_frame, text="---", style="ResultValue.TLabel"); rv_confidence_label.grid(row=res_row, column=1, sticky="ew", padx=5, pady=(3,0)); res_row += 1
    ttk.Separator(rv_details_frame, orient='horizontal').grid(row=res_row, column=0, columnspan=2, sticky='ew', pady=8); res_row += 1
    ttk.Label(rv_details_frame, text="Sensor Values Used:", font=result_title_font).grid(row=res_row, column=0, columnspan=2, sticky="w", pady=(0,3)); res_row += 1
    ttk.Label(rv_details_frame, text=" Magnetism:", font=result_title_font).grid(row=res_row, column=0, sticky="w", padx=(5,5)) # Indent sensor values
    rv_magnetism_label = ttk.Label(rv_details_frame, text="---", style="ResultValue.TLabel"); rv_magnetism_label.grid(row=res_row, column=1, sticky="ew", padx=5); res_row += 1
    ttk.Label(rv_details_frame, text=" LDC Reading:", font=result_title_font).grid(row=res_row, column=0, sticky="w", padx=(5,5))
    rv_ldc_label = ttk.Label(rv_details_frame, text="---", style="ResultValue.TLabel"); rv_ldc_label.grid(row=res_row, column=1, sticky="ew", padx=5); res_row += 1

    rv_classify_another_button = ttk.Button(rv_content_frame, text="<< Classify Another", command=show_live_view)
    rv_classify_another_button.grid(row=3, column=0, columnspan=2, pady=(15, 5))

    clear_results_display()
    show_live_view() # This will set initial classify button state via update_classify_button_state()
    update_queue_display() # Show initial (empty) queue
    update_remove_queue_button_state() # Ensure remove button is initially disabled
    print("GUI setup complete.")

# ==========================
# === Main Execution =======
# ==========================
def run_application():
    global window, lv_camera_label, lv_magnetism_label, lv_ldc_label, lv_classify_button, lv_calibrate_button
    global interpreter, camera, hall_sensor, ldc_initialized

    print("Setting up GUI...")
    try: setup_gui()
    except Exception as e:
        print(f"FATAL ERROR: Failed to set up GUI: {e}"); traceback.print_exc()
        # Try to show a simple Tk error box if GUI setup fails catastrophically
        try:
            root_err = tk.Tk(); root_err.withdraw() # Hide main error window
            messagebox.showerror("GUI Setup Error", f"GUI Initialization Failed:\n{e}\n\nCheck console for details."); root_err.destroy()
        except Exception: pass # If even this fails, console is the only hope
        return # Cannot continue without GUI

    # Set initial state of some GUI elements based on hardware/AI readiness
    update_classify_button_state() # Initial state of classify button
    if not camera and lv_camera_label:
        if placeholder_img_tk: lv_camera_label.configure(text="Camera Failed", image=placeholder_img_tk)
        else: lv_camera_label.configure(text="Camera Failed", image='')
    if not hall_sensor and lv_magnetism_label: lv_magnetism_label.config(text="N/A (No Sensor)")
    if not ldc_initialized and lv_ldc_label: lv_ldc_label.config(text="N/A (No Sensor)")
    if not (hall_sensor or ldc_initialized) and lv_calibrate_button:
        lv_calibrate_button.config(state=tk.DISABLED, text="Calibrate (No Sensors)")

    print("Starting GUI update loops and signal checks...")
    update_camera_feed()    # Start camera feed updates
    update_magnetism()      # Start magnetism sensor updates
    update_ldc_reading()    # Start LDC sensor updates
    check_enable_signal()   # Start polling for the enable classification signal
    check_gui_event_queue_for_updates() # Start checking for Arduino confirmation pulses

    print("Starting Tkinter main loop... (Press Ctrl+C in console to exit if GUI hangs)")
    try:
        window.protocol("WM_DELETE_WINDOW", on_closing) # Handle window close button
        window.mainloop()
    except KeyboardInterrupt: # Allow Ctrl+C to quit from console if GUI is responsive
        print("\nKeyboard interrupt detected by Tkinter main loop. Exiting.")
        on_closing(force_quit=True) # Attempt cleanup
    except Exception as e:
        print(f"ERROR: Exception in Tkinter main loop: {e}")
        traceback.print_exc()
    print("Tkinter main loop finished.")

def on_closing(force_quit=False):
    global window, g_incoming_pulse_timer
    print("Window close requested.")
    do_quit = False
    if force_quit:
        do_quit = True
    elif messagebox.askokcancel("Quit", "Do you want to quit the AI Metal Classifier application?"):
        do_quit = True

    if do_quit:
        print("Proceeding with shutdown...")
        # Stop the pulse timer if it's running
        if g_incoming_pulse_timer is not None and g_incoming_pulse_timer.is_alive():
            print("Cancelling incoming pulse timer...")
            g_incoming_pulse_timer.cancel()
        if window:
            print("Quitting Tkinter window...")
            window.quit() # Stops mainloop, allows finally block in main to run
    else:
        print("Shutdown cancelled by user.")


# ==========================
# === Cleanup Resources ====
# ==========================
def cleanup_resources():
    print("\n--- Cleaning up resources ---")
    global camera, spi, ldc_initialized, RPi_GPIO_AVAILABLE, g_incoming_pulse_timer

    # Cancel the pulse timer if it's still active
    if g_incoming_pulse_timer is not None and g_incoming_pulse_timer.is_alive():
        print("Final cancel of incoming pulse timer.")
        g_incoming_pulse_timer.cancel()
        # It's good practice to wait for the timer thread to finish if it was started
        # However, trying to join here can hang if the timer function itself is stuck.
        # For simplicity, we'll just cancel.

    if camera and camera.isOpened():
        try: print("Releasing camera..."); camera.release(); print("Camera released.")
        except Exception as e: print(f"Warning: Error releasing camera: {e}")

    if spi:
        try:
            if ldc_initialized and RPi_GPIO_AVAILABLE: # Check GPIO available for CS pin
                print("Putting LDC1101 to sleep...")
                if not enable_ldc_powermode(SLEEP_MODE): print("Note: Failed to send LDC sleep command.")
                else: print("LDC sleep command sent.")
        except Exception as ldc_e: print(f"Note: Error sending LDC sleep cmd: {ldc_e}")
        finally:
            try: print("Closing LDC SPI..."); spi.close(); print("LDC SPI closed.")
            except Exception as e: print(f"Warning: Error closing LDC SPI: {e}")

    if RPi_GPIO_AVAILABLE:
        try:
            # GPIO.cleanup() handles removing event detects
            print(f"Cleaning up GPIO pins..."); GPIO.cleanup(); print("GPIO cleaned up.")
        except RuntimeError as e: print(f"Note: GPIO cleanup runtime error (possibly already cleaned or not set up): {e}")
        except Exception as e: print(f"Warning: Error during GPIO cleanup: {e}")
    else:
        print("Note: RPi.GPIO not available or not set up, skipping GPIO cleanup.")
    print("--- Cleanup complete ---")

# ==========================
# === Main Entry Point =====
# ==========================
if __name__ == '__main__':
    print("="*30 + "\n Starting AI Metal Classifier (v3.2.0 - Pulse Comm) \n" + "="*30)
    hw_init_attempted = False
    try:
        initialize_hardware(); hw_init_attempted = True
        initialize_ai()
        run_application()
    except KeyboardInterrupt: # Catch Ctrl+C if it happens outside Tkinter's main loop
        print("\nKeyboard interrupt detected during script execution. Exiting application.")
    except Exception as e:
        print("\n" + "="*30 + f"\nFATAL UNHANDLED ERROR in main execution: {e}\n" + "="*30)
        traceback.print_exc()
        if 'window' in globals() and window and window.winfo_exists(): # If GUI started, try to show error
            try: messagebox.showerror("Fatal Application Error", f"Unrecoverable error:\n\n{e}\n\nPlease check console.")
            except Exception: pass # If even messagebox fails
    finally:
        # Ensure window is destroyed if it exists, which might not happen if mainloop didn't stop cleanly
        if 'window' in globals() and window:
            try:
                if window.winfo_exists():
                    print("Ensuring Tkinter window is destroyed...")
                    window.destroy() # Force destroy if quit() wasn't enough or mainloop errored
                    print("Tkinter window destroyed.")
            except tk.TclError: print("Note: Tkinter window likely already destroyed.")
            except Exception as e_destroy: print(f"Warning: Error destroying Tkinter window during final cleanup: {e_destroy}")

        if hw_init_attempted: # Only cleanup if hardware was attempted to be initialized
            cleanup_resources()
        else:
            print("Skipping resource cleanup as hardware initialization was not fully attempted.")
        print("\nApplication finished.\n" + "="*30)
