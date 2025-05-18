# CODE 3.0.11 - AI Metal Classifier GUI with Results Page and Sorting Output
# Description: Displays live sensor data and camera feed.
#              Captures image and sensor readings, classifies metal using a TFLite model,
#              displays the results on a dedicated page, and sends a sorting signal via GPIO.
# Version: 3.0.11 - Integrated GPIO signaling for sorting mechanism based on classification.
#                  Removed "None" category from sorting, merged with "Others".
#                  Added SORTING_GPIO_ENABLED flag and relevant setup/cleanup.
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

# --- AI Imports ---
# Try importing the TFLite runtime interpreter
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
    print("Ensure Adafruit Blinka is installed and configured for your Pi.")
    print("Magnetism readings will be disabled.")
except NotImplementedError:
    print("Warning: I2C not supported on this platform according to Blinka. Magnetism readings disabled.")
except Exception as e:
    print(f"Warning: Error importing I2C/ADS1115 libraries: {e}. Magnetism readings disabled.")

# --- SPI/LDC1101 Imports (for Inductive Sensor) ---
SPI_ENABLED = False # Default to False, set True if libraries import successfully
try:
    import spidev   # For SPI communication
    import RPi.GPIO as GPIO # For controlling Chip Select pin and sorting pins
    SPI_ENABLED = True # Indicates spidev is available
    # RPi.GPIO availability for sorting is checked separately
    print("SPI library (spidev) imported successfully.")
    print("RPi.GPIO library imported successfully (needed for LDC CS and Sorting).")
except ImportError:
    # Separate checks for spidev and RPi.GPIO for clarity
    try:
        import spidev
        SPI_ENABLED = True
        print("SPI library (spidev) imported successfully.")
        print("Warning: RPi.GPIO library not found. LDC CS control and Sorting will be disabled.")
    except ImportError:
        print("Warning: SPI library (spidev) not found. LDC readings will be disabled.")
        print("Warning: RPi.GPIO library likely also not found or an issue exists. LDC CS and Sorting will be disabled.")
    # If RPi.GPIO failed but spidev didn't, it's already handled for SPI_ENABLED.
    # If RPi.GPIO is the one missing, sorting pins won't work.
except RuntimeError:
    print("Warning: RPi.GPIO library likely requires root privileges (sudo). LDC and Sorting may fail if not run as root.")
except Exception as e:
    print(f"Warning: Error importing SPI/GPIO libraries: {e}. LDC/Sorting readings/control disabled.")


# --- Sorting GPIO Configuration ---
SORTING_GPIO_ENABLED = False # Default to False, set True if RPi.GPIO is available and setup succeeds
SORTING_DATA_PIN_LSB = 16 # BCM Pin for LSB of sorting data
SORTING_DATA_PIN_MID = 6  # BCM Pin for MID/MSB of sorting data (2-bit signal)
SORTING_DATA_READY_PIN = 26 # BCM Pin to signal data is ready for sorter

# ==================================
# === Constants and Configuration ===
# ==================================

# --- Accuracy/Stability/Speed ---
NUM_SAMPLES_PER_UPDATE = 3
NUM_SAMPLES_CALIBRATION = 15
GUI_UPDATE_INTERVAL_MS = 100
CAMERA_UPDATE_INTERVAL_MS = 50
LDC_DISPLAY_BUFFER_SIZE = 5
MAGNETISM_FILTER_ALPHA = 0.2

# --- Camera ---
CAMERA_INDEX = 0
DISPLAY_IMG_WIDTH = 640
DISPLAY_IMG_HEIGHT = 480
RESULT_IMG_DISPLAY_WIDTH = 280

# --- AI Model Configuration ---
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

# --- Hall Sensor (ADS1115 Configuration) ---
HALL_ADC_CHANNEL = ADS.P0 if I2C_ENABLED else None
SENSITIVITY_V_PER_TESLA = 0.0004
SENSITIVITY_V_PER_MILLITESLA = SENSITIVITY_V_PER_TESLA * 1000
IDLE_VOLTAGE = 0.0

# --- Inductive Sensor (LDC1101 Configuration) ---
SPI_BUS = 0
SPI_DEVICE = 0
SPI_SPEED = 500000
SPI_MODE = 0b00
CS_PIN = 8 # BCM Pin for LDC Chip Select

LDC_CHIP_ID = 0xD4
START_CONFIG_REG = 0x0B
RP_SET_REG = 0x01
TC1_REG = 0x02
TC2_REG = 0x03
DIG_CONFIG_REG = 0x04
ALT_CONFIG_REG = 0x05
D_CONF_REG = 0x0C
INTB_MODE_REG = 0x0A
RP_DATA_MSB_REG = 0x22
RP_DATA_LSB_REG = 0x21
CHIP_ID_REG = 0x3F
ACTIVE_CONVERSION_MODE = 0x00
SLEEP_MODE = 0x01

# --- Calibration Defaults ---
IDLE_RP_VALUE = 0

# ============================
# === Global Objects/State ===
# ============================
camera = None
i2c = None
ads = None
hall_sensor = None
spi = None # For LDC
ldc_initialized = False

interpreter = None
input_details = None
output_details = None
loaded_labels = []
numerical_scaler = None

RP_DISPLAY_BUFFER = deque(maxlen=LDC_DISPLAY_BUFFER_SIZE)
previous_filtered_mag_mT = None

window = None
main_frame = None
live_view_frame = None
results_view_frame = None

label_font, readout_font, button_font, title_font, result_title_font, result_value_font = (None,) * 6

lv_camera_label, lv_magnetism_label, lv_ldc_label, lv_classify_button, lv_calibrate_button = (None,) * 5
rv_image_label, rv_prediction_label, rv_confidence_label, rv_magnetism_label, rv_ldc_label, rv_classify_another_button = (None,) * 6
placeholder_img_tk = None

# =========================
# === Hardware Setup ===
# =========================
def initialize_hardware():
    global camera, i2c, ads, hall_sensor, spi, ldc_initialized, CS_PIN
    global SORTING_GPIO_ENABLED # For sorting mechanism

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
                print(f"ADS1115 initialized. Hall sensor assigned to channel {HALL_ADC_CHANNEL}.")
            else:
                print("Warning: HALL_ADC_CHANNEL not defined, cannot create Hall sensor input.")
                hall_sensor = None
        except Exception as e:
            print(f"ERROR: Initializing I2C/ADS1115 failed: {e}")
            i2c = ads = hall_sensor = None
    else:
        print("Skipping I2C/ADS1115 setup (libraries not found or disabled).")

    # --- GPIO and SPI/LDC1101 Initialization ---
    # GPIO setup is needed for both LDC (CS pin) and Sorting (data pins)
    gpio_setup_done_for_ldc_or_sorting = False
    try:
        # Test if RPi.GPIO is imported and usable for LDC CS pin and/or Sorting pins
        if 'GPIO' in globals() and hasattr(GPIO, 'setmode'):
            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BCM) # Use BCM pin numbering scheme
            print("GPIO general setup (BCM mode) successful.")
            gpio_setup_done_for_ldc_or_sorting = True # Base GPIO setup okay
        else:
            print("Warning: RPi.GPIO not available. LDC CS and Sorting will be disabled.")
            # Ensure SPI_ENABLED reflects this if it relies on GPIO for CS
            # For LDC, if SPI_ENABLED was true but GPIO is not, LDC cannot work.
            # The 'initialize_ldc1101' function relies on GPIO.output.
            # No specific action needed here as ldc_write/read will fail gracefully.

    except Exception as e:
        print(f"ERROR: Initial GPIO setup (BCM mode) failed: {e}")
        gpio_setup_done_for_ldc_or_sorting = False


    # --- SPI/LDC1101 Initialization (relies on SPI_ENABLED and gpio_setup_done_for_ldc_or_sorting for CS) ---
    if SPI_ENABLED and gpio_setup_done_for_ldc_or_sorting: # Need GPIO for CS
        print("Initializing SPI and LDC1101 (CS pin setup)...")
        try:
            GPIO.setup(CS_PIN, GPIO.OUT, initial=GPIO.HIGH)
            print(f"LDC CS Pin {CS_PIN} set as OUTPUT HIGH.")

            spi = spidev.SpiDev()
            spi.open(SPI_BUS, SPI_DEVICE)
            spi.max_speed_hz = SPI_SPEED
            spi.mode = SPI_MODE
            print(f"SPI initialized for LDC (Bus={SPI_BUS}, Device={SPI_DEVICE}).")

            if initialize_ldc1101():
                enable_ldc_rpmode()
                print("LDC1101 initialized and RP+L mode enabled.")
                # ldc_initialized is set True inside initialize_ldc1101
            else:
                print("ERROR: LDC1101 Initialization Failed.")
                ldc_initialized = False
        except Exception as e:
            print(f"ERROR: An error occurred during SPI/LDC initialization: {e}")
            if spi: spi.close()
            spi = None
            ldc_initialized = False
            # GPIO cleanup for CS_PIN might be needed if only part of setup failed
            # However, a general GPIO.cleanup() is done at the end of the script.
    elif SPI_ENABLED and not gpio_setup_done_for_ldc_or_sorting:
        print("Skipping LDC1101 setup because GPIO for CS pin is not available.")
        spi = None # Ensure spi object is None if it cannot be used
        ldc_initialized = False
    else: # SPI_ENABLED is False
        print("Skipping SPI/LDC1101 setup (spidev library not found or disabled).")


    # --- Sorting GPIO Pin Initialization ---
    if gpio_setup_done_for_ldc_or_sorting: # If basic RPi.GPIO setup was okay
        print("Initializing GPIO pins for Sorting Mechanism...")
        try:
            GPIO.setup(SORTING_DATA_PIN_LSB, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(SORTING_DATA_PIN_MID, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(SORTING_DATA_READY_PIN, GPIO.OUT, initial=GPIO.LOW)
            SORTING_GPIO_ENABLED = True # Set flag if setup is successful
            print(f"Sorting GPIO pins ({SORTING_DATA_PIN_LSB}, {SORTING_DATA_PIN_MID}, {SORTING_DATA_READY_PIN}) set as OUTPUT LOW.")
        except Exception as e:
            print(f"ERROR: Failed to set up sorting GPIO pins: {e}")
            SORTING_GPIO_ENABLED = False
    else:
        print("Skipping Sorting GPIO setup (RPi.GPIO not available or initial setup failed).")
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
        if not loaded_labels: raise ValueError("Labels file is empty.")
        print(f"Loaded {len(loaded_labels)} labels: {loaded_labels}")
    except Exception as e:
        print(f"ERROR: Failed to read labels file '{LABELS_FILENAME}': {e}"); ai_ready = False

    if ai_ready:
        try:
            numerical_scaler = joblib.load(SCALER_PATH)
            if not hasattr(numerical_scaler, 'transform'): raise TypeError("Scaler missing 'transform'.")
            expected_features = 2 # Magnetism, LDC RP
            if hasattr(numerical_scaler, 'n_features_in_'):
                print(f"Scaler expects {numerical_scaler.n_features_in_} features.")
                if numerical_scaler.n_features_in_ != expected_features:
                    print(f"ERROR: Scaler/script feature count mismatch ({numerical_scaler.n_features_in_} vs {expected_features}).")
                    ai_ready = False
            else: print(f"Warning: Cannot verify scaler feature count. Assuming {expected_features}.")
        except Exception as e:
            print(f"ERROR: Failed to load numerical scaler '{SCALER_FILENAME}': {e}"); ai_ready = False

    if ai_ready:
        try:
            interpreter = Interpreter(model_path=MODEL_PATH)
            interpreter.allocate_tensors()
            input_details = interpreter.get_input_details()
            output_details = interpreter.get_output_details()
            print("TFLite model loaded and tensors allocated.")
            # Sanity checks
            if len(input_details) != 2:
                print(f"ERROR: Model expected 2 inputs, found {len(input_details)}."); ai_ready = False
            if output_details and output_details[0]['shape'][-1] != len(loaded_labels):
                print(f"ERROR: Model output size ({output_details[0]['shape'][-1]}) != labels count ({len(loaded_labels)})."); ai_ready = False
        except Exception as e:
            print(f"ERROR: Failed to load TFLite model '{MODEL_FILENAME}': {e}"); ai_ready = False

    if not ai_ready:
        print("--- AI Initialization Failed ---")
        interpreter = input_details = output_details = numerical_scaler = None
        # Keep loaded_labels if they were loaded, might be useful
    else:
        print("--- AI Initialization Complete ---")
    return ai_ready

# =========================
# === LDC1101 Functions ===
# =========================
def ldc_write_register(reg_addr, value):
    if not spi or not ('GPIO' in globals() and hasattr(GPIO, 'output')): # Check GPIO for CS
        return False
    success = False
    try:
        GPIO.output(CS_PIN, GPIO.LOW)
        spi.xfer2([reg_addr & 0x7F, value])
        GPIO.output(CS_PIN, GPIO.HIGH)
        success = True
    except Exception as e:
        print(f"Warning: LDC write error (Reg 0x{reg_addr:02X}): {e}")
        try: GPIO.output(CS_PIN, GPIO.HIGH)
        except: pass # Ignore error during error handling for CS
    return success

def ldc_read_register(reg_addr):
    if not spi or not ('GPIO' in globals() and hasattr(GPIO, 'output')): # Check GPIO for CS
        return None
    read_value = None
    try:
        GPIO.output(CS_PIN, GPIO.LOW)
        result = spi.xfer2([reg_addr | 0x80, 0x00])
        GPIO.output(CS_PIN, GPIO.HIGH)
        read_value = result[1]
    except Exception as e:
        print(f"Warning: LDC read error (Reg 0x{reg_addr:02X}): {e}")
        try: GPIO.output(CS_PIN, GPIO.HIGH)
        except: pass
    return read_value

def initialize_ldc1101():
    global ldc_initialized
    ldc_initialized = False
    if not spi: return False
    print("Initializing LDC1101...")
    try:
        chip_id = ldc_read_register(CHIP_ID_REG)
        if chip_id is None or chip_id != LDC_CHIP_ID:
            print(f"ERROR: LDC Chip ID mismatch/read error. Read: {chip_id}, Expected: {LDC_CHIP_ID}")
            return False
        print(f"LDC1101 Chip ID verified (0x{chip_id:02X}).")
        # Example configurations (MUST BE ADJUSTED FOR YOUR COIL/SETUP)
        if not ldc_write_register(RP_SET_REG, 0x07): return False
        if not ldc_write_register(TC1_REG, 0x90): return False
        if not ldc_write_register(TC2_REG, 0xA0): return False
        if not ldc_write_register(DIG_CONFIG_REG, 0x03): return False
        if not ldc_write_register(ALT_CONFIG_REG, 0x00): return False
        if not ldc_write_register(D_CONF_REG, 0x00): return False
        if not ldc_write_register(INTB_MODE_REG, 0x00): return False
        if not ldc_write_register(START_CONFIG_REG, SLEEP_MODE): return False
        time.sleep(0.02)
        print("LDC1101 Configuration successful.")
        ldc_initialized = True
        return True
    except Exception as e:
        print(f"ERROR: Exception during LDC1101 Initialization: {e}")
        return False

def enable_ldc_powermode(mode):
    if not spi or not ldc_initialized: return False
    if ldc_write_register(START_CONFIG_REG, mode):
        time.sleep(0.01)
        return True
    print(f"Warning: Failed to set LDC power mode.")
    return False

def enable_ldc_rpmode():
    if not spi or not ldc_initialized: return False
    print("Enabling LDC RP+L Mode...")
    try:
        if not ldc_write_register(ALT_CONFIG_REG, 0x00): return False
        if not ldc_write_register(D_CONF_REG, 0x00): return False
        if enable_ldc_powermode(ACTIVE_CONVERSION_MODE):
            print("LDC RP+L Mode Enabled and Active.")
            return True
        print("Failed to set LDC to Active mode for RP+L.")
        return False
    except Exception as e:
        print(f"Warning: Failed to enable LDC RP mode: {e}")
        return False

def get_ldc_rpdata():
    if not spi or not ldc_initialized: return None
    try:
        msb = ldc_read_register(RP_DATA_MSB_REG)
        lsb = ldc_read_register(RP_DATA_LSB_REG)
        if msb is None or lsb is None: return None
        return (msb << 8) | lsb
    except Exception as e:
        print(f"Warning: Exception while reading LDC RP data: {e}")
        return None

# ============================
# === Sensor Reading (Avg) ===
# ============================
def get_averaged_hall_voltage(num_samples=NUM_SAMPLES_PER_UPDATE):
    if not hall_sensor: return None
    readings = []
    for _ in range(num_samples):
        try: readings.append(hall_sensor.voltage)
        except Exception as e:
            print(f"Warning: Error reading Hall sensor: {e}. Aborting average."); return None
    if readings: return statistics.mean(readings)
    return None

def get_averaged_rp_data(num_samples=NUM_SAMPLES_PER_UPDATE):
    if not ldc_initialized: return None
    readings = []
    for _ in range(num_samples):
        rp_value = get_ldc_rpdata()
        if rp_value is not None: readings.append(rp_value)
        # else: print(f"Warning: Failed LDC RP sample. Skipping.") # Optional: too verbose
    if readings: return statistics.mean(readings)
    return None


# ==========================
# === AI Processing ========
# ==========================
def preprocess_input(image_pil, mag_mT, ldc_rp_raw):
    global numerical_scaler, input_details, interpreter
    print("\n--- Preprocessing Input for AI ---")
    if interpreter is None or numerical_scaler is None:
        print("ERROR: AI Model/Scaler not initialized. Cannot preprocess."); return None

    try: # Image preprocessing
        img_resized = image_pil.resize((AI_IMG_WIDTH, AI_IMG_HEIGHT), Image.Resampling.LANCZOS)
        image_np = np.array(img_resized.convert('RGB'), dtype=np.float32) / 255.0 # Normalize to [0,1]
        image_input = np.expand_dims(image_np, axis=0)
        print(f"Image preprocessed. Shape: {image_input.shape}")
    except Exception as e:
        print(f"ERROR: Image preprocessing failed: {e}"); return None

    # Numerical preprocessing
    mag_mT_val = float(mag_mT) if mag_mT is not None else 0.0
    ldc_rp_raw_val = float(ldc_rp_raw) if ldc_rp_raw is not None else 0.0
    if mag_mT is None: print("DEBUG Preprocess: Magnetism was None, using 0.0.")
    if ldc_rp_raw is None: print("DEBUG Preprocess: LDC RP raw was None, using 0.0.")

    numerical_features = np.array([[mag_mT_val, ldc_rp_raw_val]], dtype=np.float32)
    print(f"DEBUG Preprocess: Raw numerical features: {numerical_features}")
    try:
        with warnings.catch_warnings(): # Suppress UserWarning about feature names
            warnings.filterwarnings("ignore", message="X does not have valid feature names.*", category=UserWarning)
            scaled_numerical_features = numerical_scaler.transform(numerical_features)
        print(f"DEBUG Preprocess: Scaled numerical features: {scaled_numerical_features}")
    except Exception as e:
        print(f"ERROR: Scaling numerical features failed: {e}"); return None

    # Identify input tensor indices and types
    image_input_index, numerical_input_index = -1, -1
    image_input_dtype, numerical_input_dtype = None, None

    for detail in input_details:
        shape = detail['shape']
        if len(shape) == 4 and shape[1:3] == (AI_IMG_HEIGHT, AI_IMG_WIDTH): # Image tensor
            image_input_index, image_input_dtype = detail['index'], detail['dtype']
        elif len(shape) == 2: # Numerical tensor
            numerical_input_index, numerical_input_dtype = detail['index'], detail['dtype']
            if shape[1] != scaled_numerical_features.shape[1]:
                print(f"ERROR: Model expects {shape[1]} numerical feats, got {scaled_numerical_features.shape[1]}.")
                return None
    if image_input_index == -1 or numerical_input_index == -1:
        print("ERROR: Failed to identify image/numerical input tensors."); return None

    # Prepare final model inputs with correct dtypes
    final_image_input = image_input.astype(image_input_dtype)
    if image_input_dtype == np.uint8: # If model expects UINT8, rescale from [0,1] to [0,255]
        final_image_input = (image_input * 255.0).astype(np.uint8)
        print("DEBUG Preprocess: Image input converted to UINT8 [0-255].")

    model_inputs = {
        image_input_index: final_image_input,
        numerical_input_index: scaled_numerical_features.astype(numerical_input_dtype)
    }
    print(f"  Final Image Input: Index={image_input_index}, Shape={model_inputs[image_input_index].shape}, Dtype={model_inputs[image_input_index].dtype}")
    print(f"  Final Numerical Input: Index={numerical_input_index}, Shape={model_inputs[numerical_input_index].shape}, Dtype={model_inputs[numerical_input_index].dtype}")
    print("--- Preprocessing Complete ---")
    return model_inputs

def run_inference(model_inputs):
    global interpreter, output_details
    print("\n--- Running AI Inference ---")
    if interpreter is None or model_inputs is None:
        print("ERROR: Interpreter/inputs not ready for inference."); return None
    try:
        for index, data in model_inputs.items(): interpreter.set_tensor(index, data)
        interpreter.invoke()
        output_data = interpreter.get_tensor(output_details[0]['index'])
        print(f"DEBUG Inference: Raw output data shape: {output_data.shape}, values: {output_data}")
        print("--- Inference Complete ---")
        return output_data
    except Exception as e:
        print(f"ERROR: Inference failed: {e}"); traceback.print_exc(); return None

def postprocess_output(output_data):
    global loaded_labels
    print("\n--- Postprocessing AI Output ---")
    if output_data is None or not loaded_labels:
        print("ERROR: No output data or labels for postprocessing."); return "Error", 0.0
    try:
        probabilities = output_data[0] if len(output_data.shape) == 2 else output_data
        if len(probabilities) != len(loaded_labels):
            print(f"ERROR: Probabilities count ({len(probabilities)}) != labels count ({len(loaded_labels)}).")
            return "Label Mismatch", 0.0

        # Debug print probabilities
        prob_dict = sorted(zip(loaded_labels, probabilities), key=lambda x: x[1], reverse=True)
        for label, prob in prob_dict: print(f"  - {label}: {prob:.4f}")

        predicted_index = np.argmax(probabilities)
        confidence = float(probabilities[predicted_index])
        predicted_label = loaded_labels[predicted_index]
        print(f"Final Prediction: '{predicted_label}', Confidence: {confidence:.4f}")
        print("--- Postprocessing Complete ---")
        return predicted_label, confidence
    except Exception as e:
        print(f"ERROR: Postprocessing failed: {e}"); return "Post Err", 0.0


# ==================================
# === Sorting Signal Functions ===
# ==================================
def send_sorting_signal(material_label):
    """
    Sends a 2-bit signal via GPIO based on the classified material label.
    Uses SORTING_DATA_READY_PIN to signal data transmission.
    """
    if not SORTING_GPIO_ENABLED:
        print("Sorting Signal: GPIO for sorting not enabled. Skipping send.")
        return

    if not ('GPIO' in globals() and hasattr(GPIO, 'output')):
        print("Sorting Signal: RPi.GPIO 'output' not available. Cannot send signal.")
        return

    print(f"\n--- Sending Sorting Signal for: {material_label} ---")

    # Default to "Others"
    mid_val = GPIO.LOW  # Bit 1 (MSB for our 2-bit signal)
    lsb_val = GPIO.LOW  # Bit 0 (LSB for our 2-bit signal)
    signal_desc = "Others (00)"

    if material_label == "Aluminum":
        mid_val = GPIO.LOW
        lsb_val = GPIO.HIGH
        signal_desc = "Aluminum (01)"
    elif material_label == "Copper":
        mid_val = GPIO.HIGH
        lsb_val = GPIO.LOW
        signal_desc = "Copper (10)"
    elif material_label == "Steel":
        mid_val = GPIO.HIGH
        lsb_val = GPIO.HIGH
        signal_desc = "Steel (11)"
    # Any other label will use the default "Others" signal (00)

    try:
        # 1. Ensure data ready is LOW
        GPIO.output(SORTING_DATA_READY_PIN, GPIO.LOW)
        time.sleep(0.01) # Small delay

        # 2. Set data pins
        GPIO.output(SORTING_DATA_PIN_MID, mid_val)
        GPIO.output(SORTING_DATA_PIN_LSB, lsb_val)
        print(f"Set GPIO Pins: MID={mid_val}, LSB={lsb_val} for {signal_desc}")
        time.sleep(0.01) # Allow pins to settle

        # 3. Pulse data ready pin HIGH then LOW
        GPIO.output(SORTING_DATA_READY_PIN, GPIO.HIGH)
        print(f"Pulsed {SORTING_DATA_READY_PIN} HIGH (Data Ready)")
        time.sleep(0.05) # Arduino needs time to see this
        GPIO.output(SORTING_DATA_READY_PIN, GPIO.LOW)
        print(f"Set {SORTING_DATA_READY_PIN} LOW (Data Transmitted)")
        print(f"--- Sorting signal {signal_desc} sent ---")

    except Exception as e:
        print(f"ERROR: Failed to send sorting signal via GPIO: {e}")
        # Ensure data ready pin is low in case of error during pulse
        try:
            GPIO.output(SORTING_DATA_READY_PIN, GPIO.LOW)
        except Exception as e_cleanup:
            print(f"Warning: Could not ensure data ready pin is LOW after error: {e_cleanup}")


# ==============================
# === View Switching Logic ===
# ==============================
def show_live_view():
    global live_view_frame, results_view_frame, lv_classify_button, interpreter
    if results_view_frame and results_view_frame.winfo_ismapped(): results_view_frame.pack_forget()
    if live_view_frame and not live_view_frame.winfo_ismapped():
        live_view_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
    if lv_classify_button:
        lv_classify_button.config(state=tk.NORMAL if interpreter else tk.DISABLED)

def show_results_view():
    global live_view_frame, results_view_frame
    if live_view_frame and live_view_frame.winfo_ismapped(): live_view_frame.pack_forget()
    if results_view_frame and not results_view_frame.winfo_ismapped():
        results_view_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

# ======================
# === GUI Functions ===
# ======================
def create_placeholder_image(width, height, color='#E0E0E0', text="No Image"):
    try:
        pil_img = Image.new('RGB', (width, height), color); tk_img = ImageTk.PhotoImage(pil_img)
        return tk_img
    except Exception as e: print(f"Warning: Failed to create placeholder: {e}"); return None

def clear_results_display():
    global rv_image_label, rv_prediction_label, rv_confidence_label, rv_magnetism_label, rv_ldc_label, placeholder_img_tk
    if rv_image_label:
        if placeholder_img_tk: rv_image_label.config(image=placeholder_img_tk, text=""); rv_image_label.img_tk = placeholder_img_tk
        else: rv_image_label.config(image='', text="No Image"); rv_image_label.img_tk = None
    default_text = "---"
    if rv_prediction_label: rv_prediction_label.config(text=default_text)
    if rv_confidence_label: rv_confidence_label.config(text=default_text)
    if rv_magnetism_label: rv_magnetism_label.config(text=default_text)
    if rv_ldc_label: rv_ldc_label.config(text=default_text)


def capture_and_classify():
    global lv_classify_button, window, camera, IDLE_VOLTAGE, IDLE_RP_VALUE, interpreter
    global rv_image_label, rv_prediction_label, rv_confidence_label, rv_magnetism_label, rv_ldc_label

    print("\n" + "="*10 + " Capture & Classify Triggered " + "="*10)
    if not interpreter: messagebox.showerror("Error", "AI Model not initialized."); print("Aborted: AI not ready."); return
    if not camera or not camera.isOpened(): messagebox.showerror("Error", "Camera not available."); print("Aborted: Camera not ready."); return

    if lv_classify_button: lv_classify_button.config(state=tk.DISABLED); window.update_idletasks()

    ret, frame = camera.read()
    if not ret or frame is None:
        messagebox.showerror("Capture Error", "Failed to capture image."); print("ERROR: Failed camera read.")
        if lv_classify_button: show_live_view(); return # Re-enable button implicitly by switching view
    try: img_captured_pil = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    except Exception as e:
        messagebox.showerror("Image Error", f"Image processing failed: {e}"); print(f"ERROR: PIL conversion: {e}")
        if lv_classify_button: show_live_view(); return

    print(f"Reading sensors ({NUM_SAMPLES_CALIBRATION} samples)...")
    avg_voltage = get_averaged_hall_voltage(num_samples=NUM_SAMPLES_CALIBRATION)
    current_mag_mT, mag_display_text, sensor_warning = None, "N/A", False
    if avg_voltage is not None:
        try:
            if abs(SENSITIVITY_V_PER_MILLITESLA) < 1e-9: raise ZeroDivisionError("Sensitivity zero.")
            current_mag_mT = (avg_voltage - IDLE_VOLTAGE) / SENSITIVITY_V_PER_MILLITESLA
            mag_display_text = f"{current_mag_mT:+.3f} mT" + (" (No Cal)" if IDLE_VOLTAGE == 0.0 else "")
        except Exception as e: mag_display_text = "Calc Err"; print(f"Warn: Mag calc fail: {e}"); sensor_warning = True
    else: mag_display_text = "Read Err"; print("ERROR: Hall read fail."); sensor_warning = True

    avg_rp_val = get_averaged_rp_data(num_samples=NUM_SAMPLES_CALIBRATION)
    current_rp_raw, ldc_display_text = None, "N/A"
    if avg_rp_val is not None:
        current_rp_raw = avg_rp_val; current_rp_int = int(round(avg_rp_val))
        delta_rp_display = current_rp_int - IDLE_RP_VALUE
        ldc_display_text = f"{current_rp_int}" + (f" (Δ{delta_rp_display:+,})" if IDLE_RP_VALUE != 0 else " (No Cal)")
    else: ldc_display_text = "Read Err"; print("ERROR: LDC read fail."); sensor_warning = True

    print(f"DEBUG Data for Preprocessing: Mag(mT): {current_mag_mT}, LDC_RP_Raw: {current_rp_raw}")
    if sensor_warning: print("WARNING: Sensor issues. Classification may be inaccurate.")

    model_inputs = preprocess_input(img_captured_pil, current_mag_mT, current_rp_raw)
    if model_inputs is None:
        messagebox.showerror("AI Error", "Preprocessing failed."); print("ERROR: Preprocessing abort.")
        if lv_classify_button: show_live_view(); return

    output_data = run_inference(model_inputs)
    if output_data is None:
        messagebox.showerror("AI Error", "Inference failed."); print("ERROR: Inference abort.")
        if lv_classify_button: show_live_view(); return

    predicted_label, confidence = postprocess_output(output_data)
    print(f"--- Classification Result: '{predicted_label}', Confidence={confidence:.1%} ---")

    # Attempt to send sorting signal
    send_sorting_signal(predicted_label) # New call

    # Update Results Display
    if rv_image_label:
        try:
            aspect = img_captured_pil.height / img_captured_pil.width if img_captured_pil.width > 0 else 0.75
            disp_h = int(RESULT_IMG_DISPLAY_WIDTH * aspect) if aspect > 0 else int(RESULT_IMG_DISPLAY_WIDTH * 0.75)
            img_disp = img_captured_pil.resize((RESULT_IMG_DISPLAY_WIDTH, max(1, disp_h)), Image.Resampling.LANCZOS)
            img_tk = ImageTk.PhotoImage(img_disp); rv_image_label.img_tk = img_tk
            rv_image_label.config(image=img_tk, text="")
        except Exception as e:
            print(f"ERROR: Results image update: {e}")
            rv_image_label.config(image=placeholder_img_tk or '', text="Img Err"); rv_image_label.img_tk = placeholder_img_tk

    if rv_prediction_label: rv_prediction_label.config(text=f"{predicted_label}")
    if rv_confidence_label: rv_confidence_label.config(text=f"{confidence:.1%}")
    if rv_magnetism_label: rv_magnetism_label.config(text=mag_display_text)
    if rv_ldc_label: rv_ldc_label.config(text=ldc_display_text)

    show_results_view()
    print("="*10 + " Capture & Classify Complete " + "="*10 + "\n")


def calibrate_sensors():
    global IDLE_VOLTAGE, IDLE_RP_VALUE, window, previous_filtered_mag_mT
    global lv_calibrate_button, lv_classify_button, hall_sensor, ldc_initialized

    print("\n" + "="*10 + " Sensor Calibration Triggered " + "="*10)
    hall_avail, ldc_avail = hall_sensor is not None, ldc_initialized
    if not hall_avail and not ldc_avail:
        messagebox.showwarning("Calibration", "No sensors available."); print("Aborted: No sensors."); return

    instr = "Ensure NO metal object is near sensors.\n\n"
    if hall_avail: instr += "- Hall idle voltage will be measured.\n"
    if ldc_avail: instr += "- LDC idle RP value will be measured.\n"
    if not messagebox.askokcancel("Calibration Instructions", instr + "\nClick OK to start."):
        print("Calibration cancelled."); return

    if lv_calibrate_button: lv_calibrate_button.config(state=tk.DISABLED)
    if lv_classify_button: lv_classify_button.config(state=tk.DISABLED)
    window.update_idletasks()

    hall_res, ldc_res = "Hall: N/A", "LDC: N/A"
    hall_ok, ldc_ok = False, False

    if hall_avail:
        avg_v = get_averaged_hall_voltage(num_samples=NUM_SAMPLES_CALIBRATION)
        if avg_v is not None: IDLE_VOLTAGE = avg_v; hall_res = f"Hall Idle: {IDLE_VOLTAGE:.4f} V"; hall_ok = True
        else: IDLE_VOLTAGE = 0.0; hall_res = "Hall: Read Error!"
        print(hall_res)
    if ldc_avail:
        avg_rp = get_averaged_rp_data(num_samples=NUM_SAMPLES_CALIBRATION)
        if avg_rp is not None: IDLE_RP_VALUE = int(round(avg_rp)); ldc_res = f"LDC Idle RP: {IDLE_RP_VALUE}"; ldc_ok = True
        else: IDLE_RP_VALUE = 0; ldc_res = "LDC: Read Error!"
        print(ldc_res)

    previous_filtered_mag_mT = None # Reset mag smoothing
    if lv_calibrate_button: lv_calibrate_button.config(state=tk.NORMAL)
    if lv_classify_button: lv_classify_button.config(state=tk.NORMAL if interpreter else tk.DISABLED)

    msg = f"Calibration Results:\n\n{hall_res}\n{ldc_res}"
    print("--- Calibration Complete ---")
    if (hall_avail and not hall_ok) or (ldc_avail and not ldc_ok): messagebox.showwarning("Calibration Warning", msg)
    else: messagebox.showinfo("Calibration Complete", msg)
    print("="*10 + " Sensor Calibration Finished " + "="*10 + "\n")


def update_camera_feed():
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
            except Exception as e: print(f"Error processing camera frame: {e}")
    if lv_camera_label:
        if img_tk: lv_camera_label.img_tk = img_tk; lv_camera_label.configure(image=img_tk, text="")
        else: # Placeholder logic
            if not hasattr(lv_camera_label, 'no_cam_img'):
                lv_camera_label.no_cam_img = create_placeholder_image(DISPLAY_IMG_WIDTH // 2, DISPLAY_IMG_HEIGHT // 2, '#BDBDBD', "No Feed")
            if lv_camera_label.no_cam_img and str(lv_camera_label.cget("image")) != str(lv_camera_label.no_cam_img):
                lv_camera_label.configure(image=lv_camera_label.no_cam_img, text=""); lv_camera_label.img_tk = lv_camera_label.no_cam_img
            elif not lv_camera_label.no_cam_img and lv_camera_label.cget("text") != "Camera Failed":
                lv_camera_label.configure(image='', text="Camera Failed"); lv_camera_label.img_tk = None
    if window and window.winfo_exists(): window.after(CAMERA_UPDATE_INTERVAL_MS, update_camera_feed)

def update_magnetism():
    global lv_magnetism_label, window, previous_filtered_mag_mT, IDLE_VOLTAGE, hall_sensor
    if not window or not window.winfo_exists(): return
    display_text = "N/A"
    if hall_sensor:
        avg_v = get_averaged_hall_voltage(num_samples=NUM_SAMPLES_PER_UPDATE)
        if avg_v is not None:
            try:
                if abs(SENSITIVITY_V_PER_MILLITESLA) < 1e-9: raise ZeroDivisionError("Sensitivity near zero")
                raw_mT = (avg_v - IDLE_VOLTAGE) / SENSITIVITY_V_PER_MILLITESLA
                filt_mT = (MAGNETISM_FILTER_ALPHA * raw_mT) + ((1 - MAGNETISM_FILTER_ALPHA) * (previous_filtered_mag_mT or raw_mT))
                previous_filtered_mag_mT = filt_mT
                display_text = f"{filt_mT * 1000:+.1f} µT" if abs(filt_mT) < 0.1 else f"{filt_mT:+.2f} mT"
                if IDLE_VOLTAGE == 0.0: display_text += " (No Cal)"
            except Exception: display_text = "Calc Err"; previous_filtered_mag_mT = None
        else: display_text = "Read Err"; previous_filtered_mag_mT = None
    if lv_magnetism_label and lv_magnetism_label.cget("text") != display_text: lv_magnetism_label.config(text=display_text)
    if window and window.winfo_exists(): window.after(GUI_UPDATE_INTERVAL_MS, update_magnetism)

def update_ldc_reading():
    global lv_ldc_label, window, RP_DISPLAY_BUFFER, IDLE_RP_VALUE, ldc_initialized
    if not window or not window.winfo_exists(): return
    display_text = "N/A"
    if ldc_initialized:
        avg_rp = get_averaged_rp_data(num_samples=NUM_SAMPLES_PER_UPDATE)
        if avg_rp is not None:
            RP_DISPLAY_BUFFER.append(avg_rp)
            if RP_DISPLAY_BUFFER:
                curr_rp = int(round(statistics.mean(RP_DISPLAY_BUFFER)))
                delta = curr_rp - IDLE_RP_VALUE
                display_text = f"{curr_rp}" + (f" (Δ{delta:+,})" if IDLE_RP_VALUE != 0 else " (No Cal)")
            else: display_text = "Buffering..."
        else: display_text = "Read Err"
    if lv_ldc_label and lv_ldc_label.cget("text") != display_text: lv_ldc_label.config(text=display_text)
    if window and window.winfo_exists(): window.after(GUI_UPDATE_INTERVAL_MS, update_ldc_reading)

# ======================
# === GUI Setup ========
# ======================
def setup_gui():
    global window, main_frame, placeholder_img_tk, live_view_frame, results_view_frame
    global lv_camera_label, lv_magnetism_label, lv_ldc_label, lv_classify_button, lv_calibrate_button
    global rv_image_label, rv_prediction_label, rv_confidence_label, rv_magnetism_label, rv_ldc_label, rv_classify_another_button
    global label_font, readout_font, button_font, title_font, result_title_font, result_value_font

    print("Setting up GUI...")
    window = tk.Tk()
    window.title("AI Metal Classifier v3.0.11 (RPi Combined)")
    window.geometry("800x600")
    style = ttk.Style()
    try: style.theme_use('clam') # Or 'alt', 'default'
    except tk.TclError: print("Warning: 'clam' theme not available, using default.")

    try:
        title_font = tkFont.Font(family="DejaVu Sans", size=16, weight="bold")
        label_font = tkFont.Font(family="DejaVu Sans", size=10)
        readout_font = tkFont.Font(family="DejaVu Sans Mono", size=12, weight="bold")
        button_font = tkFont.Font(family="DejaVu Sans", size=10, weight="bold")
        result_title_font = tkFont.Font(family="DejaVu Sans", size=11, weight="bold")
        result_value_font = tkFont.Font(family="DejaVu Sans Mono", size=12, weight="bold")
    except tk.TclError: # Fallback fonts
        print("Warning: Preferred fonts (DejaVu) not found, using Tkinter defaults.")
        title_font, label_font, readout_font, button_font, result_title_font, result_value_font = \
            "TkHeadingFont", "TkTextFont", "TkFixedFont", "TkDefaultFont", "TkDefaultFont", "TkFixedFont"

    style.configure("TLabel", font=label_font, padding=2)
    style.configure("TButton", font=button_font, padding=(8, 5))
    style.configure("Readout.TLabel", font=readout_font, foreground="#0000AA")
    style.configure("ResultValue.TLabel", font=result_value_font, foreground="#0000AA")
    style.configure("Prediction.TLabel", font=tkFont.Font(family="DejaVu Sans", size=16, weight="bold") if isinstance(title_font, tkFont.Font) else "TkHeadingFont", foreground="#AA0000")


    main_frame = ttk.Frame(window, padding=5); main_frame.pack(fill=tk.BOTH, expand=True)
    main_frame.rowconfigure(0, weight=1); main_frame.columnconfigure(0, weight=1)

    # Live View
    live_view_frame = ttk.Frame(main_frame, padding=5)
    live_view_frame.columnconfigure(0, weight=3); live_view_frame.columnconfigure(1, weight=1); live_view_frame.rowconfigure(0, weight=1)
    lv_camera_label = ttk.Label(live_view_frame, text="Init Cam...", anchor="center", relief="sunken", background="#CCC")
    lv_camera_label.grid(row=0, column=0, sticky="nsew", padx=(0,5))
    lv_controls = ttk.Frame(live_view_frame); lv_controls.grid(row=0, column=1, sticky="nsew", padx=(5,0))
    lv_controls.columnconfigure(0, weight=1) # Allow readings/actions frames to expand width
    lv_readings = ttk.Labelframe(lv_controls, text=" Live Readings ", padding=5)
    lv_readings.grid(row=0, column=0, sticky="new", pady=(0,10)); lv_readings.columnconfigure(1, weight=1)
    ttk.Label(lv_readings, text="Magnetism:").grid(row=0, column=0, sticky="w")
    lv_magnetism_label = ttk.Label(lv_readings, text="Init...", style="Readout.TLabel"); lv_magnetism_label.grid(row=0, column=1, sticky="ew")
    ttk.Label(lv_readings, text="LDC (Delta):").grid(row=1, column=0, sticky="w")
    lv_ldc_label = ttk.Label(lv_readings, text="Init...", style="Readout.TLabel"); lv_ldc_label.grid(row=1, column=1, sticky="ew")
    lv_actions = ttk.Labelframe(lv_controls, text=" Actions ", padding=5)
    lv_actions.grid(row=1, column=0, sticky="new"); lv_actions.columnconfigure(0, weight=1)
    lv_classify_button = ttk.Button(lv_actions, text="Capture & Classify", command=capture_and_classify); lv_classify_button.grid(row=0, column=0, sticky="ew", pady=4)
    lv_calibrate_button = ttk.Button(lv_actions, text="Calibrate Sensors", command=calibrate_sensors); lv_calibrate_button.grid(row=1, column=0, sticky="ew", pady=4)

    # Results View
    results_view_frame = ttk.Frame(main_frame, padding=10)
    results_view_frame.rowconfigure(1, weight=1); results_view_frame.columnconfigure(1, weight=1) # Center content
    rv_content = ttk.Frame(results_view_frame); rv_content.grid(row=1, column=1) # Centered content frame
    ttk.Label(rv_content, text="Classification Result", font=title_font).grid(row=0, columnspan=2, pady=(5,15))
    ph_h = int(RESULT_IMG_DISPLAY_WIDTH * 0.75); placeholder_img_tk = create_placeholder_image(RESULT_IMG_DISPLAY_WIDTH, ph_h)
    rv_image_label = ttk.Label(rv_content, anchor="center", relief="sunken")
    if placeholder_img_tk: rv_image_label.config(image=placeholder_img_tk); rv_image_label.img_tk = placeholder_img_tk
    else: rv_image_label.config(text="Image Area", width=30, height=15)
    rv_image_label.grid(row=1, columnspan=2, pady=(0,15))
    rv_details = ttk.Frame(rv_content); rv_details.grid(row=2, columnspan=2, pady=(0,15)); rv_details.columnconfigure(1, weight=1)
    r=0; ttk.Label(rv_details, text="Material:", font=result_title_font).grid(row=r, column=0, sticky="w")
    rv_prediction_label = ttk.Label(rv_details, text="---", style="Prediction.TLabel"); rv_prediction_label.grid(row=r, column=1, sticky="ew", padx=5); r+=1
    ttk.Label(rv_details, text="Confidence:", font=result_title_font).grid(row=r, column=0, sticky="w")
    rv_confidence_label = ttk.Label(rv_details, text="---", style="ResultValue.TLabel"); rv_confidence_label.grid(row=r, column=1, sticky="ew", padx=5); r+=1
    ttk.Separator(rv_details, orient='horizontal').grid(row=r, columnspan=2, sticky='ew', pady=8); r+=1
    ttk.Label(rv_details, text="Sensor Values Used:", font=result_title_font).grid(row=r, columnspan=2, sticky="w", pady=(0,3)); r+=1
    ttk.Label(rv_details, text=" Magnetism:", font=result_title_font).grid(row=r, column=0, sticky="w", padx=(5,0))
    rv_magnetism_label = ttk.Label(rv_details, text="---", style="ResultValue.TLabel"); rv_magnetism_label.grid(row=r, column=1, sticky="ew", padx=5); r+=1
    ttk.Label(rv_details, text=" LDC Reading:", font=result_title_font).grid(row=r, column=0, sticky="w", padx=(5,0))
    rv_ldc_label = ttk.Label(rv_details, text="---", style="ResultValue.TLabel"); rv_ldc_label.grid(row=r, column=1, sticky="ew", padx=5)
    rv_classify_another_button = ttk.Button(rv_content, text="<< Classify Another", command=show_live_view); rv_classify_another_button.grid(row=3, columnspan=2, pady=(15,5))

    clear_results_display()
    show_live_view()
    print("GUI setup complete.")

# ==========================
# === Main Execution =======
# ==========================
def run_application():
    global window, lv_classify_button, interpreter, camera, hall_sensor, ldc_initialized
    print("Setting up GUI...")
    try: setup_gui()
    except Exception as e:
        print(f"FATAL: GUI setup failed: {e}"); traceback.print_exc()
        try: root=tk.Tk(); root.withdraw(); messagebox.showerror("GUI Error", f"Failed to init GUI:\n{e}"); root.destroy()
        except: pass
        return

    # Update GUI based on init status
    if not camera: lv_camera_label.configure(text="Cam Fail", image='')
    if not hall_sensor: lv_magnetism_label.config(text="N/A")
    if not ldc_initialized: lv_ldc_label.config(text="N/A")
    if not interpreter: lv_classify_button.config(state=tk.DISABLED, text="Classify (AI Fail)")
    if not hall_sensor and not ldc_initialized: lv_calibrate_button.config(state=tk.DISABLED, text="Calib (No Sens)")

    print("Starting GUI update loops...")
    update_camera_feed(); update_magnetism(); update_ldc_reading()
    print("Starting Tkinter main loop... (Ctrl+C in console to exit)")
    try: window.protocol("WM_DELETE_WINDOW", on_closing); window.mainloop()
    except Exception as e: print(f"ERROR: Tkinter main loop: {e}")
    print("Tkinter main loop finished.")

def on_closing():
    global window
    print("Window close requested.")
    if messagebox.askokcancel("Quit", "Quit AI Metal Classifier?"):
        print("Proceeding with shutdown...")
        if window:
            try: window.destroy(); print("Tkinter window destroyed.")
            except: print("Note: Window may already be destroyed.")
    else: print("Shutdown cancelled.")

# ==========================
# === Cleanup Resources ====
# ==========================
import traceback # For more detailed error logging in main

def cleanup_resources():
    print("\n--- Cleaning up resources ---")
    global camera, spi, ldc_initialized, CS_PIN, SPI_ENABLED, SORTING_GPIO_ENABLED

    if camera and camera.isOpened():
        try: print("Releasing camera..."); camera.release(); print("Camera released.")
        except Exception as e: print(f"Warn: Error releasing camera: {e}")

    if spi: # LDC related SPI
        try:
            if ldc_initialized and CS_PIN is not None and ('GPIO' in globals() and hasattr(GPIO, 'output')):
                print("Putting LDC1101 to sleep...")
                try: # Attempt to send sleep command
                    GPIO.output(CS_PIN, GPIO.LOW); spi.xfer2([START_CONFIG_REG & 0x7F, SLEEP_MODE]); GPIO.output(CS_PIN, GPIO.HIGH); time.sleep(0.05)
                    print("LDC sleep command sent.")
                except Exception as ldc_e: print(f"Note: Error sending LDC sleep: {ldc_e}")
        finally:
            try: print("Closing LDC SPI..."); spi.close(); print("LDC SPI closed.")
            except Exception as e: print(f"Warn: Error closing LDC SPI: {e}")

    # General GPIO cleanup if RPi.GPIO was used for LDC CS or Sorting
    # SORTING_GPIO_ENABLED implies 'GPIO' in globals and GPIO.setmode was successful.
    # SPI_ENABLED with CS_PIN also implies 'GPIO' in globals and GPIO.setmode was successful.
    if ('GPIO' in globals() and hasattr(GPIO, 'cleanup') and
        ( (SPI_ENABLED and CS_PIN is not None) or SORTING_GPIO_ENABLED ) ):
        try:
            if GPIO.getmode() is not None: # Check if a mode was set
                print("Cleaning up GPIO (for LDC CS and/or Sorting)...")
                GPIO.cleanup()
                print("GPIO cleaned up.")
            else:
                print("Note: GPIO mode not set, skipping GPIO.cleanup().")
        except RuntimeError as e: print(f"Note: GPIO cleanup runtime error (already cleaned?): {e}")
        except Exception as e: print(f"Warn: Error during GPIO cleanup: {e}")
    else:
        print("Note: GPIO library not used or not set up for LDC/Sorting, skipping GPIO cleanup.")


    print("--- Cleanup complete ---")

# ==========================
# === Main Entry Point =====
# ==========================
if __name__ == '__main__':
    print("="*30 + "\n Starting AI Metal Classifier (RPi Combined) \n" + "="*30)
    hardware_init_ok = False # Track if initialize_hardware itself completed
    ai_init_ok = False

    try:
        initialize_hardware()
        hardware_init_ok = True # If it didn't crash
        ai_init_ok = initialize_ai()
        run_application()
    except KeyboardInterrupt:
        print("\nKeyboard interrupt. Exiting.")
        if 'window' in globals() and window and window.winfo_exists(): window.destroy()
    except Exception as e:
        print("\n" + "="*30 + f"\nFATAL ERROR in main: {e}\n" + "="*30)
        traceback.print_exc()
        if 'window' in globals() and window and window.winfo_exists():
            try: messagebox.showerror("Fatal Error", f"Unrecoverable error:\n\n{e}")
            except: pass # If GUI itself is broken
            try: window.destroy()
            except: pass
    finally:
        if hardware_init_ok: # Only cleanup if hardware init was attempted/completed
            cleanup_resources()
        else:
            print("Skipping resource cleanup as hardware initialization did not complete successfully.")
        print("\nApplication finished.\n" + "="*30)
