# CODE 3.0.14 - AI Metal Classifier GUI with Results Page and Sorting Output
# Description: Displays live sensor data and camera feed.
#              Captures image and sensor readings, classifies metal using a TFLite model,
#              displays the results on a dedicated page, sends a sorting signal via GPIO,
#              and waits for a GPIO enable signal before allowing next classification.
# Version: 3.0.14 - Added logic to disable classification after completion and re-enable
#                  it upon receiving a signal on GPIO 5 (BCM). Uses polling for GPIO input.
#                  Maintained expanded code formatting for readability.
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
    # This can happen if Blinka doesn't detect the board/platform correctly
    print("Warning: I2C not supported on this platform according to Blinka. Magnetism readings disabled.")
except Exception as e:
    print(f"Warning: Error importing I2C/ADS1115 libraries: {e}. Magnetism readings disabled.")

# --- SPI/LDC1101 & RPi.GPIO Imports ---
SPI_ENABLED = False # For spidev library itself
RPi_GPIO_AVAILABLE = False # For RPi.GPIO library itself
try:
    import spidev   # For SPI communication
    SPI_ENABLED = True
    print("SPI library (spidev) imported successfully.")
except ImportError:
    print("Warning: SPI library (spidev) not found. LDC readings will be disabled.")

try:
    import RPi.GPIO as GPIO # For controlling Chip Select pin and sorting pins
    RPi_GPIO_AVAILABLE = True
    print("RPi.GPIO library imported successfully (needed for LDC CS, Sorting, and Enable Input).")
except ImportError:
    print("Warning: RPi.GPIO library not found. LDC CS control, Sorting, and Enable Input will be disabled.")
except RuntimeError:
    print("Warning: RPi.GPIO library likely requires root privileges (sudo). LDC CS, Sorting, and Enable Input may fail.")
except Exception as e:
    print(f"Warning: Error importing RPi.GPIO library: {e}. LDC CS control, Sorting, and Enable Input disabled.")


# --- Sorting GPIO Configuration ---
SORTING_GPIO_ENABLED = False # Default to False, set True if RPi.GPIO is available and setup succeeds
SORTING_DATA_PIN_LSB = 16 # BCM Pin for LSB of sorting data
SORTING_DATA_PIN_MID = 6  # BCM Pin for MID/MSB of sorting data (2-bit signal)
SORTING_DATA_READY_PIN = 26 # BCM Pin to signal data is ready for sorter

# --- Enable Classification GPIO Configuration ---
ENABLE_CLASSIFICATION_PIN = 5 # BCM Pin to re-enable classification
ENABLE_PIN_SETUP_OK = False   # Tracks if the enable pin was set up
CHECK_ENABLE_INTERVAL_MS = 250 # How often to check the enable pin (in milliseconds)


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

# --- GUI Globals ---
window = None
main_frame = None
live_view_frame = None
results_view_frame = None
label_font, readout_font, button_font, title_font, result_title_font, result_value_font, pred_font = (None,) * 7
lv_camera_label, lv_magnetism_label, lv_ldc_label, lv_classify_button, lv_calibrate_button = (None,) * 5
rv_image_label, rv_prediction_label, rv_confidence_label, rv_magnetism_label, rv_ldc_label, rv_classify_another_button = (None,) * 6
placeholder_img_tk = None

# --- State for enabling classification ---
classification_pending_enable = False # True if waiting for GPIO signal to re-enable classification

# =========================
# === Hardware Setup ===
# =========================
def initialize_hardware():
    global camera, i2c, ads, hall_sensor, spi, ldc_initialized, CS_PIN
    global SORTING_GPIO_ENABLED, RPi_GPIO_AVAILABLE
    global ENABLE_PIN_SETUP_OK, ENABLE_CLASSIFICATION_PIN

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
            print(f"LDC CS Pin {CS_PIN} set as OUTPUT HIGH.")
            spi = spidev.SpiDev()
            spi.open(SPI_BUS, SPI_DEVICE)
            spi.max_speed_hz = SPI_SPEED
            spi.mode = SPI_MODE
            print(f"SPI initialized for LDC (Bus={SPI_BUS}, Device={SPI_DEVICE}).")
            if initialize_ldc1101():
                enable_ldc_rpmode()
                print("LDC1101 initialized and RP+L mode enabled.")
            else:
                print("ERROR: LDC1101 Low-level Initialization Failed.")
                ldc_initialized = False
        except Exception as e:
            print(f"ERROR: An error occurred during SPI/LDC initialization: {e}")
            if spi: spi.close()
            spi = None
            ldc_initialized = False
    elif SPI_ENABLED and not gpio_bcm_mode_set: # spidev present, but GPIO failed
        print("Skipping LDC1101 setup because RPi.GPIO (needed for CS pin) failed BCM mode setup.")
        spi = None
        ldc_initialized = False
    else: # SPI_ENABLED is False or RPi.GPIO not available/BCM failed
        print("Skipping SPI/LDC1101 setup.")


    # --- Sorting GPIO Pin Initialization ---
    if gpio_bcm_mode_set:
        print("Attempting to initialize GPIO pins for Sorting Mechanism...")
        try:
            GPIO.setup(SORTING_DATA_PIN_LSB, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(SORTING_DATA_PIN_MID, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(SORTING_DATA_READY_PIN, GPIO.OUT, initial=GPIO.LOW)
            SORTING_GPIO_ENABLED = True
            print(f"Sorting GPIO pins set. Sorting is ENABLED.")
        except Exception as e:
            print(f"ERROR: Failed to set up sorting GPIO pins: {e}. Sorting is DISABLED.")
            SORTING_GPIO_ENABLED = False
    else:
        print("Skipping Sorting GPIO setup (RPi.GPIO not available or BCM mode failed). Sorting is DISABLED.")
        SORTING_GPIO_ENABLED = False


    # --- Enable Classification Pin Initialization ---
    if gpio_bcm_mode_set:
        print(f"Attempting to initialize GPIO pin {ENABLE_CLASSIFICATION_PIN} for Enable Signal...")
        try:
            GPIO.setup(ENABLE_CLASSIFICATION_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
            ENABLE_PIN_SETUP_OK = True
            print(f"Enable Classification Pin {ENABLE_CLASSIFICATION_PIN} set as INPUT with PULL-DOWN. Waiting for HIGH signal.")
        except Exception as e:
            print(f"ERROR: Failed to set up Enable Classification Pin {ENABLE_CLASSIFICATION_PIN}: {e}")
            ENABLE_PIN_SETUP_OK = False
    else:
        print(f"Skipping Enable Classification Pin {ENABLE_CLASSIFICATION_PIN} setup (RPi.GPIO not available or BCM mode failed).")
        ENABLE_PIN_SETUP_OK = False

    print("--- Hardware Initialization Complete ---")

# =========================
# === AI Model Setup ======
# =========================
def initialize_ai(): # Unchanged from 3.0.13
    global interpreter, input_details, output_details, loaded_labels, numerical_scaler
    print("\n--- Initializing AI Components ---")
    ai_ready = True
    print(f"Loading labels from: {LABELS_PATH}")
    try:
        with open(LABELS_PATH, 'r') as f: loaded_labels = [line.strip() for line in f.readlines()]
        if not loaded_labels: raise ValueError("Labels file is empty.")
        print(f"Loaded {len(loaded_labels)} labels: {loaded_labels}")
    except Exception as e: print(f"ERROR: Reading labels '{LABELS_FILENAME}': {e}"); ai_ready = False

    if ai_ready:
        print(f"Loading numerical scaler from: {SCALER_PATH}")
        try:
            numerical_scaler = joblib.load(SCALER_PATH)
            if not hasattr(numerical_scaler, 'transform') or not callable(numerical_scaler.transform):
                raise TypeError("Loaded scaler invalid.")
            expected_features = 2
            if hasattr(numerical_scaler, 'n_features_in_'):
                if numerical_scaler.n_features_in_ != expected_features:
                    print(f"ERROR: Scaler features mismatch ({numerical_scaler.n_features_in_} vs {expected_features}).")
                    ai_ready = False
            else: print(f"Warning: Cannot verify scaler feature count. Assuming {expected_features}.")
        except Exception as e: print(f"ERROR: Loading scaler '{SCALER_FILENAME}': {e}"); ai_ready = False

    if ai_ready:
        print(f"Loading TFLite model from: {MODEL_PATH}")
        try:
            interpreter = Interpreter(model_path=MODEL_PATH)
            interpreter.allocate_tensors()
            input_details = interpreter.get_input_details()
            output_details = interpreter.get_output_details()
            print("TFLite model loaded and tensors allocated.")
            if len(input_details) != 2: print(f"ERROR: Model inputs != 2."); ai_ready = False
            if output_details and output_details[0]['shape'][-1] != len(loaded_labels):
                print(f"ERROR: Model output size != labels ({len(loaded_labels)})."); ai_ready = False
        except Exception as e: print(f"ERROR: Loading TFLite model '{MODEL_FILENAME}': {e}"); traceback.print_exc(); ai_ready = False

    if not ai_ready:
        print("--- AI Initialization Failed ---")
        interpreter = input_details = output_details = numerical_scaler = None
    else: print("--- AI Initialization Complete ---")
    return ai_ready

# =========================
# === LDC1101 Functions ===
# =========================
def ldc_write_register(reg_addr, value): # Unchanged from 3.0.13
    if not spi or not RPi_GPIO_AVAILABLE: return False
    success = False
    try:
        GPIO.output(CS_PIN, GPIO.LOW)
        spi.xfer2([reg_addr & 0x7F, value])
        GPIO.output(CS_PIN, GPIO.HIGH)
        success = True
    except Exception as e:
        print(f"Warning: LDC write error (Reg 0x{reg_addr:02X}): {e}")
        try:
            if RPi_GPIO_AVAILABLE: GPIO.output(CS_PIN, GPIO.HIGH)
        except Exception: pass
    return success

def ldc_read_register(reg_addr): # Unchanged from 3.0.13
    if not spi or not RPi_GPIO_AVAILABLE: return None
    read_value = None
    try:
        GPIO.output(CS_PIN, GPIO.LOW)
        result = spi.xfer2([reg_addr | 0x80, 0x00])
        GPIO.output(CS_PIN, GPIO.HIGH)
        read_value = result[1]
    except Exception as e:
        print(f"Warning: LDC read error (Reg 0x{reg_addr:02X}): {e}")
        try:
            if RPi_GPIO_AVAILABLE: GPIO.output(CS_PIN, GPIO.HIGH)
        except Exception: pass
    return read_value

def initialize_ldc1101(): # Unchanged from 3.0.13
    global ldc_initialized
    ldc_initialized = False
    if not spi: print("Cannot initialize LDC1101: SPI not available."); return False
    print("Initializing LDC1101...")
    try:
        chip_id = ldc_read_register(CHIP_ID_REG)
        if chip_id is None: print("ERROR: Failed to read LDC Chip ID."); return False
        if chip_id != LDC_CHIP_ID: print(f"ERROR: LDC Chip ID mismatch! (Read:0x{chip_id:02X})"); return False
        print(f"LDC1101 Chip ID verified (0x{chip_id:02X}).")
        regs_to_write = { RP_SET_REG: 0x07, TC1_REG: 0x90, TC2_REG: 0xA0, DIG_CONFIG_REG: 0x03,
                          ALT_CONFIG_REG: 0x00, D_CONF_REG: 0x00, INTB_MODE_REG: 0x00 }
        for reg, val in regs_to_write.items():
            if not ldc_write_register(reg, val): print(f"ERROR: LDC write reg 0x{reg:02X} failed."); return False
        if not ldc_write_register(START_CONFIG_REG, SLEEP_MODE): return False
        time.sleep(0.02)
        print("LDC1101 Configuration successful.")
        ldc_initialized = True
        return True
    except Exception as e: print(f"ERROR: Exception during LDC1101 Initialization: {e}"); ldc_initialized = False; return False

def enable_ldc_powermode(mode): # Unchanged from 3.0.13
    if not spi or not ldc_initialized: return False
    if ldc_write_register(START_CONFIG_REG, mode): time.sleep(0.01); return True
    else: print(f"Warning: Failed to set LDC power mode register."); return False

def enable_ldc_rpmode(): # Unchanged from 3.0.13
    if not spi or not ldc_initialized: print("Warning: Cannot enable LDC RP mode (SPI/LDC not ready)."); return False
    print("Enabling LDC RP+L Mode...")
    try:
        if not ldc_write_register(ALT_CONFIG_REG, 0x00): return False
        if not ldc_write_register(D_CONF_REG, 0x00): return False
        if enable_ldc_powermode(ACTIVE_CONVERSION_MODE): print("LDC RP+L Mode Enabled and Active."); return True
        else: print("Failed to set LDC to Active mode for RP+L."); return False
    except Exception as e: print(f"Warning: Failed to enable LDC RP mode: {e}"); return False

def get_ldc_rpdata(): # Unchanged from 3.0.13
    if not spi or not ldc_initialized: return None
    try:
        msb = ldc_read_register(RP_DATA_MSB_REG)
        lsb = ldc_read_register(RP_DATA_LSB_REG)
        if msb is None or lsb is None: return None
        return (msb << 8) | lsb
    except Exception as e: print(f"Warning: Exception while reading LDC RP data: {e}"); return None

# ============================
# === Sensor Reading (Avg) ===
# ============================
def get_averaged_hall_voltage(num_samples=NUM_SAMPLES_PER_UPDATE): # Unchanged from 3.0.13
    if not hall_sensor: return None
    readings = []
    for _ in range(num_samples):
        try: readings.append(hall_sensor.voltage)
        except Exception as e: print(f"Warning: Error reading Hall sensor: {e}. Aborting average."); return None
    if readings: return statistics.mean(readings)
    else: return None

def get_averaged_rp_data(num_samples=NUM_SAMPLES_PER_UPDATE): # Unchanged from 3.0.13
    if not ldc_initialized: return None
    readings = []
    for _ in range(num_samples):
        rp_value = get_ldc_rpdata()
        if rp_value is not None: readings.append(rp_value)
    if readings: return statistics.mean(readings)
    else: return None

# ==========================
# === AI Processing ========
# ==========================
def preprocess_input(image_pil, mag_mT, ldc_rp_raw): # Unchanged from 3.0.13
    global numerical_scaler, input_details, interpreter
    print("\n--- Preprocessing Input for AI ---")
    if interpreter is None or input_details is None or numerical_scaler is None:
        print("ERROR: AI Model/Scaler not initialized."); return None
    try:
        img_resized = image_pil.resize((AI_IMG_WIDTH, AI_IMG_HEIGHT), Image.Resampling.LANCZOS)
        image_np = np.array(img_resized.convert('RGB'), dtype=np.float32)
        image_np /= 255.0
        image_input = np.expand_dims(image_np, axis=0)
        print(f"Image preprocessed. Shape: {image_input.shape}")
    except Exception as e: print(f"ERROR: Image preprocessing failed: {e}"); return None

    mag_mT_val = float(mag_mT) if mag_mT is not None else 0.0
    ldc_rp_raw_val = float(ldc_rp_raw) if ldc_rp_raw is not None else 0.0
    if mag_mT is None: print("DEBUG Preprocess: Magnetism None, using 0.0.")
    if ldc_rp_raw is None: print("DEBUG Preprocess: LDC RP raw None, using 0.0.")
    numerical_features = np.array([[mag_mT_val, ldc_rp_raw_val]], dtype=np.float32)
    print(f"DEBUG Preprocess: Raw numerical features: {numerical_features}")
    try:
        with warnings.catch_warnings():
            warnings.filterwarnings("ignore", message="X does not have valid feature names.*", category=UserWarning)
            scaled_numerical_features = numerical_scaler.transform(numerical_features)
        print(f"DEBUG Preprocess: Scaled numerical features: {scaled_numerical_features}")
    except Exception as e: print(f"ERROR: Scaling numerical features failed: {e}"); return None

    image_input_index, numerical_input_index = -1, -1
    image_input_dtype, numerical_input_dtype = None, None
    for detail in input_details:
        shape = detail['shape']
        if len(shape) == 4 and shape[1] == AI_IMG_HEIGHT and shape[2] == AI_IMG_WIDTH:
            image_input_index, image_input_dtype = detail['index'], detail['dtype']
        elif len(shape) == 2:
            numerical_input_index, numerical_input_dtype = detail['index'], detail['dtype']
            if shape[1] != scaled_numerical_features.shape[1]:
                print(f"ERROR: Model expects {shape[1]} numerical feats, got {scaled_numerical_features.shape[1]}."); return None
    if image_input_index == -1 or numerical_input_index == -1:
        print("ERROR: Failed to identify image/numerical input tensors."); return None

    final_image_input = image_input.astype(image_input_dtype)
    if image_input_dtype == np.uint8:
        final_image_input = (image_input * 255.0).astype(np.uint8)
        print("DEBUG Preprocess: Image input rescaled to UINT8 [0-255].")
    model_inputs = { image_input_index: final_image_input, numerical_input_index: scaled_numerical_features.astype(numerical_input_dtype) }
    print("--- Preprocessing Complete ---")
    return model_inputs

def run_inference(model_inputs): # Unchanged from 3.0.13
    global interpreter, output_details
    print("\n--- Running AI Inference ---")
    if interpreter is None or model_inputs is None: print("ERROR: Interpreter/inputs not ready for inference."); return None
    try:
        for index, data in model_inputs.items(): interpreter.set_tensor(index, data)
        interpreter.invoke()
        output_data = interpreter.get_tensor(output_details[0]['index'])
        print(f"DEBUG Inference: Raw output data shape: {output_data.shape}, values: {output_data}")
        print("--- Inference Complete ---")
        return output_data
    except Exception as e: print(f"ERROR: Inference failed: {e}"); traceback.print_exc(); return None

def postprocess_output(output_data): # Unchanged from 3.0.13
    global loaded_labels
    print("\n--- Postprocessing AI Output ---")
    if output_data is None or not loaded_labels: print("ERROR: No output/labels for postprocessing."); return "Error", 0.0
    try:
        if len(output_data.shape) == 2 and output_data.shape[0] == 1: probabilities = output_data[0]
        elif len(output_data.shape) == 1: probabilities = output_data
        else: print(f"ERROR: Unexpected output data shape {output_data.shape}."); return "Shape Err", 0.0
        if len(probabilities) != len(loaded_labels):
            print(f"ERROR: Probabilities count ({len(probabilities)}) != labels count ({len(loaded_labels)})."); return "Label Mismatch", 0.0
        predicted_index = np.argmax(probabilities)
        confidence = float(probabilities[predicted_index])
        predicted_label = loaded_labels[predicted_index]
        print(f"Final Prediction: '{predicted_label}', Confidence: {confidence:.4f}")
        print("--- Postprocessing Complete ---")
        return predicted_label, confidence
    except Exception as e: print(f"ERROR: Postprocessing failed: {e}"); return "Post Err", 0.0

# ==================================
# === Sorting Signal Functions ===
# ==================================
def send_sorting_signal(material_label): # Unchanged from 3.0.13
    if not SORTING_GPIO_ENABLED: print("Sorting Signal: GPIO for sorting not enabled. Skipping send."); return
    if not RPi_GPIO_AVAILABLE: print("Sorting Signal: RPi.GPIO library not available. Cannot send signal."); return

    print(f"\n--- Sending Sorting Signal for: {material_label} ---")
    mid_val, lsb_val = GPIO.LOW, GPIO.LOW
    signal_desc = "Others (00)"
    if material_label == "Aluminum": mid_val, lsb_val, signal_desc = GPIO.LOW, GPIO.HIGH, "Aluminum (01)"
    elif material_label == "Copper": mid_val, lsb_val, signal_desc = GPIO.HIGH, GPIO.LOW, "Copper (10)"
    elif material_label == "Steel": mid_val, lsb_val, signal_desc = GPIO.HIGH, GPIO.HIGH, "Steel (11)"

    try:
        GPIO.output(SORTING_DATA_READY_PIN, GPIO.LOW)
        time.sleep(0.01)
        GPIO.output(SORTING_DATA_PIN_MID, mid_val)
        GPIO.output(SORTING_DATA_PIN_LSB, lsb_val)
        print(f"Set GPIO Pins: MID={mid_val}, LSB={lsb_val} for {signal_desc}")
        time.sleep(0.01)
        GPIO.output(SORTING_DATA_READY_PIN, GPIO.HIGH)
        print(f"Pulsed {SORTING_DATA_READY_PIN} HIGH (Data Ready)")
        time.sleep(0.05)
        GPIO.output(SORTING_DATA_READY_PIN, GPIO.LOW)
        print(f"Set {SORTING_DATA_READY_PIN} LOW (Data Transmitted)")
        time.sleep(0.01)
        GPIO.output(SORTING_DATA_PIN_MID, GPIO.LOW)
        GPIO.output(SORTING_DATA_PIN_LSB, GPIO.LOW)
        print(f"Data pins ({SORTING_DATA_PIN_MID}, {SORTING_DATA_PIN_LSB}) reset to LOW after signal.")
        print(f"--- Sorting signal {signal_desc} sent ---")
    except Exception as e:
        print(f"ERROR: Failed to send sorting signal via GPIO: {e}")
        try:
            if RPi_GPIO_AVAILABLE:
                GPIO.output(SORTING_DATA_READY_PIN, GPIO.LOW)
                GPIO.output(SORTING_DATA_PIN_MID, GPIO.LOW)
                GPIO.output(SORTING_DATA_PIN_LSB, GPIO.LOW)
                print("Ensured sorting pins are LOW after error.")
        except Exception as e_cleanup: print(f"Warning: Could not reset pins after error: {e_cleanup}")

# ==============================
# === View Switching Logic ===
# ==============================
def show_live_view():
    global live_view_frame, results_view_frame, lv_classify_button, interpreter, classification_pending_enable
    if results_view_frame and results_view_frame.winfo_ismapped():
        results_view_frame.pack_forget()
    if live_view_frame and not live_view_frame.winfo_ismapped():
        live_view_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

    # Manage classify button state considering the new enable logic
    if lv_classify_button:
        if classification_pending_enable:
            lv_classify_button.config(state=tk.DISABLED, text="Waiting for Enable...")
        elif interpreter: # AI is ready and not waiting for enable
            lv_classify_button.config(state=tk.NORMAL, text="Capture & Classify")
        else: # AI not ready
            lv_classify_button.config(state=tk.DISABLED, text="Classify (AI Failed)")

def show_results_view(): # Unchanged from 3.0.13
    global live_view_frame, results_view_frame
    if live_view_frame and live_view_frame.winfo_ismapped():
        live_view_frame.pack_forget()
    if results_view_frame and not results_view_frame.winfo_ismapped():
        results_view_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

# ======================
# === GUI Functions ===
# ======================
def create_placeholder_image(width, height, color='#E0E0E0', text="No Image"): # Unchanged from 3.0.13
    try:
        pil_img = Image.new('RGB', (width, height), color); tk_img = ImageTk.PhotoImage(pil_img)
        return tk_img
    except Exception as e: print(f"Warning: Failed to create placeholder image: {e}"); return None

def clear_results_display(): # Unchanged from 3.0.13
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
    global classification_pending_enable # New global state

    print("\n" + "="*10 + " Capture & Classify Triggered " + "="*10)
    if not interpreter:
        messagebox.showerror("Error", "AI Model is not initialized. Cannot classify.")
        print("Classification aborted: AI not ready."); return
    if not camera or not camera.isOpened():
        messagebox.showerror("Error", "Camera is not available. Cannot capture image.")
        print("Classification aborted: Camera not ready."); return

    # Disable button immediately - will be re-enabled by GPIO signal or if process fails early
    if lv_classify_button:
        lv_classify_button.config(state=tk.DISABLED, text="Processing...")
    window.update_idletasks()

    ret, frame = camera.read()
    if not ret or frame is None:
        messagebox.showerror("Capture Error", "Failed to capture image from camera.")
        print("ERROR: Failed to read frame from camera.")
        # Re-enable button if capture fails before classification cycle completes
        # or rely on show_live_view to set correct state based on 'classification_pending_enable'
        show_live_view()
        return
    try:
        img_captured_pil = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    except Exception as e:
        messagebox.showerror("Image Error", f"Failed to process captured image: {e}")
        print(f"ERROR: Failed converting captured frame to PIL Image: {e}")
        show_live_view()
        return

    # ... (Sensor reading and AI processing - same as 3.0.13) ...
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
    if sensor_warning: print("WARNING: Sensor issues may affect classification.")

    model_inputs = preprocess_input(img_captured_pil, current_mag_mT, current_rp_raw)
    if model_inputs is None:
        messagebox.showerror("AI Error", "Data preprocessing failed."); print("ERROR: Preprocessing abort.")
        show_live_view(); return

    output_data = run_inference(model_inputs)
    if output_data is None:
        messagebox.showerror("AI Error", "AI model inference failed."); print("ERROR: Inference abort.")
        show_live_view(); return

    predicted_label, confidence = postprocess_output(output_data)
    print(f"--- Classification Result: Prediction='{predicted_label}', Confidence={confidence:.1%} ---")
    send_sorting_signal(predicted_label)

    # Update Results Display (same as 3.0.13)
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

    # --- NEW: Disable classify button and set flag to wait for enable signal ---
    classification_pending_enable = True
    if lv_classify_button:
        lv_classify_button.config(state=tk.DISABLED, text="Waiting for Enable...")
    print("Classification complete. Waiting for enable signal on GPIO", ENABLE_CLASSIFICATION_PIN)


def calibrate_sensors(): # Unchanged from 3.0.13
    global IDLE_VOLTAGE, IDLE_RP_VALUE, window, previous_filtered_mag_mT
    global lv_calibrate_button, lv_classify_button, hall_sensor, ldc_initialized, classification_pending_enable

    print("\n" + "="*10 + " Sensor Calibration Triggered " + "="*10)
    hall_avail, ldc_avail = hall_sensor is not None, ldc_initialized
    if not hall_avail and not ldc_avail:
        messagebox.showwarning("Calibration", "Neither Hall nor LDC sensor is available."); print("Aborted: No sensors."); return
    instr = "Ensure NO metal object is near sensors.\n\n"
    if hall_avail: instr += "- Hall sensor idle voltage will be measured.\n"
    if ldc_avail: instr += "- LDC sensor idle RP value will be measured.\n"
    instr += "\nClick OK to start calibration."
    if not messagebox.askokcancel("Calibration Instructions", instr): print("Calibration cancelled."); return

    if lv_calibrate_button: lv_calibrate_button.config(state=tk.DISABLED)
    # Also disable classify button during calibration
    original_classify_text = lv_classify_button.cget("text") if lv_classify_button else "Capture & Classify"
    if lv_classify_button: lv_classify_button.config(state=tk.DISABLED)
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

    previous_filtered_mag_mT = None
    if lv_calibrate_button: lv_calibrate_button.config(state=tk.NORMAL)

    # Restore classify button state considering 'classification_pending_enable'
    if lv_classify_button:
        if classification_pending_enable:
            lv_classify_button.config(state=tk.DISABLED, text="Waiting for Enable...")
        elif interpreter:
            lv_classify_button.config(state=tk.NORMAL, text="Capture & Classify")
        else: # AI not ready
            lv_classify_button.config(state=tk.DISABLED, text="Classify (AI Failed)")


    msg = f"Calibration Results:\n\n{hall_res}\n{ldc_res}"
    print("--- Calibration Complete ---")
    if (hall_avail and not hall_ok) or (ldc_avail and not ldc_ok): messagebox.showwarning("Calibration Warning", msg)
    else: messagebox.showinfo("Calibration Complete", msg)
    print("="*10 + " Sensor Calibration Finished " + "="*10 + "\n")


def update_camera_feed(): # Unchanged from 3.0.13
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
        else:
            if not hasattr(lv_camera_label, 'no_cam_img'):
                lv_camera_label.no_cam_img = create_placeholder_image(DISPLAY_IMG_WIDTH // 2, DISPLAY_IMG_HEIGHT // 2, '#BDBDBD', "No Feed")
            if lv_camera_label.no_cam_img and str(lv_camera_label.cget("image")) != str(lv_camera_label.no_cam_img):
                 lv_camera_label.configure(image=lv_camera_label.no_cam_img, text=""); lv_camera_label.img_tk = lv_camera_label.no_cam_img
            elif not lv_camera_label.no_cam_img and lv_camera_label.cget("text") != "Camera Failed":
                 lv_camera_label.configure(image='', text="Camera Failed"); lv_camera_label.img_tk = None
    if window and window.winfo_exists(): window.after(CAMERA_UPDATE_INTERVAL_MS, update_camera_feed)

def update_magnetism(): # Unchanged from 3.0.13
    global lv_magnetism_label, window, previous_filtered_mag_mT, IDLE_VOLTAGE, hall_sensor
    if not window or not window.winfo_exists(): return
    display_text = "N/A"
    if hall_sensor:
        avg_v = get_averaged_hall_voltage(num_samples=NUM_SAMPLES_PER_UPDATE)
        if avg_v is not None:
            try:
                if abs(SENSITIVITY_V_PER_MILLITESLA) < 1e-9: raise ZeroDivisionError("Sens0")
                raw_mT = (avg_v - IDLE_VOLTAGE) / SENSITIVITY_V_PER_MILLITESLA
                if previous_filtered_mag_mT is None: previous_filtered_mag_mT = raw_mT
                filt_mT = (MAGNETISM_FILTER_ALPHA * raw_mT) + ((1-MAGNETISM_FILTER_ALPHA)*previous_filtered_mag_mT)
                previous_filtered_mag_mT = filt_mT
                if abs(filt_mT) < 0.1: display_text = f"{filt_mT*1000:+.1f}µT"
                else: display_text = f"{filt_mT:+.2f}mT"
                if IDLE_VOLTAGE == 0.0: display_text += "(NoCal)"
            except Exception: display_text = "CalcErr"; previous_filtered_mag_mT=None
        else: display_text = "ReadErr"; previous_filtered_mag_mT=None
    if lv_magnetism_label and lv_magnetism_label.cget("text") != display_text: lv_magnetism_label.config(text=display_text)
    if window and window.winfo_exists(): window.after(GUI_UPDATE_INTERVAL_MS, update_magnetism)

def update_ldc_reading(): # Unchanged from 3.0.13
    global lv_ldc_label, window, RP_DISPLAY_BUFFER, IDLE_RP_VALUE, ldc_initialized
    if not window or not window.winfo_exists(): return
    display_text = "N/A"
    if ldc_initialized:
        avg_rp = get_averaged_rp_data(num_samples=NUM_SAMPLES_PER_UPDATE)
        if avg_rp is not None:
            RP_DISPLAY_BUFFER.append(avg_rp)
            if RP_DISPLAY_BUFFER:
                cur_rp = int(round(statistics.mean(RP_DISPLAY_BUFFER)))
                delta = cur_rp - IDLE_RP_VALUE
                display_text = f"{cur_rp}"
                if IDLE_RP_VALUE != 0: display_text += f"(Δ{delta:+,})"
                else: display_text += "(NoCal)"
            else: display_text = "Buffer..."
        else: display_text = "ReadErr"
    if lv_ldc_label and lv_ldc_label.cget("text") != display_text: lv_ldc_label.config(text=display_text)
    if window and window.winfo_exists(): window.after(GUI_UPDATE_INTERVAL_MS, update_ldc_reading)

# --- NEW: Function to check the enable signal GPIO pin ---
def check_enable_signal():
    global window, lv_classify_button, classification_pending_enable, ENABLE_PIN_SETUP_OK, RPi_GPIO_AVAILABLE

    if not window or not window.winfo_exists():
        return # Stop if window is closed

    if classification_pending_enable and ENABLE_PIN_SETUP_OK and RPi_GPIO_AVAILABLE:
        try:
            if GPIO.input(ENABLE_CLASSIFICATION_PIN) == GPIO.HIGH:
                print(f"Enable signal DETECTED on GPIO {ENABLE_CLASSIFICATION_PIN}!")
                classification_pending_enable = False
                if lv_classify_button:
                    # Re-enable based on AI readiness
                    if interpreter:
                        lv_classify_button.config(state=tk.NORMAL, text="Capture & Classify")
                    else:
                        lv_classify_button.config(state=tk.DISABLED, text="Classify (AI Failed)")
                # No need to reschedule if we only act once per classification disable cycle.
                # However, for continuous operation, we always reschedule.
            # else:
                # Signal not high, do nothing, will check again.
        except Exception as e:
            print(f"Error reading Enable Classification Pin {ENABLE_CLASSIFICATION_PIN}: {e}")
            # Optionally disable further checks or handle error
            # For now, we'll let it keep trying.

    # Always reschedule the check
    if window.winfo_exists(): # Check again, as previous operations might take time
        window.after(CHECK_ENABLE_INTERVAL_MS, check_enable_signal)


# ======================
# === GUI Setup ========
# ======================
def setup_gui(): # Unchanged from 3.0.13
    global window, main_frame, placeholder_img_tk, live_view_frame, results_view_frame
    global lv_camera_label, lv_magnetism_label, lv_ldc_label, lv_classify_button, lv_calibrate_button
    global rv_image_label, rv_prediction_label, rv_confidence_label, rv_magnetism_label, rv_ldc_label, rv_classify_another_button
    global label_font, readout_font, button_font, title_font, result_title_font, result_value_font, pred_font

    print("Setting up GUI...")
    window = tk.Tk()
    window.title("AI Metal Classifier v3.0.14 (RPi Combined)")
    window.geometry("800x600")
    style = ttk.Style()
    available_themes = style.theme_names(); style.theme_use('clam' if 'clam' in available_themes else 'default')
    try:
        font_family = "DejaVu Sans"
        title_font = tkFont.Font(family=font_family, size=16, weight="bold"); label_font = tkFont.Font(family=font_family, size=10)
        readout_font = tkFont.Font(family=font_family+" Mono", size=12, weight="bold"); button_font = tkFont.Font(family=font_family, size=10, weight="bold")
        result_title_font = tkFont.Font(family=font_family, size=11, weight="bold"); result_value_font = tkFont.Font(family=font_family+" Mono", size=12, weight="bold")
        pred_font = tkFont.Font(family=font_family, size=16, weight="bold")
    except tk.TclError:
        print("Warning: DejaVu fonts not found, using Tkinter default fonts.")
        title_font=tkFont.nametofont("TkHeadingFont"); label_font=tkFont.nametofont("TkTextFont"); readout_font=tkFont.nametofont("TkFixedFont")
        button_font=tkFont.nametofont("TkDefaultFont"); result_title_font=tkFont.nametofont("TkDefaultFont"); result_value_font=tkFont.nametofont("TkFixedFont")
        pred_font=tkFont.nametofont("TkHeadingFont")

    style.configure("TLabel", font=label_font, padding=2); style.configure("TButton", font=button_font, padding=(8,5))
    style.configure("Readout.TLabel", font=readout_font, foreground="#0000AA"); style.configure("ResultValue.TLabel", font=result_value_font, foreground="#0000AA")
    style.configure("Prediction.TLabel", font=pred_font, foreground="#AA0000")

    main_frame = ttk.Frame(window, padding="5 5 5 5"); main_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
    main_frame.rowconfigure(0, weight=1); main_frame.columnconfigure(0, weight=1)

    live_view_frame = ttk.Frame(main_frame, padding="5 5 5 5"); live_view_frame.columnconfigure(0, weight=3); live_view_frame.columnconfigure(1, weight=1); live_view_frame.rowconfigure(0, weight=1)
    lv_camera_label = ttk.Label(live_view_frame, text="Initializing Camera...", anchor="center", borderwidth=1, relief="sunken", background="#CCCCCC")
    lv_camera_label.grid(row=0, column=0, padx=(0, 5), pady=0, sticky="nsew")
    lv_controls_frame = ttk.Frame(live_view_frame); lv_controls_frame.grid(row=0, column=1, sticky="nsew", padx=(5,0)); lv_controls_frame.columnconfigure(0, weight=1)
    lv_readings_frame = ttk.Labelframe(lv_controls_frame, text=" Live Readings ", padding="8 4 8 4"); lv_readings_frame.grid(row=0, column=0, sticky="new", pady=(0, 10)); lv_readings_frame.columnconfigure(1, weight=1)
    ttk.Label(lv_readings_frame, text="Magnetism:").grid(row=0, column=0, sticky="w", padx=(0, 8)); lv_magnetism_label = ttk.Label(lv_readings_frame, text="Init...", style="Readout.TLabel"); lv_magnetism_label.grid(row=0, column=1, sticky="ew")
    ttk.Label(lv_readings_frame, text="LDC (Delta):").grid(row=1, column=0, sticky="w", padx=(0, 8), pady=(2,0)); lv_ldc_label = ttk.Label(lv_readings_frame, text="Init...", style="Readout.TLabel"); lv_ldc_label.grid(row=1, column=1, sticky="ew", pady=(2,0))
    lv_actions_frame = ttk.Labelframe(lv_controls_frame, text=" Actions ", padding="8 4 8 8"); lv_actions_frame.grid(row=1, column=0, sticky="new", pady=(0,10)); lv_actions_frame.columnconfigure(0, weight=1)
    lv_classify_button = ttk.Button(lv_actions_frame, text="Capture & Classify", command=capture_and_classify); lv_classify_button.grid(row=0, column=0, sticky="ew", pady=(4,4))
    lv_calibrate_button = ttk.Button(lv_actions_frame, text="Calibrate Sensors", command=calibrate_sensors); lv_calibrate_button.grid(row=1, column=0, sticky="ew", pady=(4,4))

    results_view_frame = ttk.Frame(main_frame, padding="10 10 10 10"); results_view_frame.rowconfigure(0, weight=1); results_view_frame.rowconfigure(1, weight=0); results_view_frame.rowconfigure(2, weight=1)
    results_view_frame.columnconfigure(0, weight=1); results_view_frame.columnconfigure(1, weight=0); results_view_frame.columnconfigure(2, weight=1)
    rv_content_frame = ttk.Frame(results_view_frame); rv_content_frame.grid(row=1, column=1, sticky="")
    ttk.Label(rv_content_frame, text="Classification Result", font=title_font).grid(row=0, column=0, columnspan=2, pady=(5, 15))
    placeholder_h = int(RESULT_IMG_DISPLAY_WIDTH * 0.75); placeholder_img_tk = create_placeholder_image(RESULT_IMG_DISPLAY_WIDTH, placeholder_h)
    rv_image_label = ttk.Label(rv_content_frame, anchor="center", borderwidth=1, relief="sunken")
    if placeholder_img_tk: rv_image_label.config(image=placeholder_img_tk); rv_image_label.img_tk = placeholder_img_tk
    else: rv_image_label.config(text="Image Area", width=30, height=15)
    rv_image_label.grid(row=1, column=0, columnspan=2, pady=(0, 15))
    rv_details_frame = ttk.Frame(rv_content_frame); rv_details_frame.grid(row=2, column=0, columnspan=2, pady=(0,15)); rv_details_frame.columnconfigure(1, weight=1); res_row = 0
    ttk.Label(rv_details_frame, text="Material:", font=result_title_font).grid(row=res_row, column=0, sticky="w", padx=(0,5)); rv_prediction_label = ttk.Label(rv_details_frame, text="---", style="Prediction.TLabel"); rv_prediction_label.grid(row=res_row, column=1, sticky="ew", padx=5); res_row += 1
    ttk.Label(rv_details_frame, text="Confidence:", font=result_title_font).grid(row=res_row, column=0, sticky="w", padx=(0,5), pady=(3,0)); rv_confidence_label = ttk.Label(rv_details_frame, text="---", style="ResultValue.TLabel"); rv_confidence_label.grid(row=res_row, column=1, sticky="ew", padx=5, pady=(3,0)); res_row += 1
    ttk.Separator(rv_details_frame, orient='horizontal').grid(row=res_row, column=0, columnspan=2, sticky='ew', pady=8); res_row += 1
    ttk.Label(rv_details_frame, text="Sensor Values Used:", font=result_title_font).grid(row=res_row, column=0, columnspan=2, sticky="w", pady=(0,3)); res_row += 1
    ttk.Label(rv_details_frame, text=" Magnetism:", font=result_title_font).grid(row=res_row, column=0, sticky="w", padx=(5,5)); rv_magnetism_label = ttk.Label(rv_details_frame, text="---", style="ResultValue.TLabel"); rv_magnetism_label.grid(row=res_row, column=1, sticky="ew", padx=5); res_row += 1
    ttk.Label(rv_details_frame, text=" LDC Reading:", font=result_title_font).grid(row=res_row, column=0, sticky="w", padx=(5,5)); rv_ldc_label = ttk.Label(rv_details_frame, text="---", style="ResultValue.TLabel"); rv_ldc_label.grid(row=res_row, column=1, sticky="ew", padx=5); res_row += 1
    rv_classify_another_button = ttk.Button(rv_content_frame, text="<< Classify Another", command=show_live_view); rv_classify_another_button.grid(row=3, column=0, columnspan=2, pady=(15, 5))

    clear_results_display()
    show_live_view() # This will set initial classify button state
    print("GUI setup complete.")

# ==========================
# === Main Execution =======
# ==========================
def run_application():
    global window, lv_camera_label, lv_magnetism_label, lv_ldc_label, lv_classify_button, lv_calibrate_button
    global interpreter, camera, hall_sensor, ldc_initialized, classification_pending_enable

    print("Setting up GUI...")
    try: setup_gui()
    except Exception as e:
        print(f"FATAL ERROR: Failed to set up GUI: {e}"); traceback.print_exc()
        try: root_err = tk.Tk(); root_err.withdraw(); messagebox.showerror("GUI Setup Error", f"GUI Init Failed:\n{e}\n\nConsole for details."); root_err.destroy()
        except Exception: pass
        return

    # Initial state of classify button is handled by show_live_view called in setup_gui
    if not camera and lv_camera_label: lv_camera_label.configure(text="Camera Failed", image='')
    if not hall_sensor and lv_magnetism_label: lv_magnetism_label.config(text="N/A")
    if not ldc_initialized and lv_ldc_label: lv_ldc_label.config(text="N/A")
    if not (hall_sensor or ldc_initialized) and lv_calibrate_button:
        lv_calibrate_button.config(state=tk.DISABLED, text="Calibrate (No Sensors)")

    print("Starting GUI update loops...")
    update_camera_feed()
    update_magnetism()
    update_ldc_reading()
    check_enable_signal() # Start polling for the enable signal

    print("Starting Tkinter main loop... (Press Ctrl+C in console to exit)")
    try:
        window.protocol("WM_DELETE_WINDOW", on_closing)
        window.mainloop()
    except Exception as e: print(f"ERROR: Exception in Tkinter main loop: {e}")
    print("Tkinter main loop finished.")

def on_closing(): # Unchanged from 3.0.13
    global window
    print("Window close requested by user.")
    if messagebox.askokcancel("Quit", "Do you want to quit the AI Metal Classifier application?"):
        print("Proceeding with shutdown...")
        if window: window.quit()
    else: print("Shutdown cancelled by user.")

# ==========================
# === Cleanup Resources ====
# ==========================
def cleanup_resources(): # Unchanged from 3.0.13
    print("\n--- Cleaning up resources ---")
    global camera, spi, ldc_initialized, CS_PIN, RPi_GPIO_AVAILABLE, SORTING_GPIO_ENABLED
    if camera and camera.isOpened():
        try: print("Releasing camera..."); camera.release(); print("Camera released.")
        except Exception as e: print(f"Warning: Error releasing camera: {e}")
    if spi:
        try:
            if ldc_initialized and RPi_GPIO_AVAILABLE and CS_PIN is not None:
                print("Putting LDC1101 to sleep...")
                try:
                    if ldc_write_register(START_CONFIG_REG, SLEEP_MODE): print("LDC sleep command sent.")
                    else: print("Note: Failed to send LDC sleep command.")
                except Exception as ldc_e: print(f"Note: Error sending LDC sleep cmd: {ldc_e}")
        finally:
            try: print("Closing LDC SPI..."); spi.close(); print("LDC SPI closed.")
            except Exception as e: print(f"Warning: Error closing LDC SPI: {e}")
    if RPi_GPIO_AVAILABLE:
        try:
            current_gpio_mode = GPIO.getmode()
            if current_gpio_mode is not None:
                print(f"Cleaning up GPIO (mode: {current_gpio_mode})..."); GPIO.cleanup(); print("GPIO cleaned up.")
            else: print("Note: GPIO mode not set/already cleaned, skipping cleanup.")
        except RuntimeError as e: print(f"Note: GPIO cleanup runtime error: {e}")
        except Exception as e: print(f"Warning: Error during GPIO cleanup: {e}")
    else: print("Note: RPi.GPIO not available, skipping cleanup.")
    print("--- Cleanup complete ---")

# ==========================
# === Main Entry Point =====
# ==========================
if __name__ == '__main__': # Unchanged from 3.0.13
    print("="*30 + "\n Starting AI Metal Classifier (RPi Combined) \n" + "="*30)
    hw_init_attempted = False
    try:
        initialize_hardware(); hw_init_attempted = True
        initialize_ai()
        run_application()
    except KeyboardInterrupt: print("\nKeyboard interrupt detected. Exiting application.")
    except Exception as e:
        print("\n" + "="*30 + f"\nFATAL ERROR in main execution: {e}\n" + "="*30); traceback.print_exc()
        if 'window' in globals() and window and window.winfo_exists():
            try: messagebox.showerror("Fatal Application Error", f"Unrecoverable error:\n\n{e}\n\nPlease check console.")
            except Exception: pass
    finally:
        if 'window' in globals() and window:
            try:
                if window.winfo_exists(): print("Ensuring Tkinter window is destroyed..."); window.destroy(); print("Tkinter window destroyed.")
            except tk.TclError: print("Note: Tkinter window already destroyed.")
            except Exception as e: print(f"Warning: Error destroying Tkinter window: {e}")
        if hw_init_attempted: cleanup_resources()
        else: print("Skipping resource cleanup as hardware init not fully attempted.")
        print("\nApplication finished.\n" + "="*30)
