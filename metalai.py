# CODE 3.0.9 - AI Metal Classifier GUI with Results Page
# Description: Displays live sensor data and camera feed.
#              Captures image and sensor readings, classifies metal using a TFLite model,
#              and displays the results on a dedicated page. Includes debug prints.
# Version: 3.0.9 - Corrected poor style and potential syntax issue inside try-except block in capture_and_classify.
# FIXED:    Proper indentation and line breaks applied throughout for readability and correctness.
# DEBUG:    Added more detailed prints in capture_and_classify and preprocess_input
#           to diagnose incorrect prediction issue. Added warning if sensor data is missing.

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
        # Consider exiting gracefully or disabling AI features
        exit()

try:
    import joblib
except ImportError:
    print("ERROR: Joblib is not installed.")
    print("Please install it: pip install joblib")
    # Consider exiting gracefully or disabling AI features
    exit()

# --- I2C/ADS1115 Imports ---
I2C_ENABLED = False # Default to False
try:
    import board
    import busio
    import adafruit_ads1x15.ads1115 as ADS
    from adafruit_ads1x15.analog_in import AnalogIn
    I2C_ENABLED = True
except ImportError:
    print("Warning: I2C/ADS1115 libraries not found. Magnetism readings disabled.")
except NotImplementedError:
    print("Warning: I2C not supported on this platform. Magnetism readings disabled.")
except Exception as e:
    print(f"Warning: Error importing I2C/ADS1115 libraries: {e}. Magnetism readings disabled.")

# --- SPI/LDC1101 Imports ---
SPI_ENABLED = False # Default to False
try:
    import spidev
    import RPi.GPIO as GPIO
    SPI_ENABLED = True
except ImportError:
    print("Warning: SPI/GPIO libraries (spidev, RPi.GPIO) not found. LDC readings disabled.")
except RuntimeError:
     print("Warning: RPi.GPIO library likely requires root privileges. LDC readings disabled.")
except Exception as e:
    print(f"Warning: Error importing SPI/GPIO libraries: {e}. LDC readings disabled.")

# ==================================
# === Constants and Configuration ===
# ==================================

# Accuracy/Stability/Speed
NUM_SAMPLES_PER_UPDATE = 3      # Number of sensor reads averaged for each live GUI update
NUM_SAMPLES_CALIBRATION = 10    # Number of sensor reads averaged during calibration
GUI_UPDATE_INTERVAL_MS = 100    # How often (ms) to update sensor readings in the GUI
CAMERA_UPDATE_INTERVAL_MS = 40  # How often (ms) to update the camera feed in the GUI
LDC_DISPLAY_BUFFER_SIZE = 5     # Number of LDC readings to average for display smoothing
MAGNETISM_FILTER_ALPHA = 0.2    # Smoothing factor (0-1) for magnetism display (lower = smoother)

# Camera
CAMERA_INDEX = 0                # Index of the camera (usually 0, 1, etc.)
DISPLAY_IMG_WIDTH = 640         # Target width for the live camera feed display
DISPLAY_IMG_HEIGHT = 480        # Target height for the live camera feed display
RESULT_IMG_DISPLAY_WIDTH = 280  # Width for the captured image shown on the results page

# --- AI Model Configuration ---
try:
    # Get the directory where the script is located
    BASE_PATH = os.path.dirname(os.path.abspath(__file__))
except NameError:
    # Fallback if __file__ is not defined (e.g., in interactive mode)
    BASE_PATH = os.getcwd()

MODEL_FILENAME = "material_classifier_model.tflite"
LABELS_FILENAME = "material_labels.txt"
SCALER_FILENAME = "numerical_scaler.joblib"

MODEL_PATH = os.path.join(BASE_PATH, MODEL_FILENAME)
LABELS_PATH = os.path.join(BASE_PATH, LABELS_FILENAME)
SCALER_PATH = os.path.join(BASE_PATH, SCALER_FILENAME)

AI_IMG_WIDTH = 224              # Model's expected image width
AI_IMG_HEIGHT = 224             # Model's expected image height

# Hall Sensor (ADS1115)
# Define HALL_ADC_CHANNEL based on whether I2C is enabled
HALL_ADC_CHANNEL = ADS.P0 if I2C_ENABLED else None
SENSITIVITY_V_PER_TESLA = 0.0004 # V/T - IMPORTANT: Check your specific sensor datasheet!
SENSITIVITY_V_PER_MILLITESLA = SENSITIVITY_V_PER_TESLA * 1000 # V/mT
IDLE_VOLTAGE = 0.0              # Default idle voltage (V), recalibrated on startup/request

# Inductive Sensor (LDC1101)
SPI_BUS = 0                     # SPI bus number
SPI_DEVICE = 0                  # SPI device (chip select) number
SPI_SPEED = 500000              # SPI clock speed in Hz (500 kHz)
SPI_MODE = 0b00                 # SPI mode (CPOL=0, CPHA=0)
CS_PIN = 8                      # BCM Pin number for Chip Select (GPIO 8)

LDC_CHIP_ID = 0xD4              # Expected Chip ID for LDC1101

# LDC Registers (Hex Addresses)
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

# LDC Modes
ACTIVE_CONVERSION_MODE = 0x00
SLEEP_MODE = 0x01

# Calibration
IDLE_RP_VALUE = 0               # Default idle RP value, recalibrated

# Global Hardware Objects (initialized later)
camera = None
i2c = None
ads = None
hall_sensor = None
spi = None
ldc_initialized = False

# Global AI Objects (initialized later)
interpreter = None
input_details = None
output_details = None
loaded_labels = []
numerical_scaler = None

# Global State
RP_DISPLAY_BUFFER = deque(maxlen=LDC_DISPLAY_BUFFER_SIZE)
previous_filtered_mag_mT = None # Used for smoothing magnetism display

# --- GUI Globals ---
window = None
main_frame = None
live_view_frame = None
results_view_frame = None

# Fonts
label_font = None
readout_font = None
button_font = None
title_font = None
result_title_font = None
result_value_font = None

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

# Placeholder image object
placeholder_img_tk = None

# =========================
# === Hardware Setup ===
# =========================
def initialize_hardware():
    """Initializes Camera, I2C/ADS1115, and SPI/LDC1101."""
    global camera, i2c, ads, hall_sensor, spi, ldc_initialized
    print("--- Initializing Hardware ---")

    # --- Camera Initialization ---
    try:
        camera = cv2.VideoCapture(CAMERA_INDEX)
        if not camera or not camera.isOpened():
            # Raise an error if camera failed to open, handled by except block
            raise ValueError(f"Could not open camera at index {CAMERA_INDEX}")
        print(f"Camera {CAMERA_INDEX} opened successfully.")
    except Exception as e:
        print(f"ERROR: Failed to open camera {CAMERA_INDEX}: {e}")
        camera = None # Ensure camera is None if initialization fails

    # --- I2C/ADS1115 Initialization ---
    if I2C_ENABLED:
        try:
            # Initialize I2C bus
            i2c = busio.I2C(board.SCL, board.SDA)
            # Initialize ADS1115 ADC
            ads = ADS.ADS1115(i2c)
            # Create an analog input channel for the Hall sensor
            hall_sensor = AnalogIn(ads, HALL_ADC_CHANNEL)
            # Set gain if needed (optional, default is 2/3)
            # hall_sensor.gain = 1
            print("I2C bus and ADS1115 initialized successfully.")
        except ValueError as e:
             print(f"ERROR: Initializing I2C/ADS1115 failed. Check connections/address: {e}")
             i2c = None
             ads = None
             hall_sensor = None
        except Exception as e:
            print(f"ERROR: An unexpected error occurred during I2C/ADS1115 initialization: {e}")
            i2c = None
            ads = None
            hall_sensor = None # Ensure hall_sensor is None on failure
    else:
        print("Skipping I2C/ADS1115 setup (libraries not found or disabled).")

    # --- SPI/LDC1101 Initialization ---
    if SPI_ENABLED:
        try:
            # Configure GPIO
            GPIO.setwarnings(False) # Disable GPIO warnings
            GPIO.setmode(GPIO.BCM) # Use BCM pin numbering
            GPIO.setup(CS_PIN, GPIO.OUT) # Set CS pin as output
            GPIO.output(CS_PIN, GPIO.HIGH) # Set CS pin high (inactive)
            print("GPIO initialized successfully.")

            # Configure SPI
            spi = spidev.SpiDev()
            spi.open(SPI_BUS, SPI_DEVICE)
            spi.max_speed_hz = SPI_SPEED
            spi.mode = SPI_MODE
            print(f"SPI initialized (Bus={SPI_BUS}, Device={SPI_DEVICE}, Speed={SPI_SPEED}Hz, Mode={SPI_MODE}).")

            # Initialize LDC1101 sensor
            if initialize_ldc1101():
                # If LDC init is successful, enable RP mode
                enable_ldc_rpmode()
                print("LDC1101 initialized and RP mode enabled.")
            else:
                # Handle LDC initialization failure
                print("ERROR: LDC1101 Initialization Failed. Check connections/wiring.")
                # No need to set ldc_initialized = False here, initialize_ldc1101 does it
                # We might still have SPI, but LDC isn't usable
                # spi.close() # Optionally close SPI if LDC fails, depends on desired behavior
                # spi = None

        except Exception as e:
            print(f"ERROR: An error occurred during GPIO/SPI/LDC initialization: {e}")
            # Cleanup GPIO if SPI setup failed partway
            if spi:
                spi.close()
            # GPIO cleanup might be needed here if setup failed after setmode
            # try:
            #     GPIO.cleanup()
            # except Exception as gpio_e:
            #     print(f"Note: Error during GPIO cleanup after SPI failure: {gpio_e}")
            spi = None
            ldc_initialized = False # Ensure LDC is marked as not initialized
    else:
        print("Skipping SPI/GPIO/LDC1101 setup (libraries not found or disabled).")

    print("--- Hardware Initialization Complete ---")

# =========================
# === AI Model Setup ======
# =========================
def initialize_ai():
    """Loads labels, scaler, TFLite model, and allocates tensors."""
    global interpreter, input_details, output_details, loaded_labels, numerical_scaler
    print("--- Initializing AI Components ---")
    ai_ready = True # Flag to track overall success

    # --- Load Labels ---
    try:
        with open(LABELS_PATH, 'r') as f:
            loaded_labels = [line.strip() for line in f.readlines()]
        if not loaded_labels:
            raise ValueError("Labels file is empty.")
        print(f"Loaded {len(loaded_labels)} labels: {loaded_labels}")
    except FileNotFoundError:
        print(f"ERROR: Labels file not found: {LABELS_PATH}")
        ai_ready = False
    except Exception as e:
        print(f"ERROR: Failed to read labels file: {e}")
        ai_ready = False

    # --- Load Scaler ---
    if ai_ready: # Only proceed if previous steps were successful
        try:
            numerical_scaler = joblib.load(SCALER_PATH)
            print(f"Loaded numerical scaler from {SCALER_PATH}")
            # Basic check if the loaded object has the 'transform' method
            if not hasattr(numerical_scaler, 'transform'):
                raise TypeError("Loaded scaler object does not have a 'transform' method.")
            # Check scaler's expected number of features (if available)
            if hasattr(numerical_scaler, 'n_features_in_'):
                 print(f"Scaler expects {numerical_scaler.n_features_in_} features.")
            else:
                 print("Warning: Cannot verify number of features expected by scaler.")
        except FileNotFoundError:
            print(f"ERROR: Scaler file not found: {SCALER_PATH}")
            ai_ready = False
        except TypeError as e:
             print(f"ERROR: Scaler file seems corrupted or invalid: {e}")
             ai_ready = False
        except Exception as e:
            print(f"ERROR: Failed to load numerical scaler: {e}")
            ai_ready = False

    # --- Load TFLite Model ---
    if ai_ready:
        try:
            # Suppress potential NumPy warnings during model loading if necessary
            with warnings.catch_warnings():
                warnings.filterwarnings("ignore", message="The value of the smallest subnormal.*", category=UserWarning)
                interpreter = Interpreter(model_path=MODEL_PATH)

            # Allocate memory for the model's tensors
            interpreter.allocate_tensors()

            # Get input and output tensor details
            input_details = interpreter.get_input_details()
            output_details = interpreter.get_output_details()
            print(f"Loaded TFLite model from {MODEL_PATH}")
            print(f"Input Details: {input_details}")   # Debug print
            print(f"Output Details: {output_details}") # Debug print

            # Sanity check: Compare model output size with the number of labels
            output_shape = output_details[0]['shape']
            num_classes_in_model = output_shape[-1] # Usually the last dimension
            if num_classes_in_model != len(loaded_labels):
                print(f"WARNING: Model output size ({num_classes_in_model}) does not match the number of loaded labels ({len(loaded_labels)}). Predictions might be incorrect.")
                # Depending on severity, you might want to set ai_ready = False here

        except FileNotFoundError:
            print(f"ERROR: Model file not found: {MODEL_PATH}")
            ai_ready = False
        except ValueError as e:
             print(f"ERROR: Error initializing TFLite interpreter. Model file might be corrupted or invalid: {e}")
             ai_ready = False
        except Exception as e:
            print(f"ERROR: Failed to load TFLite model: {e}")
            import traceback
            traceback.print_exc() # Print detailed traceback for debugging
            ai_ready = False

    # --- Final Check ---
    if ai_ready:
        print("--- AI Initialization Complete ---")
    else:
        print("--- AI Initialization Failed ---")
        # Ensure AI components are None if initialization failed
        interpreter = None
        input_details = None
        output_details = None
        loaded_labels = []
        numerical_scaler = None

    return ai_ready

# =========================
# === LDC1101 Functions ===
# =========================

def ldc_write_register(reg_addr, value):
    """Writes a single byte value to an LDC1101 register via SPI."""
    if not spi:
        # print("Warning: SPI not available for LDC write.") # Optional debug
        return
    try:
        GPIO.output(CS_PIN, GPIO.LOW) # Activate chip select
        # Send register address (MSB=0 for write) and the value
        spi.xfer2([reg_addr & 0x7F, value])
        GPIO.output(CS_PIN, GPIO.HIGH) # Deactivate chip select
    except Exception as e:
        print(f"Warning: Error during LDC write (Register 0x{reg_addr:02X}). Error: {e}")
        # Attempt to ensure CS is high even if an error occurred
        try:
            GPIO.output(CS_PIN, GPIO.HIGH)
        except Exception as inner_e:
            print(f"Warning: Failed to force CS HIGH after LDC write error. Inner error: {inner_e}")
            # Continue anyway, but this might indicate a deeper issue

def ldc_read_register(reg_addr):
    """Reads a single byte value from an LDC1101 register via SPI."""
    if not spi:
        # print("Warning: SPI not available for LDC read.") # Optional debug
        return 0 # Return a default value (e.g., 0) if SPI isn't working

    read_value = 0 # Default value in case of error
    try:
        GPIO.output(CS_PIN, GPIO.LOW) # Activate chip select
        # Send register address (MSB=1 for read) and a dummy byte (0x00)
        result = spi.xfer2([reg_addr | 0x80, 0x00])
        GPIO.output(CS_PIN, GPIO.HIGH) # Deactivate chip select
        read_value = result[1] # The second byte received is the register value
    except Exception as e:
        print(f"Warning: Error during LDC read (Register 0x{reg_addr:02X}). Error: {e}")
        # Attempt to ensure CS is high even if an error occurred
        try:
            GPIO.output(CS_PIN, GPIO.HIGH)
        except Exception as inner_e:
            print(f"Warning: Failed to force CS HIGH after LDC read error. Inner error: {inner_e}")
            # Continue, returning the default value
    finally:
        # Ensure CS is high in most cases (might already be high)
        # This finally block might be redundant if the try/except covers it,
        # but adds an extra layer of safety.
        try:
             # Check if CS pin exists and is low before setting high
             if CS_PIN is not None and GPIO.input(CS_PIN) == GPIO.LOW:
                  GPIO.output(CS_PIN, GPIO.HIGH)
        except NameError: # Handle case where GPIO might not be defined
             pass
        except RuntimeError: # Handle case where GPIO might not be setup
             pass
        except Exception:
             pass # Ignore other errors during final cleanup check

    return read_value

def initialize_ldc1101():
    """Initializes and configures the LDC1101 sensor. Returns True on success, False on failure."""
    global ldc_initialized
    ldc_initialized = False # Assume failure until proven otherwise

    if not spi:
        print("Cannot initialize LDC1101: SPI is not available.")
        return False

    try:
        # 1. Verify Chip ID
        chip_id = ldc_read_register(CHIP_ID_REG)
        if chip_id != LDC_CHIP_ID:
            print(f"ERROR: LDC Chip ID mismatch! Read 0x{chip_id:02X}, Expected 0x{LDC_CHIP_ID:02X}")
            return False # Stop initialization if chip ID is wrong
        print(f"LDC1101 Chip ID verified (0x{chip_id:02X}).")

        # 2. Configure LDC Registers (adjust values based on datasheet and application)
        print("Configuring LDC1101 registers...")
        ldc_write_register(RP_SET_REG, 0x07)       # RP_MAX=16kOhm, RP_MIN=2kOhm (Example)
        ldc_write_register(TC1_REG, 0x90)          # Sensor Frequency Setting 1 (Example)
        ldc_write_register(TC2_REG, 0xA0)          # Sensor Frequency Setting 2 (Example)
        ldc_write_register(DIG_CONFIG_REG, 0x03)   # Min/Max Frequency Setting (Example)
        ldc_write_register(ALT_CONFIG_REG, 0x00)   # High Current Sensor Drive Disabled (Example)
        ldc_write_register(D_CONF_REG, 0x00)       # Configure L measurement (Example)
        ldc_write_register(INTB_MODE_REG, 0x00)    # Interrupt Pin Disabled (Example)

        # 3. Put LDC into Sleep Mode initially
        ldc_write_register(START_CONFIG_REG, SLEEP_MODE)
        time.sleep(0.01) # Short delay after configuration

        print("LDC1101 Configuration successful.")
        ldc_initialized = True
        return True

    except Exception as e:
        print(f"ERROR: Exception during LDC1101 Initialization: {e}")
        ldc_initialized = False
        return False

def enable_ldc_powermode(mode):
    """Sets the power mode (Active or Sleep) of the LDC1101."""
    if not spi or not ldc_initialized:
        # print("Warning: Cannot set LDC power mode (SPI not ready or LDC not initialized).") # Optional
        return
    try:
        ldc_write_register(START_CONFIG_REG, mode)
        time.sleep(0.01) # Allow time for mode change
        # print(f"LDC Power Mode set to {'Active' if mode == ACTIVE_CONVERSION_MODE else 'Sleep'}.") # Optional debug
    except Exception as e:
        print(f"Warning: Failed to set LDC power mode: {e}")


def enable_ldc_rpmode():
    """Enables the LDC1101 RP+L measurement mode (Resonant Impedance)."""
    if not spi or not ldc_initialized:
        print("Warning: Cannot enable LDC RP mode (SPI not ready or LDC not initialized).")
        return

    try:
        # Configure for RP+L measurement (adjust if only L is needed)
        ldc_write_register(ALT_CONFIG_REG, 0x00) # Example setting for RP+L
        ldc_write_register(D_CONF_REG, 0x00)     # Example setting for RP+L

        # Set to active conversion mode
        enable_ldc_powermode(ACTIVE_CONVERSION_MODE)
        print("LDC RP+L Mode Enabled.")
    except Exception as e:
        print(f"Warning: Failed to enable LDC RP mode: {e}")


def get_ldc_rpdata():
    """Reads the 16-bit raw RP+L data from the LDC1101. Returns integer or None on error."""
    if not spi or not ldc_initialized:
        # print("Warning: Cannot read LDC RP data (SPI not ready or LDC not initialized).") # Optional
        return None

    try:
        # Read the Most Significant Byte (MSB) and Least Significant Byte (LSB)
        msb = ldc_read_register(RP_DATA_MSB_REG)
        lsb = ldc_read_register(RP_DATA_LSB_REG)

        # Combine MSB and LSB to form the 16-bit value
        rp_data = (msb << 8) | lsb
        return rp_data
    except Exception as e:
        print(f"Warning: Failed to read LDC RP data: {e}")
        return None

# ============================
# === Sensor Reading (Avg) ===
# ============================

def get_averaged_hall_voltage(num_samples=NUM_SAMPLES_PER_UPDATE):
    """
    Reads the Hall sensor voltage multiple times and returns the average.
    Returns the average voltage (float) or None if reading fails or sensor is unavailable.
    """
    if not hall_sensor:
        # print("Debug: Hall sensor not available for reading.") # Optional debug
        return None

    readings = []
    for i in range(num_samples):
        try:
            # Attempt to read the voltage from the ADC channel
            voltage = hall_sensor.voltage
            readings.append(voltage)
            # time.sleep(0.001) # Optional small delay between reads if needed
        except OSError as e:
            # Handle specific I/O errors (e.g., connection issues)
            print(f"Warning: OS Error reading Hall sensor (sample {i+1}/{num_samples}): {e}. Aborting average.")
            return None # Abort averaging if a read fails critically
        except Exception as e:
            # Handle other potential errors during a single read
            print(f"Warning: Error reading Hall sensor voltage (sample {i+1}/{num_samples}): {e}. Aborting average.")
            return None # Abort averaging

    # If the loop completes without errors and readings were collected
    if readings:
        average_voltage = sum(readings) / len(readings)
        return average_voltage
    else:
        # Should not happen if num_samples > 0 and no exceptions occurred, but handle defensively
        print("Warning: No valid Hall sensor readings collected.")
        return None

def get_averaged_rp_data(num_samples=NUM_SAMPLES_PER_UPDATE):
    """
    Reads the LDC RP data multiple times and returns the average.
    Returns the average RP value (float) or None if reading fails or LDC is unavailable.
    """
    if not ldc_initialized:
        # print("Debug: LDC not initialized for reading.") # Optional debug
        return None

    readings = []
    for i in range(num_samples):
        rp_value = get_ldc_rpdata()
        if rp_value is not None:
            readings.append(rp_value)
        else:
            # Handle case where a single read failed
            print(f"Warning: Failed to get LDC RP data for sample {i+1}/{num_samples}. Skipping sample.")
            # Optionally decide whether to abort entirely or just skip the sample
            # return None # Uncomment to abort averaging if any read fails

    # Calculate average only from valid readings
    if readings:
        average_rp = sum(readings) / len(readings)
        return average_rp
    else:
        print("Warning: No valid LDC RP readings collected.")
        return None

# ==========================
# === AI Processing ========
# ==========================

def preprocess_input(image_pil, mag_mT, ldc_rp_delta):
    """
    Prepares the captured image and sensor data for the TFLite model.
    Args:
        image_pil (PIL.Image): The captured image.
        mag_mT (float or None): Magnetism reading in milliTesla.
        ldc_rp_delta (int or None): Change in LDC RP value from idle.
    Returns:
        dict: A dictionary mapping input tensor indices to processed numpy arrays,
              or None if preprocessing fails.
    """
    global numerical_scaler, input_details, interpreter # Need interpreter for input details

    if interpreter is None or input_details is None:
        print("ERROR: AI Model (interpreter/input details) not initialized for preprocessing.")
        return None
    if numerical_scaler is None:
         print("ERROR: Numerical scaler not loaded for preprocessing.")
         return None

    # --- 1. Preprocess Image ---
    try:
        # Resize image to the dimensions expected by the AI model
        img_resized = image_pil.resize((AI_IMG_WIDTH, AI_IMG_HEIGHT), Image.Resampling.LANCZOS) # Use LANCZOS for quality
        # Convert image to RGB (if not already)
        img_rgb = img_resized.convert('RGB')
        # Convert PIL image to NumPy array
        image_np = np.array(img_rgb, dtype=np.float32)
        # Normalize pixel values (common practice: 0-1 range)
        image_np /= 255.0
        # Add batch dimension (model expects batch_size, height, width, channels)
        image_input = np.expand_dims(image_np, axis=0)
        # print(f"Debug Preprocess: Image shape: {image_input.shape}, Type: {image_input.dtype}, Min: {image_input.min():.2f}, Max: {image_input.max():.2f}") # Less verbose debug
    except Exception as e:
        print(f"ERROR: Image preprocessing failed: {e}")
        return None

    # --- 2. Preprocess Numerical Features ---
    # Handle potential None values from sensor readings
    # *** MODIFIED: Explicitly check for None and print warning if defaulting ***
    used_default_mag = False
    used_default_ldc = False

    if mag_mT is None:
        print("DEBUG Preprocess: Magnetism reading was None, defaulting to 0.0 for scaling.")
        mag_mT_val = 0.0
        used_default_mag = True
    else:
        mag_mT_val = mag_mT

    if ldc_rp_delta is None:
        print("DEBUG Preprocess: LDC RP delta was None, defaulting to 0.0 for scaling.")
        ldc_rp_delta_val = 0.0
        used_default_ldc = True
    else:
        ldc_rp_delta_val = float(ldc_rp_delta) # Ensure float

    # Create NumPy array for numerical features (shape: [1, num_features])
    numerical_features = np.array([[mag_mT_val, ldc_rp_delta_val]], dtype=np.float32)
    print(f"DEBUG Preprocess: Raw numerical features BEFORE scaling: {numerical_features}") # Debug

    # Scale numerical features using the loaded scaler
    try:
        # Suppress potential UserWarning about feature names if scaler was trained with pandas
        with warnings.catch_warnings():
            warnings.filterwarnings("ignore", message="X does not have valid feature names.*", category=UserWarning)
            scaled_numerical_features = numerical_scaler.transform(numerical_features)
        print(f"DEBUG Preprocess: Scaled numerical features AFTER scaling: {scaled_numerical_features}") # Debug

        # *** ADDED: Check if scaled values are zero/unchanging ***
        if np.allclose(scaled_numerical_features, np.zeros_like(scaled_numerical_features)) and (used_default_mag or used_default_ldc):
             print("WARNING Preprocess: Scaled numerical features are all zero, likely due to missing sensor input.")
        elif np.allclose(scaled_numerical_features, np.zeros_like(scaled_numerical_features)):
             print("WARNING Preprocess: Scaled numerical features are all zero. Check scaler or sensor calibration/readings.")


    except Exception as e:
        print(f"ERROR: Scaling numerical features failed: {e}")
        # Fallback: Use zeros if scaling fails? Or return None? Returning None is safer.
        # scaled_numerical_features = np.zeros_like(numerical_features)
        return None

    # --- 3. Identify Input Tensor Indices ---
    # (No changes needed here, logic seems okay, relies on debug prints from initialize_ai)
    image_input_index = -1
    numerical_input_index = -1
    num_features_expected = 2 # Expecting 2 numerical features (mag, ldc_delta)

    for detail in input_details:
        shape = detail['shape']
        index = detail['index']
        # Image tensor typically has 4 dimensions (batch, height, width, channels)
        if len(shape) == 4 and shape[1] == AI_IMG_HEIGHT and shape[2] == AI_IMG_WIDTH and shape[3] == 3:
             if image_input_index == -1: # Assign only if not already found
                 image_input_index = index
             else: print("Warning: Found multiple potential image input tensors.") # Handle ambiguity
        # Numerical tensor typically has 2 dimensions (batch, num_features)
        elif len(shape) == 2 and shape[1] == num_features_expected:
             if numerical_input_index == -1: # Assign only if not already found
                 numerical_input_index = index
             else: print("Warning: Found multiple potential numerical input tensors.") # Handle ambiguity

    # --- Fallback Logic (if shape-based identification failed) ---
    if image_input_index == -1 or numerical_input_index == -1:
        print("Warning: Could not reliably determine input tensor indices from shape. Attempting fallback.")
        if len(input_details) == 2: # Only fallback if exactly two inputs exist
            # print("Attempting fallback identification based only on number of dimensions...") # Less verbose
            try:
                # Get shapes and indices of the two inputs
                i0_shape, i1_shape = input_details[0]['shape'], input_details[1]['shape']
                i0_idx, i1_idx = input_details[0]['index'], input_details[1]['index']

                # Assume the 4D tensor is image, 2D is numerical
                if len(i0_shape) == 4 and len(i1_shape) == 2:
                    image_input_index, numerical_input_index = i0_idx, i1_idx
                    print(f"Fallback Success: Assumed Input Index {i0_idx} = Image, Index {i1_idx} = Numerical.")
                # Assume the 2D tensor is numerical, 4D is image
                elif len(i0_shape) == 2 and len(i1_shape) == 4:
                    numerical_input_index, image_input_index = i0_idx, i1_idx
                    print(f"Fallback Success: Assumed Input Index {i0_idx} = Numerical, Index {i1_idx} = Image.")
                else:
                    # Fallback fails if dimensions don't match expected pattern (4D and 2D)
                    print("Fallback Failed: Input tensor dimensions are not the expected 4D and 2D.")
                    return None # Cannot proceed if fallback fails
            except Exception as e:
                 print(f"Fallback Failed: Error during fallback index identification: {e}")
                 return None
        else:
            # Fallback is not possible if the model doesn't have exactly two inputs
            print("Fallback Failed: Model does not have exactly 2 inputs.")
            return None

    # --- Final Check after fallback attempt ---
    if image_input_index == -1 or numerical_input_index == -1:
         print("ERROR: Failed to identify input tensor indices even after fallback.")
         return None

    # --- 4. Prepare Model Input Dictionary ---
    try:
        # Get the expected data types for each input tensor
        img_dtype = next(d['dtype'] for d in input_details if d['index'] == image_input_index)
        num_dtype = next(d['dtype'] for d in input_details if d['index'] == numerical_input_index)

        # Create the dictionary mapping index to the correctly typed numpy array
        model_inputs = {
            image_input_index: image_input.astype(img_dtype),
            numerical_input_index: scaled_numerical_features.astype(num_dtype)
        }
        # print("Debug Preprocess: Model inputs prepared successfully.") # Less verbose
        return model_inputs

    except StopIteration:
         print("ERROR: Could not find details for identified input indices. Model structure mismatch?")
         return None
    except Exception as e:
        print(f"ERROR: Failed to prepare model input dictionary: {e}")
        return None


def run_inference(model_inputs):
    """
    Runs inference using the loaded TFLite model and preprocessed inputs.
    Args:
        model_inputs (dict): Dictionary mapping input tensor indices to numpy arrays.
    Returns:
        numpy.ndarray: The raw output tensor from the model, or None on failure.
    """
    global interpreter, input_details, output_details

    if interpreter is None:
        print("ERROR: TFLite interpreter not initialized. Cannot run inference.")
        return None
    if model_inputs is None:
        print("ERROR: Invalid model inputs provided for inference.")
        return None
    if not output_details:
         print("ERROR: Model output details not available.")
         return None

    try:
        # --- Set Input Tensors ---
        print("DEBUG Inference: Setting input tensors...") # Debug
        for index, data in model_inputs.items():
            # Double-check data type just before setting tensor (optional but safe)
            expected_dtype = next(d['dtype'] for d in input_details if d['index'] == index)
            if data.dtype != expected_dtype:
                # print(f"Warning: Correcting input tensor {index} dtype from {data.dtype} to {expected_dtype}.") # Less verbose
                data = data.astype(expected_dtype)
            # Set the tensor data in the interpreter
            # print(f"DEBUG Inference: Setting tensor index {index} with shape {data.shape} and dtype {data.dtype}") # Verbose Debug
            interpreter.set_tensor(index, data)
        print("DEBUG Inference: Input tensors set.") # Debug

        # --- Run Inference ---
        print("DEBUG Inference: Invoking interpreter...") # Debug
        start_time = time.time() # Optional: time inference
        interpreter.invoke()
        end_time = time.time() # Optional: time inference
        print(f"DEBUG Inference: Interpreter invoked successfully (took {end_time - start_time:.4f}s).") # Debug

        # --- Get Output Tensor ---
        # Assuming the model has a single output tensor (index 0 in output_details)
        output_tensor_index = output_details[0]['index']
        output_data = interpreter.get_tensor(output_tensor_index)
        # print(f"Debug Inference: Raw output data shape: {output_data.shape}, dtype: {output_data.dtype}") # Less verbose

        return output_data

    except StopIteration:
        print("ERROR: Could not find details for input/output indices during inference.")
        return None
    except ValueError as e:
         print(f"ERROR: ValueError during inference (often shape mismatch): {e}")
         return None
    except Exception as e:
        print(f"ERROR: An unexpected error occurred during model inference: {e}")
        import traceback
        traceback.print_exc() # Print detailed traceback
        return None

def postprocess_output(output_data):
    """
    Interprets the model's raw output tensor to get the predicted label and confidence.
    Args:
        output_data (numpy.ndarray): The raw output tensor from the model.
    Returns:
        tuple: (predicted_label (str), confidence (float)), or ("Error", 0.0) on failure.
    """
    global loaded_labels

    if output_data is None:
        print("ERROR: No output data received for postprocessing.")
        return "Error", 0.0
    if not loaded_labels:
        print("ERROR: Labels not loaded. Cannot interpret output.")
        return "No Labels", 0.0

    try:
        # Assuming output_data is shape (1, num_classes) - probabilities
        probabilities = output_data[0]
        # *** MODIFIED: Print probabilities with labels for better debugging ***
        print("DEBUG Postprocess: Raw Probabilities:")
        for i, prob in enumerate(probabilities):
             label = loaded_labels[i] if i < len(loaded_labels) else f"Index {i}"
             print(f"  - {label}: {prob:.4f}")
        # *********************************************************************

        # Find the index of the highest probability
        predicted_index = np.argmax(probabilities)

        # Get the corresponding label from the loaded list
        if predicted_index < len(loaded_labels):
            predicted_label = loaded_labels[predicted_index]
        else:
             print(f"ERROR: Predicted index {predicted_index} is out of bounds for labels list (size {len(loaded_labels)}).")
             return "Index Error", 0.0

        # Get the confidence score (the probability of the predicted class)
        confidence = float(probabilities[predicted_index])

        print(f"DEBUG Postprocess: Final Prediction: '{predicted_label}', Confidence: {confidence:.4f}") # Debug
        return predicted_label, confidence

    except IndexError:
        # Handle cases where output_data might not have the expected shape (e.g., not [0])
        print(f"ERROR: Index error during postprocessing. Output shape might be unexpected: {output_data.shape}")
        return "Shape Err", 0.0
    except Exception as e:
        print(f"ERROR: An unexpected error occurred during postprocessing: {e}")
        return "Post Err", 0.0

# ==============================
# === View Switching Logic ===
# ==============================
def show_live_view():
    """Hides the results view and shows the live camera/sensor view."""
    global live_view_frame, results_view_frame, lv_classify_button

    # Hide the results frame if it exists and is currently packed
    if results_view_frame and results_view_frame.winfo_ismapped():
        results_view_frame.pack_forget()

    # Show the live view frame if it exists and is not currently packed
    if live_view_frame and not live_view_frame.winfo_ismapped():
        live_view_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

    # Re-enable the classify button when switching back to live view
    if lv_classify_button:
        # Only enable if AI is ready
        if interpreter:
             lv_classify_button.config(state=tk.NORMAL)
        else:
             lv_classify_button.config(state=tk.DISABLED)


    # print("Switched to Live View.") # Less verbose

def show_results_view():
    """Hides the live view and shows the classification results view."""
    global live_view_frame, results_view_frame

    # Hide the live view frame if it exists and is currently packed
    if live_view_frame and live_view_frame.winfo_ismapped():
        live_view_frame.pack_forget()

    # Show the results frame if it exists and is not currently packed
    if results_view_frame and not results_view_frame.winfo_ismapped():
        results_view_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

    # print("Switched to Results View.") # Less verbose

# ======================
# === GUI Functions ===
# ======================
def create_placeholder_image(width, height, color='#E0E0E0'):
    """Creates a simple PIL placeholder image and converts it to Tkinter format."""
    try:
        pil_img = Image.new('RGB', (width, height), color)
        tk_img = ImageTk.PhotoImage(pil_img)
        return tk_img
    except Exception as e:
        print(f"Warning: Failed to create placeholder image: {e}")
        return None

def clear_results_display():
    """Clears the classification results display widgets back to placeholder/default states."""
    global rv_image_label, rv_prediction_label, rv_confidence_label
    global rv_magnetism_label, rv_ldc_label, placeholder_img_tk

    # --- Clear Image ---
    if rv_image_label:
        if placeholder_img_tk:
            # Use the pre-generated placeholder if available
            rv_image_label.img_tk = placeholder_img_tk # Keep reference
            rv_image_label.config(image=placeholder_img_tk, text="")
        else:
            # Fallback text if placeholder creation failed
            rv_image_label.config(image='', text="No Image")

    # --- Clear Text Labels ---
    default_text = "..."
    if rv_prediction_label:
        rv_prediction_label.config(text=default_text)
    if rv_confidence_label:
        rv_confidence_label.config(text=default_text)
    if rv_magnetism_label:
        rv_magnetism_label.config(text=default_text)
    if rv_ldc_label:
        rv_ldc_label.config(text=default_text)

    # print("Results display cleared.") # Less verbose


def capture_and_classify():
    """
    Captures an image and sensor data, runs AI classification,
    updates the results page, and switches to the results view.
    """
    global lv_classify_button, window, camera, IDLE_VOLTAGE, IDLE_RP_VALUE
    global rv_image_label, rv_prediction_label, rv_confidence_label
    global rv_magnetism_label, rv_ldc_label
    global interpreter # Check if AI is ready

    # --- Pre-checks ---
    if not interpreter:
        messagebox.showerror("Error", "AI Model is not initialized. Cannot classify.")
        return
    if not camera or not camera.isOpened():
        messagebox.showerror("Error", "Camera is not available. Cannot capture image.")
        return

    # --- Disable Button ---
    if lv_classify_button:
        lv_classify_button.config(state=tk.DISABLED)
    window.update_idletasks() # Ensure GUI updates before blocking operations

    # --- 1. Capture Image ---
    print("--- Starting Capture & Classify ---") # Marker for start
    print("Capturing image...")
    ret, frame = camera.read()
    if not ret or frame is None:
        messagebox.showerror("Capture Error", "Failed to capture image from camera.")
        if lv_classify_button: show_live_view() # Re-enable button by switching view
        return
    # Convert captured frame (BGR) to RGB PIL Image
    try:
        img_captured_pil = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        print("Image captured successfully.")
    except Exception as e:
        messagebox.showerror("Image Error", f"Failed to process captured image: {e}")
        if lv_classify_button: show_live_view() # Re-enable button by switching view
        return

    # --- 2. Read Sensors (using more samples for better accuracy during capture) ---
    print("Reading sensors for classification...")
    # Read Hall Sensor
    avg_voltage = get_averaged_hall_voltage(num_samples=NUM_SAMPLES_CALIBRATION)
    current_mag_mT = None # Initialize default
    mag_display_text = "N/A" # Default display text
    sensor_warning = False # Flag if sensor data is missing/problematic

    if avg_voltage is not None:
        # Voltage reading was successful, proceed with calculation attempt
        try:
            # Use calibrated idle voltage if available, otherwise use current reading (less accurate)
            idle_v = IDLE_VOLTAGE if IDLE_VOLTAGE != 0.0 else avg_voltage
            # Ensure sensitivity is not zero to avoid division error
            if abs(SENSITIVITY_V_PER_MILLITESLA) < 1e-9: # Check against a small epsilon
                raise ZeroDivisionError("Hall sensor sensitivity is zero or too close to zero.")

            # Calculate magnetism
            current_mag_mT = (avg_voltage - idle_v) / SENSITIVITY_V_PER_MILLITESLA
            # Format text only on successful calculation
            mag_display_text = f"{current_mag_mT:+.3f} mT"

        except ZeroDivisionError as e:
            mag_display_text = "Div Zero Err" # Handle division by zero
            print(f"Warning: Magnetism calculation failed - {e}")
            current_mag_mT = None # Ensure it's None if calculation failed
            sensor_warning = True
        except Exception as e:
            mag_display_text = "Calc Error"
            print(f"Warning: Magnetism calculation failed - {e}")
            current_mag_mT = None # Ensure it's None if calculation failed
            sensor_warning = True

        # Append calibration status *after* try-except block, only if calc didn't fail
        if IDLE_VOLTAGE == 0.0 and current_mag_mT is not None: # Check if calc succeeded
             mag_display_text += " (No Cal)"
             # sensor_warning = True # Optionally warn if not calibrated
    else:
        # avg_voltage was None (sensor read failed)
        mag_display_text = "Read Error"
        current_mag_mT = None
        sensor_warning = True

    # Read LDC Sensor
    avg_rp_val = get_averaged_rp_data(num_samples=NUM_SAMPLES_CALIBRATION)
    current_rp_int = None # Integer value of the current reading
    delta_rp = None       # Change from idle value
    ldc_display_text = "N/A" # Default display text

    if avg_rp_val is not None:
        current_rp_int = int(avg_rp_val)
        if IDLE_RP_VALUE != 0:
            # Calculate delta if calibration value exists
            delta_rp = current_rp_int - IDLE_RP_VALUE
            ldc_display_text = f"{current_rp_int} (Delta {delta_rp:+,})" # Show current and delta
        else:
            # No calibration value, just show current reading
            ldc_display_text = f"{current_rp_int} (No Cal)"
            delta_rp = None # Delta is meaningless without calibration
            # sensor_warning = True # Optionally warn if not calibrated
    else:
        # avg_rp_val was None (sensor read failed)
        ldc_display_text = "Read Error"
        delta_rp = None
        sensor_warning = True

    # *** ADDED: Print the exact values being passed to preprocessing ***
    print(f"DEBUG Data for Preprocessing: Magnetism Value (mT): {current_mag_mT}, LDC Delta RP: {delta_rp}")

    # Optionally show a warning if sensor data was problematic before proceeding
    if sensor_warning:
         print("WARNING: One or more sensor readings failed or were uncalibrated. Classification results may be inaccurate.")
         # You could potentially show a messagebox here too, but it might be annoying
         # messagebox.showwarning("Sensor Warning", "Sensor readings missing/uncalibrated. Results may be inaccurate.")

    # --- 3. Preprocess Data for AI ---
    print("Preprocessing data for AI model...")
    # Pass the calculated mag_mT and delta_rp to preprocessing
    model_inputs = preprocess_input(img_captured_pil, current_mag_mT, delta_rp)

    if model_inputs is None:
        messagebox.showerror("AI Error", "Data preprocessing failed. Check console logs for details.")
        if lv_classify_button: show_live_view() # Re-enable button by switching view
        return

    # --- 4. Run AI Inference ---
    print("Running AI inference...")
    output_data = run_inference(model_inputs)

    if output_data is None:
        messagebox.showerror("AI Error", "AI model inference failed. Check console logs for details.")
        if lv_classify_button: show_live_view() # Re-enable button by switching view
        return

    # --- 5. Postprocess AI Output ---
    print("Postprocessing AI output...")
    predicted_label, confidence = postprocess_output(output_data)
    print(f"--- Classification Result: Prediction='{predicted_label}', Confidence={confidence:.1%} ---") # Marker for end result

    # --- 6. Update Results Display ---
    print("Updating results display widgets...")

    # Update Image Label
    if rv_image_label:
        try:
            # Calculate display height maintaining aspect ratio
            w, h = img_captured_pil.size
            aspect = h / w if w > 0 else 1 # Avoid division by zero
            display_h = int(RESULT_IMG_DISPLAY_WIDTH * aspect)
            if display_h <= 0: display_h = int(RESULT_IMG_DISPLAY_WIDTH * 0.75) # Fallback height

            # Resize image for display
            img_disp = img_captured_pil.resize((RESULT_IMG_DISPLAY_WIDTH, display_h), Image.Resampling.LANCZOS)

            # Convert to Tkinter format
            img_tk = ImageTk.PhotoImage(img_disp)

            # Update the label widget
            rv_image_label.img_tk = img_tk # IMPORTANT: Keep reference to avoid garbage collection
            rv_image_label.config(image=img_tk, text="") # Clear any previous text

        except Exception as e:
            print(f"ERROR: Failed to update results image display: {e}")
            # Display error text on the label if image processing fails
            rv_image_label.config(image='', text="Img Error") # Clear image, show error text
            rv_image_label.img_tk = None # Clear reference
    else:
         print("Warning: rv_image_label widget not found during results update.")

    # Update Text Labels
    if rv_prediction_label:
        rv_prediction_label.config(text=f"{predicted_label}")
    if rv_confidence_label:
        rv_confidence_label.config(text=f"{confidence:.1%}") # Format confidence as percentage
    if rv_magnetism_label:
        # Use the text generated during sensor reading phase
        rv_magnetism_label.config(text=mag_display_text)
    if rv_ldc_label:
        # Use the text generated during sensor reading phase
        rv_ldc_label.config(text=ldc_display_text)

    # --- 7. Switch View ---
    # print("Switching to results view.") # Less verbose
    show_results_view()
    # Note: The classify button remains disabled until the user clicks "Classify Another" (show_live_view re-enables it)


def calibrate_sensors():
    """
    Calibrates the idle voltage for the Hall sensor and the idle RP value for the LDC sensor.
    Uses the current readings with no object present as the baseline.
    """
    global IDLE_VOLTAGE, IDLE_RP_VALUE, window
    global previous_filtered_mag_mT # Reset smoothing on calibration
    global lv_calibrate_button, lv_classify_button

    print("--- Starting Sensor Calibration ---")
    # Provide instructions to the user
    if not messagebox.askokcancel("Calibration", "Ensure NO metal object is near the sensors.\n\nClick OK to start calibration, Cancel to abort."):
         print("Calibration cancelled by user.")
         return # Abort if user cancels


    # Disable buttons during calibration
    if lv_calibrate_button: lv_calibrate_button.config(state=tk.DISABLED)
    if lv_classify_button: lv_classify_button.config(state=tk.DISABLED)
    window.update_idletasks()

    hall_results = "Hall Sensor: N/A"
    hall_error = True # Assume error initially
    ldc_results = "LDC Sensor: N/A"
    ldc_error = True # Assume error initially

    # --- Calibrate Hall Sensor ---
    if hall_sensor:
        print("Calibrating Hall sensor...")
        avg_v = get_averaged_hall_voltage(num_samples=NUM_SAMPLES_CALIBRATION)
        if avg_v is not None:
            IDLE_VOLTAGE = avg_v
            hall_results = f"Hall Idle Voltage: {IDLE_VOLTAGE:.4f} V"
            hall_error = False
            print(hall_results)
        else:
            IDLE_VOLTAGE = 0.0 # Reset on error
            hall_results = "Hall Sensor: Calibration Read Error!"
            hall_error = True
            print(hall_results)
    else:
        hall_results = "Hall Sensor: Not Available"
        hall_error = True # Mark as error if not available

    # --- Calibrate LDC Sensor ---
    if ldc_initialized:
        print("Calibrating LDC sensor...")
        avg_rp = get_averaged_rp_data(num_samples=NUM_SAMPLES_CALIBRATION)
        if avg_rp is not None:
            IDLE_RP_VALUE = int(avg_rp)
            ldc_results = f"LDC Idle RP Value: {IDLE_RP_VALUE}"
            ldc_error = False
            print(ldc_results)
        else:
            IDLE_RP_VALUE = 0 # Reset on error
            ldc_results = "LDC Sensor: Calibration Read Error!"
            ldc_error = True
            print(ldc_results)
    else:
        ldc_results = "LDC Sensor: Not Available"
        ldc_error = True # Mark as error if not available

    # Reset magnetism smoothing filter
    previous_filtered_mag_mT = None
    print("Magnetism display smoothing reset.")

    # --- Re-enable Buttons ---
    if lv_calibrate_button: lv_calibrate_button.config(state=tk.NORMAL)
    # Only re-enable classify if AI is ready
    if lv_classify_button:
         if interpreter: lv_classify_button.config(state=tk.NORMAL)
         else: lv_classify_button.config(state=tk.DISABLED)


    # --- Display Results ---
    final_message = f"Calibration Results:\n\n{hall_results}\n{ldc_results}"
    print("--- Calibration Complete ---")

    if hall_error and ldc_error:
         # Both failed or were unavailable
         if not hall_sensor and not ldc_initialized:
              messagebox.showerror("Calibration Error", "Neither Hall nor LDC sensor is available for calibration.")
         else:
              messagebox.showwarning("Calibration Warning", f"Calibration failed for available sensors:\n\n{hall_results}\n{ldc_results}")
    elif hall_error or ldc_error:
        # At least one failed or was unavailable
        messagebox.showwarning("Calibration Warning", final_message)
    else:
        # Both succeeded (or were unavailable but handled gracefully)
        messagebox.showinfo("Calibration Complete", final_message)


def update_camera_feed():
    """Updates the live camera feed label in the GUI (Live View)."""
    global lv_camera_label, window, camera

    # Check if window still exists before proceeding
    if not window or not window.winfo_exists():
        # print("Camera update loop: Window closed, stopping.") # Less verbose
        return

    img_tk = None # Initialize Tkinter image object
    if camera and camera.isOpened():
        ret, frame = camera.read()
        if ret and frame is not None:
            try:
                # Convert BGR frame to RGB
                img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                # Create PIL image
                img_pil = Image.fromarray(img_rgb)
                # Resize for display (use thumbnail for aspect ratio preservation)
                # NEAREST is faster but lower quality, LANCZOS is better quality but slower
                img_pil.thumbnail((DISPLAY_IMG_WIDTH, DISPLAY_IMG_HEIGHT), Image.Resampling.NEAREST)
                # Convert PIL image to Tkinter PhotoImage
                img_tk = ImageTk.PhotoImage(img_pil)
            except Exception as e:
                print(f"Error processing camera frame: {e}")
                img_tk = None # Ensure img_tk is None on error
        # else: # Optional: Handle frame read failure
        #     print("Warning: Failed to read frame from camera.")

    # Update the label widget
    if lv_camera_label:
        if img_tk:
            # Update image and clear text
            lv_camera_label.img_tk = img_tk # Keep reference!
            lv_camera_label.configure(image=img_tk, text="")
        else:
            # Display placeholder or text if camera fails or frame processing error
            # Check if a 'no camera' image exists, create if not
            if not hasattr(lv_camera_label, 'no_cam_img'):
                lv_camera_label.no_cam_img = create_placeholder_image(
                    DISPLAY_IMG_WIDTH // 2, DISPLAY_IMG_HEIGHT // 2, '#BDBDBD'
                )

            # Update only if the current image is not already the placeholder
            # Comparing image objects directly can be tricky, compare configuration string
            current_img_str = str(lv_camera_label.cget("image"))
            placeholder_img_str = str(lv_camera_label.no_cam_img) if lv_camera_label.no_cam_img else ""

            if lv_camera_label.no_cam_img and current_img_str != placeholder_img_str:
                 lv_camera_label.configure(image=lv_camera_label.no_cam_img, text="No Camera Feed")
                 lv_camera_label.img_tk = lv_camera_label.no_cam_img # Keep reference
            elif not lv_camera_label.no_cam_img and lv_camera_label.cget("text") != "Camera Failed":
                 # Fallback text if placeholder failed
                 lv_camera_label.configure(image='', text="Camera Failed")
                 lv_camera_label.img_tk = None

    # Schedule the next update
    # Use try-except to prevent error if window is destroyed between check and after() call
    try:
        if window and window.winfo_exists():
            window.after(CAMERA_UPDATE_INTERVAL_MS, update_camera_feed)
    except tk.TclError:
        pass # Ignore error if window is destroyed


def update_magnetism():
    """Updates the live magnetism reading label in the GUI (Live View)."""
    global lv_magnetism_label, window, previous_filtered_mag_mT, IDLE_VOLTAGE

    # Check if window still exists
    if not window or not window.winfo_exists():
        # print("Magnetism update loop: Window closed, stopping.") # Less verbose
        return

    display_text = "N/A" # Default text

    if hall_sensor:
        # Get averaged voltage reading
        avg_voltage = get_averaged_hall_voltage() # Uses default NUM_SAMPLES_PER_UPDATE

        if avg_voltage is not None:
            try:
                # Use calibrated idle voltage if available, else current reading
                idle_v = IDLE_VOLTAGE if IDLE_VOLTAGE != 0.0 else avg_voltage
                # Check sensitivity before division
                if abs(SENSITIVITY_V_PER_MILLITESLA) < 1e-9:
                    raise ZeroDivisionError("Sensitivity near zero")

                # Calculate raw magnetism
                raw_mag_mT = (avg_voltage - idle_v) / SENSITIVITY_V_PER_MILLITESLA

                # Apply simple exponential smoothing filter for display stability
                if previous_filtered_mag_mT is None:
                    # Initialize filter on first valid reading or after calibration
                    filtered_mag_mT = raw_mag_mT
                else:
                    # Apply smoothing formula
                    filtered_mag_mT = (MAGNETISM_FILTER_ALPHA * raw_mag_mT) + \
                                      ((1 - MAGNETISM_FILTER_ALPHA) * previous_filtered_mag_mT)

                # Update the previous value for the next iteration
                previous_filtered_mag_mT = filtered_mag_mT

                # --- Determine Unit and Format Display Text ---
                # Display in microTesla (µT) if magnitude is small, else milliTesla (mT)
                if abs(filtered_mag_mT) < 0.1:
                    value_uT = filtered_mag_mT * 1000
                    unit = "µT"
                    display_text = f"{value_uT:+.2f} {unit}" # Show sign and 2 decimal places for µT
                else:
                    value_mT = filtered_mag_mT
                    unit = "mT"
                    display_text = f"{value_mT:+.2f} {unit}" # Show sign and 2 decimal places for mT

                # Append calibration status if needed
                if IDLE_VOLTAGE == 0.0:
                    display_text += " (No Cal)"

            except ZeroDivisionError:
                display_text = "Div Zero Err"
                previous_filtered_mag_mT = None # Reset filter on error
            except Exception as e:
                display_text = "Calc Error"
                # print(f"Warning: Magnetism calculation error: {e}") # Less verbose
                previous_filtered_mag_mT = None # Reset filter on error
        else:
            # avg_voltage was None (read error)
            display_text = "Read Err"
            previous_filtered_mag_mT = None # Reset filter on error
    # else: # Hall sensor not available, display_text remains "N/A"

    # Update the GUI label
    if lv_magnetism_label:
        # Avoid updating if text is the same to reduce flicker (optional)
        if lv_magnetism_label.cget("text") != display_text:
             lv_magnetism_label.config(text=display_text)

    # Schedule the next update
    # Use try-except to prevent error if window is destroyed between check and after() call
    try:
        if window and window.winfo_exists():
             window.after(GUI_UPDATE_INTERVAL_MS, update_magnetism)
    except tk.TclError:
        pass # Ignore error if window is destroyed


def update_ldc_reading():
    """Updates the live LDC RP reading label in the GUI (Live View)."""
    global lv_ldc_label, window, RP_DISPLAY_BUFFER, IDLE_RP_VALUE

    # Check if window still exists
    if not window or not window.winfo_exists():
        # print("LDC update loop: Window closed, stopping.") # Less verbose
        return

    display_rp_text = "N/A" # Default text

    if ldc_initialized:
        # Get averaged RP data
        avg_rp_val = get_averaged_rp_data() # Uses default NUM_SAMPLES_PER_UPDATE

        if avg_rp_val is not None:
            # Add the new average reading to the display buffer
            RP_DISPLAY_BUFFER.append(avg_rp_val)

            # Calculate the average of the buffer for smoother display
            if RP_DISPLAY_BUFFER:
                buffer_avg = sum(RP_DISPLAY_BUFFER) / len(RP_DISPLAY_BUFFER)
                current_rp_int = int(buffer_avg) # Use integer part for display

                # Format display text based on calibration status
                if IDLE_RP_VALUE != 0:
                    delta = current_rp_int - IDLE_RP_VALUE
                    display_rp_text = f"{current_rp_int} (Delta {delta:+,})" # Show current and delta
                else:
                    display_rp_text = f"{current_rp_int} (No Cal)" # Show only current value
            else:
                # Buffer might be empty if reads consistently fail right at the start
                display_rp_text = "Buffering..."
        else:
            # avg_rp_val was None (read error)
            RP_DISPLAY_BUFFER.clear() # Clear buffer on read error
            display_rp_text = "Read Err"
    # else: # LDC not initialized, display_rp_text remains "N/A"

    # Update the GUI label
    if lv_ldc_label:
         # Avoid updating if text is the same to reduce flicker (optional)
        if lv_ldc_label.cget("text") != display_rp_text:
            lv_ldc_label.config(text=display_rp_text)

    # Schedule the next update
    # Use try-except to prevent error if window is destroyed between check and after() call
    try:
        if window and window.winfo_exists():
            window.after(GUI_UPDATE_INTERVAL_MS, update_ldc_reading)
    except tk.TclError:
        pass # Ignore error if window is destroyed


# ======================
# === GUI Setup ========
# ======================
def setup_gui():
    """Sets up the main Tkinter GUI window, frames, widgets, and styles."""
    global window, main_frame, placeholder_img_tk, live_view_frame, results_view_frame
    global lv_camera_label, lv_magnetism_label, lv_ldc_label, lv_classify_button, lv_calibrate_button
    global rv_image_label, rv_prediction_label, rv_confidence_label, rv_magnetism_label, rv_ldc_label, rv_classify_another_button
    global label_font, readout_font, button_font, title_font, result_title_font, result_value_font

    # --- Window Setup ---
    window = tk.Tk()
    window.title("AI Metal Classifier v3.0.9 (Debug)") # Updated title
    window.geometry("1000x650") # Initial size, can be adjusted

    # --- Style Configuration ---
    style = ttk.Style()
    # Try to use a modern theme if available
    available_themes = style.theme_names()
    # print(f"Available themes: {available_themes}") # Debug available themes
    if 'clam' in available_themes:
        style.theme_use('clam')
    elif 'alt' in available_themes:
         style.theme_use('alt')
    elif 'vista' in available_themes: # Common on Windows
         style.theme_use('vista')
    else:
         style.theme_use('default') # Fallback

    # --- Define Fonts ---
    # Use common fonts, adjust sizes as needed
    try:
        title_font = tkFont.Font(family="Helvetica", size=16, weight="bold")
        label_font = tkFont.Font(family="Helvetica", size=11)
        readout_font = tkFont.Font(family="Consolas", size=14, weight="bold") # Monospaced for numbers
        button_font = tkFont.Font(family="Helvetica", size=11, weight="bold")
        result_title_font = tkFont.Font(family="Helvetica", size=12, weight="bold")
        result_value_font = tkFont.Font(family="Consolas", size=14, weight="bold") # Monospaced
    except tk.TclError: # Fallback if fonts aren't available
        print("Warning: Preferred fonts not found, using defaults.")
        title_font = tkFont.nametofont("TkHeadingFont")
        label_font = tkFont.nametofont("TkTextFont")
        readout_font = tkFont.nametofont("TkFixedFont")
        button_font = tkFont.nametofont("TkDefaultFont")
        result_title_font = tkFont.nametofont("TkDefaultFont")
        result_value_font = tkFont.nametofont("TkFixedFont")


    # --- Configure Widget Styles ---
    style.configure("TLabel", font=label_font, padding=2)
    style.configure("TButton", font=button_font, padding=(10, 6)) # More padding for buttons
    style.configure("TLabelframe", padding=8)
    style.configure("TLabelframe.Label", font=tkFont.Font(family="Helvetica", size=12, weight="bold"))
    # Custom styles for specific labels
    style.configure("Readout.TLabel", font=readout_font, padding=(5, 1), anchor=tk.E) # Right-align readouts
    style.configure("ResultTitle.TLabel", font=label_font, padding=(5, 2), anchor=tk.W) # Left-align result titles
    style.configure("ResultValue.TLabel", font=result_value_font, padding=(5, 1), anchor=tk.E) # Right-align result values

    # --- Main Content Frame ---
    # This frame holds either the live view or the results view
    main_frame = ttk.Frame(window, padding="5 5 5 5")
    main_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
    # Configure grid weights for resizing
    main_frame.rowconfigure(0, weight=1)
    main_frame.columnconfigure(0, weight=1)

    # ===================================
    # === Live View Frame Construction ===
    # ===================================
    live_view_frame = ttk.Frame(main_frame, padding="5 5 5 5")
    # Grid configuration: Camera feed takes more space
    live_view_frame.columnconfigure(0, weight=3) # Camera column
    live_view_frame.columnconfigure(1, weight=1) # Controls column
    live_view_frame.rowconfigure(0, weight=1)    # Allow vertical expansion

    # --- Live Camera Feed Label ---
    lv_camera_label = ttk.Label(live_view_frame, text="Initializing Camera...",
                                anchor="center", borderwidth=1, relief="sunken",
                                background="#DDDDDD") # Light gray background
    lv_camera_label.grid(row=0, column=0, padx=(0, 10), pady=0, sticky="nsew") # Fill space

    # --- Controls Frame (Right Side) ---
    lv_controls_frame = ttk.Frame(live_view_frame)
    lv_controls_frame.grid(row=0, column=1, sticky="nsew", padx=(5,0))
    lv_controls_frame.columnconfigure(0, weight=1) # Allow controls to expand horizontally if needed

    # --- Live Readings Section ---
    lv_readings_frame = ttk.Labelframe(lv_controls_frame, text=" Live Readings ", padding="10 5 10 5")
    lv_readings_frame.grid(row=0, column=0, sticky="new", pady=(0, 10)) # Stick to top-width
    lv_readings_frame.columnconfigure(1, weight=1) # Allow value labels to expand

    # Magnetism Reading
    ttk.Label(lv_readings_frame, text="Magnetism:").grid(row=0, column=0, sticky="w", padx=(0, 10))
    lv_magnetism_label = ttk.Label(lv_readings_frame, text="Init...", style="Readout.TLabel")
    lv_magnetism_label.grid(row=0, column=1, sticky="ew")

    # LDC Reading
    ttk.Label(lv_readings_frame, text="LDC (Delta):").grid(row=1, column=0, sticky="w", padx=(0, 10), pady=(3, 0))
    lv_ldc_label = ttk.Label(lv_readings_frame, text="Init...", style="Readout.TLabel")
    lv_ldc_label.grid(row=1, column=1, sticky="ew", pady=(3, 0))

    # --- Actions Section ---
    lv_actions_frame = ttk.Labelframe(lv_controls_frame, text=" Actions ", padding="10 5 10 10")
    lv_actions_frame.grid(row=1, column=0, sticky="new", pady=(0, 10)) # Below readings
    lv_actions_frame.columnconfigure(0, weight=1) # Allow buttons to expand horizontally

    # Classify Button
    lv_classify_button = ttk.Button(lv_actions_frame, text="Capture & Classify", command=capture_and_classify)
    lv_classify_button.grid(row=0, column=0, sticky="ew", pady=(5, 5))

    # Calibrate Button
    lv_calibrate_button = ttk.Button(lv_actions_frame, text="Calibrate Sensors", command=calibrate_sensors)
    lv_calibrate_button.grid(row=1, column=0, sticky="ew", pady=(5, 5))

    # =====================================
    # === Results View Frame Construction ===
    # =====================================
    results_view_frame = ttk.Frame(main_frame, padding="10 10 10 10")
    # Configure grid to center content vertically and horizontally
    results_view_frame.rowconfigure(0, weight=1)
    results_view_frame.columnconfigure(0, weight=1)

    # --- Centering Frame for Content ---
    # Use an inner frame to hold the actual results content and center it
    rv_content_frame = ttk.Frame(results_view_frame)
    # Place the content frame in the center of the results_view_frame grid
    rv_content_frame.grid(row=0, column=0, sticky="") # Don't make it sticky to keep it centered

    # --- Results Title ---
    ttk.Label(rv_content_frame, text="Classification Result", font=title_font).grid(row=0, column=0, columnspan=2, pady=(5, 15))

    # --- Placeholder Image for Results ---
    placeholder_h = int(RESULT_IMG_DISPLAY_WIDTH * 0.75) # Default 4:3 aspect ratio
    placeholder_img_tk = create_placeholder_image(RESULT_IMG_DISPLAY_WIDTH, placeholder_h, '#E0E0E0')

    # --- Result Image Label ---
    rv_image_label = ttk.Label(rv_content_frame, anchor="center", borderwidth=1, relief="sunken")
    if placeholder_img_tk:
        rv_image_label.config(image=placeholder_img_tk)
        rv_image_label.img_tk = placeholder_img_tk # Keep reference
    else:
        rv_image_label.config(text="Image Area") # Fallback text
    rv_image_label.grid(row=1, column=0, columnspan=2, pady=(0, 15))

    # --- Results Details Frame ---
    rv_details_frame = ttk.Frame(rv_content_frame)
    rv_details_frame.grid(row=2, column=0, columnspan=2, pady=(0, 15))
    rv_details_frame.columnconfigure(1, weight=1) # Allow value labels to expand

    res_row = 0 # Row counter for details grid

    # Predicted Material
    ttk.Label(rv_details_frame, text="Material:", style="ResultTitle.TLabel").grid(row=res_row, column=0, sticky="w", padx=5)
    rv_prediction_label = ttk.Label(rv_details_frame, text="...", style="ResultValue.TLabel")
    rv_prediction_label.grid(row=res_row, column=1, sticky="ew", padx=5)
    res_row += 1

    # Confidence Score
    ttk.Label(rv_details_frame, text="Confidence:", style="ResultTitle.TLabel").grid(row=res_row, column=0, sticky="w", padx=5)
    rv_confidence_label = ttk.Label(rv_details_frame, text="...", style="ResultValue.TLabel")
    rv_confidence_label.grid(row=res_row, column=1, sticky="ew", padx=5)
    res_row += 1

    # Separator
    ttk.Separator(rv_details_frame, orient='horizontal').grid(row=res_row, column=0, columnspan=2, sticky='ew', pady=8)
    res_row += 1

    # Magnetism Reading Used
    ttk.Label(rv_details_frame, text="Magnetism Used:", style="ResultTitle.TLabel").grid(row=res_row, column=0, sticky="w", padx=5)
    rv_magnetism_label = ttk.Label(rv_details_frame, text="...", style="ResultValue.TLabel")
    rv_magnetism_label.grid(row=res_row, column=1, sticky="ew", padx=5)
    res_row += 1

    # LDC Reading Used
    ttk.Label(rv_details_frame, text="LDC (Delta) Used:", style="ResultTitle.TLabel").grid(row=res_row, column=0, sticky="w", padx=5)
    rv_ldc_label = ttk.Label(rv_details_frame, text="...", style="ResultValue.TLabel")
    rv_ldc_label.grid(row=res_row, column=1, sticky="ew", padx=5)
    res_row += 1

    # --- Classify Another Button ---
    # Place this button below the details frame, centered
    rv_classify_another_button = ttk.Button(rv_content_frame, text="<< Classify Another", command=show_live_view)
    # Use the main content frame's grid for this button
    rv_classify_another_button.grid(row=3, column=0, columnspan=2, pady=(10, 5)) # Use row 3

    # --- Set Initial State ---
    clear_results_display() # Initialize results widgets with placeholders
    show_live_view()        # Start by showing the live view

    # print("GUI setup complete.") # Less verbose


# ==========================
# === Main Execution =======
# ==========================
def run_application():
    """Sets up the GUI, initializes update loops, and runs the main Tkinter event loop."""
    global window, lv_camera_label, lv_magnetism_label, lv_ldc_label, lv_classify_button, interpreter

    # --- Setup GUI ---
    print("Setting up GUI...")
    try:
        setup_gui()
    except Exception as e:
        print(f"FATAL ERROR: Failed to set up GUI: {e}")
        # Attempt to show error in a basic Tkinter window if main setup failed
        try:
            root = tk.Tk()
            root.withdraw() # Hide the main window
            messagebox.showerror("GUI Setup Error", f"Failed to initialize the application GUI:\n{e}")
            root.destroy()
        except Exception:
            pass # Ignore if even basic Tk window fails
        return # Exit if GUI setup fails

    # --- Update Initial Status Labels based on Hardware/AI Init ---
    if not camera:
        if lv_camera_label: lv_camera_label.configure(text="Camera Failed", image='')
        else: print("Warning: lv_camera_label not available to show camera status.")

    if not hall_sensor:
        if lv_magnetism_label: lv_magnetism_label.config(text="N/A")
        else: print("Warning: lv_magnetism_label not available to show Hall status.")

    if not ldc_initialized:
        if lv_ldc_label: lv_ldc_label.config(text="N/A")
        else: print("Warning: lv_ldc_label not available to show LDC status.")

    if not interpreter:
        if lv_classify_button:
            lv_classify_button.config(state=tk.DISABLED)
            # Don't show messagebox here, it's annoying on startup, rely on console/disabled button
            # messagebox.showwarning("AI Initialization Failed",
            #                        "AI components failed to initialize.\nClassification is disabled.")
            print("AI Initialization Failed - Classification Disabled.")
        else:
            print("Warning: lv_classify_button not available to disable.")
            print("AI Initialization Failed - Classification Disabled.")


    # --- Start Update Loops ---
    print("Starting GUI update loops...")
    # Start camera feed update first (more frequent)
    update_camera_feed()
    # Start sensor updates
    update_magnetism()
    update_ldc_reading()

    # --- Run Tkinter Main Loop ---
    print("Starting Tkinter main loop...")
    try:
        # Add protocol handler for window close
        window.protocol("WM_DELETE_WINDOW", on_closing)
        window.mainloop()
    except Exception as e:
         print(f"ERROR: An exception occurred in the Tkinter main loop: {e}")
         # Log the error, cleanup will happen in the finally block of main execution

    print("Tkinter main loop finished.")


# ==========================
# === Window Closing =======
# ==========================
def on_closing():
    """Handles window close event, ensuring cleanup."""
    global window
    print("Window close requested.")
    if messagebox.askokcancel("Quit", "Do you want to quit the application?"):
        print("Proceeding with shutdown...")
        # Stop update loops if possible (by setting a flag or destroying window)
        # Destroying the window should stop the 'after' calls eventually
        if window:
            window.destroy() # This will break the mainloop
        # Cleanup should happen in the finally block of the main execution
    else:
        print("Shutdown cancelled.")


# ==========================
# === Cleanup Resources ====
# ==========================
def cleanup_resources():
    """Releases hardware resources gracefully."""
    print("--- Cleaning up resources ---")

    # --- Release Camera ---
    global camera
    if camera and camera.isOpened():
        try:
            camera.release()
            print("Camera released.")
        except Exception as e:
            print(f"Warning: Error releasing camera: {e}")
    # Close any OpenCV windows that might have been opened (less likely here)
    cv2.destroyAllWindows()

    # --- Cleanup SPI/LDC ---
    global spi, ldc_initialized, SPI_ENABLED
    if spi: # Check if spi object exists
        try:
            # Put LDC to sleep before closing SPI if it was initialized
            if ldc_initialized:
                print("Putting LDC1101 to sleep...")
                # Use a direct write here, avoid functions that might check spi again
                try:
                     # Ensure CS pin is configured before using it
                     if CS_PIN is not None:
                         GPIO.output(CS_PIN, GPIO.LOW)
                         spi.xfer2([START_CONFIG_REG & 0x7F, SLEEP_MODE])
                         GPIO.output(CS_PIN, GPIO.HIGH)
                         time.sleep(0.05) # Short delay
                         print("LDC sleep command sent.")
                     else:
                          print("Note: CS_PIN not defined, cannot send sleep command.")
                except NameError: # Handle case where GPIO might not be defined
                     print("Note: GPIO not available, cannot send sleep command.")
                except RuntimeError: # Handle case where GPIO might not be setup
                     print("Note: GPIO not setup, cannot send sleep command.")
                except Exception as ldc_e:
                     print(f"Note: Error sending sleep command to LDC: {ldc_e}")
        except Exception as e:
            # Catch errors related to accessing ldc_initialized or spi during cleanup
             print(f"Note: Error during LDC sleep attempt in cleanup: {e}")
        finally:
            # Always try to close SPI if the object exists
            try:
                spi.close()
                print("SPI closed.")
            except Exception as e:
                print(f"Warning: Error closing SPI: {e}")

    # --- Cleanup GPIO ---
    # Only cleanup GPIO if the SPI library was successfully imported initially
    if SPI_ENABLED:
        try:
            # Check if GPIO library was imported and mode was set
            if 'GPIO' in globals() and GPIO.getmode() is not None:
                 GPIO.cleanup()
                 print("GPIO cleaned up.")
            else:
                 print("Note: GPIO mode not set or library not fully available, skipping cleanup.")
        except NameError:
             print("Note: GPIO library was enabled but likely failed import/setup; cleanup skipped.")
        except RuntimeError as e:
             # Catch runtime errors like "cannot determine pin numbering system" if setup failed badly
             print(f"Note: GPIO cleanup runtime error (may be normal if setup failed): {e}")
        except Exception as e:
            # Catch any other unexpected errors during cleanup
            print(f"Warning: Error during GPIO cleanup: {e}")

    print("--- Cleanup complete ---")


# ==========================
# === Main Entry Point =====
# ==========================
if __name__ == '__main__':
    hardware_init_attempted = False
    ai_init_ok = False
    app_running = True # Flag to control loops if needed (though window destroy is primary)

    try:
        # --- Initialize Hardware ---
        hardware_init_attempted = True # Mark that we tried
        initialize_hardware()
        # Note: initialize_hardware prints its own errors/warnings

        # --- Initialize AI ---
        # Only proceed if hardware init didn't cause immediate exit (though it shouldn't)
        ai_init_ok = initialize_ai()
        # Note: initialize_ai prints its own errors/warnings and returns status

        # --- Run Application ---
        # The application should run even if some components failed,
        # as run_application handles disabling features based on init status.
        run_application() # This enters the mainloop

    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        print("\nKeyboard interrupt detected. Exiting application.")
        app_running = False
        # Try to destroy window if it exists
        try:
             if window and window.winfo_exists():
                  window.destroy()
        except Exception:
             pass
        # Cleanup will happen in the finally block

    except Exception as e:
        # Catch any unexpected fatal errors during initialization or the main loop run
        app_running = False
        print("\n" + "="*30)
        print(f"FATAL ERROR in main execution: {e}")
        print("="*30)
        # Attempt to show error in GUI if possible (might fail if error was in GUI setup)
        try:
            if window and window.winfo_exists():
                messagebox.showerror("Fatal Application Error",
                                     f"An unrecoverable error occurred:\n\n{e}\n\nCheck console for details.")
        except Exception:
            pass # Ignore if GUI isn't available or fails

        # Print full traceback to console for debugging
        import traceback
        traceback.print_exc()
        # Try to destroy window if it exists after fatal error
        try:
             if window and window.winfo_exists():
                  window.destroy()
        except Exception:
             pass


    finally:
        # --- Cleanup ---
        # Ensure cleanup runs regardless of how the application exits (normal, interrupt, error)
        # Only attempt hardware cleanup if initialization was attempted
        if hardware_init_attempted:
            cleanup_resources()
        else:
             print("Skipping resource cleanup as hardware initialization was not attempted.")

        print("\nApplication finished.")
```

**Summary of Changes:**

1.  **`capture_and_classify`:**
    * Added a `DEBUG` print statement right before calling `preprocess_input` to show the exact `current_mag_mT` and `delta_rp` values being used.
    * Added a `sensor_warning` flag. If any sensor read fails or is uncalibrated (`None` value passed to preprocessing), a warning is printed to the console.
2.  **`preprocess_input`:**
    * Added `DEBUG` prints to explicitly state when `mag_mT` or `ldc_rp_delta` is `None` and is being defaulted to `0.0`.
    * Added `DEBUG` prints to show the `numerical_features` array *before* and *after* scaling by `numerical_scaler.transform()`.
    * Added a `WARNING` print if the scaled features become all zeros, which is a strong indicator of a problem.
3.  **`postprocess_output`:**
    * Modified the debug print to show the raw probabilities alongside their corresponding labels from `loaded_labels`. This makes it much easier to see *why* a certain class is being chosen.
4.  **`calibrate_sensors`:** Added a confirmation dialog (`askokcancel`) before starting calibration.
5.  **`on_closing`:** Added a basic window closing handler (`WM_DELETE_WINDOW` protocol) to ask the user for confirmation before quitting and ensure cleanup runs.
6.  **Minor:** Reduced verbosity of some less critical debug prints (e.g., "Switched to view"). Made button re-enabling dependent on `interpreter` being ready. Improved robustness of `cleanup_resources` slightly.

**How to Use for Debugging:**

1.  **Run the updated script.**
2.  **Calibrate:** Perform the sensor calibration first (`Calibrate Sensors` button). Check the console output to ensure `IDLE_VOLTAGE` and `IDLE_RP_VALUE` are set to reasonable non-zero values.
3.  **Classify:** Place a known material (e.g., Aluminum, which should be non-magnetic and affect LDC differently than Steel) and press `Capture & Classify`.
4.  **Examine Console Output:** Look closely at the `DEBUG` and `WARNING` messages printed during the "Capture & Classify" process:
    * **`DEBUG Data for Preprocessing:`**: Are the `Magnetism Value (mT)` and `LDC Delta RP` non-zero and different from what you'd expect for an empty reading or for Steel?
    * **`DEBUG Preprocess: Raw numerical features BEFORE scaling:`**: Does this match the values printed above?
    * **`DEBUG Preprocess: Scaled numerical features AFTER scaling:`**: Are these values non-zero? Do they change significantly when you test different materials? If they are always `[[0. 0.]]` or very similar, the problem is likely the raw sensor data or the scaler itself.
    * **`DEBUG Postprocess: Raw Probabilities:`**: Look at the list of probabilities printed with labels. Is the probability for "Steel" always close to 1.0 (100%) and others near 0.0, even if the scaled numerical features look reasonable? If yes, the issue might be more complex (e.g., model expecting different scaling, feature importance issues). If the probabilities *do* change but Steel still wins, the numerical features might still not be distinct enough.

By examining these specific printouts, you should get a much clearer idea of where the data pipeline is going wrong and why "Steel" is always being predict
