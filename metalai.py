# CODE 3.0.9 - AI Metal Classifier GUI with Results Page
# Description: Displays live sensor data and camera feed.
#              Captures image and sensor readings, classifies metal using a TFLite model,
#              and displays the results on a dedicated page. Includes debug prints.
# Version: 3.0.9 - Corrected poor style and potential syntax issue inside try-except block in capture_and_classify.
# FIXED:     Proper indentation and line breaks applied throughout for readability and correctness.
# DEBUG:     Added more detailed prints in capture_and_classify and preprocess_input
#            to diagnose incorrect prediction issue. Added warning if sensor data is missing.

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
        # Consider exiting gracefully or disabling AI features
        exit()

# Try importing joblib for loading the scaler
try:
    import joblib
except ImportError:
    print("ERROR: Joblib is not installed.")
    print("Please install it: pip install joblib")
    # Consider exiting gracefully or disabling AI features
    exit()

# --- I2C/ADS1115 Imports (for Hall Sensor/Magnetism) ---
I2C_ENABLED = False # Default to False, set True if libraries import successfully
try:
    import board        # Adafruit Blinka library for hardware pins
    import busio        # For I2C communication
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

# --- SPI/LDC1101 Imports (for Inductive Sensor) ---
SPI_ENABLED = False # Default to False, set True if libraries import successfully
try:
    import spidev     # For SPI communication
    import RPi.GPIO as GPIO # For controlling Chip Select pin
    SPI_ENABLED = True
    print("SPI/GPIO libraries imported successfully.")
except ImportError:
    print("Warning: SPI/GPIO libraries (spidev, RPi.GPIO) not found.")
    print("Please install them (e.g., 'pip install spidev RPi.GPIO').")
    print("LDC readings will be disabled.")
except RuntimeError:
     print("Warning: RPi.GPIO library likely requires root privileges (sudo). LDC readings may fail if not run as root.")
     # SPI_ENABLED might still be technically true, but GPIO access will fail later
except Exception as e:
    print(f"Warning: Error importing SPI/GPIO libraries: {e}. LDC readings disabled.")

# ==================================
# === Constants and Configuration ===
# ==================================

# --- Accuracy/Stability/Speed ---
NUM_SAMPLES_PER_UPDATE = 3      # Number of sensor reads averaged for each live GUI update (faster, less smooth)
NUM_SAMPLES_CALIBRATION = 15    # Number of sensor reads averaged during calibration (slower, more stable)
GUI_UPDATE_INTERVAL_MS = 100    # How often (ms) to update sensor readings in the GUI (e.g., 100ms = 10 Hz)
CAMERA_UPDATE_INTERVAL_MS = 50  # How often (ms) to update the camera feed (e.g., 50ms = 20 FPS) - Adjust based on performance
LDC_DISPLAY_BUFFER_SIZE = 5     # Number of recent LDC readings to average for display smoothing
MAGNETISM_FILTER_ALPHA = 0.2    # Smoothing factor (0-1) for magnetism display (lower = smoother, more lag)

# --- Camera ---
CAMERA_INDEX = 0                # Index of the camera (usually 0 for built-in/first USB, 1, etc.)
DISPLAY_IMG_WIDTH = 640         # Target width for the live camera feed display in the GUI
DISPLAY_IMG_HEIGHT = 480        # Target height for the live camera feed display in the GUI
RESULT_IMG_DISPLAY_WIDTH = 280  # Width for the captured image shown on the results page

# --- AI Model Configuration ---
try:
    # Get the directory where the script is located
    BASE_PATH = os.path.dirname(os.path.abspath(__file__))
except NameError:
    # Fallback if __file__ is not defined (e.g., running in an interactive interpreter)
    BASE_PATH = os.getcwd()
    print(f"Warning: __file__ not defined, using current working directory as base path: {BASE_PATH}")

# Filenames for the AI components (expected in the same directory as the script)
MODEL_FILENAME = "material_classifier_model.tflite"
LABELS_FILENAME = "material_labels.txt"
SCALER_FILENAME = "numerical_scaler.joblib" # For scaling numerical sensor data

# Construct full paths
MODEL_PATH = os.path.join(BASE_PATH, MODEL_FILENAME)
LABELS_PATH = os.path.join(BASE_PATH, LABELS_FILENAME)
SCALER_PATH = os.path.join(BASE_PATH, SCALER_FILENAME)

# AI Model's Expected Input Dimensions (IMPORTANT: Must match your trained model)
AI_IMG_WIDTH = 224              # Model's expected image width
AI_IMG_HEIGHT = 224             # Model's expected image height
# Note: The number of numerical features is implicitly determined by the scaler

# --- Hall Sensor (ADS1115 Configuration) ---
# Define HALL_ADC_CHANNEL based on whether I2C is enabled and libraries loaded
# Adjust ADS.P0 to ADS.P1, ADS.P2, or ADS.P3 depending on which ADC pin the sensor is connected to
HALL_ADC_CHANNEL = ADS.P0 if I2C_ENABLED else None
# IMPORTANT: Check your specific Hall effect sensor datasheet for these values!
# Sensitivity (e.g., Volts per Tesla or milliTesla)
SENSITIVITY_V_PER_TESLA = 0.0004 # Example: 0.4mV/mT = 0.0004 V/mT = 0.4 V/T
SENSITIVITY_V_PER_MILLITESLA = SENSITIVITY_V_PER_TESLA * 1000 # V/mT
# Voltage output when no magnetic field is present (Zero Field Output Voltage)
# This will be recalibrated on startup/request, so 0.0 is a safe default.
IDLE_VOLTAGE = 0.0

# --- Inductive Sensor (LDC1101 Configuration) ---
SPI_BUS = 0                     # SPI bus number (usually 0 or 1 on Raspberry Pi)
SPI_DEVICE = 0                  # SPI device (Chip Select/CE number, usually 0 or 1)
SPI_SPEED = 500000              # SPI clock speed in Hz (e.g., 500 kHz, 1 MHz = 1000000) - Check LDC datasheet limits
SPI_MODE = 0b00                 # SPI mode (CPOL=0, CPHA=0 is common) - Check LDC datasheet
# IMPORTANT: Use BCM pin numbering for GPIO
CS_PIN = 8                      # BCM Pin number for Chip Select (e.g., GPIO 8 corresponds to physical pin 24, CE0) - MUST match the SPI_DEVICE selected if using hardware CS

# LDC1101 Specific Constants
LDC_CHIP_ID = 0xD4              # Expected Chip ID for LDC1101

# LDC Registers (Hex Addresses) - Refer to LDC1101 Datasheet
START_CONFIG_REG = 0x0B
RP_SET_REG = 0x01
TC1_REG = 0x02
TC2_REG = 0x03
DIG_CONFIG_REG = 0x04
ALT_CONFIG_REG = 0x05
D_CONF_REG = 0x0C
INTB_MODE_REG = 0x0A
RP_DATA_MSB_REG = 0x22 # RP+L Data MSB
RP_DATA_LSB_REG = 0x21 # RP+L Data LSB
CHIP_ID_REG = 0x3F

# LDC Modes
ACTIVE_CONVERSION_MODE = 0x00
SLEEP_MODE = 0x01

# --- Calibration Defaults ---
IDLE_RP_VALUE = 0               # Default idle RP value, recalibrated on startup/request

# ============================
# === Global Objects/State ===
# ============================

# --- Hardware Objects (initialized later in initialize_hardware) ---
camera = None         # OpenCV VideoCapture object
i2c = None            # I2C bus object
ads = None            # ADS1115 object
hall_sensor = None    # AnalogIn object for Hall sensor pin
spi = None            # SPI device object
ldc_initialized = False # Flag to track successful LDC setup

# --- AI Objects (initialized later in initialize_ai) ---
interpreter = None      # TFLite Interpreter object
input_details = None    # Details about the model's input tensors
output_details = None   # Details about the model's output tensors
loaded_labels = []      # List to store class labels loaded from file
numerical_scaler = None # Scaler object loaded from file

# --- Global State Variables ---
RP_DISPLAY_BUFFER = deque(maxlen=LDC_DISPLAY_BUFFER_SIZE) # Buffer for smoothing LDC display
previous_filtered_mag_mT = None # Used for exponential smoothing of magnetism display

# --- GUI Globals (Tkinter widgets, initialized in setup_gui) ---
window = None               # Main Tkinter window
main_frame = None           # Top-level frame holding views
live_view_frame = None      # Frame for camera feed and live controls
results_view_frame = None   # Frame for displaying classification results

# Fonts (defined in setup_gui)
label_font = None
readout_font = None
button_font = None
title_font = None
result_title_font = None
result_value_font = None

# Live View Widgets
lv_camera_label = None      # Label to display camera feed
lv_magnetism_label = None   # Label to display live magnetism reading
lv_ldc_label = None         # Label to display live LDC reading
lv_classify_button = None   # Button to trigger classification
lv_calibrate_button = None  # Button to trigger sensor calibration

# Results View Widgets
rv_image_label = None           # Label to display captured image
rv_prediction_label = None      # Label for predicted material
rv_confidence_label = None      # Label for confidence score
rv_magnetism_label = None       # Label for magnetism reading used in classification
rv_ldc_label = None             # Label for LDC reading used in classification
rv_classify_another_button = None # Button to go back to live view

# Placeholder image object (Tkinter PhotoImage)
placeholder_img_tk = None

# =========================
# === Hardware Setup ===
# =========================
def initialize_hardware():
    """Initializes Camera, I2C/ADS1115, and SPI/LDC1101 based on enabled flags."""
    global camera, i2c, ads, hall_sensor, spi, ldc_initialized, CS_PIN

    print("\n--- Initializing Hardware ---")

    # --- Camera Initialization ---
    print(f"Attempting to open camera at index {CAMERA_INDEX}...")
    try:
        camera = cv2.VideoCapture(CAMERA_INDEX)
        # Some cameras need a moment to initialize
        time.sleep(0.5)
        if not camera or not camera.isOpened():
            # Raise an error if camera failed to open, handled by except block
            raise ValueError(f"Could not open camera at index {CAMERA_INDEX}. Check connection and permissions.")
        # Set camera resolution (optional, might improve performance)
        # camera.set(cv2.CAP_PROP_FRAME_WIDTH, DISPLAY_IMG_WIDTH)
        # camera.set(cv2.CAP_PROP_FRAME_HEIGHT, DISPLAY_IMG_HEIGHT)
        print(f"Camera {CAMERA_INDEX} opened successfully.")
    except Exception as e:
        print(f"ERROR: Failed to open camera {CAMERA_INDEX}: {e}")
        camera = None # Ensure camera is None if initialization fails

    # --- I2C/ADS1115 Initialization ---
    if I2C_ENABLED:
        print("Initializing I2C and ADS1115...")
        try:
            # Initialize I2C bus using board pins SCL and SDA
            i2c = busio.I2C(board.SCL, board.SDA)
            # Check if ADS1115 is detected at the default address (0x48)
            # You might need to scan for the address if it's different
            # i2c.scan() -> returns list of addresses
            ads = ADS.ADS1115(i2c) # Default address is 0x48
            # Create an analog input channel for the Hall sensor
            if HALL_ADC_CHANNEL is not None:
                hall_sensor = AnalogIn(ads, HALL_ADC_CHANNEL)
                # Set gain if needed (optional, affects voltage range and resolution)
                # Gain options: 2/3 (±6.144V), 1 (±4.096V), 2 (±2.048V), 4 (±1.024V), 8 (±0.512V), 16 (±0.256V)
                # ads.gain = 1 # Example: Set gain to 1
                print(f"ADS1115 initialized. Hall sensor assigned to channel {HALL_ADC_CHANNEL}.")
            else:
                print("Warning: HALL_ADC_CHANNEL not defined, cannot create Hall sensor input.")
                hall_sensor = None # Ensure it's None if channel wasn't defined
        except ValueError as e:
            # Common error if device not found at the address
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
        print("Initializing SPI, GPIO, and LDC1101...")
        try:
            # Configure GPIO
            GPIO.setwarnings(False) # Disable GPIO warnings (optional)
            GPIO.setmode(GPIO.BCM) # Use BCM pin numbering scheme
            # Setup Chip Select pin as output and set it high (inactive) initially
            GPIO.setup(CS_PIN, GPIO.OUT, initial=GPIO.HIGH)
            print(f"GPIO initialized (BCM mode). CS Pin {CS_PIN} set as OUTPUT HIGH.")

            # Configure SPI
            spi = spidev.SpiDev()
            spi.open(SPI_BUS, SPI_DEVICE) # Open SPI bus and device
            spi.max_speed_hz = SPI_SPEED   # Set clock speed
            spi.mode = SPI_MODE            # Set SPI mode
            print(f"SPI initialized (Bus={SPI_BUS}, Device={SPI_DEVICE}, Speed={SPI_SPEED}Hz, Mode={SPI_MODE}).")

            # Initialize LDC1101 sensor via SPI
            if initialize_ldc1101(): # This function attempts LDC setup
                # If LDC init is successful, enable RP mode for readings
                enable_ldc_rpmode()
                print("LDC1101 initialized and RP+L mode enabled.")
                # ldc_initialized is set True inside initialize_ldc1101 on success
            else:
                # Handle LDC initialization failure (error message printed inside initialize_ldc1101)
                print("ERROR: LDC1101 Initialization Failed. Check connections/wiring/configuration.")
                # SPI might still be open, but LDC isn't usable. Keep SPI open or close it?
                # Depending on if other SPI devices are used, decide whether to close SPI here.
                # spi.close()
                # spi = None
                ldc_initialized = False # Ensure flag is False

        except Exception as e:
            print(f"ERROR: An error occurred during GPIO/SPI/LDC initialization: {e}")
            # Cleanup GPIO/SPI if setup failed partway
            if spi:
                try:
                    spi.close()
                    print("SPI closed due to initialization error.")
                except Exception as spi_e:
                    print(f"Note: Error closing SPI after main init error: {spi_e}")
            # GPIO cleanup might be needed here if setup failed after setmode
            try:
                if 'GPIO' in globals() and GPIO.getmode() is not None:
                     GPIO.cleanup()
                     print("GPIO cleaned up due to initialization error.")
            except Exception as gpio_e:
                print(f"Note: Error during GPIO cleanup after SPI/LDC failure: {gpio_e}")
            spi = None
            ldc_initialized = False # Ensure LDC is marked as not initialized
    else:
        print("Skipping SPI/GPIO/LDC1101 setup (libraries not found or disabled).")

    print("--- Hardware Initialization Complete ---")

# =========================
# === AI Model Setup ======
# =========================
def initialize_ai():
    """Loads labels, scaler, TFLite model, and allocates tensors. Returns True if successful."""
    global interpreter, input_details, output_details, loaded_labels, numerical_scaler
    print("\n--- Initializing AI Components ---")
    ai_ready = True # Flag to track overall success

    # --- Load Labels ---
    print(f"Loading labels from: {LABELS_PATH}")
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
        print(f"ERROR: Failed to read labels file '{LABELS_FILENAME}': {e}")
        ai_ready = False

    # --- Load Scaler ---
    if ai_ready: # Only proceed if previous steps were successful
        print(f"Loading numerical scaler from: {SCALER_PATH}")
        try:
            numerical_scaler = joblib.load(SCALER_PATH)
            # Basic check if the loaded object looks like a scaler
            if not hasattr(numerical_scaler, 'transform') or not callable(numerical_scaler.transform):
                raise TypeError("Loaded scaler object does not have a callable 'transform' method.")

            # Check scaler's expected number of features (if available)
            expected_features = 2 # We expect Magnetism and LDC Delta
            if hasattr(numerical_scaler, 'n_features_in_'):
                print(f"Scaler expects {numerical_scaler.n_features_in_} features.")
                if numerical_scaler.n_features_in_ != expected_features:
                     print(f"WARNING: Scaler was trained on {numerical_scaler.n_features_in_} features, but script expects {expected_features} (Magnetism, LDC Delta). Ensure this is correct!")
                     # Decide if this is a fatal error:
                     # ai_ready = False
            else:
                print(f"Warning: Cannot verify number of features expected by scaler. Assuming {expected_features}.")

        except FileNotFoundError:
            print(f"ERROR: Scaler file not found: {SCALER_PATH}")
            ai_ready = False
        except TypeError as e:
             print(f"ERROR: Scaler file '{SCALER_FILENAME}' seems corrupted or invalid: {e}")
             ai_ready = False
        except Exception as e:
            print(f"ERROR: Failed to load numerical scaler: {e}")
            ai_ready = False

    # --- Load TFLite Model ---
    if ai_ready:
        print(f"Loading TFLite model from: {MODEL_PATH}")
        try:
            # Load the TFLite model using the interpreter
            interpreter = Interpreter(model_path=MODEL_PATH)
            # Allocate memory for the model's tensors
            interpreter.allocate_tensors()

            # Get input and output tensor details (important for feeding data correctly)
            input_details = interpreter.get_input_details()
            output_details = interpreter.get_output_details()
            print("TFLite model loaded and tensors allocated.")
            print(f"Input Details:")
            for detail in input_details: print(f"  - Index: {detail['index']}, Shape: {detail['shape']}, Dtype: {detail['dtype']}")
            print(f"Output Details:")
            for detail in output_details: print(f"  - Index: {detail['index']}, Shape: {detail['shape']}, Dtype: {detail['dtype']}")

            # --- Sanity Checks ---
            # 1. Check number of inputs (expecting 2: image and numerical)
            if len(input_details) != 2:
                print(f"WARNING: Model expected 2 inputs, but found {len(input_details)}. Preprocessing might fail.")
                # Decide if this is fatal: ai_ready = False

            # 2. Check output shape vs number of labels
            if output_details:
                output_shape = output_details[0]['shape']
                # Output shape is often (1, num_classes)
                num_classes_in_model = output_shape[-1] # Get the last dimension size
                if num_classes_in_model != len(loaded_labels):
                    print(f"WARNING: Model output size ({num_classes_in_model}) does not match the number of loaded labels ({len(loaded_labels)}). Predictions might be assigned to wrong labels.")
                    # Decide if this is fatal: ai_ready = False
            else:
                print("WARNING: Could not get model output details for sanity check.")

        except FileNotFoundError:
            print(f"ERROR: Model file not found: {MODEL_PATH}")
            ai_ready = False
        except ValueError as e:
             print(f"ERROR: Error initializing TFLite interpreter. Model file '{MODEL_FILENAME}' might be corrupted or invalid: {e}")
             ai_ready = False
        except Exception as e:
            print(f"ERROR: Failed to load TFLite model: {e}")
            import traceback
            traceback.print_exc() # Print detailed traceback for debugging
            ai_ready = False

    # --- Final Status ---
    if ai_ready:
        print("--- AI Initialization Complete ---")
    else:
        print("--- AI Initialization Failed ---")
        # Ensure AI components are None if initialization failed at any point
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
        return False # Indicate failure

    success = False
    try:
        GPIO.output(CS_PIN, GPIO.LOW) # Activate chip select (CS low)
        # Send command byte: Register address (7 bits) with MSB=0 for write
        # Send data byte: The value to write
        spi.xfer2([reg_addr & 0x7F, value])
        GPIO.output(CS_PIN, GPIO.HIGH) # Deactivate chip select (CS high)
        success = True
    except Exception as e:
        print(f"Warning: Error during LDC write (Register 0x{reg_addr:02X}). Error: {e}")
        # Attempt to ensure CS is high even if an error occurred
        try:
            GPIO.output(CS_PIN, GPIO.HIGH)
        except Exception as inner_e:
            print(f"Warning: Failed to force CS HIGH after LDC write error. Inner error: {inner_e}")
            # Continue anyway, but this might indicate a deeper issue
    return success

def ldc_read_register(reg_addr):
    """Reads a single byte value from an LDC1101 register via SPI. Returns integer or None on error."""
    if not spi:
        # print("Warning: SPI not available for LDC read.") # Optional debug
        return None # Return None if SPI isn't working

    read_value = None # Default value in case of error
    try:
        GPIO.output(CS_PIN, GPIO.LOW) # Activate chip select (CS low)
        # Send command byte: Register address (7 bits) with MSB=1 for read
        # Send dummy byte: 0x00 (value doesn't matter for read)
        result = spi.xfer2([reg_addr | 0x80, 0x00])
        GPIO.output(CS_PIN, GPIO.HIGH) # Deactivate chip select (CS high)
        # The second byte received in the result list is the register value
        read_value = result[1]
    except Exception as e:
        print(f"Warning: Error during LDC read (Register 0x{reg_addr:02X}). Error: {e}")
        # Attempt to ensure CS is high even if an error occurred
        try:
            GPIO.output(CS_PIN, GPIO.HIGH)
        except Exception as inner_e:
            print(f"Warning: Failed to force CS HIGH after LDC read error. Inner error: {inner_e}")
            # Continue, returning the default value (None)
    # No finally block needed here as try/except covers CS high setting on error

    return read_value

def initialize_ldc1101():
    """Initializes and configures the LDC1101 sensor. Returns True on success, False on failure."""
    global ldc_initialized
    ldc_initialized = False # Assume failure until proven otherwise

    if not spi:
        print("Cannot initialize LDC1101: SPI is not available.")
        return False

    print("Initializing LDC1101...")
    try:
        # 1. Verify Chip ID
        chip_id = ldc_read_register(CHIP_ID_REG)
        if chip_id is None:
             print("ERROR: Failed to read LDC Chip ID (SPI communication error?).")
             return False
        if chip_id != LDC_CHIP_ID:
            print(f"ERROR: LDC Chip ID mismatch! Read 0x{chip_id:02X}, Expected 0x{LDC_CHIP_ID:02X}")
            return False # Stop initialization if chip ID is wrong
        print(f"LDC1101 Chip ID verified (0x{chip_id:02X}).")

        # 2. Configure LDC Registers (adjust values based on datasheet and application)
        # These are EXAMPLE values, you MUST configure them based on your coil, target, and desired performance.
        print("Configuring LDC1101 registers (Using EXAMPLE values - Adjust as needed!)...")
        # RP_MAX=16kOhm, RP_MIN=2kOhm (Example - affects RP measurement range)
        if not ldc_write_register(RP_SET_REG, 0x07): return False
        # Sensor Frequency Setting 1 (Example - affects operating frequency)
        if not ldc_write_register(TC1_REG, 0x90): return False
        # Sensor Frequency Setting 2 (Example - affects operating frequency)
        if not ldc_write_register(TC2_REG, 0xA0): return False
        # Min/Max Frequency Setting (Example - related to frequency limits)
        if not ldc_write_register(DIG_CONFIG_REG, 0x03): return False
        # High Current Sensor Drive Disabled (Example - use if needed for low impedance coils)
        if not ldc_write_register(ALT_CONFIG_REG, 0x00): return False
        # Configure L measurement (Example - affects L measurement settings)
        if not ldc_write_register(D_CONF_REG, 0x00): return False
        # Interrupt Pin Disabled (Example - enable if using INTB pin)
        if not ldc_write_register(INTB_MODE_REG, 0x00): return False

        # 3. Put LDC into Sleep Mode initially after configuration
        if not ldc_write_register(START_CONFIG_REG, SLEEP_MODE): return False
        time.sleep(0.02) # Short delay after configuration

        print("LDC1101 Configuration successful.")
        ldc_initialized = True # Mark as initialized ONLY on success
        return True

    except Exception as e:
        print(f"ERROR: Exception during LDC1101 Initialization: {e}")
        ldc_initialized = False # Ensure flag is false on any exception
        return False

def enable_ldc_powermode(mode):
    """Sets the power mode (Active or Sleep) of the LDC1101."""
    if not spi or not ldc_initialized:
        # print("Warning: Cannot set LDC power mode (SPI not ready or LDC not initialized).") # Optional
        return False # Indicate failure
    # print(f"Setting LDC Power Mode to {'Active' if mode == ACTIVE_CONVERSION_MODE else 'Sleep'}...") # Debug
    if ldc_write_register(START_CONFIG_REG, mode):
        time.sleep(0.01) # Allow time for mode change
        return True
    else:
        print(f"Warning: Failed to set LDC power mode register.")
        return False


def enable_ldc_rpmode():
    """Enables the LDC1101 RP+L measurement mode (Resonant Impedance)."""
    if not spi or not ldc_initialized:
        print("Warning: Cannot enable LDC RP mode (SPI not ready or LDC not initialized).")
        return False

    print("Enabling LDC RP+L Mode...")
    try:
        # Configure for RP+L measurement (These might need adjustment based on your specific needs)
        # Refer to LDC1101 datasheet for ALT_CONFIG and D_CONF register settings for RP+L mode.
        # The example values 0x00 might be correct, but verify.
        if not ldc_write_register(ALT_CONFIG_REG, 0x00): return False # Example setting for RP+L
        if not ldc_write_register(D_CONF_REG, 0x00): return False   # Example setting for RP+L

        # Set to active conversion mode to start measurements
        if enable_ldc_powermode(ACTIVE_CONVERSION_MODE):
            print("LDC RP+L Mode Enabled and Active.")
            return True
        else:
            print("Failed to set LDC to Active mode after configuring RP+L.")
            return False
    except Exception as e:
        print(f"Warning: Failed to enable LDC RP mode: {e}")
        return False


def get_ldc_rpdata():
    """Reads the 16-bit raw RP+L data from the LDC1101. Returns integer or None on error."""
    if not spi or not ldc_initialized:
        # print("Warning: Cannot read LDC RP data (SPI not ready or LDC not initialized).") # Optional
        return None

    try:
        # Read the Most Significant Byte (MSB) and Least Significant Byte (LSB)
        msb = ldc_read_register(RP_DATA_MSB_REG)
        lsb = ldc_read_register(RP_DATA_LSB_REG)

        # Check if reads were successful
        if msb is None or lsb is None:
            print("Warning: Failed to read MSB or LSB for LDC RP data.")
            return None

        # Combine MSB and LSB to form the 16-bit value
        rp_data = (msb << 8) | lsb
        return rp_data
    except Exception as e:
        # Catch any unexpected errors during the read process
        print(f"Warning: Exception while reading LDC RP data: {e}")
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
            # time.sleep(0.001) # Optional small delay between reads if needed for stability
        except OSError as e:
            # Handle specific I/O errors (e.g., I2C communication issues)
            print(f"Warning: OS Error reading Hall sensor (sample {i+1}/{num_samples}): {e}. Aborting average.")
            return None # Abort averaging if a read fails critically
        except Exception as e:
            # Handle other potential errors during a single read
            print(f"Warning: Error reading Hall sensor voltage (sample {i+1}/{num_samples}): {e}. Aborting average.")
            return None # Abort averaging

    # If the loop completes without errors and readings were collected
    if readings:
        try:
            average_voltage = statistics.mean(readings) # Use statistics.mean for robustness
            # average_voltage = sum(readings) / len(readings) # Alternative basic average
            return average_voltage
        except statistics.StatisticsError:
             print("Warning: Statistics error calculating average Hall voltage (should not happen with valid readings).")
             return None
        except ZeroDivisionError:
             print("Warning: ZeroDivisionError calculating average Hall voltage (readings list was empty?).")
             return None
    else:
        # Should not happen if num_samples > 0 and no exceptions occurred, but handle defensively
        print("Warning: No valid Hall sensor readings collected for averaging.")
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
        rp_value = get_ldc_rpdata() # This function handles individual read errors and returns None
        if rp_value is not None:
            readings.append(rp_value)
        else:
            # Handle case where a single read failed within get_ldc_rpdata()
            print(f"Warning: Failed to get LDC RP data for sample {i+1}/{num_samples}. Skipping sample.")
            # Optionally decide whether to abort entirely or just skip the sample
            # return None # Uncomment to abort averaging if any read fails

    # Calculate average only from valid readings collected
    if readings:
        try:
            average_rp = statistics.mean(readings) # Use statistics.mean
            # average_rp = sum(readings) / len(readings) # Alternative basic average
            return average_rp
        except statistics.StatisticsError:
             print("Warning: Statistics error calculating average LDC RP data.")
             return None
        except ZeroDivisionError:
             print("Warning: ZeroDivisionError calculating average LDC RP data (readings list was empty?).")
             return None
    else:
        print("Warning: No valid LDC RP readings collected for averaging.")
        return None

# ==========================
# === AI Processing ========
# ==========================

def preprocess_input(image_pil, mag_mT, ldc_rp_delta):
    """
    Prepares the captured image and sensor data for the TFLite model.
    Args:
        image_pil (PIL.Image): The captured image (RGB).
        mag_mT (float or None): Magnetism reading in milliTesla.
        ldc_rp_delta (int or None): Change in LDC RP value from idle (raw integer delta).
    Returns:
        dict: A dictionary mapping input tensor indices to processed numpy arrays,
              or None if preprocessing fails or AI components are not ready.
    """
    global numerical_scaler, input_details, interpreter, loaded_labels # Need details for validation

    print("\n--- Preprocessing Input for AI ---") # Marker

    # --- Pre-checks ---
    if interpreter is None or input_details is None or output_details is None:
        print("ERROR: AI Model (interpreter/details) not initialized. Cannot preprocess.")
        return None
    if numerical_scaler is None:
         print("ERROR: Numerical scaler not loaded. Cannot preprocess numerical features.")
         return None
    if not loaded_labels:
        print("Warning: Labels not loaded, proceeding but postprocessing might fail.")
        # Decide if this should be fatal: return None

    # --- 1. Preprocess Image ---
    print(f"Preprocessing image (resizing to {AI_IMG_WIDTH}x{AI_IMG_HEIGHT})...")
    try:
        # Resize image to the dimensions expected by the AI model
        # Use LANCZOS resampling for potentially better quality than default/NEAREST
        img_resized = image_pil.resize((AI_IMG_WIDTH, AI_IMG_HEIGHT), Image.Resampling.LANCZOS)
        # Ensure image is RGB (should be already if captured correctly)
        img_rgb = img_resized.convert('RGB')
        # Convert PIL image to NumPy array
        image_np = np.array(img_rgb, dtype=np.float32)

        # Normalize pixel values (IMPORTANT: Match normalization used during training!)
        # Common methods:
        # a) Scale to [0, 1]
        image_np /= 255.0
        # b) Scale to [-1, 1]
        # image_np = (image_np / 127.5) - 1.0
        print(f"Image normalized (assuming [0, 1] range). Min: {image_np.min():.2f}, Max: {image_np.max():.2f}")

        # Add batch dimension (model expects batch_size, height, width, channels)
        image_input = np.expand_dims(image_np, axis=0)
        print(f"Image preprocessed. Shape: {image_input.shape}, Dtype: {image_input.dtype}")

    except Exception as e:
        print(f"ERROR: Image preprocessing failed: {e}")
        return None

    # --- 2. Preprocess Numerical Features ---
    print("Preprocessing numerical features...")
    # Handle potential None values from sensor readings by defaulting to 0.0
    # This assumes the scaler was trained with 0 representing the 'neutral' or 'missing' state,
    # or that the model can handle scaled zeros appropriately.
    used_default_mag = False
    used_default_ldc = False

    if mag_mT is None:
        print("DEBUG Preprocess: Magnetism reading was None, defaulting to 0.0 for scaling.")
        mag_mT_val = 0.0
        used_default_mag = True
    else:
        mag_mT_val = float(mag_mT) # Ensure float

    if ldc_rp_delta is None:
        print("DEBUG Preprocess: LDC RP delta was None, defaulting to 0.0 for scaling.")
        ldc_rp_delta_val = 0.0
        used_default_ldc = True
    else:
        ldc_rp_delta_val = float(ldc_rp_delta) # Ensure float

    # Create NumPy array for numerical features (shape: [1, num_features])
    # IMPORTANT: The order MUST match the order the scaler was trained on!
    # Assuming order: [Magnetism, LDC_Delta]
    numerical_features = np.array([[mag_mT_val, ldc_rp_delta_val]], dtype=np.float32)
    print(f"DEBUG Preprocess: Raw numerical features BEFORE scaling: {numerical_features}")

    # Scale numerical features using the loaded scaler
    try:
        print("Applying numerical scaler...")
        # Suppress potential UserWarning about feature names if scaler was trained with pandas
        with warnings.catch_warnings():
            warnings.filterwarnings("ignore", message="X does not have valid feature names.*", category=UserWarning)
            scaled_numerical_features = numerical_scaler.transform(numerical_features)
        print(f"DEBUG Preprocess: Scaled numerical features AFTER scaling: {scaled_numerical_features}")

        # Check if scaled values became zero/problematic
        if np.allclose(scaled_numerical_features, 0.0) and (used_default_mag or used_default_ldc):
            print("WARNING Preprocess: Scaled numerical features are all zero, likely due to missing sensor input being defaulted.")
        elif np.allclose(scaled_numerical_features, 0.0):
            print("WARNING Preprocess: Scaled numerical features are all zero. Check scaler, sensor calibration, or if input values were actually zero.")

    except Exception as e:
        print(f"ERROR: Scaling numerical features failed: {e}")
        # Fallback: Use zeros if scaling fails? Or return None? Returning None is safer.
        return None

    # --- 3. Identify Input Tensor Indices and Validate ---
    print("Identifying model input tensor indices...")
    image_input_index = -1
    image_input_dtype = None
    numerical_input_index = -1
    numerical_input_dtype = None
    num_features_expected_by_model = -1 # Will try to find this from model details

    # Iterate through model's input details to find the image and numerical tensors
    for detail in input_details:
        shape = detail['shape']
        index = detail['index']
        dtype = detail['dtype']

        # Image tensor check: 4 dimensions, matches AI H/W, 3 channels (RGB)
        # Note: shape[0] is batch size (often 1 or dynamic)
        if len(shape) == 4 and shape[1] == AI_IMG_HEIGHT and shape[2] == AI_IMG_WIDTH and shape[3] == 3:
            if image_input_index == -1: # Assign only if not already found
                image_input_index = index
                image_input_dtype = dtype
                print(f"Found potential Image Input: Index={index}, Shape={shape}, Dtype={dtype}")
            else:
                print(f"Warning: Found multiple potential image input tensors (Indices {image_input_index} and {index}). Using first one found.")

        # Numerical tensor check: 2 dimensions (batch, num_features)
        elif len(shape) == 2:
            if numerical_input_index == -1: # Assign only if not already found
                numerical_input_index = index
                numerical_input_dtype = dtype
                num_features_expected_by_model = shape[1] # Get expected feature count from model
                print(f"Found potential Numerical Input: Index={index}, Shape={shape}, Dtype={dtype}")
                # Validate against scaler's expected features
                if hasattr(numerical_scaler, 'n_features_in_') and num_features_expected_by_model != numerical_scaler.n_features_in_:
                     print(f"ERROR: Model expects {num_features_expected_by_model} numerical features, but scaler was trained on {numerical_scaler.n_features_in_}. Mismatch!")
                     return None # Fatal error if counts don't match
                elif scaled_numerical_features.shape[1] != num_features_expected_by_model:
                     print(f"ERROR: Model expects {num_features_expected_by_model} numerical features, but preprocessed data has {scaled_numerical_features.shape[1]}. Mismatch!")
                     return None # Fatal error
            else:
                print(f"Warning: Found multiple potential numerical input tensors (Indices {numerical_input_index} and {index}). Using first one found.")

    # --- Check if both inputs were found ---
    if image_input_index == -1 or numerical_input_index == -1:
        print("ERROR: Failed to identify distinct image and numerical input tensors based on shape and dimension checks.")
        print("Input Details were:")
        for detail in input_details: print(f"  - Index: {detail['index']}, Shape: {detail['shape']}, Dtype: {detail['dtype']}")
        return None # Cannot proceed if inputs aren't identified

    print(f"Identified Image Input Index: {image_input_index}, Numerical Input Index: {numerical_input_index}")

    # --- 4. Prepare Model Input Dictionary with Correct Dtypes ---
    print("Preparing final model input dictionary...")
    try:
        # Ensure data types match what the model expects
        final_image_input = image_input.astype(image_input_dtype)
        final_numerical_input = scaled_numerical_features.astype(numerical_input_dtype)

        # Check if image normalization needs adjustment for UINT8 models
        if image_input_dtype == np.uint8:
            # If model expects UINT8, input data should typically be in [0, 255] range
            # Our current normalization is [0, 1] float32. We need to rescale and cast.
            print("Model expects UINT8 image input. Rescaling normalized image from [0,1] to [0,255] and casting.")
            final_image_input = (image_input * 255.0).astype(np.uint8)
            # Verify range after conversion (optional debug)
            # print(f"UINT8 Image Range: Min={final_image_input.min()}, Max={final_image_input.max()}")

        # Create the dictionary mapping index to the correctly typed numpy array
        model_inputs = {
            image_input_index: final_image_input,
            numerical_input_index: final_numerical_input
        }
        print("Model inputs prepared successfully.")
        print(f"  Image Input final shape: {model_inputs[image_input_index].shape}, dtype: {model_inputs[image_input_index].dtype}")
        print(f"  Numerical Input final shape: {model_inputs[numerical_input_index].shape}, dtype: {model_inputs[numerical_input_index].dtype}")
        print("--- Preprocessing Complete ---") # Marker
        return model_inputs

    except Exception as e:
        print(f"ERROR: Failed to prepare final model input dictionary (dtype conversion?): {e}")
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

    print("\n--- Running AI Inference ---") # Marker

    # --- Pre-checks ---
    if interpreter is None:
        print("ERROR: TFLite interpreter not initialized. Cannot run inference.")
        return None
    if model_inputs is None or not isinstance(model_inputs, dict):
        print("ERROR: Invalid or missing model inputs provided for inference.")
        return None
    if not output_details:
         print("ERROR: Model output details not available. Cannot get output.")
         return None

    try:
        # --- Set Input Tensors ---
        print("Setting input tensors...")
        for index, data in model_inputs.items():
            # Optional: Verify shape one last time before setting
            # expected_shape = next(d['shape'] for d in input_details if d['index'] == index)
            # if list(data.shape) != expected_shape.tolist(): # Compare shapes carefully
            #     print(f"ERROR: Shape mismatch for input tensor {index}. Expected {expected_shape}, got {data.shape}")
            #     return None

            # Set the tensor data in the interpreter
            interpreter.set_tensor(index, data)
            # print(f"  Set tensor index {index} with shape {data.shape} and dtype {data.dtype}") # Verbose Debug
        print("Input tensors set.")

        # --- Run Inference ---
        print("Invoking interpreter...")
        start_time = time.monotonic() # Use monotonic clock for timing intervals
        interpreter.invoke()
        end_time = time.monotonic()
        inference_time_ms = (end_time - start_time) * 1000
        print(f"Interpreter invoked successfully (took {inference_time_ms:.2f} ms).")

        # --- Get Output Tensor ---
        # Assuming the model has a single output tensor (common case)
        output_tensor_index = output_details[0]['index']
        print(f"Getting output tensor at index {output_tensor_index}...")
        output_data = interpreter.get_tensor(output_tensor_index)
        print(f"Raw output data retrieved. Shape: {output_data.shape}, Dtype: {output_data.dtype}")
        print("--- Inference Complete ---") # Marker

        return output_data

    except Exception as e:
        print(f"ERROR: An unexpected error occurred during model inference: {e}")
        import traceback
        traceback.print_exc() # Print detailed traceback for debugging
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

    print("\n--- Postprocessing AI Output ---") # Marker

    # --- Pre-checks ---
    if output_data is None:
        print("ERROR: No output data received for postprocessing.")
        return "Error", 0.0
    if not loaded_labels:
        print("ERROR: Labels not loaded. Cannot interpret output.")
        return "No Labels", 0.0

    try:
        # --- Extract Probabilities ---
        # Common output shape is (1, num_classes). Extract the inner array.
        if len(output_data.shape) == 2 and output_data.shape[0] == 1:
            probabilities = output_data[0]
        # Handle potential flat output (num_classes,)
        elif len(output_data.shape) == 1:
             probabilities = output_data
        else:
            print(f"ERROR: Unexpected output data shape {output_data.shape}. Expected (1, num_classes) or (num_classes,).")
            return "Shape Err", 0.0

        # Check if number of probabilities matches number of labels
        if len(probabilities) != len(loaded_labels):
             print(f"ERROR: Number of probabilities ({len(probabilities)}) does not match number of labels ({len(loaded_labels)}).")
             return "Label Mismatch", 0.0

        # --- Print Probabilities (Debug) ---
        print("DEBUG Postprocess: Probabilities per Class:")
        for i, prob in enumerate(probabilities):
            label = loaded_labels[i]
            print(f"  - {label}: {prob:.4f}")

        # --- Find Best Prediction ---
        predicted_index = np.argmax(probabilities)
        confidence = float(probabilities[predicted_index]) # Get the highest probability

        # Get the corresponding label
        predicted_label = loaded_labels[predicted_index]

        print(f"Final Prediction: '{predicted_label}', Confidence: {confidence:.4f}")
        print("--- Postprocessing Complete ---") # Marker
        return predicted_label, confidence

    except IndexError as e:
        # Handle cases where indexing fails (e.g., output_data[0] if shape was wrong)
        print(f"ERROR: Index error during postprocessing. Output shape might be unexpected: {output_data.shape}. Error: {e}")
        return "Index Err", 0.0
    except Exception as e:
        print(f"ERROR: An unexpected error occurred during postprocessing: {e}")
        return "Post Err", 0.0

# ==============================
# === View Switching Logic ===
# ==============================
def show_live_view():
    """Hides the results view and shows the live camera/sensor view."""
    global live_view_frame, results_view_frame, lv_classify_button, interpreter

    # Hide the results frame if it exists and is currently packed
    if results_view_frame and results_view_frame.winfo_ismapped():
        results_view_frame.pack_forget()

    # Show the live view frame if it exists and is not currently packed
    if live_view_frame and not live_view_frame.winfo_ismapped():
        live_view_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

    # Re-enable the classify button when switching back to live view, ONLY if AI is ready
    if lv_classify_button:
        if interpreter: # Check if AI model loaded successfully
            lv_classify_button.config(state=tk.NORMAL)
        else:
            lv_classify_button.config(state=tk.DISABLED) # Keep disabled if AI failed

    # print("Switched to Live View.") # Less verbose debug

def show_results_view():
    """Hides the live view and shows the classification results view."""
    global live_view_frame, results_view_frame

    # Hide the live view frame if it exists and is currently packed
    if live_view_frame and live_view_frame.winfo_ismapped():
        live_view_frame.pack_forget()

    # Show the results frame if it exists and is not currently packed
    if results_view_frame and not results_view_frame.winfo_ismapped():
        results_view_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

    # print("Switched to Results View.") # Less verbose debug

# ======================
# === GUI Functions ===
# ======================
def create_placeholder_image(width, height, color='#E0E0E0', text="No Image"):
    """Creates a simple PIL placeholder image with optional text and converts it to Tkinter format."""
    try:
        # Create a base image with the specified background color
        pil_img = Image.new('RGB', (width, height), color)

        # Optionally add text to the placeholder (basic centering)
        # This requires Pillow to be built with freetype support potentially
        # try:
        #     from PIL import ImageDraw, ImageFont
        #     draw = ImageDraw.Draw(pil_img)
        #     # Use a default font if possible
        #     # font = ImageFont.truetype("arial.ttf", 15) # Example, requires font file
        #     font = ImageFont.load_default()
        #     text_width, text_height = draw.textsize(text, font=font) # Use textbbox in newer Pillow
        #     text_x = (width - text_width) // 2
        #     text_y = (height - text_height) // 2
        #     draw.text((text_x, text_y), text, fill="#666666", font=font) # Dark gray text
        # except ImportError:
        #     print("Note: PIL ImageDraw/ImageFont not available for placeholder text.")
        # except OSError:
        #      print("Note: Default font not found for placeholder text.")

        # Convert the final PIL image to Tkinter format
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
            rv_image_label.config(image=placeholder_img_tk, text="") # Clear any text
        else:
            # Fallback text if placeholder creation failed
            rv_image_label.config(image='', text="No Image") # Clear image, show text
            rv_image_label.img_tk = None # Clear reference

    # --- Clear Text Labels ---
    default_text = "---" # Use a clearer default than "..."
    if rv_prediction_label:
        rv_prediction_label.config(text=default_text)
    if rv_confidence_label:
        rv_confidence_label.config(text=default_text)
    if rv_magnetism_label:
        rv_magnetism_label.config(text=default_text)
    if rv_ldc_label:
        rv_ldc_label.config(text=default_text)

    # print("Results display cleared.") # Less verbose debug


def capture_and_classify():
    """
    Captures an image and sensor data, runs AI classification,
    updates the results page, and switches to the results view.
    """
    global lv_classify_button, window, camera, IDLE_VOLTAGE, IDLE_RP_VALUE
    global rv_image_label, rv_prediction_label, rv_confidence_label
    global rv_magnetism_label, rv_ldc_label
    global interpreter # Check if AI is ready

    print("\n" + "="*10 + " Capture & Classify Triggered " + "="*10) # Marker

    # --- Pre-checks ---
    if not interpreter:
        messagebox.showerror("Error", "AI Model is not initialized. Cannot classify.")
        print("Classification aborted: AI not ready.")
        return
    if not camera or not camera.isOpened():
        messagebox.showerror("Error", "Camera is not available. Cannot capture image.")
        print("Classification aborted: Camera not ready.")
        return

    # --- Disable Button & Update GUI ---
    if lv_classify_button:
        lv_classify_button.config(state=tk.DISABLED)
    window.update_idletasks() # Ensure GUI updates before potentially blocking operations

    # --- 1. Capture Image ---
    print("Capturing image...")
    ret, frame = camera.read()
    if not ret or frame is None:
        messagebox.showerror("Capture Error", "Failed to capture image from camera. Check camera connection.")
        print("ERROR: Failed to read frame from camera.")
        if lv_classify_button: show_live_view() # Re-enable button by switching view
        return
    # Convert captured frame (OpenCV uses BGR by default) to RGB PIL Image
    try:
        img_captured_pil = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        print(f"Image captured successfully. Size: {img_captured_pil.size}")
    except Exception as e:
        messagebox.showerror("Image Error", f"Failed to process captured image: {e}")
        print(f"ERROR: Failed converting captured frame to PIL Image: {e}")
        if lv_classify_button: show_live_view() # Re-enable button by switching view
        return

    # --- 2. Read Sensors (using more samples for better accuracy during capture) ---
    print(f"Reading sensors for classification (using {NUM_SAMPLES_CALIBRATION} samples)...")
    # --- Read Hall Sensor ---
    avg_voltage = get_averaged_hall_voltage(num_samples=NUM_SAMPLES_CALIBRATION)
    current_mag_mT = None # Initialize default for AI input
    mag_display_text = "N/A" # Default display text for results page
    sensor_warning = False # Flag if sensor data is missing/problematic

    if avg_voltage is not None:
        print(f"  Avg Hall Voltage: {avg_voltage:.4f} V")
        # Voltage reading was successful, proceed with calculation attempt
        try:
            # Use calibrated idle voltage if available, otherwise warn
            idle_v = IDLE_VOLTAGE
            if idle_v == 0.0:
                 print("  Warning: Hall sensor idle voltage not calibrated. Using 0.0V as baseline.")
                 # Optionally use current reading as baseline, but less accurate: idle_v = avg_voltage

            # Ensure sensitivity is not zero to avoid division error
            if abs(SENSITIVITY_V_PER_MILLITESLA) < 1e-9: # Check against a small epsilon
                raise ZeroDivisionError("Hall sensor sensitivity (V/mT) is zero or too close to zero.")

            # Calculate magnetism relative to idle voltage
            current_mag_mT = (avg_voltage - idle_v) / SENSITIVITY_V_PER_MILLITESLA
            print(f"  Calculated Magnetism: {current_mag_mT:+.4f} mT")
            # Format text for display on results page
            mag_display_text = f"{current_mag_mT:+.3f} mT"
            if idle_v == 0.0: mag_display_text += " (No Cal)" # Append warning if not calibrated

        except ZeroDivisionError as e:
            mag_display_text = "Div Zero Err" # Handle division by zero
            print(f"  Warning: Magnetism calculation failed - {e}")
            current_mag_mT = None # Ensure it's None if calculation failed
            sensor_warning = True
        except Exception as e:
            mag_display_text = "Calc Error"
            print(f"  Warning: Magnetism calculation failed - {e}")
            current_mag_mT = None # Ensure it's None if calculation failed
            sensor_warning = True
    else:
        # avg_voltage was None (sensor read failed)
        mag_display_text = "Read Error"
        print("  ERROR: Failed to read Hall sensor voltage.")
        current_mag_mT = None
        sensor_warning = True

    # --- Read LDC Sensor ---
    avg_rp_val = get_averaged_rp_data(num_samples=NUM_SAMPLES_CALIBRATION)
    current_rp_int = None # Integer value of the current reading
    delta_rp = None       # Change from idle value (for AI input)
    ldc_display_text = "N/A" # Default display text for results page

    if avg_rp_val is not None:
        current_rp_int = int(round(avg_rp_val)) # Round avg float to nearest int
        print(f"  Avg LDC RP Value: {avg_rp_val:.2f} (Rounded: {current_rp_int})")
        if IDLE_RP_VALUE != 0:
            # Calculate delta if calibration value exists
            delta_rp = current_rp_int - IDLE_RP_VALUE
            print(f"  LDC Idle RP: {IDLE_RP_VALUE}, Delta RP: {delta_rp:+,}")
            ldc_display_text = f"{current_rp_int} (Delta {delta_rp:+,})" # Show current and delta
        else:
            # No calibration value, cannot calculate meaningful delta for AI
            print("  Warning: LDC sensor idle RP not calibrated. Cannot calculate delta.")
            ldc_display_text = f"{current_rp_int} (No Cal)"
            delta_rp = None # Delta is meaningless without calibration, pass None to AI
            # sensor_warning = True # Optionally warn if not calibrated affects AI
    else:
        # avg_rp_val was None (sensor read failed)
        ldc_display_text = "Read Error"
        print("  ERROR: Failed to read LDC sensor RP value.")
        delta_rp = None
        sensor_warning = True

    # --- Log values being passed to AI ---
    print(f"DEBUG Data for Preprocessing: Magnetism (mT): {current_mag_mT}, LDC Delta RP: {delta_rp}")

    # Optionally show a warning if sensor data was problematic before proceeding
    if sensor_warning:
         print("WARNING: One or more sensor readings failed or were uncalibrated. Classification results may be inaccurate.")
         # messagebox.showwarning("Sensor Warning", "Sensor readings missing/uncalibrated. Results may be inaccurate.") # Can be annoying

    # --- 3. Preprocess Data for AI ---
    # Pass the captured PIL image and the calculated mag_mT and delta_rp
    model_inputs = preprocess_input(img_captured_pil, current_mag_mT, delta_rp)

    if model_inputs is None:
        messagebox.showerror("AI Error", "Data preprocessing failed. Check console logs for details.")
        print("ERROR: Preprocessing failed. Aborting classification.")
        if lv_classify_button: show_live_view() # Re-enable button by switching view
        return

    # --- 4. Run AI Inference ---
    output_data = run_inference(model_inputs)

    if output_data is None:
        messagebox.showerror("AI Error", "AI model inference failed. Check console logs for details.")
        print("ERROR: Inference failed. Aborting classification.")
        if lv_classify_button: show_live_view() # Re-enable button by switching view
        return

    # --- 5. Postprocess AI Output ---
    predicted_label, confidence = postprocess_output(output_data)
    # Postprocessing handles its own errors and returns default values

    print(f"--- Classification Result: Prediction='{predicted_label}', Confidence={confidence:.1%} ---") # Marker

    # --- 6. Update Results Display Widgets ---
    print("Updating results display widgets...")

    # Update Image Label on Results Page
    if rv_image_label:
        try:
            # Calculate display height maintaining aspect ratio
            w, h = img_captured_pil.size
            aspect = h / w if w > 0 else 1 # Avoid division by zero
            display_h = int(RESULT_IMG_DISPLAY_WIDTH * aspect)
            if display_h <= 0: display_h = int(RESULT_IMG_DISPLAY_WIDTH * 0.75) # Fallback height

            # Resize image for display using LANCZOS for better quality
            img_disp = img_captured_pil.resize((RESULT_IMG_DISPLAY_WIDTH, display_h), Image.Resampling.LANCZOS)

            # Convert to Tkinter format
            img_tk = ImageTk.PhotoImage(img_disp)

            # Update the label widget
            rv_image_label.img_tk = img_tk # IMPORTANT: Keep reference to avoid garbage collection
            rv_image_label.config(image=img_tk, text="") # Clear any previous text

        except Exception as e:
            print(f"ERROR: Failed to update results image display: {e}")
            # Display error text on the label if image processing fails
            rv_image_label.config(image=placeholder_img_tk if placeholder_img_tk else '', # Show placeholder or clear
                                  text="Img Error")
            rv_image_label.img_tk = placeholder_img_tk if placeholder_img_tk else None # Keep/clear reference
    else:
         print("Warning: rv_image_label widget not found during results update.")

    # Update Text Labels on Results Page
    if rv_prediction_label:
        rv_prediction_label.config(text=f"{predicted_label}")
    if rv_confidence_label:
        rv_confidence_label.config(text=f"{confidence:.1%}") # Format confidence as percentage
    if rv_magnetism_label:
        # Use the text generated during sensor reading phase (includes units/cal status)
        rv_magnetism_label.config(text=mag_display_text)
    if rv_ldc_label:
        # Use the text generated during sensor reading phase (includes units/cal status)
        rv_ldc_label.config(text=ldc_display_text)

    # --- 7. Switch View ---
    print("Switching to results view.")
    show_results_view()
    # Note: The classify button remains disabled until the user clicks "Classify Another"
    #       (show_live_view re-enables it if AI is ready)
    print("="*10 + " Capture & Classify Complete " + "="*10 + "\n") # Marker


def calibrate_sensors():
    """
    Calibrates the idle voltage for the Hall sensor and the idle RP value for the LDC sensor.
    Uses the current readings with no object present as the baseline.
    """
    global IDLE_VOLTAGE, IDLE_RP_VALUE, window
    global previous_filtered_mag_mT # Reset smoothing on calibration
    global lv_calibrate_button, lv_classify_button, hall_sensor, ldc_initialized

    print("\n" + "="*10 + " Sensor Calibration Triggered " + "="*10) # Marker

    # --- Check if sensors are available ---
    hall_available = hall_sensor is not None
    ldc_available = ldc_initialized

    if not hall_available and not ldc_available:
         messagebox.showwarning("Calibration", "Neither Hall nor LDC sensor is available for calibration.")
         print("Calibration aborted: No sensors available.")
         return

    # --- Provide instructions to the user ---
    instruction_text = "Ensure NO metal object is near the sensors.\n\n"
    if hall_available: instruction_text += "- Hall sensor idle voltage will be measured.\n"
    if ldc_available: instruction_text += "- LDC sensor idle RP value will be measured.\n"
    instruction_text += "\nClick OK to start calibration, Cancel to abort."

    if not messagebox.askokcancel("Calibration Instructions", instruction_text):
        print("Calibration cancelled by user.")
        return # Abort if user cancels

    # --- Disable buttons during calibration ---
    if lv_calibrate_button: lv_calibrate_button.config(state=tk.DISABLED)
    if lv_classify_button: lv_classify_button.config(state=tk.DISABLED)
    window.update_idletasks() # Ensure GUI updates

    hall_results = "Hall Sensor: N/A"
    hall_success = False # Track success specifically for Hall
    ldc_results = "LDC Sensor: N/A"
    ldc_success = False # Track success specifically for LDC

    # --- Calibrate Hall Sensor ---
    if hall_available:
        print(f"Calibrating Hall sensor ({NUM_SAMPLES_CALIBRATION} samples)...")
        avg_v = get_averaged_hall_voltage(num_samples=NUM_SAMPLES_CALIBRATION)
        if avg_v is not None:
            IDLE_VOLTAGE = avg_v
            hall_results = f"Hall Idle Voltage: {IDLE_VOLTAGE:.4f} V"
            hall_success = True
            print(hall_results)
        else:
            IDLE_VOLTAGE = 0.0 # Reset on error
            hall_results = "Hall Sensor: Calibration Read Error!"
            hall_success = False
            print(hall_results)
    else:
        hall_results = "Hall Sensor: Not Available"
        hall_success = False # Not available = not successful calibration

    # --- Calibrate LDC Sensor ---
    if ldc_available:
        print(f"Calibrating LDC sensor ({NUM_SAMPLES_CALIBRATION} samples)...")
        avg_rp = get_averaged_rp_data(num_samples=NUM_SAMPLES_CALIBRATION)
        if avg_rp is not None:
            IDLE_RP_VALUE = int(round(avg_rp)) # Store rounded integer
            ldc_results = f"LDC Idle RP Value: {IDLE_RP_VALUE}"
            ldc_success = True
            print(ldc_results)
        else:
            IDLE_RP_VALUE = 0 # Reset on error
            ldc_results = "LDC Sensor: Calibration Read Error!"
            ldc_success = False
            print(ldc_results)
    else:
        ldc_results = "LDC Sensor: Not Available"
        ldc_success = False # Not available = not successful calibration

    # Reset magnetism smoothing filter after calibration
    previous_filtered_mag_mT = None
    print("Magnetism display smoothing filter reset.")

    # --- Re-enable Buttons ---
    if lv_calibrate_button: lv_calibrate_button.config(state=tk.NORMAL)
    # Only re-enable classify if AI is ready
    if lv_classify_button:
        if interpreter: lv_classify_button.config(state=tk.NORMAL)
        else: lv_classify_button.config(state=tk.DISABLED)

    # --- Display Results ---
    final_message = f"Calibration Results:\n\n{hall_results}\n{ldc_results}"
    print("--- Calibration Complete ---")

    # Determine message type based on success
    if hall_success and ldc_success:
        messagebox.showinfo("Calibration Complete", final_message)
    elif (hall_available and not hall_success) or (ldc_available and not ldc_success):
        # At least one available sensor failed calibration
        messagebox.showwarning("Calibration Warning", f"Calibration finished with errors:\n\n{hall_results}\n{ldc_results}")
    else:
        # Both were unavailable or failed (if available) - covered by unavailable case?
        # This case might be redundant if unavailable implies !success
         messagebox.showinfo("Calibration Finished", final_message) # Show info even if unavailable

    print("="*10 + " Sensor Calibration Finished " + "="*10 + "\n") # Marker


def update_camera_feed():
    """Updates the live camera feed label in the GUI (Live View)."""
    global lv_camera_label, window, camera

    # Check if window still exists before proceeding
    if not window or not window.winfo_exists():
        # print("Camera update loop: Window closed, stopping.") # Less verbose
        return

    img_tk = None # Initialize Tkinter image object for this update cycle

    # --- Read Frame ---
    if camera and camera.isOpened():
        ret, frame = camera.read()
        if ret and frame is not None:
            # --- Process Frame ---
            try:
                # Convert BGR frame (from OpenCV) to RGB
                img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                # Create PIL image from numpy array
                img_pil = Image.fromarray(img_rgb)
                # Resize for display using thumbnail (preserves aspect ratio)
                # NEAREST is faster but lower quality, LANCZOS is better quality but slower
                img_pil.thumbnail((DISPLAY_IMG_WIDTH, DISPLAY_IMG_HEIGHT), Image.Resampling.NEAREST)
                # Convert PIL image to Tkinter PhotoImage
                img_tk = ImageTk.PhotoImage(img_pil)
            except Exception as e:
                print(f"Error processing camera frame: {e}")
                img_tk = None # Ensure img_tk is None on error
        # else: # Optional: Handle frame read failure more explicitly
        #      print("Warning: Failed to read frame from camera in update loop.")
    # else: # Camera not open or not initialized
         # img_tk will remain None

    # --- Update GUI Label ---
    if lv_camera_label:
        if img_tk:
            # Valid frame processed, update image and clear any text
            lv_camera_label.img_tk = img_tk # Keep reference!
            lv_camera_label.configure(image=img_tk, text="")
        else:
            # No valid frame (camera failed, read error, processing error)
            # Display placeholder or text
            # Check if a 'no camera' placeholder image exists, create if not
            if not hasattr(lv_camera_label, 'no_cam_img'):
                lv_camera_label.no_cam_img = create_placeholder_image(
                    DISPLAY_IMG_WIDTH // 2, DISPLAY_IMG_HEIGHT // 2, '#BDBDBD', "No Feed"
                )

            # Get the string representation of the current image in the label
            # Comparing Tkinter image objects directly is unreliable
            current_img_str = str(lv_camera_label.cget("image"))
            placeholder_img_str = str(lv_camera_label.no_cam_img) if lv_camera_label.no_cam_img else ""

            # Update only if the current image is not already the placeholder
            if lv_camera_label.no_cam_img and current_img_str != placeholder_img_str:
                 lv_camera_label.configure(image=lv_camera_label.no_cam_img, text="") # Show placeholder, clear text
                 lv_camera_label.img_tk = lv_camera_label.no_cam_img # Keep reference
            elif not lv_camera_label.no_cam_img and lv_camera_label.cget("text") != "Camera Failed":
                 # Fallback text if placeholder creation failed
                 lv_camera_label.configure(image='', text="Camera Failed") # Clear image, show text
                 lv_camera_label.img_tk = None

    # --- Schedule Next Update ---
    # Use try-except to gracefully handle error if window is destroyed
    # between the initial check and the after() call.
    try:
        if window and window.winfo_exists():
            window.after(CAMERA_UPDATE_INTERVAL_MS, update_camera_feed)
    except tk.TclError:
        pass # Ignore error if window is destroyed


def update_magnetism():
    """Updates the live magnetism reading label in the GUI (Live View) with smoothing."""
    global lv_magnetism_label, window, previous_filtered_mag_mT, IDLE_VOLTAGE, hall_sensor

    # Check if window still exists
    if not window or not window.winfo_exists():
        # print("Magnetism update loop: Window closed, stopping.") # Less verbose
        return

    display_text = "N/A" # Default text if sensor unavailable

    if hall_sensor:
        # Get averaged voltage reading using the faster update rate
        avg_voltage = get_averaged_hall_voltage(num_samples=NUM_SAMPLES_PER_UPDATE)

        if avg_voltage is not None:
            # --- Calculate Magnetism ---
            try:
                # Use calibrated idle voltage if available
                idle_v = IDLE_VOLTAGE
                # Check sensitivity before division
                if abs(SENSITIVITY_V_PER_MILLITESLA) < 1e-9:
                    raise ZeroDivisionError("Sensitivity near zero")

                # Calculate raw magnetism based on voltage difference from idle
                raw_mag_mT = (avg_voltage - idle_v) / SENSITIVITY_V_PER_MILLITESLA

                # --- Apply Smoothing Filter ---
                # Apply simple exponential smoothing filter for display stability
                if previous_filtered_mag_mT is None:
                    # Initialize filter on first valid reading or after calibration reset
                    filtered_mag_mT = raw_mag_mT
                else:
                    # Apply smoothing formula: new = alpha*raw + (1-alpha)*previous
                    filtered_mag_mT = (MAGNETISM_FILTER_ALPHA * raw_mag_mT) + \
                                      ((1 - MAGNETISM_FILTER_ALPHA) * previous_filtered_mag_mT)

                # Update the stored previous value for the next iteration
                previous_filtered_mag_mT = filtered_mag_mT

                # --- Format Display Text ---
                # Display in microTesla (µT) if magnitude is small, else milliTesla (mT)
                # Adjust threshold (e.g., 0.1 mT) as needed
                if abs(filtered_mag_mT) < 0.1:
                    value_uT = filtered_mag_mT * 1000
                    unit = "µT"
                    # Show sign and 1 or 2 decimal places for µT
                    display_text = f"{value_uT:+.1f} {unit}"
                else:
                    value_mT = filtered_mag_mT
                    unit = "mT"
                    # Show sign and 2 decimal places for mT
                    display_text = f"{value_mT:+.2f} {unit}"

                # Append calibration status if idle voltage is still default (0.0)
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

    # --- Update GUI Label ---
    if lv_magnetism_label:
        # Avoid updating if text is the same to reduce potential flicker (optional)
        if lv_magnetism_label.cget("text") != display_text:
            lv_magnetism_label.config(text=display_text)

    # --- Schedule Next Update ---
    try:
        if window and window.winfo_exists():
            window.after(GUI_UPDATE_INTERVAL_MS, update_magnetism)
    except tk.TclError:
        pass # Ignore error if window is destroyed


def update_ldc_reading():
    """Updates the live LDC RP reading label in the GUI (Live View) with smoothing."""
    global lv_ldc_label, window, RP_DISPLAY_BUFFER, IDLE_RP_VALUE, ldc_initialized

    # Check if window still exists
    if not window or not window.winfo_exists():
        # print("LDC update loop: Window closed, stopping.") # Less verbose
        return

    display_rp_text = "N/A" # Default text if sensor unavailable

    if ldc_initialized:
        # Get averaged RP data using the faster update rate
        avg_rp_val = get_averaged_rp_data(num_samples=NUM_SAMPLES_PER_UPDATE)

        if avg_rp_val is not None:
            # Add the new average reading to the display buffer
            RP_DISPLAY_BUFFER.append(avg_rp_val)

            # Calculate the average of the buffer for smoother display
            if RP_DISPLAY_BUFFER: # Check if buffer has readings
                buffer_avg = statistics.mean(RP_DISPLAY_BUFFER)
                current_rp_int = int(round(buffer_avg)) # Use integer part of buffer average

                # Format display text based on calibration status
                if IDLE_RP_VALUE != 0:
                    delta = current_rp_int - IDLE_RP_VALUE
                    # Show current smoothed value and the delta from calibrated idle
                    display_rp_text = f"{current_rp_int} (Δ{delta:+,})" # Use delta symbol
                else:
                    # Show only current smoothed value if not calibrated
                    display_rp_text = f"{current_rp_int} (No Cal)"
            else:
                # Buffer might be empty if reads consistently fail right at the start
                display_rp_text = "Buffering..."
        else:
            # avg_rp_val was None (read error)
            # RP_DISPLAY_BUFFER.clear() # Optionally clear buffer on read error? Or let old values fade?
            display_rp_text = "Read Err"
    # else: # LDC not initialized, display_rp_text remains "N/A"

    # --- Update GUI Label ---
    if lv_ldc_label:
         # Avoid updating if text is the same to reduce potential flicker (optional)
        if lv_ldc_label.cget("text") != display_rp_text:
            lv_ldc_label.config(text=display_rp_text)

    # --- Schedule Next Update ---
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

    print("Setting up GUI...")

    # --- Window Setup ---
    window = tk.Tk()
    window.title("AI Metal Classifier v3.0.9 (RPi)") # Indicate Raspberry Pi version
    # Set initial size, user can resize. Consider Pi screen resolution.
    window.geometry("800x600") # Adjusted default size
    # window.attributes('-fullscreen', True) # Uncomment for fullscreen mode on Pi

    # --- Style Configuration (using ttk for better look) ---
    style = ttk.Style()
    # Try to use a theme that looks good on Linux/Pi (clam, alt, default)
    available_themes = style.theme_names()
    # print(f"Available themes: {available_themes}") # Debug available themes
    if 'clam' in available_themes: style.theme_use('clam')
    elif 'alt' in available_themes: style.theme_use('alt')
    else: style.theme_use('default') # Fallback

    # --- Define Fonts (adjust family/size as needed) ---
    try:
        # Use common sans-serif fonts, adjust sizes for Pi display
        title_font = tkFont.Font(family="DejaVu Sans", size=16, weight="bold")
        label_font = tkFont.Font(family="DejaVu Sans", size=10)
        # Use monospaced font for sensor readouts for alignment
        readout_font = tkFont.Font(family="DejaVu Sans Mono", size=12, weight="bold")
        button_font = tkFont.Font(family="DejaVu Sans", size=10, weight="bold")
        result_title_font = tkFont.Font(family="DejaVu Sans", size=11, weight="bold")
        result_value_font = tkFont.Font(family="DejaVu Sans Mono", size=12, weight="bold") # Monospaced
    except tk.TclError: # Fallback if specific fonts aren't available
        print("Warning: Preferred fonts not found, using Tkinter defaults.")
        title_font = tkFont.nametofont("TkHeadingFont")
        label_font = tkFont.nametofont("TkTextFont")
        readout_font = tkFont.nametofont("TkFixedFont")
        button_font = tkFont.nametofont("TkDefaultFont")
        result_title_font = tkFont.nametofont("TkDefaultFont")
        result_value_font = tkFont.nametofont("TkFixedFont")
        # Optionally adjust default font sizes
        # label_font.config(size=10)
        # button_font.config(size=10)


    # --- Configure Widget Styles ---
    style.configure("TLabel", font=label_font, padding=2)
    style.configure("TButton", font=button_font, padding=(8, 5)) # Adjust button padding
    style.configure("TLabelframe", padding=6)
    style.configure("TLabelframe.Label", font=tkFont.Font(family="DejaVu Sans", size=11, weight="bold")) # Label frame title font
    # Custom styles for specific label types
    style.configure("Readout.TLabel", font=readout_font, padding=(4, 1), anchor=tk.E, foreground="#0000AA") # Blue readout text, right-aligned
    style.configure("ResultTitle.TLabel", font=label_font, padding=(4, 2), anchor=tk.W) # Left-align result titles
    style.configure("ResultValue.TLabel", font=result_value_font, padding=(4, 1), anchor=tk.E, foreground="#0000AA") # Blue result values, right-aligned
    style.configure("Prediction.TLabel", font=tkFont.Font(family="DejaVu Sans", size=16, weight="bold"), padding=(4, 1), anchor=tk.E, foreground="#AA0000") # Larger red prediction


    # --- Main Content Frame (holds either live or results view) ---
    main_frame = ttk.Frame(window, padding="5 5 5 5")
    main_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
    # Configure grid weights for resizing behavior
    main_frame.rowconfigure(0, weight=1)    # Allow row 0 to expand vertically
    main_frame.columnconfigure(0, weight=1) # Allow column 0 to expand horizontally

    # ===================================
    # === Live View Frame Construction ===
    # ===================================
    live_view_frame = ttk.Frame(main_frame, padding="5 5 5 5")
    # Grid configuration: Camera feed takes more space horizontally
    live_view_frame.columnconfigure(0, weight=3) # Camera column (larger weight)
    live_view_frame.columnconfigure(1, weight=1) # Controls column (smaller weight)
    live_view_frame.rowconfigure(0, weight=1)    # Allow row to expand vertically

    # --- Live Camera Feed Label ---
    lv_camera_label = ttk.Label(live_view_frame, text="Initializing Camera...",
                                anchor="center", borderwidth=1, relief="sunken",
                                background="#CCCCCC") # Slightly darker gray background
    # Place in grid, expanding in all directions
    lv_camera_label.grid(row=0, column=0, padx=(0, 5), pady=0, sticky="nsew")

    # --- Controls Frame (Right Side) ---
    lv_controls_frame = ttk.Frame(live_view_frame)
    lv_controls_frame.grid(row=0, column=1, sticky="nsew", padx=(5,0))
    lv_controls_frame.columnconfigure(0, weight=1) # Allow controls column content to expand horizontally if needed
    # Configure rows for spacing: Readings (0), Actions (1), Spacer (2, weight=1)
    lv_controls_frame.rowconfigure(0, weight=0)
    lv_controls_frame.rowconfigure(1, weight=0)
    lv_controls_frame.rowconfigure(2, weight=1) # Spacer row pushes controls up

    # --- Live Readings Section ---
    lv_readings_frame = ttk.Labelframe(lv_controls_frame, text=" Live Readings ", padding="8 4 8 4")
    lv_readings_frame.grid(row=0, column=0, sticky="new", pady=(0, 10)) # Stick to top-width
    lv_readings_frame.columnconfigure(1, weight=1) # Allow value labels to expand horizontally

    # Magnetism Reading
    ttk.Label(lv_readings_frame, text="Magnetism:").grid(row=0, column=0, sticky="w", padx=(0, 8))
    lv_magnetism_label = ttk.Label(lv_readings_frame, text="Init...", style="Readout.TLabel")
    lv_magnetism_label.grid(row=0, column=1, sticky="ew")

    # LDC Reading
    ttk.Label(lv_readings_frame, text="LDC (Delta):").grid(row=1, column=0, sticky="w", padx=(0, 8), pady=(2, 0))
    lv_ldc_label = ttk.Label(lv_readings_frame, text="Init...", style="Readout.TLabel")
    lv_ldc_label.grid(row=1, column=1, sticky="ew", pady=(2, 0))

    # --- Actions Section ---
    lv_actions_frame = ttk.Labelframe(lv_controls_frame, text=" Actions ", padding="8 4 8 8")
    lv_actions_frame.grid(row=1, column=0, sticky="new", pady=(0, 10)) # Below readings
    lv_actions_frame.columnconfigure(0, weight=1) # Allow buttons to expand horizontally

    # Classify Button
    lv_classify_button = ttk.Button(lv_actions_frame, text="Capture & Classify", command=capture_and_classify)
    lv_classify_button.grid(row=0, column=0, sticky="ew", pady=(4, 4))

    # Calibrate Button
    lv_calibrate_button = ttk.Button(lv_actions_frame, text="Calibrate Sensors", command=calibrate_sensors)
    lv_calibrate_button.grid(row=1, column=0, sticky="ew", pady=(4, 4))

    # =====================================
    # === Results View Frame Construction ===
    # =====================================
    results_view_frame = ttk.Frame(main_frame, padding="10 10 10 10")
    # Configure grid to allow centering of content
    results_view_frame.rowconfigure(0, weight=1)    # Spacer row above
    results_view_frame.rowconfigure(1, weight=0)    # Content row (no extra weight)
    results_view_frame.rowconfigure(2, weight=1)    # Spacer row below
    results_view_frame.columnconfigure(0, weight=1) # Spacer col left
    results_view_frame.columnconfigure(1, weight=0) # Content col (no extra weight)
    results_view_frame.columnconfigure(2, weight=1) # Spacer col right

    # --- Centering Frame for Content ---
    # Use an inner frame to hold the actual results content
    rv_content_frame = ttk.Frame(results_view_frame)
    # Place the content frame in the center cell of the results_view_frame grid
    rv_content_frame.grid(row=1, column=1, sticky="") # Don't make it sticky to keep it centered

    # --- Results Title ---
    ttk.Label(rv_content_frame, text="Classification Result", font=title_font).grid(row=0, column=0, columnspan=2, pady=(5, 15))

    # --- Placeholder Image for Results ---
    # Calculate default placeholder height (e.g., 3:4 aspect ratio for the width)
    placeholder_h = int(RESULT_IMG_DISPLAY_WIDTH * 0.75)
    placeholder_img_tk = create_placeholder_image(RESULT_IMG_DISPLAY_WIDTH, placeholder_h, '#E0E0E0', "Result")

    # --- Result Image Label ---
    rv_image_label = ttk.Label(rv_content_frame, anchor="center", borderwidth=1, relief="sunken")
    if placeholder_img_tk:
        rv_image_label.config(image=placeholder_img_tk)
        rv_image_label.img_tk = placeholder_img_tk # Keep reference
    else:
        rv_image_label.config(text="Image Area", width=30, height=15) # Fallback text/size
    rv_image_label.grid(row=1, column=0, columnspan=2, pady=(0, 15))

    # --- Results Details Frame ---
    rv_details_frame = ttk.Frame(rv_content_frame)
    rv_details_frame.grid(row=2, column=0, columnspan=2, pady=(0, 15))
    rv_details_frame.columnconfigure(1, weight=1) # Allow value labels to expand

    res_row = 0 # Row counter for details grid

    # Predicted Material
    ttk.Label(rv_details_frame, text="Material:", style="ResultTitle.TLabel").grid(row=res_row, column=0, sticky="w", padx=(0,5))
    # Use a custom style for the prediction label to make it stand out
    rv_prediction_label = ttk.Label(rv_details_frame, text="---", style="Prediction.TLabel")
    rv_prediction_label.grid(row=res_row, column=1, sticky="ew", padx=5)
    res_row += 1

    # Confidence Score
    ttk.Label(rv_details_frame, text="Confidence:", style="ResultTitle.TLabel").grid(row=res_row, column=0, sticky="w", padx=(0,5), pady=(3,0))
    rv_confidence_label = ttk.Label(rv_details_frame, text="---", style="ResultValue.TLabel")
    rv_confidence_label.grid(row=res_row, column=1, sticky="ew", padx=5, pady=(3,0))
    res_row += 1

    # Separator
    ttk.Separator(rv_details_frame, orient='horizontal').grid(row=res_row, column=0, columnspan=2, sticky='ew', pady=8)
    res_row += 1

    # --- Sensor Readings Used ---
    ttk.Label(rv_details_frame, text="Sensor Values Used:", style="ResultTitle.TLabel", font=tkFont.Font(weight='bold')).grid(row=res_row, column=0, columnspan=2, sticky="w", pady=(0,3))
    res_row += 1

    # Magnetism Reading Used
    ttk.Label(rv_details_frame, text="  Magnetism:", style="ResultTitle.TLabel").grid(row=res_row, column=0, sticky="w", padx=(5,5))
    rv_magnetism_label = ttk.Label(rv_details_frame, text="---", style="ResultValue.TLabel")
    rv_magnetism_label.grid(row=res_row, column=1, sticky="ew", padx=5)
    res_row += 1

    # LDC Reading Used
    ttk.Label(rv_details_frame, text="  LDC Delta:", style="ResultTitle.TLabel").grid(row=res_row, column=0, sticky="w", padx=(5,5))
    rv_ldc_label = ttk.Label(rv_details_frame, text="---", style="ResultValue.TLabel")
    rv_ldc_label.grid(row=res_row, column=1, sticky="ew", padx=5)
    res_row += 1

    # --- Classify Another Button ---
    # Place this button below the details frame, within the centered content frame
    rv_classify_another_button = ttk.Button(rv_content_frame, text="<< Classify Another", command=show_live_view)
    # Use the content frame's grid for this button
    rv_classify_another_button.grid(row=3, column=0, columnspan=2, pady=(15, 5)) # Use row 3 within rv_content_frame

    # --- Set Initial State ---
    clear_results_display() # Initialize results widgets with placeholders
    show_live_view()        # Start by showing the live view

    print("GUI setup complete.")


# ==========================
# === Main Execution =======
# ==========================
def run_application():
    """Sets up the GUI, initializes update loops, and runs the main Tkinter event loop."""
    global window, lv_camera_label, lv_magnetism_label, lv_ldc_label, lv_classify_button, interpreter
    global camera, hall_sensor, ldc_initialized # Needed to check status

    # --- Setup GUI ---
    print("Setting up GUI...")
    try:
        setup_gui() # This creates the window and all widgets
    except Exception as e:
        print(f"FATAL ERROR: Failed to set up GUI: {e}")
        # Attempt to show error in a basic Tkinter window if main setup failed
        try:
            root = tk.Tk()
            root.withdraw() # Hide the empty root window
            messagebox.showerror("GUI Setup Error", f"Failed to initialize the application GUI:\n{e}\n\nPlease check console output for details.")
            root.destroy()
        except Exception:
            pass # Ignore if even basic Tk window fails
        return # Exit application if GUI setup fails critically

    # --- Update Initial Status Labels/Buttons based on Hardware/AI Init ---
    print("Updating initial GUI status based on hardware/AI checks...")
    if not camera:
        if lv_camera_label: lv_camera_label.configure(text="Camera Failed", image='') # Clear image if camera failed
        else: print("Warning: lv_camera_label not available to show camera status.")

    if not hall_sensor:
        if lv_magnetism_label: lv_magnetism_label.config(text="N/A")
        else: print("Warning: lv_magnetism_label not available to show Hall status.")

    if not ldc_initialized:
        if lv_ldc_label: lv_ldc_label.config(text="N/A")
        else: print("Warning: lv_ldc_label not available to show LDC status.")

    # Disable classify button if AI failed to initialize
    if not interpreter:
        if lv_classify_button:
            lv_classify_button.config(state=tk.DISABLED, text="Classify (AI Failed)")
            print("AI Initialization Failed - Classification Disabled.")
        else:
            print("Warning: lv_classify_button not available to disable.")
            print("AI Initialization Failed - Classification Disabled.")
    # Disable calibrate button if both sensors are unavailable
    if not hall_sensor and not ldc_initialized:
         if lv_calibrate_button:
              lv_calibrate_button.config(state=tk.DISABLED, text="Calibrate (No Sensors)")
              print("Sensor Calibration Disabled - No sensors available.")
         else:
              print("Warning: lv_calibrate_button not available.")
              print("Sensor Calibration Disabled - No sensors available.")


    # --- Start Update Loops ---
    # Start background update loops for camera and sensors
    # These loops will schedule themselves to run repeatedly using window.after()
    print("Starting GUI update loops...")
    update_camera_feed()    # Start camera feed updates
    update_magnetism()      # Start magnetism updates
    update_ldc_reading()    # Start LDC updates

    # --- Run Tkinter Main Loop ---
    print("Starting Tkinter main loop... (Press Ctrl+C in console to exit)")
    try:
        # Add protocol handler for window close button (the 'X')
        window.protocol("WM_DELETE_WINDOW", on_closing)
        window.mainloop() # Blocks here until the window is closed
    except Exception as e:
         print(f"ERROR: An exception occurred in the Tkinter main loop: {e}")
         # Log the error, cleanup will happen in the finally block of main execution

    print("Tkinter main loop finished.")


# ==========================
# === Window Closing =======
# ==========================
def on_closing():
    """Handles window close event (clicking the 'X'), ensuring cleanup."""
    global window
    print("Window close requested by user.")
    # Ask for confirmation before quitting
    if messagebox.askokcancel("Quit", "Do you want to quit the AI Metal Classifier application?"):
        print("Proceeding with shutdown...")
        # Stop update loops if possible (by destroying window)
        # Destroying the window should stop the 'after' calls eventually and break mainloop
        if window:
            try:
                window.destroy() # This will break the mainloop
                print("Tkinter window destroyed.")
            except tk.TclError:
                 print("Note: Window already destroyed during shutdown.")
            except Exception as e:
                 print(f"Warning: Error destroying Tkinter window: {e}")
        # Actual hardware cleanup should happen in the finally block of the main execution
    else:
        print("Shutdown cancelled by user.")


# ==========================
# === Cleanup Resources ====
# ==========================
def cleanup_resources():
    """Releases hardware resources (Camera, SPI, GPIO) gracefully."""
    print("\n--- Cleaning up resources ---")

    # --- Release Camera ---
    global camera
    if camera and camera.isOpened():
        try:
            print("Releasing camera...")
            camera.release()
            print("Camera released.")
        except Exception as e:
            print(f"Warning: Error releasing camera: {e}")
    # Close any OpenCV windows that might have been opened (less likely in this GUI app)
    # cv2.destroyAllWindows()

    # --- Cleanup SPI/LDC ---
    global spi, ldc_initialized, SPI_ENABLED, CS_PIN
    if spi: # Check if spi object exists (meaning initialization was attempted)
        try:
            # Put LDC to sleep before closing SPI if it was initialized successfully
            if ldc_initialized:
                print("Putting LDC1101 to sleep...")
                # Use a direct write here, avoid functions that might check spi/ldc_initialized again
                try:
                    # Ensure CS pin is configured before using it (check if GPIO setup succeeded)
                    if CS_PIN is not None and 'GPIO' in globals() and GPIO.getmode() is not None:
                         GPIO.output(CS_PIN, GPIO.LOW)
                         spi.xfer2([START_CONFIG_REG & 0x7F, SLEEP_MODE])
                         GPIO.output(CS_PIN, GPIO.HIGH)
                         time.sleep(0.05) # Short delay
                         print("LDC sleep command sent.")
                    else:
                         print("Note: CS_PIN/GPIO not available, cannot send sleep command.")
                except Exception as ldc_e:
                    print(f"Note: Error sending sleep command to LDC during cleanup: {ldc_e}")
        except Exception as e:
            # Catch errors related to accessing ldc_initialized or spi during cleanup
             print(f"Note: Error during LDC sleep attempt in cleanup: {e}")
        finally:
            # Always try to close SPI if the object exists
            try:
                print("Closing SPI...")
                spi.close()
                print("SPI closed.")
            except Exception as e:
                print(f"Warning: Error closing SPI: {e}")

    # --- Cleanup GPIO ---
    # Only cleanup GPIO if the SPI library was successfully imported initially AND GPIO setup likely happened
    if SPI_ENABLED and 'GPIO' in globals():
        try:
            # Check if GPIO mode was set (indicates setup was likely successful)
            if GPIO.getmode() is not None: # Check if BCM or BOARD mode was set
                 print("Cleaning up GPIO...")
                 GPIO.cleanup()
                 print("GPIO cleaned up.")
            else:
                 print("Note: GPIO mode not set, skipping cleanup (likely failed during init).")
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
    print("="*30)
    print(" Starting AI Metal Classifier (RPi) ")
    print("="*30)

    hardware_init_attempted = False
    ai_init_ok = False
    # app_running flag might not be strictly necessary as window closing handles exit

    try:
        # --- Initialize Hardware ---
        # Set flag first in case init itself raises an unexpected exception
        hardware_init_attempted = True
        initialize_hardware()
        # Note: initialize_hardware prints its own errors/warnings

        # --- Initialize AI ---
        # Proceed even if some hardware failed, AI might still work partially or GUI can show status
        ai_init_ok = initialize_ai()
        # Note: initialize_ai prints its own errors/warnings and returns status

        # --- Run Application's Main Loop ---
        # The application GUI should launch even if some components failed,
        # as run_application/setup_gui handles disabling features based on init status.
        run_application() # This creates the GUI and enters the mainloop

    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully in the console
        print("\nKeyboard interrupt detected. Exiting application.")
        # Try to destroy window if it exists to exit mainloop cleanly
        try:
            if window and window.winfo_exists():
                window.destroy()
        except Exception:
            pass # Ignore errors during forced shutdown
        # Cleanup will happen in the finally block

    except Exception as e:
        # Catch any unexpected fatal errors during initialization or the main loop run
        print("\n" + "="*30)
        print(f"FATAL ERROR in main execution: {e}")
        print("="*30)
        # Attempt to show error in GUI if possible (might fail if error was in GUI setup)
        try:
            if window and window.winfo_exists():
                messagebox.showerror("Fatal Application Error",
                                     f"An unrecoverable error occurred:\n\n{e}\n\nPlease check console for details.")
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
            pass # Ignore errors during forced shutdown

    finally:
        # --- Cleanup ---
        # Ensure cleanup runs regardless of how the application exits (normal close, interrupt, error)
        # Only attempt hardware cleanup if initialization was actually attempted
        if hardware_init_attempted:
            cleanup_resources()
        else:
             print("Skipping resource cleanup as hardware initialization was not attempted.")

        print("\nApplication finished.")
        print("="*30)
