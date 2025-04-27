# CODE 3.0.10 - AI Metal Classifier GUI with Results Page
# Description: Displays live sensor data and camera feed.
#              Captures image and sensor readings, classifies metal using a TFLite model,
#              and displays the results on a dedicated page. Includes enhanced debug prints.
# Version: 3.0.10 - Changed numerical preprocessing to use raw sensor values (Mag mT, LDC RP)
#                  instead of LDC delta before scaling, aligning with likely scaler training.
#                  Added more detailed debugging prints throughout the capture/classify process.
# FIXED:     Potential mismatch between sensor data processing and scaler expectation.
# DEBUG:     Enhanced prints in capture_and_classify, preprocess_input, run_inference, postprocess_output.

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
            expected_features = 2 # We expect Magnetism and LDC RP value
            if hasattr(numerical_scaler, 'n_features_in_'):
                print(f"Scaler expects {numerical_scaler.n_features_in_} features.")
                if numerical_scaler.n_features_in_ != expected_features:
                     print(f"ERROR: Scaler was trained on {numerical_scaler.n_features_in_} features, but script expects {expected_features} (Magnetism, LDC RP). Ensure this is correct!")
                     # This is likely a fatal error for accurate predictions
                     ai_ready = False
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
                print(f"ERROR: Model expected 2 inputs, but found {len(input_details)}. Preprocessing will likely fail.")
                # This is likely fatal
                ai_ready = False

            # 2. Check output shape vs number of labels
            if output_details:
                output_shape = output_details[0]['shape']
                # Output shape is often (1, num_classes)
                num_classes_in_model = output_shape[-1] # Get the last dimension size
                if num_classes_in_model != len(loaded_labels):
                    print(f"ERROR: Model output size ({num_classes_in_model}) does not match the number of loaded labels ({len(loaded_labels)}). Predictions will be incorrect.")
                    # This is likely fatal
                    ai_ready = False
            else:
                print("ERROR: Could not get model output details for sanity check.")
                ai_ready = False

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
        # Keep labels loaded if they were successful, might be useful for display
        # loaded_labels = []
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
            # print("Warning: Failed to read MSB or LSB for LDC RP data.") # Reduced verbosity
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
            # print(f"Warning: Failed to get LDC RP data for sample {i+1}/{num_samples}. Skipping sample.") # Reduced verbosity
            # Optionally decide whether to abort entirely or just skip the sample
            # return None # Uncomment to abort averaging if any read fails
            pass # Just skip the failed sample

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
        # print("Warning: No valid LDC RP readings collected for averaging.") # Reduced verbosity
        return None

# ==========================
# === AI Processing ========
# ==========================

# MODIFIED: Function now accepts raw LDC RP value instead of delta
def preprocess_input(image_pil, mag_mT, ldc_rp_raw):
    """
    Prepares the captured image and sensor data for the TFLite model.
    Args:
        image_pil (PIL.Image): The captured image (RGB).
        mag_mT (float or None): Magnetism reading in milliTesla.
        ldc_rp_raw (float or None): Raw (averaged) LDC RP value.
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
        # Resize image
        img_resized = image_pil.resize((AI_IMG_WIDTH, AI_IMG_HEIGHT), Image.Resampling.LANCZOS)
        img_rgb = img_resized.convert('RGB')
        image_np = np.array(img_rgb, dtype=np.float32)

        # Normalize pixel values (Match training!)
        image_np /= 255.0 # Assuming [0, 1] normalization
        print(f"Image normalized (assuming [0, 1] range). Min: {image_np.min():.2f}, Max: {image_np.max():.2f}")

        # Add batch dimension
        image_input = np.expand_dims(image_np, axis=0)
        print(f"Image preprocessed. Shape: {image_input.shape}, Dtype: {image_input.dtype}")

    except Exception as e:
        print(f"ERROR: Image preprocessing failed: {e}")
        return None

    # --- 2. Preprocess Numerical Features ---
    print("Preprocessing numerical features...")
    # Handle potential None values from sensor readings by defaulting to 0.0
    used_default_mag = False
    used_default_ldc = False

    if mag_mT is None:
        print("DEBUG Preprocess: Magnetism reading was None, defaulting to 0.0 for scaling.")
        mag_mT_val = 0.0
        used_default_mag = True
    else:
        mag_mT_val = float(mag_mT) # Ensure float

    # MODIFIED: Use raw LDC RP value
    if ldc_rp_raw is None:
        print("DEBUG Preprocess: LDC RP raw value was None, defaulting to 0.0 for scaling.")
        ldc_rp_raw_val = 0.0
        used_default_ldc = True
    else:
        ldc_rp_raw_val = float(ldc_rp_raw) # Ensure float

    # Create NumPy array for numerical features (shape: [1, num_features])
    # IMPORTANT: The order MUST match the order the scaler was trained on!
    # Assuming order: [Magnetism_mT, LDC_RP_Raw] - VERIFY THIS!
    numerical_features = np.array([[mag_mT_val, ldc_rp_raw_val]], dtype=np.float32)
    print(f"DEBUG Preprocess: Raw numerical features BEFORE scaling: {numerical_features}")

    # Scale numerical features using the loaded scaler
    try:
        print("Applying numerical scaler...")
        # Suppress potential UserWarning about feature names
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

        # Image tensor check
        if len(shape) == 4 and shape[1] == AI_IMG_HEIGHT and shape[2] == AI_IMG_WIDTH and shape[3] == 3:
            if image_input_index == -1:
                image_input_index = index
                image_input_dtype = dtype
                print(f"Found potential Image Input: Index={index}, Shape={shape}, Dtype={dtype}")
            else:
                print(f"Warning: Found multiple potential image input tensors (Indices {image_input_index} and {index}). Using first one found.")

        # Numerical tensor check
        elif len(shape) == 2:
            if numerical_input_index == -1:
                numerical_input_index = index
                numerical_input_dtype = dtype
                num_features_expected_by_model = shape[1]
                print(f"Found potential Numerical Input: Index={index}, Shape={shape}, Dtype={dtype}")
                # Validate against scaler's expected features
                if hasattr(numerical_scaler, 'n_features_in_') and num_features_expected_by_model != numerical_scaler.n_features_in_:
                     print(f"ERROR: Model expects {num_features_expected_by_model} numerical features, but scaler was trained on {numerical_scaler.n_features_in_}. Mismatch!")
                     return None
                elif scaled_numerical_features.shape[1] != num_features_expected_by_model:
                     print(f"ERROR: Model expects {num_features_expected_by_model} numerical features, but preprocessed data has {scaled_numerical_features.shape[1]}. Mismatch!")
                     return None
            else:
                print(f"Warning: Found multiple potential numerical input tensors (Indices {numerical_input_index} and {index}). Using first one found.")

    # --- Check if both inputs were found ---
    if image_input_index == -1 or numerical_input_index == -1:
        print("ERROR: Failed to identify distinct image and numerical input tensors based on shape and dimension checks.")
        print("Input Details were:")
        for detail in input_details: print(f"  - Index: {detail['index']}, Shape: {detail['shape']}, Dtype: {detail['dtype']}")
        return None

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
            print("Model expects UINT8 image input. Rescaling normalized image from [0,1] to [0,255] and casting.")
            final_image_input = (image_input * 255.0).astype(np.uint8)

        # Create the dictionary mapping index to the correctly typed numpy array
        model_inputs = {
            image_input_index: final_image_input,
            numerical_input_index: final_numerical_input
        }
        print("Model inputs prepared successfully.")
        # More detailed print of final inputs being sent to model
        print(f"  Final Image Input: Index={image_input_index}, Shape={model_inputs[image_input_index].shape}, Dtype={model_inputs[image_input_index].dtype}, Min={model_inputs[image_input_index].min()}, Max={model_inputs[image_input_index].max()}")
        print(f"  Final Numerical Input: Index={numerical_input_index}, Shape={model_inputs[numerical_input_index].shape}, Dtype={model_inputs[numerical_input_index].dtype}, Value={model_inputs[numerical_input_index]}")
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
            # Set the tensor data in the interpreter
            interpreter.set_tensor(index, data)
            print(f"  DEBUG Inference: Set tensor index {index} with shape {data.shape} and dtype {data.dtype}") # Debug print
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
        print(f"DEBUG Inference: Raw output data retrieved. Shape: {output_data.shape}, Dtype: {output_data.dtype}")
        # Print the actual raw output values for debugging
        print(f"DEBUG Inference: Raw output values: {output_data}")
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
        if len(output_data.shape) == 2 and output_data.shape[0] == 1:
            probabilities = output_data[0]
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
        # Combine labels and probabilities for printing
        prob_dict = {label: prob for label, prob in zip(loaded_labels, probabilities)}
        # Sort by probability descending for clarity
        sorted_probs = sorted(prob_dict.items(), key=lambda item: item[1], reverse=True)
        for label, prob in sorted_probs:
            print(f"  - {label}: {prob:.6f}") # Increased precision

        # --- Find Best Prediction ---
        predicted_index = np.argmax(probabilities)
        confidence = float(probabilities[predicted_index]) # Get the highest probability
        predicted_label = loaded_labels[predicted_index]

        print(f"Final Prediction: '{predicted_label}', Confidence: {confidence:.4f}")
        print("--- Postprocessing Complete ---") # Marker
        return predicted_label, confidence

    except IndexError as e:
        print(f"ERROR: Index error during postprocessing. Output shape: {output_data.shape}. Error: {e}")
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

    if results_view_frame and results_view_frame.winfo_ismapped():
        results_view_frame.pack_forget()
    if live_view_frame and not live_view_frame.winfo_ismapped():
        live_view_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
    if lv_classify_button:
        lv_classify_button.config(state=tk.NORMAL if interpreter else tk.DISABLED)

def show_results_view():
    """Hides the live view and shows the classification results view."""
    global live_view_frame, results_view_frame

    if live_view_frame and live_view_frame.winfo_ismapped():
        live_view_frame.pack_forget()
    if results_view_frame and not results_view_frame.winfo_ismapped():
        results_view_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

# ======================
# === GUI Functions ===
# ======================
def create_placeholder_image(width, height, color='#E0E0E0', text="No Image"):
    """Creates a simple PIL placeholder image with optional text and converts it to Tkinter format."""
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

    if rv_image_label:
        if placeholder_img_tk:
            rv_image_label.img_tk = placeholder_img_tk
            rv_image_label.config(image=placeholder_img_tk, text="")
        else:
            rv_image_label.config(image='', text="No Image")
            rv_image_label.img_tk = None

    default_text = "---"
    if rv_prediction_label: rv_prediction_label.config(text=default_text)
    if rv_confidence_label: rv_confidence_label.config(text=default_text)
    if rv_magnetism_label: rv_magnetism_label.config(text=default_text)
    if rv_ldc_label: rv_ldc_label.config(text=default_text)


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
    if lv_classify_button: lv_classify_button.config(state=tk.DISABLED)
    window.update_idletasks()

    # --- 1. Capture Image ---
    print("Capturing image...")
    ret, frame = camera.read()
    if not ret or frame is None:
        messagebox.showerror("Capture Error", "Failed to capture image from camera. Check camera connection.")
        print("ERROR: Failed to read frame from camera.")
        if lv_classify_button: show_live_view()
        return
    try:
        img_captured_pil = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        print(f"Image captured successfully. Size: {img_captured_pil.size}")
    except Exception as e:
        messagebox.showerror("Image Error", f"Failed to process captured image: {e}")
        print(f"ERROR: Failed converting captured frame to PIL Image: {e}")
        if lv_classify_button: show_live_view()
        return

    # --- 2. Read Sensors (using more samples for better accuracy during capture) ---
    print(f"Reading sensors for classification (using {NUM_SAMPLES_CALIBRATION} samples)...")
    # --- Read Hall Sensor ---
    avg_voltage = get_averaged_hall_voltage(num_samples=NUM_SAMPLES_CALIBRATION)
    current_mag_mT = None
    mag_display_text = "N/A"
    sensor_warning = False

    if avg_voltage is not None:
        print(f"  Avg Hall Voltage: {avg_voltage:.4f} V")
        try:
            idle_v = IDLE_VOLTAGE
            if idle_v == 0.0: print("  Warning: Hall sensor idle voltage not calibrated. Using 0.0V as baseline.")
            if abs(SENSITIVITY_V_PER_MILLITESLA) < 1e-9: raise ZeroDivisionError("Hall sensor sensitivity (V/mT) is zero.")

            current_mag_mT = (avg_voltage - idle_v) / SENSITIVITY_V_PER_MILLITESLA
            print(f"  Calculated Magnetism: {current_mag_mT:+.4f} mT")
            mag_display_text = f"{current_mag_mT:+.3f} mT"
            if idle_v == 0.0: mag_display_text += " (No Cal)"

        except ZeroDivisionError as e:
            mag_display_text = "Div Zero Err"
            print(f"  Warning: Magnetism calculation failed - {e}")
            current_mag_mT = None
            sensor_warning = True
        except Exception as e:
            mag_display_text = "Calc Error"
            print(f"  Warning: Magnetism calculation failed - {e}")
            current_mag_mT = None
            sensor_warning = True
    else:
        mag_display_text = "Read Error"
        print("  ERROR: Failed to read Hall sensor voltage.")
        current_mag_mT = None
        sensor_warning = True

    # --- Read LDC Sensor ---
    # MODIFIED: Get the raw average value, not the delta yet
    avg_rp_val = get_averaged_rp_data(num_samples=NUM_SAMPLES_CALIBRATION)
    current_rp_raw = None # Raw value (float or None) to pass to AI
    ldc_display_text = "N/A"

    if avg_rp_val is not None:
        current_rp_raw = avg_rp_val # Keep the float value for potential scaling precision
        current_rp_int = int(round(avg_rp_val)) # Rounded int for display/delta calc
        print(f"  Avg LDC RP Value: {avg_rp_val:.2f} (Rounded: {current_rp_int})")
        if IDLE_RP_VALUE != 0:
            delta_rp_display = current_rp_int - IDLE_RP_VALUE # Delta for display only
            print(f"  LDC Idle RP: {IDLE_RP_VALUE}, Delta RP (for display): {delta_rp_display:+,}")
            ldc_display_text = f"{current_rp_int} (Δ{delta_rp_display:+,})"
        else:
            print("  Warning: LDC sensor idle RP not calibrated.")
            ldc_display_text = f"{current_rp_int} (No Cal)"
    else:
        ldc_display_text = "Read Error"
        print("  ERROR: Failed to read LDC sensor RP value.")
        current_rp_raw = None
        sensor_warning = True

    # --- Log values being passed to AI ---
    # MODIFIED: Log raw LDC value being passed
    print(f"DEBUG Data for Preprocessing: Magnetism (mT): {current_mag_mT}, LDC RP Raw: {current_rp_raw}")

    if sensor_warning:
         print("WARNING: One or more sensor readings failed or were uncalibrated. Classification results may be inaccurate.")

    # --- 3. Preprocess Data for AI ---
    # MODIFIED: Pass raw LDC value
    model_inputs = preprocess_input(img_captured_pil, current_mag_mT, current_rp_raw)

    if model_inputs is None:
        messagebox.showerror("AI Error", "Data preprocessing failed. Check console logs.")
        print("ERROR: Preprocessing failed. Aborting classification.")
        if lv_classify_button: show_live_view()
        return

    # --- 4. Run AI Inference ---
    output_data = run_inference(model_inputs)

    if output_data is None:
        messagebox.showerror("AI Error", "AI model inference failed. Check console logs.")
        print("ERROR: Inference failed. Aborting classification.")
        if lv_classify_button: show_live_view()
        return

    # --- 5. Postprocess AI Output ---
    predicted_label, confidence = postprocess_output(output_data)

    print(f"--- Classification Result: Prediction='{predicted_label}', Confidence={confidence:.1%} ---") # Marker

    # --- 6. Update Results Display Widgets ---
    print("Updating results display widgets...")

    # Update Image Label
    if rv_image_label:
        try:
            w, h = img_captured_pil.size
            aspect = h / w if w > 0 else 1
            display_h = int(RESULT_IMG_DISPLAY_WIDTH * aspect)
            if display_h <= 0: display_h = int(RESULT_IMG_DISPLAY_WIDTH * 0.75)
            img_disp = img_captured_pil.resize((RESULT_IMG_DISPLAY_WIDTH, display_h), Image.Resampling.LANCZOS)
            img_tk = ImageTk.PhotoImage(img_disp)
            rv_image_label.img_tk = img_tk
            rv_image_label.config(image=img_tk, text="")
        except Exception as e:
            print(f"ERROR: Failed to update results image display: {e}")
            rv_image_label.config(image=placeholder_img_tk if placeholder_img_tk else '', text="Img Error")
            rv_image_label.img_tk = placeholder_img_tk if placeholder_img_tk else None

    # Update Text Labels
    if rv_prediction_label: rv_prediction_label.config(text=f"{predicted_label}")
    if rv_confidence_label: rv_confidence_label.config(text=f"{confidence:.1%}")
    # Display the values *used* for classification (Mag mT, Raw LDC RP) along with cal status
    if rv_magnetism_label: rv_magnetism_label.config(text=mag_display_text) # Already has cal status
    if rv_ldc_label: rv_ldc_label.config(text=ldc_display_text) # Already has cal status and delta

    # --- 7. Switch View ---
    print("Switching to results view.")
    show_results_view()
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

    hall_available = hall_sensor is not None
    ldc_available = ldc_initialized

    if not hall_available and not ldc_available:
         messagebox.showwarning("Calibration", "Neither Hall nor LDC sensor is available for calibration.")
         print("Calibration aborted: No sensors available.")
         return

    instruction_text = "Ensure NO metal object is near the sensors.\n\n"
    if hall_available: instruction_text += "- Hall sensor idle voltage will be measured.\n"
    if ldc_available: instruction_text += "- LDC sensor idle RP value will be measured.\n"
    instruction_text += "\nClick OK to start calibration, Cancel to abort."

    if not messagebox.askokcancel("Calibration Instructions", instruction_text):
        print("Calibration cancelled by user.")
        return

    if lv_calibrate_button: lv_calibrate_button.config(state=tk.DISABLED)
    if lv_classify_button: lv_classify_button.config(state=tk.DISABLED)
    window.update_idletasks()

    hall_results = "Hall Sensor: N/A"
    hall_success = False
    ldc_results = "LDC Sensor: N/A"
    ldc_success = False

    # Calibrate Hall Sensor
    if hall_available:
        print(f"Calibrating Hall sensor ({NUM_SAMPLES_CALIBRATION} samples)...")
        avg_v = get_averaged_hall_voltage(num_samples=NUM_SAMPLES_CALIBRATION)
        if avg_v is not None:
            IDLE_VOLTAGE = avg_v
            hall_results = f"Hall Idle Voltage: {IDLE_VOLTAGE:.4f} V"
            hall_success = True
            print(hall_results)
        else:
            IDLE_VOLTAGE = 0.0
            hall_results = "Hall Sensor: Calibration Read Error!"
            hall_success = False
            print(hall_results)
    else:
        hall_results = "Hall Sensor: Not Available"
        hall_success = False

    # Calibrate LDC Sensor
    if ldc_available:
        print(f"Calibrating LDC sensor ({NUM_SAMPLES_CALIBRATION} samples)...")
        avg_rp = get_averaged_rp_data(num_samples=NUM_SAMPLES_CALIBRATION)
        if avg_rp is not None:
            IDLE_RP_VALUE = int(round(avg_rp))
            ldc_results = f"LDC Idle RP Value: {IDLE_RP_VALUE}"
            ldc_success = True
            print(ldc_results)
        else:
            IDLE_RP_VALUE = 0
            ldc_results = "LDC Sensor: Calibration Read Error!"
            ldc_success = False
            print(ldc_results)
    else:
        ldc_results = "LDC Sensor: Not Available"
        ldc_success = False

    previous_filtered_mag_mT = None
    print("Magnetism display smoothing filter reset.")

    if lv_calibrate_button: lv_calibrate_button.config(state=tk.NORMAL)
    if lv_classify_button:
        if interpreter: lv_classify_button.config(state=tk.NORMAL)
        else: lv_classify_button.config(state=tk.DISABLED)

    final_message = f"Calibration Results:\n\n{hall_results}\n{ldc_results}"
    print("--- Calibration Complete ---")

    if hall_success and ldc_success:
        messagebox.showinfo("Calibration Complete", final_message)
    elif (hall_available and not hall_success) or (ldc_available and not ldc_success):
        messagebox.showwarning("Calibration Warning", f"Calibration finished with errors:\n\n{hall_results}\n{ldc_results}")
    else:
         messagebox.showinfo("Calibration Finished", final_message)

    print("="*10 + " Sensor Calibration Finished " + "="*10 + "\n") # Marker


def update_camera_feed():
    """Updates the live camera feed label in the GUI (Live View)."""
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
            except Exception as e:
                print(f"Error processing camera frame: {e}")
                img_tk = None

    if lv_camera_label:
        if img_tk:
            lv_camera_label.img_tk = img_tk
            lv_camera_label.configure(image=img_tk, text="")
        else:
            if not hasattr(lv_camera_label, 'no_cam_img'):
                lv_camera_label.no_cam_img = create_placeholder_image(
                    DISPLAY_IMG_WIDTH // 2, DISPLAY_IMG_HEIGHT // 2, '#BDBDBD', "No Feed"
                )
            current_img_str = str(lv_camera_label.cget("image"))
            placeholder_img_str = str(lv_camera_label.no_cam_img) if lv_camera_label.no_cam_img else ""
            if lv_camera_label.no_cam_img and current_img_str != placeholder_img_str:
                 lv_camera_label.configure(image=lv_camera_label.no_cam_img, text="")
                 lv_camera_label.img_tk = lv_camera_label.no_cam_img
            elif not lv_camera_label.no_cam_img and lv_camera_label.cget("text") != "Camera Failed":
                 lv_camera_label.configure(image='', text="Camera Failed")
                 lv_camera_label.img_tk = None
    try:
        if window and window.winfo_exists():
            window.after(CAMERA_UPDATE_INTERVAL_MS, update_camera_feed)
    except tk.TclError: pass


def update_magnetism():
    """Updates the live magnetism reading label in the GUI (Live View) with smoothing."""
    global lv_magnetism_label, window, previous_filtered_mag_mT, IDLE_VOLTAGE, hall_sensor

    if not window or not window.winfo_exists(): return

    display_text = "N/A"
    if hall_sensor:
        avg_voltage = get_averaged_hall_voltage(num_samples=NUM_SAMPLES_PER_UPDATE)
        if avg_voltage is not None:
            try:
                idle_v = IDLE_VOLTAGE
                if abs(SENSITIVITY_V_PER_MILLITESLA) < 1e-9: raise ZeroDivisionError("Sensitivity near zero")
                raw_mag_mT = (avg_voltage - idle_v) / SENSITIVITY_V_PER_MILLITESLA

                if previous_filtered_mag_mT is None: filtered_mag_mT = raw_mag_mT
                else: filtered_mag_mT = (MAGNETISM_FILTER_ALPHA * raw_mag_mT) + ((1 - MAGNETISM_FILTER_ALPHA) * previous_filtered_mag_mT)
                previous_filtered_mag_mT = filtered_mag_mT

                if abs(filtered_mag_mT) < 0.1: display_text = f"{filtered_mag_mT * 1000:+.1f} µT"
                else: display_text = f"{filtered_mag_mT:+.2f} mT"
                if IDLE_VOLTAGE == 0.0: display_text += " (No Cal)"

            except ZeroDivisionError: display_text = "Div Zero Err"; previous_filtered_mag_mT = None
            except Exception: display_text = "Calc Error"; previous_filtered_mag_mT = None
        else: display_text = "Read Err"; previous_filtered_mag_mT = None

    if lv_magnetism_label and lv_magnetism_label.cget("text") != display_text:
        lv_magnetism_label.config(text=display_text)

    try:
        if window and window.winfo_exists(): window.after(GUI_UPDATE_INTERVAL_MS, update_magnetism)
    except tk.TclError: pass


def update_ldc_reading():
    """Updates the live LDC RP reading label in the GUI (Live View) with smoothing."""
    global lv_ldc_label, window, RP_DISPLAY_BUFFER, IDLE_RP_VALUE, ldc_initialized

    if not window or not window.winfo_exists(): return

    display_rp_text = "N/A"
    if ldc_initialized:
        avg_rp_val = get_averaged_rp_data(num_samples=NUM_SAMPLES_PER_UPDATE)
        if avg_rp_val is not None:
            RP_DISPLAY_BUFFER.append(avg_rp_val)
            if RP_DISPLAY_BUFFER:
                buffer_avg = statistics.mean(RP_DISPLAY_BUFFER)
                current_rp_int = int(round(buffer_avg))
                if IDLE_RP_VALUE != 0:
                    delta = current_rp_int - IDLE_RP_VALUE
                    display_rp_text = f"{current_rp_int} (Δ{delta:+,})"
                else: display_rp_text = f"{current_rp_int} (No Cal)"
            else: display_rp_text = "Buffering..."
        else: display_rp_text = "Read Err"

    if lv_ldc_label and lv_ldc_label.cget("text") != display_rp_text:
        lv_ldc_label.config(text=display_rp_text)

    try:
        if window and window.winfo_exists(): window.after(GUI_UPDATE_INTERVAL_MS, update_ldc_reading)
    except tk.TclError: pass


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
    window = tk.Tk()
    window.title("AI Metal Classifier v3.0.10 (RPi Debug)")
    window.geometry("800x600")
    style = ttk.Style()
    available_themes = style.theme_names()
    if 'clam' in available_themes: style.theme_use('clam')
    elif 'alt' in available_themes: style.theme_use('alt')
    else: style.theme_use('default')

    try:
        title_font = tkFont.Font(family="DejaVu Sans", size=16, weight="bold")
        label_font = tkFont.Font(family="DejaVu Sans", size=10)
        readout_font = tkFont.Font(family="DejaVu Sans Mono", size=12, weight="bold")
        button_font = tkFont.Font(family="DejaVu Sans", size=10, weight="bold")
        result_title_font = tkFont.Font(family="DejaVu Sans", size=11, weight="bold")
        result_value_font = tkFont.Font(family="DejaVu Sans Mono", size=12, weight="bold")
    except tk.TclError:
        print("Warning: Preferred fonts not found, using Tkinter defaults.")
        title_font=tkFont.nametofont("TkHeadingFont"); label_font=tkFont.nametofont("TkTextFont"); readout_font=tkFont.nametofont("TkFixedFont"); button_font=tkFont.nametofont("TkDefaultFont"); result_title_font=tkFont.nametofont("TkDefaultFont"); result_value_font=tkFont.nametofont("TkFixedFont")

    style.configure("TLabel", font=label_font, padding=2)
    style.configure("TButton", font=button_font, padding=(8, 5))
    style.configure("TLabelframe", padding=6)
    style.configure("TLabelframe.Label", font=tkFont.Font(family="DejaVu Sans", size=11, weight="bold"))
    style.configure("Readout.TLabel", font=readout_font, padding=(4, 1), anchor=tk.E, foreground="#0000AA")
    style.configure("ResultTitle.TLabel", font=label_font, padding=(4, 2), anchor=tk.W)
    style.configure("ResultValue.TLabel", font=result_value_font, padding=(4, 1), anchor=tk.E, foreground="#0000AA")
    style.configure("Prediction.TLabel", font=tkFont.Font(family="DejaVu Sans", size=16, weight="bold"), padding=(4, 1), anchor=tk.E, foreground="#AA0000")

    main_frame = ttk.Frame(window, padding="5 5 5 5")
    main_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
    main_frame.rowconfigure(0, weight=1)
    main_frame.columnconfigure(0, weight=1)

    # Live View Frame
    live_view_frame = ttk.Frame(main_frame, padding="5 5 5 5")
    live_view_frame.columnconfigure(0, weight=3); live_view_frame.columnconfigure(1, weight=1); live_view_frame.rowconfigure(0, weight=1)
    lv_camera_label = ttk.Label(live_view_frame, text="Initializing Camera...", anchor="center", borderwidth=1, relief="sunken", background="#CCCCCC")
    lv_camera_label.grid(row=0, column=0, padx=(0, 5), pady=0, sticky="nsew")
    lv_controls_frame = ttk.Frame(live_view_frame)
    lv_controls_frame.grid(row=0, column=1, sticky="nsew", padx=(5,0))
    lv_controls_frame.columnconfigure(0, weight=1); lv_controls_frame.rowconfigure(0, weight=0); lv_controls_frame.rowconfigure(1, weight=0); lv_controls_frame.rowconfigure(2, weight=1)
    lv_readings_frame = ttk.Labelframe(lv_controls_frame, text=" Live Readings ", padding="8 4 8 4")
    lv_readings_frame.grid(row=0, column=0, sticky="new", pady=(0, 10)); lv_readings_frame.columnconfigure(1, weight=1)
    ttk.Label(lv_readings_frame, text="Magnetism:").grid(row=0, column=0, sticky="w", padx=(0, 8))
    lv_magnetism_label = ttk.Label(lv_readings_frame, text="Init...", style="Readout.TLabel"); lv_magnetism_label.grid(row=0, column=1, sticky="ew")
    ttk.Label(lv_readings_frame, text="LDC (Delta):").grid(row=1, column=0, sticky="w", padx=(0, 8), pady=(2, 0))
    lv_ldc_label = ttk.Label(lv_readings_frame, text="Init...", style="Readout.TLabel"); lv_ldc_label.grid(row=1, column=1, sticky="ew", pady=(2, 0))
    lv_actions_frame = ttk.Labelframe(lv_controls_frame, text=" Actions ", padding="8 4 8 8")
    lv_actions_frame.grid(row=1, column=0, sticky="new", pady=(0, 10)); lv_actions_frame.columnconfigure(0, weight=1)
    lv_classify_button = ttk.Button(lv_actions_frame, text="Capture & Classify", command=capture_and_classify); lv_classify_button.grid(row=0, column=0, sticky="ew", pady=(4, 4))
    lv_calibrate_button = ttk.Button(lv_actions_frame, text="Calibrate Sensors", command=calibrate_sensors); lv_calibrate_button.grid(row=1, column=0, sticky="ew", pady=(4, 4))

    # Results View Frame
    results_view_frame = ttk.Frame(main_frame, padding="10 10 10 10")
    results_view_frame.rowconfigure(0, weight=1); results_view_frame.rowconfigure(1, weight=0); results_view_frame.rowconfigure(2, weight=1)
    results_view_frame.columnconfigure(0, weight=1); results_view_frame.columnconfigure(1, weight=0); results_view_frame.columnconfigure(2, weight=1)
    rv_content_frame = ttk.Frame(results_view_frame); rv_content_frame.grid(row=1, column=1, sticky="")
    ttk.Label(rv_content_frame, text="Classification Result", font=title_font).grid(row=0, column=0, columnspan=2, pady=(5, 15))
    placeholder_h = int(RESULT_IMG_DISPLAY_WIDTH * 0.75); placeholder_img_tk = create_placeholder_image(RESULT_IMG_DISPLAY_WIDTH, placeholder_h, '#E0E0E0', "Result")
    rv_image_label = ttk.Label(rv_content_frame, anchor="center", borderwidth=1, relief="sunken")
    if placeholder_img_tk: rv_image_label.config(image=placeholder_img_tk); rv_image_label.img_tk = placeholder_img_tk
    else: rv_image_label.config(text="Image Area", width=30, height=15)
    rv_image_label.grid(row=1, column=0, columnspan=2, pady=(0, 15))
    rv_details_frame = ttk.Frame(rv_content_frame); rv_details_frame.grid(row=2, column=0, columnspan=2, pady=(0, 15)); rv_details_frame.columnconfigure(1, weight=1)
    res_row = 0
    ttk.Label(rv_details_frame, text="Material:", style="ResultTitle.TLabel").grid(row=res_row, column=0, sticky="w", padx=(0,5))
    rv_prediction_label = ttk.Label(rv_details_frame, text="---", style="Prediction.TLabel"); rv_prediction_label.grid(row=res_row, column=1, sticky="ew", padx=5); res_row += 1
    ttk.Label(rv_details_frame, text="Confidence:", style="ResultTitle.TLabel").grid(row=res_row, column=0, sticky="w", padx=(0,5), pady=(3,0))
    rv_confidence_label = ttk.Label(rv_details_frame, text="---", style="ResultValue.TLabel"); rv_confidence_label.grid(row=res_row, column=1, sticky="ew", padx=5, pady=(3,0)); res_row += 1
    ttk.Separator(rv_details_frame, orient='horizontal').grid(row=res_row, column=0, columnspan=2, sticky='ew', pady=8); res_row += 1
    ttk.Label(rv_details_frame, text="Sensor Values Used:", style="ResultTitle.TLabel", font=tkFont.Font(weight='bold')).grid(row=res_row, column=0, columnspan=2, sticky="w", pady=(0,3)); res_row += 1
    ttk.Label(rv_details_frame, text="  Magnetism:", style="ResultTitle.TLabel").grid(row=res_row, column=0, sticky="w", padx=(5,5))
    rv_magnetism_label = ttk.Label(rv_details_frame, text="---", style="ResultValue.TLabel"); rv_magnetism_label.grid(row=res_row, column=1, sticky="ew", padx=5); res_row += 1
    ttk.Label(rv_details_frame, text="  LDC Reading:", style="ResultTitle.TLabel").grid(row=res_row, column=0, sticky="w", padx=(5,5)) # Changed label
    rv_ldc_label = ttk.Label(rv_details_frame, text="---", style="ResultValue.TLabel"); rv_ldc_label.grid(row=res_row, column=1, sticky="ew", padx=5); res_row += 1
    rv_classify_another_button = ttk.Button(rv_content_frame, text="<< Classify Another", command=show_live_view); rv_classify_another_button.grid(row=3, column=0, columnspan=2, pady=(15, 5))

    clear_results_display()
    show_live_view()
    print("GUI setup complete.")


# ==========================
# === Main Execution =======
# ==========================
def run_application():
    """Sets up the GUI, initializes update loops, and runs the main Tkinter event loop."""
    global window, lv_camera_label, lv_magnetism_label, lv_ldc_label, lv_classify_button, interpreter
    global camera, hall_sensor, ldc_initialized

    print("Setting up GUI...")
    try: setup_gui()
    except Exception as e:
        print(f"FATAL ERROR: Failed to set up GUI: {e}")
        try: root=tk.Tk(); root.withdraw(); messagebox.showerror("GUI Setup Error", f"Failed to initialize GUI:\n{e}\n\nCheck console."); root.destroy()
        except Exception: pass
        return

    print("Updating initial GUI status based on hardware/AI checks...")
    if not camera:
        if lv_camera_label: lv_camera_label.configure(text="Camera Failed", image='')
        else: print("Warning: lv_camera_label not available.")
    if not hall_sensor:
        if lv_magnetism_label: lv_magnetism_label.config(text="N/A")
        else: print("Warning: lv_magnetism_label not available.")
    if not ldc_initialized:
        if lv_ldc_label: lv_ldc_label.config(text="N/A")
        else: print("Warning: lv_ldc_label not available.")

    if not interpreter:
        if lv_classify_button: lv_classify_button.config(state=tk.DISABLED, text="Classify (AI Failed)")
        print("AI Initialization Failed - Classification Disabled.")
    if not hall_sensor and not ldc_initialized:
         if lv_calibrate_button: lv_calibrate_button.config(state=tk.DISABLED, text="Calibrate (No Sensors)")
         print("Sensor Calibration Disabled - No sensors available.")

    print("Starting GUI update loops...")
    update_camera_feed()
    update_magnetism()
    update_ldc_reading()

    print("Starting Tkinter main loop... (Press Ctrl+C in console to exit)")
    try:
        window.protocol("WM_DELETE_WINDOW", on_closing)
        window.mainloop()
    except Exception as e: print(f"ERROR: Exception in Tkinter main loop: {e}")
    print("Tkinter main loop finished.")


# ==========================
# === Window Closing =======
# ==========================
def on_closing():
    """Handles window close event (clicking the 'X'), ensuring cleanup."""
    global window
    print("Window close requested by user.")
    if messagebox.askokcancel("Quit", "Do you want to quit the AI Metal Classifier application?"):
        print("Proceeding with shutdown...")
        if window:
            try: window.destroy(); print("Tkinter window destroyed.")
            except tk.TclError: print("Note: Window already destroyed.")
            except Exception as e: print(f"Warning: Error destroying Tkinter window: {e}")
    else: print("Shutdown cancelled by user.")


# ==========================
# === Cleanup Resources ====
# ==========================
def cleanup_resources():
    """Releases hardware resources (Camera, SPI, GPIO) gracefully."""
    print("\n--- Cleaning up resources ---")
    global camera, spi, ldc_initialized, SPI_ENABLED, CS_PIN

    if camera and camera.isOpened():
        try: print("Releasing camera..."); camera.release(); print("Camera released.")
        except Exception as e: print(f"Warning: Error releasing camera: {e}")

    if spi:
        try:
            if ldc_initialized:
                print("Putting LDC1101 to sleep...")
                try:
                    if CS_PIN is not None and 'GPIO' in globals() and GPIO.getmode() is not None:
                         GPIO.output(CS_PIN, GPIO.LOW); spi.xfer2([START_CONFIG_REG & 0x7F, SLEEP_MODE]); GPIO.output(CS_PIN, GPIO.HIGH); time.sleep(0.05)
                         print("LDC sleep command sent.")
                    else: print("Note: CS_PIN/GPIO not available, cannot send sleep command.")
                except Exception as ldc_e: print(f"Note: Error sending sleep command to LDC during cleanup: {ldc_e}")
        except Exception as e: print(f"Note: Error during LDC sleep attempt in cleanup: {e}")
        finally:
            try: print("Closing SPI..."); spi.close(); print("SPI closed.")
            except Exception as e: print(f"Warning: Error closing SPI: {e}")

    if SPI_ENABLED and 'GPIO' in globals():
        try:
            if GPIO.getmode() is not None: print("Cleaning up GPIO..."); GPIO.cleanup(); print("GPIO cleaned up.")
            else: print("Note: GPIO mode not set, skipping cleanup.")
        except RuntimeError as e: print(f"Note: GPIO cleanup runtime error: {e}")
        except Exception as e: print(f"Warning: Error during GPIO cleanup: {e}")

    print("--- Cleanup complete ---")


# ==========================
# === Main Entry Point =====
# ==========================
if __name__ == '__main__':
    print("="*30); print(" Starting AI Metal Classifier (RPi) "); print("="*30)
    hardware_init_attempted = False; ai_init_ok = False

    try:
        hardware_init_attempted = True
        initialize_hardware()
        ai_init_ok = initialize_ai()
        run_application()
    except KeyboardInterrupt:
        print("\nKeyboard interrupt detected. Exiting application.")
        try:
            if window and window.winfo_exists(): window.destroy()
        except Exception: pass
    except Exception as e:
        print("\n" + "="*30); print(f"FATAL ERROR in main execution: {e}"); print("="*30)
        try:
            if window and window.winfo_exists(): messagebox.showerror("Fatal Application Error", f"An unrecoverable error occurred:\n\n{e}\n\nPlease check console.")
        except Exception: pass
        import traceback; traceback.print_exc()
        try:
            if window and window.winfo_exists(): window.destroy()
        except Exception: pass
    finally:
        if hardware_init_attempted: cleanup_resources()
        else: print("Skipping resource cleanup as hardware initialization was not attempted.")
        print("\nApplication finished."); print("="*30)

