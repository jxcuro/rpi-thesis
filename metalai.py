# CODE 3.0.4 - AI Metal Classifier GUI with Results Page
# Description: Displays live sensor data and camera feed.
#              Captures image and sensor readings, classifies metal using a TFLite model,
#              and displays the results on a dedicated page. Includes debug prints.
# Version: 3.0.4 - Corrected SyntaxError in get_averaged_hall_voltage by properly indenting try-except.

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
NUM_SAMPLES_CALIBRATION = 10
GUI_UPDATE_INTERVAL_MS = 100
CAMERA_UPDATE_INTERVAL_MS = 40
LDC_DISPLAY_BUFFER_SIZE = 5
MAGNETISM_FILTER_ALPHA = 0.2

# Camera
CAMERA_INDEX = 0
DISPLAY_IMG_WIDTH = 640
DISPLAY_IMG_HEIGHT = 480
RESULT_IMG_DISPLAY_WIDTH = 280

# --- AI Model Configuration ---
try: BASE_PATH = os.path.dirname(os.path.abspath(__file__))
except NameError: BASE_PATH = os.getcwd()
MODEL_FILENAME = "material_classifier_model.tflite"
LABELS_FILENAME = "material_labels.txt"
SCALER_FILENAME = "numerical_scaler.joblib"
MODEL_PATH = os.path.join(BASE_PATH, MODEL_FILENAME)
LABELS_PATH = os.path.join(BASE_PATH, LABELS_FILENAME)
SCALER_PATH = os.path.join(BASE_PATH, SCALER_FILENAME)

AI_IMG_WIDTH = 224
AI_IMG_HEIGHT = 224

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
main_frame = None
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
def initialize_hardware():
    global camera, i2c, ads, hall_sensor, spi, ldc_initialized
    print("--- Initializing Hardware ---")
    try:
        camera = cv2.VideoCapture(CAMERA_INDEX)
        if camera and not camera.isOpened(): raise ValueError("Could not open camera")
        else: print(f"Camera {CAMERA_INDEX} opened.")
    except Exception as e: print(f"Error opening camera {CAMERA_INDEX}: {e}"); camera = None

    if I2C_ENABLED:
        try:
            i2c = busio.I2C(board.SCL, board.SDA); ads = ADS.ADS1115(i2c)
            hall_sensor = AnalogIn(ads, HALL_ADC_CHANNEL); print("ADS1115 Initialized.")
        except Exception as e: print(f"Error initializing I2C/ADS1115: {e}"); hall_sensor = None
    else: print("Skipping I2C/ADS1115 setup.")

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
    try:
        with open(LABELS_PATH, 'r') as f:
            loaded_labels = [line.strip() for line in f.readlines()]
        if not loaded_labels: raise ValueError("Labels file is empty.")
        print(f"Loaded {len(loaded_labels)} labels: {loaded_labels}")
    except FileNotFoundError: print(f"ERROR: Labels file not found at {LABELS_PATH}"); return False
    except Exception as e: print(f"ERROR: Could not read labels file {LABELS_PATH}: {e}"); return False

    try:
        numerical_scaler = joblib.load(SCALER_PATH)
        print(f"Loaded numerical scaler from {SCALER_PATH}")
        if not hasattr(numerical_scaler, 'transform'): raise TypeError("Loaded scaler missing 'transform'.")
    except FileNotFoundError: print(f"ERROR: Scaler file not found at {SCALER_PATH}"); return False
    except Exception as e: print(f"ERROR: Could not load scaler from {SCALER_PATH}: {e}"); return False

    try:
        with warnings.catch_warnings():
            warnings.filterwarnings("ignore", message="The value of the smallest subnormal.*", category=UserWarning)
            interpreter = Interpreter(model_path=MODEL_PATH)
            interpreter.allocate_tensors()
        input_details = interpreter.get_input_details()
        output_details = interpreter.get_output_details()
        print(f"Loaded TFLite model from {MODEL_PATH}")
        output_shape = output_details[0]['shape']
        if output_shape[-1] != len(loaded_labels): print(f"WARNING: Model output size ({output_shape[-1]}) != labels ({len(loaded_labels)}).")
    except FileNotFoundError: print(f"ERROR: Model file not found at {MODEL_PATH}"); return False
    except Exception as e: print(f"ERROR: Failed to load TFLite model or allocate tensors: {e}"); import traceback; traceback.print_exc(); return False
    print("--- AI Initialization Complete ---")
    return True

# =========================
# === LDC1101 Functions ===
# =========================
# Includes syntax fix from v3.0.3
def ldc_write_register(reg_addr, value):
    """Writes a value to an LDC1101 register via SPI."""
    if not spi: return
    try:
        GPIO.output(CS_PIN, GPIO.LOW)
        spi.xfer2([reg_addr & 0x7F, value]) # Write command MSB=0
        GPIO.output(CS_PIN, GPIO.HIGH)
    except Exception as e:
        # Properly Indented Error Handling
        print(f"Warning: Error during LDC write (Register 0x{reg_addr:02X}), attempting CS HIGH. Error: {e}")
        try:
            GPIO.output(CS_PIN, GPIO.HIGH) # Attempt to ensure CS is high on error
        except Exception as inner_e:
            print(f"Warning: Failed to force CS HIGH after write error. Inner error: {inner_e}")
            pass # Ignore error during cleanup attempt

def ldc_read_register(reg_addr):
    """Reads a value from an LDC1101 register via SPI."""
    if not spi: return 0
    result = [0, 0]
    try:
        GPIO.output(CS_PIN, GPIO.LOW)
        result = spi.xfer2([reg_addr | 0x80, 0x00]) # Read command MSB=1
        GPIO.output(CS_PIN, GPIO.HIGH)
        return result[1]
    except Exception as e:
        # Properly Indented Error Handling
        print(f"Warning: Error during LDC read (Register 0x{reg_addr:02X}), attempting CS HIGH. Error: {e}")
        try:
            GPIO.output(CS_PIN, GPIO.HIGH) # Attempt to ensure CS is high on error
        except Exception as inner_e:
            print(f"Warning: Failed to force CS HIGH after read error. Inner error: {inner_e}")
            pass # Ignore error during cleanup attempt
        return 0 # Return 0 on error after attempting cleanup

def initialize_ldc1101():
    """Initializes and configures the LDC1101 sensor."""
    global ldc_initialized; ldc_initialized = False;
    if not spi: return False
    try:
        chip_id = ldc_read_register(CHIP_ID_REG)
        assert chip_id == LDC_CHIP_ID, f"LDC Mismatch: Read 0x{chip_id:02X}, Expected 0x{LDC_CHIP_ID:02X}"
        print("Configuring LDC1101...")
        ldc_write_register(RP_SET_REG, 0x07); ldc_write_register(TC1_REG, 0x90)
        ldc_write_register(TC2_REG, 0xA0); ldc_write_register(DIG_CONFIG_REG, 0x03)
        ldc_write_register(ALT_CONFIG_REG, 0x00); ldc_write_register(D_CONF_REG, 0x00)
        ldc_write_register(INTB_MODE_REG, 0x00); ldc_write_register(START_CONFIG_REG, SLEEP_MODE)
        time.sleep(0.01); print("LDC1101 Configured."); ldc_initialized = True; return True
    except Exception as e: print(f"LDC Init Exception: {e}"); ldc_initialized = False; return False

def enable_ldc_powermode(mode):
    """Sets the power mode of the LDC1101."""
    if not spi or not ldc_initialized: return
    ldc_write_register(START_CONFIG_REG, mode)
    time.sleep(0.01)

def enable_ldc_rpmode():
    """Enables the LDC1101 RP+L measurement mode."""
    if not spi or not ldc_initialized: print("Cannot enable RP mode"); return
    ldc_write_register(ALT_CONFIG_REG, 0x00)
    ldc_write_register(D_CONF_REG, 0x00)
    enable_ldc_powermode(ACTIVE_CONVERSION_MODE)

def get_ldc_rpdata():
    """Reads the raw RP+L data from the LDC1101."""
    if not spi or not ldc_initialized: return None
    try:
        msb = ldc_read_register(RP_DATA_MSB_REG)
        lsb = ldc_read_register(RP_DATA_LSB_REG)
        # Combine MSB and LSB for 16-bit value
        return (msb << 8) | lsb
    except Exception: # Catch potential SPI errors during read
        return None

# ============================
# === Sensor Reading (Avg) ===
# ============================

# --- CORRECTED Syntax for get_averaged_hall_voltage ---
def get_averaged_hall_voltage(num_samples=NUM_SAMPLES_PER_UPDATE):
    """Reads the Hall sensor voltage multiple times and returns the average."""
    if not hall_sensor:
        return None
    readings = []
    for _ in range(num_samples):
        # --- Start of properly indented block ---
        try:
            # Attempt to read the voltage
            readings.append(hall_sensor.voltage)
        except Exception as e:
            # Handle potential errors during a single read
            # print(f"Warning: Error reading Hall sensor voltage: {e}") # Optional debug
            # If any read fails, abort the averaging and return None immediately
            return None
        # --- End of properly indented block ---

    # If the loop completes without errors, calculate and return the average
    return sum(readings) / len(readings) if readings else None
# ----------------------------------------------------

def get_averaged_rp_data(num_samples=NUM_SAMPLES_PER_UPDATE):
    """Reads the LDC RP data multiple times and returns the average."""
    if not ldc_initialized: return None
    readings = [get_ldc_rpdata() for _ in range(num_samples)]
    valid = [r for r in readings if r is not None] # Filter out None values from failed reads
    return sum(valid) / len(valid) if valid else None

# ==========================
# === AI Processing ========
# ==========================
# (Identical to previous version with debug prints)
def preprocess_input(image_pil, mag_mT, ldc_rp_delta):
    """Prepares image and sensor data for the TFLite model."""
    global numerical_scaler, input_details
    if numerical_scaler is None or input_details is None: print("ERROR: Scaler or model details missing."); return None
    try: img_resized = image_pil.resize((AI_IMG_WIDTH, AI_IMG_HEIGHT), Image.Resampling.LANCZOS); image_np = np.array(img_resized.convert('RGB'), dtype=np.float32) / 255.0; image_input = np.expand_dims(image_np, axis=0)
    except Exception as e: print(f"ERROR: Image preprocessing failed: {e}"); return None
    mag_mT_val=mag_mT if mag_mT is not None else 0.0; ldc_rp_delta_val=ldc_rp_delta if ldc_rp_delta is not None else 0.0; numerical_features=np.array([[mag_mT_val,ldc_rp_delta_val]],dtype=np.float32)
    try:
        with warnings.catch_warnings(): warnings.filterwarnings("ignore",message="X does not have valid feature names.*",category=UserWarning); scaled_numerical_features=numerical_scaler.transform(numerical_features)
    except Exception as e: print(f"ERROR: Scaling failed: {e}"); scaled_numerical_features=np.zeros_like(numerical_features)
    print(f"Debug Preprocess: Image shape: {image_input.shape}, min: {image_input.min():.2f}, max: {image_input.max():.2f}"); print(f"Debug Preprocess: Raw numerical: {numerical_features}"); print(f"Debug Preprocess: Scaled numerical: {scaled_numerical_features}")
    image_input_index,numerical_input_index = -1,-1; num_features_expected = 2
    for detail in input_details:
        shape, index = detail['shape'], detail['index']
        if len(shape)==4 and shape[1:4]==(AI_IMG_HEIGHT,AI_IMG_WIDTH,3): image_input_index=index
        elif len(shape)==2 and shape[1]==num_features_expected: numerical_input_index=index
    if image_input_index==-1 or numerical_input_index==-1: print("Warning: Trying input index fallback.");
        if len(input_details)==2:
            try: i0_s,i1_s=input_details[0]['shape'],input_details[1]['shape']; i0_i,i1_i=input_details[0]['index'],input_details[1]['index']
                if len(i0_s)==4 and len(i1_s)==2: image_input_index,numerical_input_index=i0_i,i1_i; print(f"Fallback: Assumed idx {i0_i}=img,{i1_i}=num.")
                elif len(i0_s)==2 and len(i1_s)==4: numerical_input_index,image_input_index=i0_i,i1_i; print(f"Fallback: Assumed idx {i0_i}=num,{i1_i}=img.")
                else: print("Fallback fail: Dim mismatch."); return None
            except Exception as e: print(f"Fallback fail: {e}"); return None
        else: print("Cannot fallback: Not 2 inputs."); return None
        if image_input_index==-1 or numerical_input_index==-1: print("Fallback ID failed."); return None
    try: img_dtype=next(d['dtype'] for d in input_details if d['index']==image_input_index); num_dtype=next(d['dtype'] for d in input_details if d['index']==numerical_input_index); model_inputs={image_input_index:image_input.astype(img_dtype),numerical_input_index:scaled_numerical_features.astype(num_dtype)}
    except Exception as e: print(f"ERROR: Setting input types failed: {e}"); return None
    return model_inputs

def run_inference(model_inputs):
    """Runs inference using the loaded TFLite model."""
    global interpreter, input_details, output_details
    if interpreter is None or model_inputs is None: print("ERROR: Interpreter or input data missing."); return None
    try:
        for index, data in model_inputs.items(): expected_dtype=next(d['dtype'] for d in input_details if d['index']==index); data=data.astype(expected_dtype) if data.dtype!=expected_dtype else data; interpreter.set_tensor(index,data)
        interpreter.invoke(); output_data=interpreter.get_tensor(output_details[0]['index']); return output_data
    except Exception as e: print(f"ERROR: Failed during model inference: {e}"); return None

def postprocess_output(output_data):
    """Interprets the model's output to get prediction and confidence."""
    global loaded_labels
    if output_data is None or not loaded_labels: return "Error", 0.0
    try:
        probabilities = output_data[0]; print(f"Debug Postprocess: Raw output tensor: {probabilities}") # DEBUG PRINT
        predicted_index = np.argmax(probabilities); predicted_label = loaded_labels[predicted_index]; confidence = float(probabilities[predicted_index])
        return predicted_label, confidence
    except IndexError: print(f"ERROR: Output shape unexpected: {output_data.shape}"); return "Shape Err", 0.0
    except Exception as e: print(f"ERROR: Postprocessing failed: {e}"); return "Post Err", 0.0

# ==============================
# === View Switching Logic ===
# ==============================
# (Identical to previous version)
def show_live_view():
    global live_view_frame, results_view_frame, lv_classify_button;
    if results_view_frame: results_view_frame.pack_forget()
    if live_view_frame: live_view_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
    if lv_classify_button: lv_classify_button.config(state=tk.NORMAL)
def show_results_view():
    global live_view_frame, results_view_frame;
    if live_view_frame: live_view_frame.pack_forget()
    if results_view_frame: results_view_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

# ======================
# === GUI Functions ===
# ======================
# (Identical to previous version)
def clear_results_display():
    global rv_image_label, rv_prediction_label, rv_confidence_label, rv_magnetism_label, rv_ldc_label, placeholder_img_tk;
    if rv_image_label: rv_image_label.img_tk=placeholder_img_tk; rv_image_label.config(image=placeholder_img_tk,text="")
    if rv_prediction_label: rv_prediction_label.config(text="...")
    if rv_confidence_label: rv_confidence_label.config(text="...")
    if rv_magnetism_label: rv_magnetism_label.config(text="...")
    if rv_ldc_label: rv_ldc_label.config(text="...")

def capture_and_classify():
    global lv_classify_button, window, camera, IDLE_VOLTAGE, IDLE_RP_VALUE, rv_image_label, rv_prediction_label, rv_confidence_label, rv_magnetism_label, rv_ldc_label, interpreter;
    if not interpreter or not camera: messagebox.showerror("Error", "AI/Cam not ready."); return
    lv_classify_button.config(state=tk.DISABLED); window.update_idletasks(); print("Capturing image..."); ret,frame=camera.read()
    if not ret: messagebox.showerror("Capture Error", "Failed capture."); lv_classify_button.config(state=tk.NORMAL); return
    img_captured_pil=Image.fromarray(cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)); print("Image captured.")
    print("Reading sensors..."); avg_voltage=get_averaged_hall_voltage(NUM_SAMPLES_CALIBRATION); current_mag_mT,mag_display_text=None,"N/A"
    if avg_voltage is not None: try: idle_v=IDLE_VOLTAGE if IDLE_VOLTAGE!=0 else avg_voltage; current_mag_mT=(avg_voltage-idle_v)/SENSITIVITY_V_PER_MILLITESLA; mag_display_text=f"{current_mag_mT:+.3f} mT" except: mag_display_text="Calc Error"; if IDLE_VOLTAGE==0: mag_display_text+=" (No Cal)"
    current_rp_val_avg=get_averaged_rp_data(NUM_SAMPLES_CALIBRATION); current_rp_val,delta_rp,ldc_display_text=None,None,"N/A"
    if current_rp_val_avg is not None: current_rp_val=int(current_rp_val_avg);
        if IDLE_RP_VALUE!=0: delta_rp=current_rp_val-IDLE_RP_VALUE; ldc_display_text=f"{current_rp_val} (Delta {delta_rp:+,})" # Use "Delta"
        else: ldc_display_text=f"{current_rp_val} (No Cal)"
    else: ldc_display_text="Read Error"
    print(f"Sensor readings: Mag={mag_display_text}, LDC={ldc_display_text}")
    print("Preprocessing data..."); model_inputs=preprocess_input(img_captured_pil,current_mag_mT,delta_rp)
    if model_inputs is None: messagebox.showerror("AI Error", "Preprocess fail."); lv_classify_button.config(state=tk.NORMAL); return
    print("Running inference..."); output_data=run_inference(model_inputs)
    if output_data is None: messagebox.showerror("AI Error", "Inference fail."); lv_classify_button.config(state=tk.NORMAL); return
    predicted_label,confidence=postprocess_output(output_data); print(f"Inference complete: Prediction={predicted_label}, Confidence={confidence:.1%}")
    print("Updating results display...");
    try: w,h=img_captured_pil.size; aspect=h/w; img_h=int(RESULT_IMG_DISPLAY_WIDTH*aspect); img_disp=img_captured_pil.resize((RESULT_IMG_DISPLAY_WIDTH,img_h),Image.Resampling.LANCZOS); img_tk=ImageTk.PhotoImage(img_disp);
        if rv_image_label: rv_image_label.img_tk=img_tk; rv_image_label.config(image=img_tk)
    except Exception as e: print(f"Img display error: {e}"); rv_image_label.config(image='', text="Img Error")
    if rv_prediction_label: rv_prediction_label.config(text=f"{predicted_label}")
    if rv_confidence_label: rv_confidence_label.config(text=f"{confidence:.1%}")
    if rv_magnetism_label: rv_magnetism_label.config(text=mag_display_text)
    if rv_ldc_label: rv_ldc_label.config(text=ldc_display_text); show_results_view()

def calibrate_sensors():
    global IDLE_VOLTAGE, IDLE_RP_VALUE, window, previous_filtered_mag_mT, lv_calibrate_button, lv_classify_button; print("Starting Calibration...");
    if lv_calibrate_button: lv_calibrate_button.config(state=tk.DISABLED); if lv_classify_button: lv_classify_button.config(state=tk.DISABLED); window.update_idletasks(); hall_results,hall_error="Hall N/A",False
    if hall_sensor: avg_v=get_averaged_hall_voltage(NUM_SAMPLES_CALIBRATION); hall_results,hall_error,IDLE_VOLTAGE=(f"Hall Idle: {avg_v:.4f} V",False,avg_v) if avg_v is not None else ("Hall Cal Err",True,0.0)
    ldc_results,ldc_error="LDC N/A",False
    if ldc_initialized: avg_rp=get_averaged_rp_data(NUM_SAMPLES_CALIBRATION); ldc_results,ldc_error,IDLE_RP_VALUE=(f"LDC Idle: {int(avg_rp)}",False,int(avg_rp)) if avg_rp is not None else ("LDC Cal Err",True,0)
    previous_filtered_mag_mT=None; print(f"Calibration Results: {hall_results}, {ldc_results}");
    if lv_calibrate_button: lv_calibrate_button.config(state=tk.NORMAL); if lv_classify_button: lv_classify_button.config(state=tk.NORMAL); final_message=f"{hall_results}\n{ldc_results}"
    if hall_error or ldc_error: messagebox.showwarning("Cal Warning", final_message)
    elif not hall_sensor and not ldc_initialized: messagebox.showerror("Cal Error", "Sensors N/A.")
    else: messagebox.showinfo("Calibration Complete", final_message)

def update_camera_feed():
    global lv_camera_label, window; img_tk=None
    if camera and camera.isOpened(): ret,frame=camera.read();
        if ret: try: img=Image.fromarray(cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)); img.thumbnail((DISPLAY_IMG_WIDTH,DISPLAY_IMG_HEIGHT),Image.Resampling.NEAREST); img_tk=ImageTk.PhotoImage(img) except Exception as e: print(f"Cam frame error: {e}")
    if lv_camera_label:
        if img_tk: lv_camera_label.img_tk=img_tk; lv_camera_label.configure(image=img_tk,text="")
        else:
            if not hasattr(lv_camera_label,'no_cam_img'): try: ph=Image.new('RGB',(DISPLAY_IMG_WIDTH//2,DISPLAY_IMG_HEIGHT//2),'#BDBDBD'); lv_camera_label.no_cam_img=ImageTk.PhotoImage(ph) except: lv_camera_label.no_cam_img=None
            if lv_camera_label.no_cam_img and lv_camera_label.cget("image")!=str(lv_camera_label.no_cam_img): lv_camera_label.configure(image=lv_camera_label.no_cam_img,text="No Cam")
    if window and window.winfo_exists(): window.after(CAMERA_UPDATE_INTERVAL_MS,update_camera_feed)

def update_magnetism():
    global lv_magnetism_label, window, previous_filtered_mag_mT, IDLE_VOLTAGE; avg_voltage=get_averaged_hall_voltage(); display_text="N/A"
    if hall_sensor:
        if avg_voltage is not None:
            try: idle_v=IDLE_VOLTAGE if IDLE_VOLTAGE!=0 else avg_voltage; raw_mag_mT=(avg_voltage-idle_v)/SENSITIVITY_V_PER_MILLITESLA
                if previous_filtered_mag_mT is None: filtered_mag_mT=raw_mag_mT
                else: filtered_mag_mT=(MAGNETISM_FILTER_ALPHA*raw_mag_mT)+((1-MAGNETISM_FILTER_ALPHA)*previous_filtered_mag_mT); previous_filtered_mag_mT=filtered_mag_mT
                unit,value=("mT",filtered_mag_mT) if abs(filtered_mag_mT)>=0.1 else ("µT",filtered_mag_mT*1000); display_text=f"{value:+.2f} {unit}";
                if IDLE_VOLTAGE==0: display_text+=" (No Cal)"
            except: display_text="Error"; previous_filtered_mag_mT=None
        else: display_text="Read Err"; previous_filtered_mag_mT=None
    if lv_magnetism_label: lv_magnetism_label.config(text=display_text)
    if window and window.winfo_exists(): window.after(GUI_UPDATE_INTERVAL_MS,update_magnetism)

def update_ldc_reading():
    global lv_ldc_label, window, RP_DISPLAY_BUFFER, IDLE_RP_VALUE; avg_rp_val=get_averaged_rp_data(); display_rp_text="N/A"
    if ldc_initialized:
        if avg_rp_val is not None: RP_DISPLAY_BUFFER.append(avg_rp_val)
            if RP_DISPLAY_BUFFER: buffer_avg=sum(RP_DISPLAY_BUFFER)/len(RP_DISPLAY_BUFFER); current_rp_int=int(buffer_avg)
                if IDLE_RP_VALUE!=0: delta=current_rp_int-IDLE_RP_VALUE; display_rp_text=f"{current_rp_int} (Delta {delta:+,})" # Use "Delta"
                else: display_rp_text=f"{current_rp_int} (No Cal)"
            else: display_rp_text="..."
        else: RP_DISPLAY_BUFFER.clear(); display_rp_text="Read Err"
    if lv_ldc_label: lv_ldc_label.config(text=display_rp_text)
    if window and window.winfo_exists(): window.after(GUI_UPDATE_INTERVAL_MS,update_ldc_reading)

# ======================
# === GUI Setup ========
# ======================
# (Structure identical to previous version)
def setup_gui():
    global window, main_frame, placeholder_img_tk, live_view_frame, results_view_frame, lv_camera_label, lv_magnetism_label, lv_ldc_label, lv_classify_button, lv_calibrate_button, rv_image_label, rv_prediction_label, rv_confidence_label, rv_magnetism_label, rv_ldc_label, rv_classify_another_button, label_font, readout_font, button_font, title_font, result_title_font, result_value_font
    window=tk.Tk(); window.title("AI Metal Classifier v3.0.4"); window.geometry("1000x650"); style=ttk.Style(); style.theme_use('clam' if 'clam' in style.theme_names() else 'default')
    title_font=tkFont.Font(family="Helvetica",size=16,weight="bold"); label_font=tkFont.Font(family="Helvetica",size=11); readout_font=tkFont.Font(family="Consolas",size=14,weight="bold"); button_font=tkFont.Font(family="Helvetica",size=11,weight="bold"); result_title_font=tkFont.Font(family="Helvetica",size=12,weight="bold"); result_value_font=tkFont.Font(family="Consolas",size=14,weight="bold")
    style.configure("TLabel",font=label_font,padding=2); style.configure("TButton",font=button_font,padding=(10,6)); style.configure("TLabelframe",padding=8); style.configure("TLabelframe.Label",font=tkFont.Font(family="Helvetica",size=12,weight="bold")); style.configure("Readout.TLabel",font=readout_font,padding=(5,1)); style.configure("ResultTitle.TLabel",font=label_font,padding=(5,2)); style.configure("ResultValue.TLabel",font=result_value_font,padding=(5,1),anchor=tk.E)
    main_frame=ttk.Frame(window,padding="5 5 5 5"); main_frame.pack(side=tk.TOP,fill=tk.BOTH,expand=True); main_frame.rowconfigure(0,weight=1); main_frame.columnconfigure(0,weight=1)
    live_view_frame=ttk.Frame(main_frame,padding="5 5 5 5"); live_view_frame.columnconfigure(0,weight=3); live_view_frame.columnconfigure(1,weight=1); live_view_frame.rowconfigure(0,weight=1); lv_camera_label=ttk.Label(live_view_frame,text="Init Cam...",anchor="center",borderwidth=1,relief="sunken"); lv_camera_label.grid(row=0,column=0,padx=(0,10),pady=0,sticky="nsew"); lv_controls_frame=ttk.Frame(live_view_frame); lv_controls_frame.grid(row=0,column=1,sticky="nsew"); lv_controls_frame.columnconfigure(0,weight=1); lv_readings_frame=ttk.Labelframe(lv_controls_frame,text=" Live Readings ",padding="10 5 10 5"); lv_readings_frame.grid(row=0,column=0,sticky="new",pady=(0,10)); lv_readings_frame.columnconfigure(1,weight=1); ttk.Label(lv_readings_frame,text="Magnetism:").grid(row=0,column=0,sticky="w",padx=(0,10)); lv_magnetism_label=ttk.Label(lv_readings_frame,text="Init...",style="Readout.TLabel",anchor="e"); lv_magnetism_label.grid(row=0,column=1,sticky="ew"); ttk.Label(lv_readings_frame,text="LDC (Delta):").grid(row=1,column=0,sticky="w",padx=(0,10),pady=(3,0)); lv_ldc_label=ttk.Label(lv_readings_frame,text="Init...",style="Readout.TLabel",anchor="e"); lv_ldc_label.grid(row=1,column=1,sticky="ew",pady=(3,0)); lv_actions_frame=ttk.Labelframe(lv_controls_frame,text=" Actions ",padding="10 5 10 10"); lv_actions_frame.grid(row=1,column=0,sticky="new",pady=(0,10)); lv_actions_frame.columnconfigure(0,weight=1); lv_classify_button=ttk.Button(lv_actions_frame,text="Capture & Classify",command=capture_and_classify); lv_classify_button.grid(row=0,column=0,sticky="ew",pady=(5,5)); lv_calibrate_button=ttk.Button(lv_actions_frame,text="Calibrate Sensors",command=calibrate_sensors); lv_calibrate_button.grid(row=1,column=0,sticky="ew",pady=(5,5))
    results_view_frame=ttk.Frame(main_frame,padding="10 10 10 10"); results_view_frame.columnconfigure(0,weight=1); rv_content_frame=ttk.Frame(results_view_frame); rv_content_frame.grid(row=0,column=0,sticky="n"); ttk.Label(rv_content_frame,text="Classification Result",font=title_font).grid(row=0,column=0,columnspan=2,pady=(5,15)); try: placeholder_img_tk=ImageTk.PhotoImage(Image.new('RGB',(RESULT_IMG_DISPLAY_WIDTH,int(RESULT_IMG_DISPLAY_WIDTH*0.75)),'#E0E0E0')) except: placeholder_img_tk=None; rv_image_label=ttk.Label(rv_content_frame,anchor="center",borderwidth=1,relief="sunken",image=placeholder_img_tk); rv_image_label.grid(row=1,column=0,columnspan=2,pady=(0,15)); rv_details_frame=ttk.Frame(rv_content_frame); rv_details_frame.grid(row=2,column=0,columnspan=2,pady=(0,15)); rv_details_frame.columnconfigure(1,weight=1); ttk.Label(rv_details_frame,text="Material:",style="ResultTitle.TLabel").grid(row=0,column=0,sticky="w",padx=5); rv_prediction_label=ttk.Label(rv_details_frame,text="...",style="ResultValue.TLabel"); rv_prediction_label.grid(row=0,column=1,sticky="ew",padx=5); ttk.Label(rv_details_frame,text="Confidence:",style="ResultTitle.TLabel").grid(row=1,column=0,sticky="w",padx=5); rv_confidence_label=ttk.Label(rv_details_frame,text="...",style="ResultValue.TLabel"); rv_confidence_label.grid(row=1,column=1,sticky="ew",padx=5); ttk.Separator(rv_details_frame,orient='horizontal').grid(row=2,column=0,columnspan=2,sticky='ew',pady=8); ttk.Label(rv_details_frame,text="Magnetism Used:",style="ResultTitle.TLabel").grid(row=3,column=0,sticky="w",padx=5); rv_magnetism_label=ttk.Label(rv_details_frame,text="...",style="ResultValue.TLabel"); rv_magnetism_label.grid(row=3,column=1,sticky="ew",padx=5); ttk.Label(rv_details_frame,text="LDC (Delta) Used:",style="ResultTitle.TLabel").grid(row=4,column=0,sticky="w",padx=5); rv_ldc_label=ttk.Label(rv_details_frame,text="...",style="ResultValue.TLabel"); rv_ldc_label.grid(row=4,column=1,sticky="ew",padx=5); rv_classify_another_button=ttk.Button(rv_content_frame,text="<< Classify Another",command=show_live_view); rv_classify_another_button.grid(row=3,column=0,columnspan=2,pady=(10,5))
    clear_results_display(); show_live_view()

# ==========================
# === Main Execution =======
# ==========================
# (Identical to previous version)
def run_application():
    global window; setup_gui()
    if not camera: lv_camera_label.configure(text="Camera Failed", image='')
    if not hall_sensor: lv_magnetism_label.config(text="N/A")
    if not ldc_initialized: lv_ldc_label.config(text="N/A")
    if not interpreter:
        if lv_classify_button: lv_classify_button.config(state=tk.DISABLED); messagebox.showwarning("AI Init Failed", "AI components failed.")
    print("Starting update loops..."); update_camera_feed(); update_magnetism(); update_ldc_reading()
    print("Starting Tkinter main loop..."); window.mainloop()

# --- Cleanup ---
# (Identical to previous version)
def cleanup_resources():
    print("Cleaning up resources...")
    if camera and camera.isOpened(): camera.release(); print("Camera released.")
    cv2.destroyAllWindows()
    if spi:
        try:
            if ldc_initialized: print("Putting LDC to sleep..."); ldc_write_register(START_CONFIG_REG, SLEEP_MODE); time.sleep(0.05)
        except Exception as e: print(f"Note: LDC sleep error: {e}")
        finally: spi.close(); print("SPI closed.")
    if SPI_ENABLED: try: GPIO.cleanup(); print("GPIO cleaned up.") except Exception as e: print(f"Note: GPIO cleanup error: {e}")
    print("Cleanup complete.")

# --- Run ---
# (Identical to previous version)
if __name__ == '__main__':
    hardware_ok, ai_ok = False, False
    try: initialize_hardware(); hardware_ok = True; ai_ok = initialize_ai(); run_application()
    except KeyboardInterrupt: print("\nKeyboard interrupt detected. Exiting.")
    except Exception as e: print(f"FATAL ERROR: {e}"); try: messagebox.showerror("Fatal Error", f"Error:\n{e}") except: pass; import traceback; traceback.print_exc()
    finally: if hardware_ok: cleanup_resources()
