# CODE 4.0.1 - AI Metal Classifier GUI with Hierarchical Inference
# Description: This version replaces the single AI model with three specialized TFLite models
#              for visual, magnetic, and resistivity data. It uses a hierarchical
#              (or "cascading") inference strategy for efficient and accurate classification.
#
# Hierarchy Logic:
# 1. Magnetic Model: First, check if the material is Steel (highly magnetic).
# 2. Resistivity Model: If not Steel, use resistivity to differentiate Aluminum, Copper, etc.
# 3. Visual Model: Used as a final check or fallback if sensor data is ambiguous.
#
# Version: 4.0.1
# MODIFIED: Inserted the final scaler parameters calculated from the training data.
# KEPT: All other code from version 4.0.0 is unchanged.

import tkinter as tk
from tkinter import ttk
from tkinter import font as tkFont
from tkinter import messagebox
import cv2 # OpenCV for camera access
from PIL import Image, ImageTk, ImageDraw, ImageFont
import time
import os
import statistics
from collections import deque
import numpy as np
import math
import warnings
import traceback

# --- AI Imports ---
try:
    # Preferred import for performance
    from tflite_runtime.interpreter import Interpreter
    print("Successfully imported tflite_runtime.interpreter")
except ImportError:
    # Fallback for standard TensorFlow installation
    print("tflite_runtime not found. Falling back to tensorflow.lite")
    from tensorflow.lite.python.interpreter import Interpreter

# --- Hardware Imports (with fallback for non-RPi testing) ---
try:
    import RPi.GPIO as GPIO
    import board
    import busio
    import adafruit_ldc1101
    from adafruit_lsm303_accel import LSM303_ACCEL
    from adafruit_lsm303dlh_mag import LSM303DLH_MAG
    print("Hardware drivers imported successfully.")
    IS_RPI = True
except (ImportError, NotImplementedError):
    print("WARNING: RPi.GPIO or other hardware libraries not found. Running in simulation mode.")
    IS_RPI = False

# ==================================
# === Global Config & State Vars ===
# ==================================

# --- System State ---
class AppState:
    WAITING_FOR_TRIGGER = 1 # Armed and waiting for GPIO HIGH signal
    PROCESSING = 2          # Trigger received, currently classifying
    PAUSED = 3              # Classification complete, waiting for user action
    CALIBRATING = 4         # Initial auto-calibration phase

current_state = AppState.CALIBRATING
last_state_change_time = time.time()

# --- AI Model Configuration ---
MODELS_DIR = './' # Assumes models are in the same directory as the script
MODEL_FILES = {
    'visual': os.path.join(MODELS_DIR, 'visual_model.tflite'),
    'magnetic': os.path.join(MODELS_DIR, 'magnetic_model.tflite'),
    'resistivity': os.path.join(MODELS_DIR, 'resistivity_model.tflite')
}
CLASS_NAMES = ['Aluminum', 'Copper', 'Others', 'Steel'] # IMPORTANT: Must match training order

# --- AI Global Variables ---
interpreters = {}
input_details = {}
output_details = {}

# IMPORTANT: SCALER VALUES FROM TRAINING
# These values have been calculated by running the training script on your dataset.
SCALER_PARAMS = {
    'magnetic': {'mean': 0.89751622, 'scale': 1.64319851},
    'resistivity': {'mean': [61284.47187500, -338.52812500], 'scale': [695.56832167, 721.43209181]}
}


# --- Hardware Configuration ---
CAMERA_INDEX = 0
LDC_CALIBRATION_SAMPLES = 100
MAG_STABILITY_SAMPLES = 10
MAG_STABILITY_THRESHOLD = 0.05
LDC_STABILITY_SAMPLES = 10
LDC_STABILITY_THRESHOLD = 50
TRIGGER_PIN = 23 # GPIO pin to start classification
ldc_baseline = 0.0

# --- Hardware Global Variables ---
i2c = None
ldc = None
mag = None
cap = None # OpenCV camera capture object

# --- GUI Elements ---
window = None
canvas = None
font_style_large = None
font_style_small = None
font_style_status = None
style = None
sensor_labels = {}
status_label = None
class_label = None
confidence_label = None
classify_button = None

# =========================================
# =========== AI Core Functions ===========
# =========================================

def initialize_ai():
    """Load all three TFLite models and allocate tensors."""
    global interpreters, input_details, output_details
    print("\n--- Initializing AI Models ---")
    for name, path in MODEL_FILES.items():
        if not os.path.exists(path):
            raise FileNotFoundError(f"Model file not found for '{name}': {path}")
        
        print(f"Loading '{name}' model from {path}...")
        interpreters[name] = Interpreter(model_path=path)
        interpreters[name].allocate_tensors()
        
        input_details[name] = interpreters[name].get_input_details()
        output_details[name] = interpreters[name].get_output_details()
        
        print(f"  - Model '{name}' loaded successfully.")
        print(f"  - Input details: {input_details[name][0]['shape']}, dtype: {input_details[name][0]['dtype'].__name__}")
        print(f"  - Output details: {output_details[name][0]['shape']}, dtype: {output_details[name][0]['dtype'].__name__}")

    print("\nAI Scaler parameters loaded.")
    print("--- AI Initialization Complete ---")

def run_inference(model_name, input_data):
    """A generic inference runner for a given model and input data."""
    interpreter = interpreters[model_name]
    
    # Set the input tensor
    interpreter.set_tensor(input_details[model_name][0]['index'], input_data)
    
    # Run inference
    interpreter.invoke()
    
    # Get the output tensor
    output_data = interpreter.get_tensor(output_details[model_name][0]['index'])
    return output_data[0] # Return the 1D array of probabilities

# ==================================
# === Hardware Control Functions ===
# ==================================

def initialize_hardware():
    """Initialize I2C, sensors, GPIO, and camera."""
    global i2c, ldc, mag, cap
    print("\n--- Initializing Hardware ---")
    if IS_RPI:
        # I2C and Sensors
        print("Initializing I2C bus...")
        i2c = busio.I2C(board.SCL, board.SDA)
        print("Initializing LDC1101 sensor...")
        ldc = adafruit_ldc1101.LDC1101(i2c)
        print("Initializing LSM303DLH_MAG sensor...")
        mag = LSM303DLH_MAG(i2c)

        # GPIO
        print(f"Setting up GPIO, trigger pin: {TRIGGER_PIN}...")
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(TRIGGER_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        print("GPIO setup complete.")
    else:
        print("SIM MODE: Skipping hardware initialization.")

    # Camera
    print(f"Initializing camera at index {CAMERA_INDEX}...")
    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        messagebox.showerror("Camera Error", f"Cannot open camera at index {CAMERA_INDEX}. Check connection.")
        raise ConnectionError("Failed to initialize camera.")
    print("Camera initialized successfully.")
    print("--- Hardware Initialization Complete ---")


def get_stable_sensor_readings():
    """
    Waits for and returns stable readings from both LDC and Magnetometer.
    This is crucial for getting a reliable measurement before classification.
    """
    print("Acquiring stable sensor readings...")
    
    if not IS_RPI:
        print("SIM MODE: Returning dummy sensor values.")
        return 0.0, (61000, -500) # (magnetism, (ldc_rp, delta_rp))

    mag_readings = deque(maxlen=MAG_STABILITY_SAMPLES)
    ldc_readings = deque(maxlen=LDC_STABILITY_SAMPLES)

    # Initial fill
    for _ in range(max(MAG_STABILITY_SAMPLES, LDC_STABILITY_SAMPLES)):
        mag_readings.append(mag.magnetic[2]) # Z-axis
        ldc_readings.append(ldc.rp_data)
        time.sleep(0.02)

    while True:
        mag_readings.append(mag.magnetic[2])
        ldc_readings.append(ldc.rp_data)
        
        mag_std_dev = statistics.stdev(mag_readings)
        ldc_std_dev = statistics.stdev(ldc_readings)

        # Check for stability
        if mag_std_dev < MAG_STABILITY_THRESHOLD and ldc_std_dev < LDC_STABILITY_THRESHOLD:
            stable_mag = statistics.mean(mag_readings)
            stable_ldc = statistics.mean(ldc_readings)
            stable_delta = stable_ldc - ldc_baseline
            print(f"Sensors stable! Mag: {stable_mag:.4f} mT, LDC_RP: {stable_ldc:.0f}, Delta: {stable_delta:.0f}")
            return stable_mag, (stable_ldc, stable_delta)
        
        time.sleep(0.01) # Small delay to prevent busy-waiting


def auto_calibrate_ldc():
    """
    Measures the baseline LDC value when no object is present.
    This is critical for calculating the 'delta_rp' value.
    """
    global ldc_baseline
    if not IS_RPI:
        ldc_baseline = 61623.0 # A typical baseline from the dataset
        print(f"SIM MODE: Set LDC baseline to {ldc_baseline}")
        return
        
    print(f"Auto-calibrating LDC sensor with {LDC_CALIBRATION_SAMPLES} samples...")
    readings = []
    for _ in range(LDC_CALIBRATION_SAMPLES):
        readings.append(ldc.rp_data)
        time.sleep(0.01)
    
    ldc_baseline = statistics.mean(readings)
    print(f"--- LDC Calibration Complete. Baseline set to: {ldc_baseline:.2f} ---")


# ========================================
# === Application Logic and Main Loop ====
# ========================================

def set_state(new_state):
    """Manages state transitions and GUI updates."""
    global current_state, last_state_change_time
    if current_state == new_state:
        return
        
    current_state = new_state
    last_state_change_time = time.time()
    print(f"STATE CHANGE -> { {v: k for k, v in AppState.__dict__.items() if not k.startswith('_')}[new_state] }")
    
    # Update GUI based on state
    if new_state == AppState.WAITING_FOR_TRIGGER:
        update_status_display("ARMED: Waiting for object...", "green")
        classify_button.config(state=tk.DISABLED)
    elif new_state == AppState.PROCESSING:
        update_status_display("PROCESSING: Do not move object...", "orange")
        classify_button.config(state=tk.DISABLED)
    elif new_state == AppState.PAUSED:
        update_status_display("PAUSED: Classification complete.", "blue")
        classify_button.config(state=tk.NORMAL)
    elif new_state == AppState.CALIBRATING:
        update_status_display("CALIBRATING: Remove all objects...", "purple")
        classify_button.config(state=tk.DISABLED)


def rearm_system():
    """Resets the state to wait for a new classification trigger."""
    print("System re-armed by user.")
    update_classification_display("?", 0.0)
    set_state(AppState.CALIBRATING)


def capture_and_classify():
    """
    The core hierarchical classification function.
    This function is triggered by the GPIO pin going HIGH.
    """
    set_state(AppState.PROCESSING)

    # --- Step 1: Get Stable Sensor Readings ---
    try:
        stable_mag, (stable_ldc_rp, stable_delta_rp) = get_stable_sensor_readings()
    except Exception as e:
        messagebox.showerror("Sensor Error", f"Failed to get stable sensor readings: {e}")
        set_state(AppState.PAUSED)
        return

    # --- Step 2: Run Magnetic Model (Check for Steel) ---
    mag_input = np.array([[(stable_mag - SCALER_PARAMS['magnetic']['mean']) / SCALER_PARAMS['magnetic']['scale']]], dtype=np.float32)
    mag_probs = run_inference('magnetic', mag_input)
    mag_pred_idx = np.argmax(mag_probs)
    mag_confidence = mag_probs[mag_pred_idx]
    
    print(f"Magnetic Model -> Probs: {mag_probs}, Pred: {CLASS_NAMES[mag_pred_idx]}, Conf: {mag_confidence:.2f}")

    if CLASS_NAMES[mag_pred_idx] == 'Steel' and mag_confidence > 0.90:
        print("High confidence STEEL detected by magnetic sensor. Concluding early.")
        update_classification_display('Steel', mag_confidence)
        set_state(AppState.PAUSED)
        return

    # --- Step 3: Run Resistivity Model (If not clearly Steel) ---
    res_mean = SCALER_PARAMS['resistivity']['mean']
    res_scale = SCALER_PARAMS['resistivity']['scale']
    res_input_scaled = np.array([[(stable_ldc_rp - res_mean[0]) / res_scale[0], 
                                  (stable_delta_rp - res_mean[1]) / res_scale[1]]], dtype=np.float32)
    res_probs = run_inference('resistivity', res_input_scaled)
    res_pred_idx = np.argmax(res_probs)
    res_confidence = res_probs[res_pred_idx]

    print(f"Resistivity Model -> Probs: {res_probs}, Pred: {CLASS_NAMES[res_pred_idx]}, Conf: {res_confidence:.2f}")

    # --- Step 4: Run Visual Model (Always, for fusion/fallback) ---
    ret, frame = cap.read()
    if not ret:
        messagebox.showwarning("Camera Error", "Could not capture frame for visual classification.")
        # If camera fails, we have to rely on the resistivity model
        update_classification_display(CLASS_NAMES[res_pred_idx], res_confidence)
        set_state(AppState.PAUSED)
        return

    # Preprocess image for MobileNetV2
    image_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    image_resized = cv2.resize(image_rgb, (224, 224))
    image_normalized = image_resized.astype(np.float32) / 255.0
    vis_input = np.expand_dims(image_normalized, axis=0)
    
    vis_probs = run_inference('visual', vis_input)
    vis_pred_idx = np.argmax(vis_probs)
    vis_confidence = vis_probs[vis_pred_idx]

    print(f"Visual Model -> Probs: {vis_probs}, Pred: {CLASS_NAMES[vis_pred_idx]}, Conf: {vis_confidence:.2f}")

    # --- Step 5: Fuse Results & Make Final Decision ---
    # Simple weighted average fusion. Give more weight to the sensor that is
    # generally more reliable for non-ferrous metals.
    
    # Exclude 'Steel' from fusion as the magnetic model already handled it.
    # Create a copy of probabilities and set the 'Steel' probability to 0.
    res_probs_no_steel = np.copy(res_probs)
    vis_probs_no_steel = np.copy(vis_probs)
    try:
        steel_index = CLASS_NAMES.index('Steel')
        res_probs_no_steel[steel_index] = 0
        vis_probs_no_steel[steel_index] = 0
    except ValueError:
        pass # 'Steel' not in class list for some reason, ignore.

    # Weighted average: 70% Resistivity, 30% Visual
    fused_probs = (0.7 * res_probs_no_steel) + (0.3 * vis_probs_no_steel)
    
    final_pred_idx = np.argmax(fused_probs)
    final_confidence = fused_probs[final_pred_idx] / np.sum(fused_probs) # Renormalize confidence
    final_class_name = CLASS_NAMES[final_pred_idx]

    print(f"--- FUSION RESULT ---")
    print(f"Fused Probs: {fused_probs}")
    print(f"Final Prediction: {final_class_name} with confidence {final_confidence:.2f}")

    update_classification_display(final_class_name, final_confidence)
    set_state(AppState.PAUSED)


def update_main_loop():
    """The main Tkinter loop, called repeatedly to update GUI and check state."""
    
    # --- State Machine Logic ---
    if current_state == AppState.CALIBRATING:
        if time.time() - last_state_change_time > 2.0: # Calibrate for 2 seconds
            auto_calibrate_ldc()
            set_state(AppState.WAITING_FOR_TRIGGER)
            
    elif current_state == AppState.WAITING_FOR_TRIGGER:
        if IS_RPI and GPIO.input(TRIGGER_PIN):
            print(f"TRIGGER DETECTED on GPIO {TRIGGER_PIN}!")
            # Use after() to avoid blocking the GUI loop during classification
            window.after(50, capture_and_classify)

    # --- Update Live Camera Feed ---
    ret, frame = cap.read()
    if ret:
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img = Image.fromarray(frame_rgb)
        img_tk = ImageTk.PhotoImage(image=img)
        canvas.create_image(0, 0, anchor=tk.NW, image=img_tk)
        canvas.img_tk = img_tk # Keep a reference!
    
    # --- Update Live Sensor Readings on GUI ---
    if IS_RPI:
        try:
            mag_z = mag.magnetic[2]
            ldc_rp = ldc.rp_data
            delta_rp = ldc_rp - ldc_baseline
            
            sensor_labels["Magnetism"].config(text=f"Magnetism (Z): {mag_z:.4f} mT")
            sensor_labels["LDC RP"].config(text=f"LDC RP: {ldc_rp}")
            sensor_labels["Delta RP"].config(text=f"Delta RP: {delta_rp:.0f}")
        except Exception as e:
            # Handle potential I2C read errors without crashing
            sensor_labels["Magnetism"].config(text="Magnetism (Z): ERROR")
            sensor_labels["LDC RP"].config(text="LDC RP: ERROR")
            sensor_labels["Delta RP"].config(text="Delta RP: ERROR")

    # Schedule the next update
    window.after(30, update_main_loop) # ~33 FPS refresh rate

# ==================================
# ===== GUI Creation Functions =====
# ==================================

def create_main_window():
    """Creates and configures the main Tkinter window and widgets."""
    global window, canvas, font_style_large, font_style_small, font_style_status
    global style, sensor_labels, status_label, class_label, confidence_label, classify_button

    window = tk.Tk()
    window.title("AI Metal Classifier")
    window.geometry("800x480")
    window.configure(bg="#2E2E2E")

    # --- Fonts and Styles ---
    font_style_large = tkFont.Font(family="Helvetica", size=24, weight="bold")
    font_style_small = tkFont.Font(family="Helvetica", size=12)
    font_style_status = tkFont.Font(family="Helvetica", size=14, weight="bold")
    
    style = ttk.Style()
    style.configure("TButton", font=font_style_small, padding=10)
    style.configure("TLabel", background="#2E2E2E", foreground="white", font=font_style_small)

    # --- Layout Frames ---
    main_frame = tk.Frame(window, bg="#2E2E2E")
    main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
    
    left_frame = tk.Frame(main_frame, bg="#2E2E2E")
    left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 10))

    right_frame = tk.Frame(main_frame, bg="#2E2E2E", width=250)
    right_frame.pack(side=tk.RIGHT, fill=tk.Y)
    right_frame.pack_propagate(False)

    # --- Left Frame Widgets (Camera) ---
    canvas = tk.Canvas(left_frame, width=640, height=480, bg="black", highlightthickness=0)
    canvas.pack(fill=tk.BOTH, expand=True)

    # --- Right Frame Widgets (Info and Controls) ---
    info_frame = tk.Frame(right_frame, bg="#3C3C3C", relief=tk.RIDGE, borderwidth=2)
    info_frame.pack(fill=tk.X, pady=(0, 10))
    
    ttk.Label(info_frame, text="Live Sensor Data", font=tkFont.Font(family="Helvetica", size=12, weight="bold"), foreground="#00BFFF").pack(pady=(5,10))
    
    sensor_labels["Magnetism"] = ttk.Label(info_frame, text="Magnetism (Z): ...")
    sensor_labels["Magnetism"].pack(anchor='w', padx=10, pady=2)
    sensor_labels["LDC RP"] = ttk.Label(info_frame, text="LDC RP: ...")
    sensor_labels["LDC RP"].pack(anchor='w', padx=10, pady=2)
    sensor_labels["Delta RP"] = ttk.Label(info_frame, text="Delta RP: ...")
    sensor_labels["Delta RP"].pack(anchor='w', padx=10, pady=5)

    status_frame = tk.Frame(right_frame, bg="#3C3C3C", relief=tk.RIDGE, borderwidth=2)
    status_frame.pack(fill=tk.X, pady=(0, 10))
    status_label = ttk.Label(status_frame, text="INITIALIZING...", font=font_style_status, anchor='center')
    status_label.pack(pady=10, padx=10)

    result_frame = tk.Frame(right_frame, bg="#2E2E2E")
    result_frame.pack(fill=tk.BOTH, expand=True)
    
    class_label = ttk.Label(result_frame, text="?", font=("Helvetica", 60, "bold"), foreground="#FFD700")
    class_label.pack(pady=(10, 0), fill=tk.X)
    
    confidence_label = ttk.Label(result_frame, text="Confidence: -", font=font_style_large, foreground="white")
    confidence_label.pack(pady=(0, 10), fill=tk.X)
    
    classify_button = ttk.Button(right_frame, text="Classify Another", command=rearm_system, state=tk.DISABLED)
    classify_button.pack(side=tk.BOTTOM, fill=tk.X, pady=(10, 0))

def update_status_display(text, color):
    """Updates the status label text and color."""
    status_label.config(text=text, foreground=color)

def update_classification_display(class_name, confidence):
    """Updates the GUI with the final classification result."""
    class_label.config(text=class_name)
    confidence_label.config(text=f"Confidence: {confidence:.1%}")

def run_application():
    """Create the GUI and start the main loop."""
    create_main_window()
    window.after(100, update_main_loop) # Start the loop after a short delay
    window.mainloop()

def cleanup_resources():
    """Release all hardware resources gracefully."""
    print("\n--- Cleaning up resources ---")
    if cap and cap.isOpened():
        print("Releasing camera...")
        cap.release()
    if IS_RPI:
        try:
            print("Cleaning up GPIO...")
            GPIO.cleanup()
        except Exception as e:
            print(f"Warning: Error during GPIO cleanup: {e}")
    else:
        print("Note: RPi.GPIO not available, skipping cleanup.")
    print("--- Cleanup complete ---")

# =========================
# === Main Entry Point ====
# =========================
if __name__ == '__main__':
    print("="*30 + "\n Starting AI Metal Classifier (RPi Gated Automation) \n" + "="*30)
    hw_init_attempted = False
    try:
        initialize_hardware(); hw_init_attempted = True
        initialize_ai()
        run_application()
    except KeyboardInterrupt:
        print("\nKeyboard interrupt detected. Exiting application.")
    except Exception as e:
        print("\n" + "="*30 + f"\nFATAL ERROR in main execution: {e}\n" + "="*30); traceback.print_exc()
        if 'window' in globals() and window and window.winfo_exists():
            try:
                messagebox.showerror("Fatal Application Error", f"Unrecoverable error:\n\n{e}\n\nPlease check console.")
            except Exception:
                pass
    finally:
        if 'window' in globals() and window:
            try:
                if window.winfo_exists():
                    print("Ensuring Tkinter window is destroyed...")
                    window.destroy()
                    print("Tkinter window destroyed.")
            except tk.TclError:
                print("Note: Tkinter window already destroyed.")
            except Exception as e:
                print(f"Warning: Error destroying Tkinter window: {e}")
        if hw_init_attempted:
            cleanup_resources()
        else:
            print("Skipping resource cleanup as hardware init not fully attempted.")
        print("\nApplication finished.\n" + "="*30)

