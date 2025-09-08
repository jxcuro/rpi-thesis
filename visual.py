# CODE 3.0.22 - VISUAL ONLY VERSION
# Description: This version has been modified to use only the camera feed for classification.
#              The magnetic and LDC sensor inputs have been disabled in the AI pipeline.

import tkinter as tk
from tkinter import ttk
from tkinter import font as tkFont
from tkinter import messagebox
import cv2
from PIL import Image, ImageTk, ImageDraw, ImageFont
import time
import os
import numpy as np
import traceback

# --- AI Imports ---
try:
    import tflite_runtime.interpreter as tflite
except ImportError:
    import tensorflow.lite as tflite

# --- Hardware Imports ---
try:
    import RPi.GPIO as GPIO
    import board
    import busio
    import adafruit_ads1x15.ads1115 as ADS
    from adafruit_ads1x15.analog_in import AnalogIn
    IS_RASPBERRY_PI = True
except ImportError:
    IS_RASPBERRY_PI = False
    print("WARNING: Raspberry Pi specific libraries not found. Running in simulation mode.")

# =================================
# === Global Configuration ========
# =================================
# --- GUI Config ---
WINDOW_WIDTH = 1024
WINDOW_HEIGHT = 600
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
FONT_LARGE = ("Helvetica", 24, "bold")
FONT_MEDIUM = ("Helvetica", 16)
FONT_SMALL = ("Helvetica", 12)
UPDATE_INTERVAL_MS = 20 # ~50 FPS for GUI updates

# --- Hardware Config ---
TRIGGER_PIN = 23 # GPIO pin to start classification

# --- AI Model Config ---
# VISUAL-ONLY: Changed model path to the visual-only TFLite file.
TFLITE_MODEL_PATH = 'visual_model.tflite'
CLASS_LABELS = ['Aluminum', 'Copper', 'Others', 'Steel'] # Ensure this order matches your model's output

# --- Global Variables ---
# Hardware & System State
cap = None
window = None
state = "INITIALIZING" # INITIALIZING, READY, CLASSIFYING, PAUSED
ads = None
ldc_channel = None
mag_channel = None

# AI components
interpreter = None

# GUI Elements
camera_label = None
mag_label, ldc_label = None, None
status_label, result_label, confidence_label = None, None, None
classify_button = None

# =================================
# === AI Functions ================
# =================================
def initialize_ai():
    global interpreter
    print("--- Initializing AI ---")
    try:
        print(f"Loading model from: {TFLITE_MODEL_PATH}")
        interpreter = tflite.Interpreter(model_path=TFLITE_MODEL_PATH)
        interpreter.allocate_tensors()
        print("AI Model loaded and interpreter allocated successfully.")
    except Exception as e:
        print(f"FATAL ERROR: Failed to initialize TFLite interpreter: {e}")
        messagebox.showerror("AI Error", f"Could not load model '{TFLITE_MODEL_PATH}'.\n\n{e}")
        if window: window.destroy()
        raise

# VISUAL-ONLY: This function has been completely rewritten for an image-only input.
def preprocess_input(image):
    try:
        # The model expects a 224x224 RGB image.
        img = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        img = cv2.resize(img, (224, 224))
        
        # Normalize the image to [0, 1] range, as done during training.
        img_array = np.array(img, dtype=np.float32) / 255.0
        
        # Add a batch dimension: (224, 224, 3) -> (1, 224, 224, 3)
        img_array = np.expand_dims(img_array, axis=0)
        return img_array
    except Exception as e:
        print(f"Error during image preprocessing: {e}")
        return None

# VISUAL-ONLY: This function is simplified to handle only one input tensor.
def run_inference(processed_image):
    try:
        input_details = interpreter.get_input_details()
        output_details = interpreter.get_output_details()
        
        # Set the image tensor.
        interpreter.set_tensor(input_details[0]['index'], processed_image)
        
        # Run inference.
        interpreter.invoke()
        
        # Get the output tensor.
        output_data = interpreter.get_tensor(output_details[0]['index'])
        return output_data
    except Exception as e:
        print(f"Error during TFLite inference: {e}")
        return None

def postprocess_output(output_data):
    try:
        probabilities = output_data[0]
        confidence = np.max(probabilities)
        predicted_class_index = np.argmax(probabilities)
        predicted_class_label = CLASS_LABELS[predicted_class_index]
        return predicted_class_label, confidence
    except Exception as e:
        print(f"Error during output postprocessing: {e}")
        return "Error", 0.0

# =================================
# === Hardware Functions ==========
# =================================
def initialize_hardware():
    global cap, ads, ldc_channel, mag_channel
    print("--- Initializing Hardware ---")
    
    # Initialize Camera
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        messagebox.showerror("Camera Error", "Cannot open camera. Is it connected?")
        if window: window.destroy()
        raise RuntimeError("Cannot open camera.")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
    print("Camera initialized.")
    
    # Initialize GPIO and Sensors if on Pi
    if IS_RASPBERRY_PI:
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(TRIGGER_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        print(f"GPIO {TRIGGER_PIN} initialized as input.")
        # VISUAL-ONLY: The sensor initialization is no longer needed.
        # It's safe to leave this commented out. If you get errors, you can re-enable it,
        # but the readings won't be used by the AI.
        # i2c = busio.I2C(board.SCL, board.SDA)
        # ads = ADS.ADS1115(i2c)
        # ldc_channel = AnalogIn(ads, ADS.P0)
        # mag_channel = AnalogIn(ads, ADS.P1)
        print("Sensors disabled for visual-only mode.")
    else:
        print("Running in simulation mode. Hardware calls will be skipped.")

# =================================
# === Core Application Logic ======
# =================================
def capture_and_classify(channel=None):
    global state
    if state != "READY":
        print(f"Ignoring trigger: System not ready (State: {state})")
        return
        
    state = "CLASSIFYING"
    print("\n--- Trigger Received: Starting Classification ---")
    
    try:
        # 1. Update GUI
        status_label.config(text="Status: CLASSIFYING...", foreground="orange")
        result_label.config(text="Result: ...")
        confidence_label.config(text="Confidence: ...")
        window.update()
        
        # 2. Capture a fresh image for classification
        ret, frame = cap.read()
        if not ret:
            print("Failed to capture frame for classification.")
            raise RuntimeError("Failed to capture frame.")
        print("Frame captured.")
        
        # 3. Preprocess the input
        # VISUAL-ONLY: Call the new preprocess function that only takes an image.
        processed_image = preprocess_input(frame)
        if processed_image is None: raise RuntimeError("Preprocessing failed.")
        print("Input preprocessed.")

        # 4. Run Inference
        output = run_inference(processed_image)
        if output is None: raise RuntimeError("Inference failed.")
        print("Inference complete.")

        # 5. Postprocess and Display Result
        label, conf = postprocess_output(output)
        print(f"Result: {label}, Confidence: {conf:.2f}")
        
        result_label.config(text=f"Result: {label}")
        confidence_label.config(text=f"Confidence: {conf*100:.1f}%")

    except Exception as e:
        print(f"ERROR during classification: {e}")
        traceback.print_exc()
        status_label.config(text="Status: ERROR", foreground="red")
        result_label.config(text="Result: Failed")
    
    finally:
        # 6. Move to PAUSED state to prevent re-triggering
        state = "PAUSED"
        status_label.config(text="Status: PAUSED", foreground="blue")
        classify_button.config(state=tk.NORMAL) # Re-enable the manual button
        print("--- Classification cycle finished. System is PAUSED. ---")

def rearm_system():
    global state
    state = "READY"
    status_label.config(text="Status: READY", foreground="green")
    result_label.config(text="Result: --")
    confidence_label.config(text="Confidence: --")
    classify_button.config(state=tk.DISABLED)
    print("\n--- System Re-Armed and Ready for next trigger ---")

# =================================
# === GUI Functions ===============
# =================================
def create_gui():
    global window, camera_label, mag_label, ldc_label, status_label, result_label, confidence_label, classify_button
    
    window = tk.Tk()
    window.title("AI Metal Classifier - Visual Only Mode")
    window.geometry(f"{WINDOW_WIDTH}x{WINDOW_HEIGHT}")
    window.configure(bg='black')

    # Main frame
    main_frame = tk.Frame(window, bg='black')
    main_frame.pack(expand=True, fill='both', padx=10, pady=10)

    # Left side (Camera Feed)
    left_frame = tk.Frame(main_frame, bg='black')
    left_frame.pack(side='left', fill='both', expand=True, padx=(0, 10))
    camera_label = tk.Label(left_frame, bg='black')
    camera_label.pack(expand=True)

    # Right side (Data and Controls)
    right_frame = tk.Frame(main_frame, bg='#222', width=350)
    right_frame.pack(side='right', fill='y')
    right_frame.pack_propagate(False)

    # Title
    title = tk.Label(right_frame, text="VISUAL AI", font=FONT_LARGE, bg='#222', fg='cyan')
    title.pack(pady=20)
    
    # Status Display
    status_label = tk.Label(right_frame, text="Status: INITIALIZING", font=FONT_MEDIUM, bg='#222', fg='yellow')
    status_label.pack(pady=10)
    
    # Sensor Readings (Now display N/A)
    # VISUAL-ONLY: Changed text to indicate sensors are not used.
    mag_label = tk.Label(right_frame, text="Magnetism: N/A", font=FONT_MEDIUM, bg='#222', fg='white')
    mag_label.pack(pady=10, anchor='w', padx=20)
    ldc_label = tk.Label(right_frame, text="Conductivity: N/A", font=FONT_MEDIUM, bg='#222', fg='white')
    ldc_label.pack(pady=10, anchor='w', padx=20)

    ttk.Separator(right_frame, orient='horizontal').pack(fill='x', pady=20, padx=20)

    # Classification Result
    result_label = tk.Label(right_frame, text="Result: --", font=FONT_LARGE, bg='#222', fg='white')
    result_label.pack(pady=10)
    confidence_label = tk.Label(right_frame, text="Confidence: --", font=FONT_MEDIUM, bg='#222', fg='white')
    confidence_label.pack(pady=5)

    # Controls
    classify_button = tk.Button(right_frame, text="Classify Another", font=FONT_MEDIUM, command=rearm_system, state=tk.DISABLED, bg='#00529B', fg='white', relief='flat', width=20, height=2)
    classify_button.pack(side='bottom', pady=20)

def update_gui():
    # Update camera feed
    ret, frame = cap.read()
    if ret:
        # Draw a bounding box/reticle for alignment
        h, w, _ = frame.shape
        cx, cy = w // 2, h // 2
        size = 150 # Size of the square
        cv2.rectangle(frame, (cx-size, cy-size), (cx+size, cy+size), (0, 255, 0), 2)
        
        img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img_pil = Image.fromarray(img)
        img_tk = ImageTk.PhotoImage(image=img_pil)
        camera_label.imgtk = img_tk
        camera_label.config(image=img_tk)

    # VISUAL-ONLY: Removed the sensor reading updates from the GUI loop.

    # Schedule next update
    window.after(UPDATE_INTERVAL_MS, update_gui)

def run_application():
    global state
    create_gui()
    
    # Start the GUI update loop
    update_gui()
    
    # Set the initial state
    rearm_system()
    
    # If on Pi, set up the GPIO interrupt for the trigger
    if IS_RASPBERRY_PI:
        GPIO.add_event_detect(TRIGGER_PIN, GPIO.RISING, callback=capture_and_classify, bouncetime=500)
        print(f"GPIO interrupt attached to pin {TRIGGER_PIN}.")
    else:
        # In simulation, we can use the 'Classify Another' button to trigger
        # We need to change its purpose for the first run
        classify_button.config(text="Manual Classify", command=lambda: capture_and_classify(None), state=tk.NORMAL)

    window.mainloop()

def cleanup_resources():
    print("\n--- Cleaning up resources ---")
    if cap and cap.isOpened():
        cap.release()
        print("Camera released.")
    if IS_RASPBERRY_PI:
        try:
            GPIO.cleanup()
            print("GPIO cleaned up.")
        except Exception as e:
            print(f"Warning during GPIO cleanup: {e}")
    else:
        print("Note: RPi.GPIO not available, skipping cleanup.")
    print("--- Cleanup complete ---")

# ==========================
# === Main Entry Point =====
# ==========================
if __name__ == '__main__':
    print("="*30 + "\n Starting AI Metal Classifier (VISUAL ONLY) \n" + "="*30)
    hw_init_attempted = False
    try:
        initialize_hardware()
        hw_init_attempted = True
        initialize_ai()
        run_application()
    except KeyboardInterrupt:
        print("\nKeyboard interrupt detected. Exiting application.")
    except Exception as e:
        print("\n" + "="*30 + f"\nFATAL ERROR in main execution: {e}\n" + "="*30)
        traceback.print_exc()
        if 'window' in globals() and window and window.winfo_exists():
            try:
                messagebox.showerror("Fatal Application Error", f"Unrecoverable error:\n\n{e}\n\nPlease check console.")
            except Exception: pass
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
        print("\nApplication finished.\n" + "="*40)
