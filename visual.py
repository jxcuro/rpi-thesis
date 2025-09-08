# CODE 3.0.22 - VISUAL-ONLY VERSION BASED ON USER'S EXACT FILE
# Description: This is a direct modification of patapos (1).py.
#              The GUI is identical.
#              The AI backend has been changed to use a visual-only model.

import tkinter as tk
from tkinter import ttk
from tkinter import font as tkFont
from tkinter import messagebox
import cv2
from PIL import Image, ImageTk
import time
import os
import numpy as np
import traceback
import pickle

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

# =================================
# === Global Configuration ========
# =================================
WINDOW_WIDTH = 1024; WINDOW_HEIGHT = 600; CAMERA_WIDTH = 640; CAMERA_HEIGHT = 480
FONT_LARGE = ("Helvetica", 24, "bold"); FONT_MEDIUM = ("Helvetica", 16); FONT_SMALL = ("Helvetica", 12)
UPDATE_INTERVAL_MS = 20
TRIGGER_PIN = 23
READING_STABILITY_THRESHOLD = 5
READING_STABILITY_DURATION_S = 0.25

# === AI Model Config ===
# MODIFIED: Changed model path and removed scaler path
TFLITE_MODEL_PATH = 'visual_model.tflite'
CLASS_LABELS = ['Aluminum', 'Copper', 'Others', 'Steel']

# === Global Variables ===
cap=None; window=None; state="INITIALIZING"; ads=None; ldc_channel=None; mag_channel=None
interpreter=None; scaler=None; camera_label=None; mag_label=None; ldc_label=None
status_label=None; result_label=None; confidence_label=None; classify_button=None
ldc_baseline = 0; ldc_buffer = []; mag_buffer = []

# =================================
# === AI Functions ================
# =================================
def initialize_ai():
    global interpreter
    print("--- Initializing Visual-Only AI ---")
    try:
        # MODIFIED: Removed scaler loading logic
        print(f"Loading model from: {TFLITE_MODEL_PATH}")
        interpreter = tflite.Interpreter(model_path=TFLITE_MODEL_PATH)
        interpreter.allocate_tensors()
        print("Visual AI Model loaded successfully.")
    except Exception as e:
        messagebox.showerror("AI Error", f"Could not load model '{TFLITE_MODEL_PATH}'.\n\n{e}")
        if window: window.destroy(); raise

def preprocess_input(image):
    """MODIFIED: This entire function is replaced to handle only an image for the visual model."""
    try:
        img_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        img_resized = cv2.resize(img_rgb, (224, 224))
        img_array = np.array(img_resized, dtype=np.float32) / 255.0
        img_expanded = np.expand_dims(img_array, axis=0)
        return img_expanded
    except Exception as e:
        print(f"Error during image preprocessing: {e}"); return None

def run_inference(processed_image):
    """MODIFIED: This entire function is replaced to handle a single-input visual model."""
    try:
        input_details = interpreter.get_input_details()
        output_details = interpreter.get_output_details()
        interpreter.set_tensor(input_details[0]['index'], processed_image)
        interpreter.invoke()
        output_data = interpreter.get_tensor(output_details[0]['index'])
        return output_data
    except Exception as e:
        print(f"Error during TFLite inference: {e}"); return None

def postprocess_output(output_data):
    # UNCHANGED
    try:
        probabilities = output_data[0]
        confidence = np.max(probabilities)
        predicted_class_index = np.argmax(probabilities)
        predicted_class_label = CLASS_LABELS[predicted_class_index]
        return predicted_class_label, confidence
    except Exception as e:
        print(f"Error during output postprocessing: {e}"); return "Error", 0.0

# =================================
# === Hardware Functions ==========
# =================================
def initialize_hardware():
    # UNCHANGED (except adding cleanup for reliability)
    global cap, ads, ldc_channel, mag_channel
    print("--- Initializing Hardware ---")
    cap = cv2.VideoCapture(0)
    if not cap.isOpened(): messagebox.showerror("Camera Error", "Cannot open camera."); window.destroy(); raise RuntimeError("Cannot open camera.")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
    print("Camera initialized.")
    if IS_RASPBERRY_PI:
        try: GPIO.cleanup()
        except: pass
        GPIO.setmode(GPIO.BCM); GPIO.setup(TRIGGER_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        print(f"GPIO {TRIGGER_PIN} initialized as input.")
        i2c = busio.I2C(board.SCL, board.SDA); ads = ADS.ADS1115(i2c)
        ldc_channel = AnalogIn(ads, ADS.P0); mag_channel = AnalogIn(ads, ADS.P1)
        print("Sensors initialized (but readings will be ignored by AI).")
    else: print("Running in simulation mode.")

# =================================
# === Core Application Logic ======
# =================================
def capture_and_classify(channel=None):
    global state
    if state != "READY": return
    state = "CLASSIFYING"
    print("\n--- Trigger Received: Starting Visual Classification ---")
    try:
        status_label.config(text="Status: CLASSIFYING...", foreground="orange")
        result_label.config(text="Result: ...")
        confidence_label.config(text="Confidence: ...")
        window.update()
        
        # MODIFIED: Removed the entire `while True` loop for stabilizing sensor readings.
        # Now, it just captures one frame and proceeds.
        ret, frame = cap.read()
        if not ret: raise RuntimeError("Failed to capture frame.")
        print("Frame captured for classification.")
        
        processed_input = preprocess_input(frame)
        if processed_input is None: raise RuntimeError("Preprocessing failed.")
        
        output = run_inference(processed_input)
        if output is None: raise RuntimeError("Inference failed.")
        
        label, conf = postprocess_output(output)
        
        result_label.config(text=f"Result: {label}")
        confidence_label.config(text=f"Confidence: {conf*100:.1f}%")
        print(f"Result: {label}, Confidence: {conf:.2f}")
    except Exception as e:
        print(f"ERROR during classification: {e}"); traceback.print_exc()
        status_label.config(text="Status: ERROR", foreground="red"); result_label.config(text="Result: Failed")
    finally:
        state = "PAUSED"
        status_label.config(text="Status: PAUSED", foreground="blue")
        classify_button.config(state=tk.NORMAL)
        print("--- Classification cycle finished. System is PAUSED. ---")

def rearm_system():
    # UNCHANGED
    global state
    state = "READY"
    status_label.config(text="Status: READY", foreground="green"); result_label.config(text="Result: --")
    confidence_label.config(text="Confidence: --"); classify_button.config(state=tk.DISABLED)
    print("\n--- System Re-Armed and Ready for next trigger ---")

# =================================
# === GUI Functions ===============
# =================================
def create_gui():
    # UNCHANGED
    global window, camera_label, mag_label, ldc_label, status_label, result_label, confidence_label, classify_button
    window = tk.Tk(); window.title("AI Metal Classifier"); window.geometry(f"{WINDOW_WIDTH}x{WINDOW_HEIGHT}"); window.configure(bg='black')
    main_frame = tk.Frame(window, bg='black'); main_frame.pack(expand=True, fill='both', padx=10, pady=10)
    left_frame = tk.Frame(main_frame, bg='black'); left_frame.pack(side='left', fill='both', expand=True, padx=(0, 10))
    camera_label = tk.Label(left_frame, bg='black'); camera_label.pack(expand=True)
    right_frame = tk.Frame(main_frame, bg='#222', width=350); right_frame.pack(side='right', fill='y'); right_frame.pack_propagate(False)
    title = tk.Label(right_frame, text="AI CLASSIFIER", font=FONT_LARGE, bg='#222', fg='cyan'); title.pack(pady=20)
    status_label = tk.Label(right_frame, text="Status: INITIALIZING", font=FONT_MEDIUM, bg='#222', fg='yellow'); status_label.pack(pady=10)
    mag_label = tk.Label(right_frame, text="Magnetism (mT):  --", font=FONT_MEDIUM, bg='#222', fg='white'); mag_label.pack(pady=10, anchor='w', padx=20)
    ldc_label = tk.Label(right_frame, text="LDC (Rp):              --", font=FONT_MEDIUM, bg='#222', fg='white'); ldc_label.pack(pady=10, anchor='w', padx=20)
    ttk.Separator(right_frame, orient='horizontal').pack(fill='x', pady=20, padx=20)
    result_label = tk.Label(right_frame, text="Result: --", font=FONT_LARGE, bg='#222', fg='white'); result_label.pack(pady=10)
    confidence_label = tk.Label(right_frame, text="Confidence: --", font=FONT_MEDIUM, bg='#222', fg='white'); confidence_label.pack(pady=5)
    classify_button = tk.Button(right_frame, text="Classify Another", font=FONT_MEDIUM, command=rearm_system, state=tk.DISABLED, bg='#00529B', fg='white', relief='flat', width=20, height=2)
    classify_button.pack(side='bottom', pady=20)

def update_gui():
    # UNCHANGED
    global ldc_baseline
    ret, frame = cap.read()
    if ret:
        h, w, _ = frame.shape; cx, cy = w // 2, h // 2; size = 150
        cv2.rectangle(frame, (cx-size, cy-size), (cx+size, cy+size), (0, 255, 0), 2)
        img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB); img_pil = Image.fromarray(img); img_tk = ImageTk.PhotoImage(image=img_pil)
        camera_label.imgtk = img_tk; camera_label.config(image=img_tk)
    if IS_RASPBERRY_PI and state == "READY":
        # MODIFIED: Commented out the lines that update the sensor text. The hardware still runs, but the GUI text won't change.
        # ldc_val = ldc_channel.value; mag_val = mag_channel.value
        # ldc_label.config(text=f"LDC (Rp):              {ldc_val:5.0f}")
        # mag_label.config(text=f"Magnetism (mT):  {mag_val:.4f}")
        pass
    window.after(UPDATE_INTERVAL_MS, update_gui)

# --- The rest of the file is UNCHANGED ---
def run_application():
    global state; create_gui(); update_gui(); rearm_system()
    if IS_RASPBERRY_PI: GPIO.add_event_detect(TRIGGER_PIN, GPIO.RISING, callback=capture_and_classify, bouncetime=500)
    else: classify_button.config(text="Manual Classify", command=lambda: capture_and_classify(None), state=tk.NORMAL)
    window.mainloop()

def cleanup_resources():
    print("\n--- Cleaning up resources ---")
    if cap and cap.isOpened(): cap.release(); print("Camera released.")
    if IS_RASPBERRY_PI:
        try: GPIO.cleanup(); print("GPIO cleaned up.")
        except Exception as e: print(f"Warning during GPIO cleanup: {e}")
    else: print("Note: RPi.GPIO not available, skipping cleanup.")
    print("--- Cleanup complete ---")

if __name__ == '__main__':
    print("="*30 + "\n Starting AI Metal Classifier (RPi Gated Automation) \n" + "="*30)
    hw_init_attempted = False
    try:
        initialize_hardware(); hw_init_attempted = True; initialize_ai(); run_application()
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
        print("\nApplication finished.\n" + "="*40)
