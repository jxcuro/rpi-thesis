# Combined Code: Camera, Magnetism, Inductance Measurement with Dataset Creation

import tkinter as tk
from tkinter import ttk # For themed widgets
from tkinter import font as tkFont # For font control
import cv2
from PIL import Image, ImageTk
import time
import os
import uuid
import csv # For writing metadata
from datetime import datetime

# --- I2C/ADS1115 Imports ---
import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

# --- SPI/LDC1101 Imports ---
import spidev
import RPi.GPIO as GPIO

# ==================================
# === Constants and Configuration ===
# ==================================

# --- Camera ---
CAMERA_INDEX = 0 # Usually 0 for default USB webcam
IMG_WIDTH = 640
IMG_HEIGHT = 480

# --- Hall Sensor (ADS1115) ---
HALL_ADC_CHANNEL = ADS.P0
SENSITIVITY_V_PER_TESLA = 0.0004
SENSITIVITY_V_PER_MILLITESLA = SENSITIVITY_V_PER_TESLA * 1000
IDLE_VOLTAGE = 1.7256  # Initial guess, updated by calibration

# --- Inductive Sensor (LDC1101) ---
SPI_BUS = 0
SPI_DEVICE = 0
SPI_SPEED = 500000
SPI_MODE = 0b00
CS_PIN = 8
LDC_CHIP_ID = 0xD4

# LDC1101 Register Addresses (abbreviated for brevity, using constants later)
START_CONFIG_REG, RP_SET_REG, TC1_REG, TC2_REG, DIG_CONFIG_REG = 0x0B, 0x01, 0x02, 0x03, 0x04
ALT_CONFIG_REG, D_CONF_REG, INTB_MODE_REG = 0x05, 0x0C, 0x0A
RP_DATA_MSB_REG, RP_DATA_LSB_REG = 0x22, 0x21
CHIP_ID_REG = 0x3F
# (Add other registers if needed)

# LDC1101 Power States
ACTIVE_CONVERSION_MODE, SLEEP_MODE = 0x00, 0x01

# --- Calibration ---
IDLE_RP_VALUE = 0 # Initial guess, updated by calibration

# --- Dataset / File Saving ---
PROJECT_FOLDER_NAME = "Project_Dataset" # Changed name slightly
IMAGES_FOLDER_NAME = "images"
METADATA_FILENAME = "metadata.csv"
METADATA_HEADER = ['image_path', 'magnetism_mT', 'ldc_rp', 'target_material']
# Determine base path (e.g., script directory or home)
# Using script directory for this example
BASE_PATH = os.path.dirname(os.path.abspath(__file__))
PROJECT_PATH = os.path.join(BASE_PATH, PROJECT_FOLDER_NAME)
IMAGES_PATH = os.path.join(PROJECT_PATH, IMAGES_FOLDER_NAME)
METADATA_PATH = os.path.join(PROJECT_PATH, METADATA_FILENAME)

# --- Target Materials ---
TARGET_OPTIONS = [
    "Copper", "Steel", "Iron", "Aluminum", "Zinc",
    "Brass", "Gold", "Silver", "Bronze", "Others"
]

# --- Global Hardware Objects ---
camera = None
i2c = None
ads = None
hall_sensor = None
spi = None
ldc_initialized = False


# =========================
# === Directory Setup =====
# =========================
def setup_project_directory():
    """Creates the main project and images directories if they don't exist."""
    print(f"Ensuring dataset directory exists at: {PROJECT_PATH}")
    try:
        os.makedirs(PROJECT_PATH, exist_ok=True)
        os.makedirs(IMAGES_PATH, exist_ok=True)
        print("Dataset directories checked/created.")
        return True
    except OSError as e:
        print(f"Error creating directories: {e}")
        # Handle error in GUI later if needed
        return False

# =========================
# === Hardware Setup ===
# =========================
def initialize_hardware():
    """Initializes Camera, I2C/ADS1115, SPI/GPIO/LDC1101."""
    global camera, i2c, ads, hall_sensor, spi, ldc_initialized

    print("--- Initializing Hardware ---")
    # --- Initialize Camera ---
    print("Initializing Camera...")
    camera = cv2.VideoCapture(CAMERA_INDEX)
    if not camera.isOpened():
        print(f"Error: Could not open camera at index {CAMERA_INDEX}")
        camera = None # Indicate failure

    # --- Initialize I2C and ADS1115 ---
    print("Initializing I2C and ADS1115...")
    try:
        i2c = busio.I2C(board.SCL, board.SDA)
        ads = ADS.ADS1115(i2c)
        hall_sensor = AnalogIn(ads, HALL_ADC_CHANNEL)
        print("ADS1115 Initialized.")
    except Exception as e:
        print(f"Error initializing I2C/ADS1115: {e}")
        hall_sensor = None

    # --- Initialize SPI, GPIO, and LDC1101 ---
    print("Initializing GPIO, SPI...")
    try:
        GPIO.setwarnings(False) # Disable warnings if re-running script
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(CS_PIN, GPIO.OUT)
        GPIO.output(CS_PIN, GPIO.HIGH)
        print("GPIO Initialized.")

        spi = spidev.SpiDev()
        spi.open(SPI_BUS, SPI_DEVICE)
        spi.max_speed_hz = SPI_SPEED
        spi.mode = SPI_MODE
        print(f"SPI Initialized (Bus={SPI_BUS}, Dev={SPI_DEVICE}, Speed={SPI_SPEED}Hz).")

        # Initialize LDC1101 itself
        if initialize_ldc1101():
            enable_ldc_rpmode()
        else:
             print("LDC1101 Initialization Failed.")
             # GUI should reflect this later
    except Exception as e:
        print(f"Error initializing GPIO/SPI: {e}")
        spi = None
        ldc_initialized = False # Explicitly set false
    print("--- Hardware Initialization Complete ---")

# =========================
# === LDC1101 Functions ===
# =========================
# (Using same functions as before, just ensure they check 'spi' object exists)
def ldc_write_register(reg_addr, value):
    if not spi: return
    try:
        GPIO.output(CS_PIN, GPIO.LOW)
        spi.xfer2([reg_addr & 0x7F, value])
        GPIO.output(CS_PIN, GPIO.HIGH)
    except Exception as e:
        print(f"Error writing LDC 0x{reg_addr:02X}: {e}")

def ldc_read_register(reg_addr):
    if not spi: return 0
    try:
        GPIO.output(CS_PIN, GPIO.LOW)
        result = spi.xfer2([reg_addr | 0x80, 0x00])
        GPIO.output(CS_PIN, GPIO.HIGH)
        return result[1]
    except Exception as e:
        print(f"Error reading LDC 0x{reg_addr:02X}: {e}")
        return 0

def initialize_ldc1101():
    global ldc_initialized
    if not spi: return False
    print("Checking LDC1101 Chip ID...")
    chip_id = ldc_read_register(CHIP_ID_REG)
    print(f"LDC1101 Chip ID Read: 0x{chip_id:02X} (Expected: 0x{LDC_CHIP_ID:02X})")
    if chip_id != LDC_CHIP_ID: return False

    print("Configuring LDC1101 for RP Mode...")
    ldc_write_register(RP_SET_REG, 0x1B)
    ldc_write_register(TC1_REG, 0x80)
    ldc_write_register(TC2_REG, 0x88)
    ldc_write_register(DIG_CONFIG_REG, 0x07)
    ldc_write_register(ALT_CONFIG_REG, 0x02)
    ldc_write_register(D_CONF_REG, 0x00)
    ldc_write_register(INTB_MODE_REG, 0x00)
    ldc_write_register(START_CONFIG_REG, SLEEP_MODE) # Start asleep
    time.sleep(0.05)
    print("LDC1101 Configuration Complete.")
    ldc_initialized = True
    return True

def enable_ldc_powermode(mode):
    if not spi: return
    print(f"Setting LDC1101 Power Mode to: {mode}")
    ldc_write_register(START_CONFIG_REG, mode)
    time.sleep(0.02)

def enable_ldc_rpmode():
    if not spi: return
    print("Enabling LDC1101 RP Mode...")
    ldc_write_register(ALT_CONFIG_REG, 0x02)
    ldc_write_register(D_CONF_REG, 0x00)
    enable_ldc_powermode(ACTIVE_CONVERSION_MODE)
    print("LDC1101 RP Mode Active.")

def get_ldc_rpdata():
    if not spi or not ldc_initialized: return None # Return None if unavailable/error
    try:
        msb = ldc_read_register(RP_DATA_MSB_REG)
        lsb = ldc_read_register(RP_DATA_LSB_REG)
        value = (msb << 8) | lsb
        return value
    except Exception as e:
        print(f"Error in get_ldc_rpdata: {e}")
        return None # Return None on read error

# =========================
# === CSV Handling =========
# =========================
def append_metadata(image_rel_path, mag_mT, rp_value, target):
    """Appends a row of data to the metadata CSV file."""
    file_exists = os.path.isfile(METADATA_PATH)
    try:
        with open(METADATA_PATH, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            if not file_exists:
                writer.writerow(METADATA_HEADER) # Write header if new file
            # Handle potential None values from sensor errors
            mag_mT_str = f"{mag_mT:.4f}" if mag_mT is not None else "N/A"
            rp_value_str = str(rp_value) if rp_value is not None else "N/A"
            writer.writerow([image_rel_path, mag_mT_str, rp_value_str, target])
        return True
    except IOError as e:
        print(f"Error writing to metadata file {METADATA_PATH}: {e}")
        return False
    except Exception as e:
         print(f"An unexpected error occurred during CSV writing: {e}")
         return False


# ======================
# === GUI Functions ===
# ======================

def capture_and_save_data():
    """Captures image, reads sensors, gets target, saves image, appends metadata."""
    global feedback_label # Access feedback label directly

    if not camera:
         feedback_label.config(text="Camera not available", foreground="red")
         return

    # Disable button, show feedback
    capture_button.config(state=tk.DISABLED)
    feedback_label.config(text="Capturing data...", foreground="orange")
    window.update()

    ret, frame = camera.read()
    if not ret:
        feedback_label.config(text="Failed to capture photo", foreground="red")
        capture_button.config(state=tk.NORMAL)
        return

    # --- Process Image ---
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    img = Image.fromarray(frame_rgb)
    # No resize here unless needed for specific model input - save original captured size
    # img_resized = img.resize((IMG_WIDTH, IMG_HEIGHT))

    # --- Generate Unique Filename & Paths ---
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    unique_id = uuid.uuid4().hex[:6]
    image_filename = f"{timestamp}_{unique_id}.jpg"
    image_save_path = os.path.join(IMAGES_PATH, image_filename)
    image_relative_path = os.path.join(IMAGES_FOLDER_NAME, image_filename) # Path for CSV

    # --- Get Sensor Readings ---
    current_mag_mT = None
    if hall_sensor:
        try:
            voltage = hall_sensor.voltage
            adjusted_voltage = voltage - IDLE_VOLTAGE
            current_mag_mT = adjusted_voltage / SENSITIVITY_V_PER_MILLITESLA
        except Exception as e:
            print(f"Error reading Hall Sensor during capture: {e}")
            current_mag_mT = None # Explicitly None on error

    current_rp_val = get_ldc_rpdata() # Already returns None on error

    # --- Get Selected Target ---
    selected_target = target_var.get()

    # --- Save Image ---
    save_success = False
    try:
        img.save(image_save_path)
        print(f"Image saved: {image_save_path}")
        save_success = True
    except Exception as e:
        feedback_label.config(text=f"Error saving image: {e}", foreground="red")
        print(f"Error saving photo to {image_save_path}: {e}")
        capture_button.config(state=tk.NORMAL) # Re-enable button on save error
        return # Don't save metadata if image save failed

    # --- Append Metadata to CSV ---
    meta_success = append_metadata(image_relative_path, current_mag_mT, current_rp_val, selected_target)

    if meta_success:
        feedback_label.config(text=f"Data Added: {image_filename}", foreground="green")
    else:
        feedback_label.config(text="Image saved, metadata FAILED", foreground="red")
        # Consider deleting the saved image if metadata fails? Or flagging it?

    # Re-enable button after a delay
    window.after(1500, lambda: capture_button.config(state=tk.NORMAL))
    window.after(3000, lambda: feedback_label.config(text="")) # Clear feedback


def calibrate_sensors():
    """Calibrates idle voltage for Hall sensor and idle RP value for LDC1101."""
    global IDLE_VOLTAGE, IDLE_RP_VALUE, feedback_label
    feedback_text = ""
    feedback_color = "blue"

    # Calibrate Hall Sensor
    if hall_sensor:
        try:
            voltages = [hall_sensor.voltage for _ in range(5)] # Read multiple times
            time.sleep(0.05)
            voltages.append(hall_sensor.voltage)
            IDLE_VOLTAGE = sum(voltages) / len(voltages) # Average
            feedback_text += f"Hall Idle: {IDLE_VOLTAGE:.4f} V\n"
        except Exception as e:
            feedback_text += f"Hall Cal Error: {e}\n"
            feedback_color = "orange"
    else:
        feedback_text += "Hall Sensor N/A\n"
        feedback_color = "orange"

    # Calibrate LDC1101
    if ldc_initialized:
        try:
            rp_readings = [get_ldc_rpdata() for _ in range(5)] # Read multiple times
            time.sleep(0.05)
            rp_readings.append(get_ldc_rpdata())
            valid_readings = [r for r in rp_readings if r is not None]
            if valid_readings:
                 IDLE_RP_VALUE = sum(valid_readings) / len(valid_readings)
                 feedback_text += f"LDC RP Idle: {IDLE_RP_VALUE:.0f}" # Display as integer
            else:
                 feedback_text += "LDC Cal Error: No valid readings\n"
                 feedback_color = "red" if feedback_color != "orange" else "orange"
        except Exception as e:
            feedback_text += f"LDC Cal Error: {e}"
            feedback_color = "red" if feedback_color != "orange" else "orange"
    else:
        feedback_text += "LDC Sensor N/A"
        feedback_color = "red" if feedback_color != "orange" else "orange"

    feedback_label.config(text=feedback_text.strip(), foreground=feedback_color)
    window.after(4000, lambda: feedback_label.config(text=""))


def update_camera_feed():
    """Updates the camera feed label in the GUI."""
    if camera:
        ret, frame = camera.read()
        if ret:
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(frame_rgb)
            img_resized = img.resize((IMG_WIDTH, IMG_HEIGHT)) # Resize for display
            img_tk = ImageTk.PhotoImage(img_resized)
            camera_label.img_tk = img_tk # Keep reference
            camera_label.configure(image=img_tk)
    else:
        # Configure camera_label to show "Camera N/A" or similar
         if not hasattr(camera_label, 'no_cam_img'):
             # Create a placeholder image/text if first time
             placeholder = Image.new('RGB', (IMG_WIDTH, IMG_HEIGHT), color = 'grey')
             # You could draw text onto the placeholder here
             camera_label.no_cam_img = ImageTk.PhotoImage(placeholder)
         camera_label.configure(image=camera_label.no_cam_img)

    window.after(50, update_camera_feed) # Update approx 20 FPS


def update_magnetism():
    """Updates the magnetism reading label in the GUI."""
    if hall_sensor:
        try:
            voltage = hall_sensor.voltage
            adjusted_voltage = voltage - IDLE_VOLTAGE
            magnetism_mT = adjusted_voltage / SENSITIVITY_V_PER_MILLITESLA

            if abs(magnetism_mT) < 1:
                magnetism_uT = magnetism_mT * 1000
                magnetism_label.config(text=f"Magnetism: {magnetism_uT:.2f} µT")
            else:
                magnetism_label.config(text=f"Magnetism: {magnetism_mT:.2f} mT")
        except Exception as e:
             magnetism_label.config(text="Magnetism: Error")
    else:
        magnetism_label.config(text="Magnetism: N/A")

    window.after(100, update_magnetism)


def update_ldc_reading():
    """Updates the LDC1101 RP value label in the GUI."""
    if ldc_initialized:
        rp_val = get_ldc_rpdata()
        if rp_val is not None:
            # Optionally calculate difference: diff = rp_val - IDLE_RP_VALUE
            ldc_label.config(text=f"LDC RP Value: {rp_val}")
        else:
            ldc_label.config(text="LDC RP Value: Error")
    else:
        ldc_label.config(text="LDC RP Value: N/A")

    window.after(100, update_ldc_reading)


# ======================
# === GUI Setup ========
# ======================
def setup_gui():
    """Sets up the Tkinter GUI elements."""
    global window, camera_label, controls_frame, feedback_label
    global magnetism_label, ldc_label, target_var, capture_button, calibrate_button

    window = tk.Tk()
    window.title("Sensor Data Acquisition Tool")
    # window.geometry("1024x600") # Optional: Set initial size

    # --- Configure Style ---
    style = ttk.Style()
    # Try different themes ('clam', 'alt', 'default', 'classic')
    try:
        style.theme_use('clam')
    except tk.TclError:
        print("Clam theme not available, using default.")

    # Configure font sizes
    default_font = tkFont.nametofont("TkDefaultFont")
    default_font.configure(size=11)
    label_font = tkFont.Font(family="Helvetica", size=12)
    button_font = tkFont.Font(family="Helvetica", size=12, weight="bold")
    feedback_font = tkFont.Font(family="Helvetica", size=10)

    style.configure("TLabel", font=label_font, padding=5)
    style.configure("TButton", font=button_font, padding=10)
    style.configure("TMenubutton", font=label_font, padding=5) # For OptionMenu

    # --- Main Frame ---
    main_frame = ttk.Frame(window, padding="10 10 10 10")
    main_frame.pack(fill=tk.BOTH, expand=True)
    main_frame.columnconfigure(0, weight=3) # Camera feed wider
    main_frame.columnconfigure(1, weight=1) # Controls narrower
    main_frame.rowconfigure(0, weight=1)

    # --- Camera Feed Label (Left) ---
    camera_label = ttk.Label(main_frame, text="Initializing Camera...")
    camera_label.grid(row=0, column=0, padx=(0, 10), pady=5, sticky="nsew")
    # Placeholder aspect ratio (optional, adjust)
    # camera_label.configure(width=IMG_WIDTH // 4, height=IMG_HEIGHT // 4)


    # --- Controls Frame (Right) ---
    controls_frame = ttk.Frame(main_frame, padding="10 10 10 10")
    controls_frame.grid(row=0, column=1, padx=(10, 0), pady=5, sticky="nsew")
    controls_frame.columnconfigure(0, weight=1) # Make column expandable

    # Configure rows for spacing
    row_idx = 0
    controls_frame.rowconfigure(row_idx, weight=0) # Feedback
    controls_frame.rowconfigure(row_idx + 1, weight=0) # Mag Label
    controls_frame.rowconfigure(row_idx + 2, weight=0) # LDC Label
    controls_frame.rowconfigure(row_idx + 3, weight=1) # Spacer
    controls_frame.rowconfigure(row_idx + 4, weight=0) # Target Label
    controls_frame.rowconfigure(row_idx + 5, weight=0) # Target Dropdown
    controls_frame.rowconfigure(row_idx + 6, weight=1) # Spacer
    controls_frame.rowconfigure(row_idx + 7, weight=0) # Capture Button
    controls_frame.rowconfigure(row_idx + 8, weight=0) # Calibrate Button
    controls_frame.rowconfigure(row_idx + 9, weight=1) # Spacer


    # --- Widgets in Controls Frame ---
    feedback_label = ttk.Label(controls_frame, text="", anchor="w", font=feedback_font)
    feedback_label.grid(row=row_idx, column=0, sticky="ew", pady=(0, 10))
    row_idx += 1

    magnetism_label = ttk.Label(controls_frame, text="Magnetism: Initializing...", anchor="w")
    magnetism_label.grid(row=row_idx, column=0, sticky="ew", pady=5)
    row_idx += 1

    ldc_label = ttk.Label(controls_frame, text="LDC RP Value: Initializing...", anchor="w")
    ldc_label.grid(row=row_idx, column=0, sticky="ew", pady=(0, 5))
    row_idx += 1

    # Spacer
    ttk.Frame(controls_frame, height=20).grid(row=row_idx, column=0)
    row_idx += 1

    target_title_label = ttk.Label(controls_frame, text="Target Material:", anchor="w")
    target_title_label.grid(row=row_idx, column=0, sticky="ew", pady=(10, 0))
    row_idx += 1

    target_var = tk.StringVar(window)
    target_var.set(TARGET_OPTIONS[0]) # Set default value
    target_dropdown = ttk.OptionMenu(controls_frame, target_var, TARGET_OPTIONS[0], *TARGET_OPTIONS)
    target_dropdown.grid(row=row_idx, column=0, sticky="ew", pady=(0, 5))
    row_idx += 1

     # Spacer
    ttk.Frame(controls_frame, height=20).grid(row=row_idx, column=0)
    row_idx += 1

    capture_button = ttk.Button(controls_frame, text="Capture & Add Data", command=capture_and_save_data)
    capture_button.grid(row=row_idx, column=0, sticky="ew", pady=10)
    row_idx += 1

    calibrate_button = ttk.Button(controls_frame, text="Calibrate Sensors", command=calibrate_sensors)
    calibrate_button.grid(row=row_idx, column=0, sticky="ew", pady=5)
    row_idx += 1

# ==========================
# === Main Execution =======
# ==========================
def run_application():
    """Sets up GUI, starts loops, runs the main loop."""
    if not setup_project_directory():
        # Handle directory creation failure (e.g., show error message)
        print("Fatal Error: Could not create project directories. Exiting.")
        # Maybe show a tk.messagebox error here before exiting
        return # Don't proceed if directories failed

    setup_gui() # Create the GUI elements

    # Start the update loops
    update_camera_feed()
    update_magnetism()
    update_ldc_reading()

    print("Starting Tkinter main loop...")
    window.mainloop() # Blocks until window is closed

# --- Cleanup ---
def cleanup_resources():
    print("Cleaning up resources...")
    if camera and camera.isOpened():
        print("Releasing camera...")
        camera.release()
    cv2.destroyAllWindows()
    if spi:
        print("Closing SPI...")
        spi.close()
    # GPIO cleanup is important
    try:
         print("Cleaning up GPIO...")
         GPIO.cleanup()
    except Exception as e:
         print(f"Note: Error during GPIO cleanup (maybe already cleaned?): {e}")

    print("Cleanup complete.")

# --- Run with Hardware Init and Cleanup ---
if __name__ == '__main__':
    initialize_hardware() # Initialize sensors *before* GUI setup
    try:
        run_application()
    except Exception as e:
        print(f"An unexpected error occurred in the main application: {e}")
        import traceback
        traceback.print_exc() # Print detailed traceback
    finally:
        cleanup_resources()
