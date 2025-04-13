import tkinter as tk
import cv2
from PIL import Image, ImageTk
import time
import os
import uuid
import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
from datetime import datetime

# Initialize OpenCV camera (0 is usually the default USB webcam)
camera = cv2.VideoCapture(0)

# Initialize I2C and ADS1115
i2c = busio.I2C(board.SCL, board.SDA)
ads = ADS.ADS1115(i2c)

# Use A0 channel for Hall sensor
hall_sensor = AnalogIn(ads, ADS.P0)

# Sensitivity factor for the Hall sensor
SENSITIVITY_V_PER_TESLA = 0.0004
SENSITIVITY_V_PER_MILLITESLA = SENSITIVITY_V_PER_TESLA * 1000
IDLE_VOLTAGE = 1.7256  # Adjust as needed

# Function to capture and save the image with magnetism-based filename
def capture_photo():
    capture_button.config(state=tk.DISABLED)
    feedback_label.config(text="Capturing photo...", fg="orange")
    window.update()

    ret, frame = camera.read()
    if not ret:
        feedback_label.config(text="Failed to capture photo", fg="red")
        return

    # Convert and resize
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    img = Image.fromarray(frame_rgb)
    img = img.resize((640, 480))

    # Get magnetism value
    voltage = hall_sensor.voltage
    adjusted_voltage = voltage - IDLE_VOLTAGE
    magnetism_mT = adjusted_voltage / SENSITIVITY_V_PER_MILLITESLA

    if abs(magnetism_mT) < 1:
        magnetism_uT = magnetism_mT * 1000
        magnetism_value = f"{magnetism_uT:.2f}"
        unit = "uT"
    else:
        magnetism_value = f"{magnetism_mT:.2f}"
        unit = "mT"

    save_path = os.path.expanduser('~') + "/Pictures/Thesis/"
    os.makedirs(save_path, exist_ok=True)

    unique_id = uuid.uuid4().hex[:8]
    file_name = f"mag_{magnetism_value}_{unit}_id_{unique_id}.jpg"
    file_path = os.path.join(save_path, file_name)

    img.save(file_path)
    feedback_label.config(text=f"Photo Captured: {file_name}", fg="green")
    window.after(2000, lambda: capture_button.config(state=tk.NORMAL))


def calibrate_idle_voltage():
    global IDLE_VOLTAGE
    IDLE_VOLTAGE = hall_sensor.voltage
    feedback_label.config(text=f"Calibrated Idle Voltage: {IDLE_VOLTAGE:.4f} V", fg="blue")


# GUI Setup
window = tk.Tk()
window.title("Camera Feed with Magnetism Measurement")

frame = tk.Frame(window)
frame.pack(padx=10, pady=10, fill='both', expand=True)

camera_label = tk.Label(frame)
camera_label.grid(row=0, column=0, padx=10, pady=10, rowspan=5)

controls_frame = tk.Frame(frame)
controls_frame.grid(row=0, column=1, padx=10, pady=10, sticky='n')

feedback_label = tk.Label(controls_frame, text="", fg="green", font=("Helvetica", 14))
feedback_label.grid(row=0, column=0)

magnetism_label = tk.Label(controls_frame, text="Magnetism: 0.00 mT", font=("Helvetica", 14))
magnetism_label.grid(row=1, column=0)

capture_button = tk.Button(controls_frame, text="Capture Photo", command=capture_photo, height=3, width=20,
                           font=("Helvetica", 14))
capture_button.grid(row=2, column=0, pady=10)

calibrate_button = tk.Button(controls_frame, text="Calibrate Idle Voltage", command=calibrate_idle_voltage,
                             height=2, width=20, font=("Helvetica", 12))
calibrate_button.grid(row=3, column=0, pady=5)


def update_camera_feed():
    ret, frame = camera.read()
    if ret:
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img = Image.fromarray(frame_rgb)
        img = img.resize((640, 480))
        img_tk = ImageTk.PhotoImage(img)
        camera_label.img_tk = img_tk
        camera_label.configure(image=img_tk)
    window.after(60, update_camera_feed)


def update_magnetism():
    voltage = hall_sensor.voltage
    adjusted_voltage = voltage - IDLE_VOLTAGE
    magnetism_mT = adjusted_voltage / SENSITIVITY_V_PER_MILLITESLA

    if abs(magnetism_mT) < 1:
        magnetism_uT = magnetism_mT * 1000
        magnetism_label.config(text=f"Magnetism: {magnetism_uT:.2f} μT")
    else:
        magnetism_label.config(text=f"Magnetism: {magnetism_mT:.2f} mT")

    window.after(60, update_magnetism)


update_camera_feed()
update_magnetism()
window.mainloop()

# Release the webcam when the window closes
camera.release()
cv2.destroyAllWindows()
