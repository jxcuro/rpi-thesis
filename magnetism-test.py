import tkinter as tk
from picamera2 import Picamera2
from PIL import Image, ImageTk
import time
import os
import uuid
import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
from datetime import datetime

# Initialize camera
camera = Picamera2()
camera.configure(camera.create_still_configuration())  # Use still configuration for faster capture
camera.start()

# Initialize I2C and ADS1115
i2c = busio.I2C(board.SCL, board.SDA)
ads = ADS.ADS1115(i2c)

# Use A0 channel for Hall sensor
hall_sensor = AnalogIn(ads, ADS.P0)

# Sensitivity factor for the Hall sensor (example for a typical sensor, adjust based on your sensor)
SENSITIVITY_V_PER_TESLA = 0.0004  # Voltage per Tesla (e.g., 0.0004 V/T for a typical sensor)
SENSITIVITY_V_PER_MILLITESLA = SENSITIVITY_V_PER_TESLA * 1000  # 1 T = 1000 mT

# Adjusted idle voltage based on actual reading (~1.04V when idle and showing -1.66mT)
IDLE_VOLTAGE = 1.704  # Now should show ~0 mT when idle

# Optional: print idle voltage for reference
print(f"[Calibration] Measured idle voltage: {hall_sensor.voltage:.4f} V")


# Function to capture and save the image with magnetism-based filename
def capture_photo():
    capture_button.config(state=tk.DISABLED)
    feedback_label.config(text="Capturing photo...", fg="orange")
    window.update()

    frame = camera.capture_array()
    img = Image.fromarray(frame)
    img = img.resize((640, 480))

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
    print(f"Image saved at {file_path}")
    feedback_label.config(text=f"Photo Captured: {file_name}", fg="green")
    window.after(2000, lambda: capture_button.config(state=tk.NORMAL))


# Create main window
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


# Update camera feed
def update_camera_feed():
    frame = camera.capture_array()
    img = Image.fromarray(frame)
    img = img.resize((640, 480))
    img_tk = ImageTk.PhotoImage(img)
    camera_label.img_tk = img_tk
    camera_label.configure(image=img_tk)
    window.after(60, update_camera_feed)


# Update magnetism label
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
camera.close()
