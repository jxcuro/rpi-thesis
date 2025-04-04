import tkinter as tk
from tkinter import messagebox
from picamera2 import Picamera2, Preview
from PIL import Image, ImageTk
import time
import os
import uuid
import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
from datetime import datetime
from libcamera import controls
import sys

# Initialize the Picamera2
picam2 = Picamera2()
camera_config = picam2.create_preview_configuration()
picam2.configure(camera_config)
picam2.start_preview(Preview.QTGL)
picam2.set_controls({"AfMode": controls.AfModeEnum.Continuous})
picam2.start()

# Initialize I2C and ADS1115
i2c = busio.I2C(board.SCL, board.SDA)
ads = ADS.ADS1115(i2c)

# Use A0 channel for Hall sensor
hall_sensor = AnalogIn(ads, ADS.P0)

# Sensitivity factor for the Hall sensor (example for a typical sensor, adjust based on your sensor)
SENSITIVITY_V_PER_TESLA = 0.0004  # Voltage per Tesla (e.g., 0.0004 V/T for a typical sensor)
SENSITIVITY_V_PER_MILLITESLA = SENSITIVITY_V_PER_TESLA * 1000  # 1 T = 1000 mT

# Idle voltage (baseline) for your Hall sensor
IDLE_VOLTAGE = 1.7  # Adjust this based on your actual idle voltage reading

# Function to capture and save the image with magnetism-based filename
def capture_photo():
    # Capture image
    timeStamp = time.strftime("%Y%m%d-%H%M%S")
    targetPath = f"/home/pi/Desktop/img_{timeStamp}.jpg"
    picam2.capture_file(targetPath)

    # Get magnetism value
    voltage = hall_sensor.voltage
    adjusted_voltage = voltage - IDLE_VOLTAGE  # Subtract idle voltage to get actual value
    magnetism_mT = adjusted_voltage / SENSITIVITY_V_PER_MILLITESLA  # Convert to mT

    # Get the path to save the image
    save_path = os.path.expanduser('~') + "/Pictures/Thesis/"
    os.makedirs(save_path, exist_ok=True)

    # Create a filename based on magnetism value and a unique ID
    unique_id = uuid.uuid4().hex[:8]  # Generate short unique ID
    file_name = f"mag_{magnetism_mT:.2f}_mT_id_{unique_id}.jpg"
    file_path = os.path.join(save_path, file_name)

    # Save the image
    print(f"Image saved at {file_path}")
    feedback_label.config(text=f"Photo Captured: {file_name}", fg="green")
    window.after(2000, lambda: feedback_label.config(text=""))

# Function to update the camera feed in the GUI
def update_camera_feed():
    frame = picam2.capture_array()  # Capture frame from Picamera2
    img = Image.fromarray(frame)
    img = img.resize((640, 480))  # Resize the image to match display size
    img_tk = ImageTk.PhotoImage(img)
    camera_label.img_tk = img_tk
    camera_label.configure(image=img_tk)
    window.after(30, update_camera_feed)

# Function to update magnetism measurement with scaling and units switching
def update_magnetism():
    # Get the raw voltage from the Hall sensor
    voltage = hall_sensor.voltage

    # Subtract the idle voltage (baseline) to get the actual magnetic field
    adjusted_voltage = voltage - IDLE_VOLTAGE

    # Convert adjusted voltage to milliTesla (mT)
    magnetism_mT = adjusted_voltage / SENSITIVITY_V_PER_MILLITESLA  # Using mT for scaling

    # Check if the magnetism is below 1 mT, convert to microTesla (μT) if so
    if abs(magnetism_mT) < 1:
        magnetism_uT = magnetism_mT * 1000  # Convert mT to μT
        magnetism_label.config(text=f"Magnetism: {magnetism_uT:.2f} μT")
    else:
        magnetism_label.config(text=f"Magnetism: {magnetism_mT:.2f} mT")

    # Update every 30ms (same as the camera feed)
    window.after(30, update_magnetism)

# Create main window
window = tk.Tk()
window.title("Camera Feed with Magnetism Measurement")

# Create a frame for organizing camera feed and controls
frame = tk.Frame(window)
frame.pack(padx=10, pady=10, fill='both', expand=True)

# Create label for displaying the camera feed
camera_label = tk.Label(frame)
camera_label.grid(row=0, column=0, padx=10, pady=10, rowspan=5)

# Create a frame for controls (text and button)
controls_frame = tk.Frame(frame)
controls_frame.grid(row=0, column=1, padx=10, pady=10, sticky='n')

# Create label to show feedback when a photo is captured
feedback_label = tk.Label(controls_frame, text="", fg="green", font=("Helvetica", 14))
feedback_label.grid(row=0, column=0)

# Create label to display magnetism value
magnetism_label = tk.Label(controls_frame, text="Magnetism: 0.00 mT", font=("Helvetica", 14))
magnetism_label.grid(row=1, column=0)

# Create a larger button to capture the photo
capture_button = tk.Button(controls_frame, text="Capture Photo", command=capture_photo, height=3, width=20, font=("Helvetica", 14))
capture_button.grid(row=2, column=0, pady=10)

# Start the camera feed and magnetism measurement updates
update_camera_feed()
update_magnetism()

# Function to handle window close and stop the camera preview
def on_closing():
    if messagebox.askokcancel("Quit", "Do you want to quit?"):
        picam2.stop_preview()
        window.destroy()

window.protocol("WM_DELETE_WINDOW", on_closing)

# Run the GUI loop
window.mainloop()

# Stop the camera when the GUI is closed
picam2.stop_preview()
