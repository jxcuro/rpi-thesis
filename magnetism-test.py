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
camera.configure(camera.create_still_configuration())
camera.start()

# Initialize I2C and ADS1115
i2c = busio.I2C(board.SCL, board.SDA)
ads = ADS.ADS1115(i2c)

# Use A0 channel for Hall sensor
hall_sensor = AnalogIn(ads, ADS.P0)

# Sensitivity factor for the Hall sensor
SENSITIVITY_V_PER_TESLA = 0.0004
SENSITIVITY_V_PER_MILLITESLA = SENSITIVITY_V_PER_TESLA * 1000

IDLE_VOLTAGE = 1.7

# Function to capture and save the image with magnetism-based filename
def capture_photo():
    frame = camera.capture_array()  # Get frame from camera
    img = Image.fromarray(frame)
    img = img.resize((640, 480))  # Resize image to match display size

    # Get magnetism value
    voltage = hall_sensor.voltage
    adjusted_voltage = voltage - IDLE_VOLTAGE  # Subtract idle voltage
    magnetism_mT = adjusted_voltage / SENSITIVITY_V_PER_MILLITESLA

    save_path = os.path.expanduser('~') + "/Pictures/Thesis/"
    os.makedirs(save_path, exist_ok=True)

    unique_id = uuid.uuid4().hex[:8]
    file_name = f"mag_{magnetism_mT:.2f}_mT_id_{unique_id}.jpg"
    file_path = os.path.join(save_path, file_name)

    img.save(file_path)
    print(f"Image saved at {file_path}")

    feedback_label.config(text=f"Photo Captured: {file_name}", fg="green")
    window.after(2000, lambda: feedback_label.config(text=""))

# Create main window
window = tk.Tk()
window.title("Camera Feed with Magnetism Measurement")

# Create frame for organizing camera feed and controls
frame = tk.Frame(window)
frame.pack(padx=10, pady=10, fill='both', expand=True)

# Create label for displaying the camera feed
camera_label = tk.Label(frame)
camera_label.grid(row=0, column=0, padx=10, pady=10, rowspan=5)

# Create frame for controls (text and button)
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

# Function to update the camera feed in the GUI
def update_camera_feed():
    frame = camera.capture_array()  # Capture a single frame
    img = Image.fromarray(frame)  # Convert the captured frame to an image
    img_tk = ImageTk.PhotoImage(img)
    
    # Display the image in the Tkinter label
    camera_label.img_tk = img_tk
    camera_label.configure(image=img_tk)
    
    # Update the frame every 20ms for smoother performance
    window.after(20, update_camera_feed)

# Function to update magnetism measurement with scaling and units switching
def update_magnetism():
    voltage = hall_sensor.voltage
    adjusted_voltage = voltage - IDLE_VOLTAGE
    magnetism_mT = adjusted_voltage / SENSITIVITY_V_PER_MILLITESLA

    if abs(magnetism_mT) < 1:
        magnetism_uT = magnetism_mT * 1000  # Convert mT to μT
        magnetism_label.config(text=f"Magnetism: {magnetism_uT:.2f} μT")
    else:
        magnetism_label.config(text=f"Magnetism: {magnetism_mT:.2f} mT")

    window.after(20, update_magnetism)

# Start the camera feed and magnetism measurement updates
update_camera_feed()
update_magnetism()

# Run the GUI loop
window.mainloop()

# Stop the camera when the GUI is closed
camera.close()
