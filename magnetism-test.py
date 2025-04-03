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

# Sensitivity factor for the Hall sensor (1.75 mV/G)
SENSITIVITY_MV_PER_GAUSS = 1.75  # Sensitivity in mV/G (max)
SENSITIVITY_V_PER_GAUSS = SENSITIVITY_MV_PER_GAUSS / 1000  # Convert mV to V (1 mV = 0.001 V)

# Conversion constants
GAUSS_TO_TESLA = 1e-4  # 1 Gauss = 0.0001 Tesla
TESLA_TO_MILLITESLA = 1000  # 1 Tesla = 1000 milliTesla (mT)

# Create main window
window = tk.Tk()
window.title("Camera Feed with Magnetism Measurement")

# Create label for displaying the camera feed
camera_label = tk.Label(window)
camera_label.pack(padx=10, pady=10)

# Create label to show feedback when a photo is captured
feedback_label = tk.Label(window, text="", fg="green", font=("Helvetica", 14))
feedback_label.pack()

# Create label to display magnetism value in mT
magnetism_label = tk.Label(window, text="Magnetism: 0.00 mT", font=("Helvetica", 14))
magnetism_label.pack()

# Function to update the camera feed in the GUI
def update_camera_feed():
    frame = camera.capture_array()
    img = Image.fromarray(frame)
    img = img.resize((224, 224))
    img_tk = ImageTk.PhotoImage(img)
    camera_label.img_tk = img_tk
    camera_label.configure(image=img_tk)
    window.after(30, update_camera_feed)

# Function to update magnetism measurement with scaling
def update_magnetism():
    # Get the raw voltage from the Hall sensor
    voltage = hall_sensor.voltage

    # Debugging: Print the raw voltage from the Hall sensor
    print(f"Hall Sensor Voltage: {voltage:.4f} V")  # Debug print to check raw voltage

    # Convert voltage to Gauss (G) using the sensitivity factor (1.75 mV/G)
    magnetism_gauss = voltage / SENSITIVITY_V_PER_GAUSS  # Using Gauss for scaling

    # Convert Gauss to millitesla (mT)
    magnetism_mT = magnetism_gauss * GAUSS_TO_TESLA * TESLA_TO_MILLITESLA  # Convert to mT

    # Display the magnetism value in mT
    magnetism_label.config(text=f"Magnetism: {magnetism_mT:.2f} mT")

    # Update every 30ms (same as the camera feed)
    window.after(30, update_magnetism)

# Function to capture and save the image with magnetism-based filename
def capture_photo():
    frame = camera.capture_array()
    img = Image.fromarray(frame)
    img = img.resize((224, 224))

    # Get magnetism value
    voltage = hall_sensor.voltage
    magnetism_gauss = voltage / SENSITIVITY_V_PER_GAUSS  # Convert to Gauss
    magnetism_mT = magnetism_gauss * GAUSS_TO_TESLA * TESLA_TO_MILLITESLA  # Convert to mT

    # Get the path to save the image
    save_path = os.path.expanduser('~') + "/Pictures/Thesis/"
    os.makedirs(save_path, exist_ok=True)

    # Create a filename based on magnetism value and a unique ID
    unique_id = uuid.uuid4().hex[:8]  # Generate short unique ID
    file_name = f"mag_{magnetism_mT:.2f}_mT_id_{unique_id}.jpg"
    file_path = os.path.join(save_path, file_name)

    # Save the image
    img.save(file_path)
    print(f"Image saved at {file_path}")

    # Provide feedback to the user
    feedback_label.config(text=f"Photo Captured: {file_name}", fg="green")
    window.after(2000, lambda: feedback_label.config(text=""))

# Create a larger button to capture the photo
capture_button = tk.Button(window, text="Capture Photo", command=capture_photo, height=3, width=20,
                           font=("Helvetica", 14))
capture_button.pack(pady=10)

# Start the camera feed and magnetism measurement updates
update_camera_feed()
update_magnetism()

# Run the GUI loop
window.mainloop()

# Stop the camera when the GUI is closed
camera.close()
