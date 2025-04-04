import tkinter as tk
from picamera2 import Picamera2
from PIL import Image, ImageTk
import time
import os
import uuid
import board
import busio
import smbus2
from adafruit_ads1x15.ads1115 import ADS1115
from adafruit_ads1x15.analog_in import AnalogIn
from datetime import datetime

# Initialize camera
camera = Picamera2()
camera.configure(camera.create_still_configuration())  # Use still configuration for faster capture
camera.start()

# Initialize I2C for both LDC1101 and ADS1115
i2c = smbus2.SMBus(1)  # For LDC1101
ads = ADS1115(i2c)  # Initialize ADS1115 using the same I2C bus

# Create AnalogIn instance for Hall sensor (connected to A0 channel)
hall_sensor = AnalogIn(ads, ADS1115.P0)

# Sensitivity factor for Hall sensor
SENSITIVITY_V_PER_TESLA = 0.0004  # Adjust as needed for your sensor
IDLE_VOLTAGE = 1.7  # Adjust based on your sensor's idle voltage

# Sensitivity factor for LDC1101 (resistivity measurement)
SENSITIVITY_RESISTIVITY = 1  # Adjust based on LDC1101 resistivity calculations

# Function to capture and save the image with magnetism-based filename
def capture_photo():
    # Disable the button to prevent multiple clicks
    capture_button.config(state=tk.DISABLED)

    # Provide instant feedback to the user
    feedback_label.config(text="Capturing photo...", fg="orange")
    window.update()  # Ensure the UI updates immediately

    # Capture the image and process the magnetism
    frame = camera.capture_array()  # Use capture_array to get the frame directly
    img = Image.fromarray(frame)
    img = img.resize((640, 480))  # Resize the image to match display size

    # Get magnetism value
    voltage = hall_sensor.voltage
    adjusted_voltage = voltage - IDLE_VOLTAGE  # Subtract idle voltage to get actual value
    magnetism_mT = adjusted_voltage / (SENSITIVITY_V_PER_TESLA * 1000)  # Convert to mT

    # Determine the correct unit (mT or μT)
    if abs(magnetism_mT) < 1:  # If magnetism is less than 1 mT, use microTesla
        magnetism_uT = magnetism_mT * 1000  # Convert mT to μT
        magnetism_value = f"{magnetism_uT:.2f}"
        unit = "uT"  # Use microTesla (μT)
    else:
        magnetism_value = f"{magnetism_mT:.2f}"
        unit = "mT"  # Use milliTesla (mT)

    # Get the path to save the image
    save_path = os.path.expanduser('~') + "/Pictures/Thesis/"
    os.makedirs(save_path, exist_ok=True)

    # Create a filename based on magnetism value, unit, and a unique ID
    unique_id = uuid.uuid4().hex[:8]  # Generate short unique ID
    file_name = f"mag_{magnetism_value}_{unit}_id_{unique_id}.jpg"
    file_path = os.path.join(save_path, file_name)

    # Save the image
    img.save(file_path)
    print(f"Image saved at {file_path}")

    # Provide feedback to the user once the image is saved
    feedback_label.config(text=f"Photo Captured: {file_name}", fg="green")

    # Re-enable the button after a short delay (e.g., 2 seconds)
    window.after(2000, lambda: capture_button.config(state=tk.NORMAL))


# Function to get magnetism in milliTesla (mT) or microTesla (uT)
def get_magnetism():
    voltage = hall_sensor.voltage  # Directly use the voltage from the hall sensor
    adjusted_voltage = voltage - IDLE_VOLTAGE
    magnetism_mT = adjusted_voltage / (SENSITIVITY_V_PER_TESLA * 1000)  # Convert to milliTesla (mT)
    
    # Display magnetism in appropriate unit
    if abs(magnetism_mT) < 1:
        magnetism_uT = magnetism_mT * 1000  # Convert to microTesla (μT)
        magnetism_label.config(text=f"Magnetism: {magnetism_uT:.2f} μT")
    else:
        magnetism_label.config(text=f"Magnetism: {magnetism_mT:.2f} mT")


# Function to measure resistivity (adjust based on your LDC1101 implementation)
def get_resistivity():
    # LDC1101 specific resistivity reading logic (replace with actual LDC1101 readings)
    resistivity_value = 100  # Example static value; replace with actual reading
    resistivity_label.config(text=f"Resistivity: {resistivity_value:.2f} Ω")
    return resistivity_value


# Function to update camera feed in the GUI
def update_camera_feed():
    frame = camera.capture_array()  # Capture a single frame
    img = Image.fromarray(frame)  # Open the captured image
    img = img.resize((640, 480))  # Resize the image to fit the screen
    img_tk = ImageTk.PhotoImage(img)
    camera_label.img_tk = img_tk
    camera_label.configure(image=img_tk)
    window.after(60, update_camera_feed)  # Update every 60ms for better performance


# Function to update magnetism and resistivity measurements
def update_measurements():
    get_magnetism()
    get_resistivity()
    window.after(60, update_measurements)  # Update every 60ms


# Create main window
window = tk.Tk()
window.title("Camera Feed with Magnetism and Resistivity Measurement")

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

# Create label to display resistivity value
resistivity_label = tk.Label(controls_frame, text="Resistivity: 0.00 Ω", font=("Helvetica", 14))
resistivity_label.grid(row=2, column=0)

# Create a larger button to capture the photo
capture_button = tk.Button(controls_frame, text="Capture Photo", command=capture_photo, height=3, width=20,
                           font=("Helvetica", 14))
capture_button.grid(row=3, column=0, pady=10)


# Start the camera feed, magnetism, and resistivity updates
update_camera_feed()
update_measurements()

# Run the GUI loop
window.mainloop()

# Stop the camera when the GUI is closed
camera.close()
