import tkinter as tk
from picamera2 import Picamera2
from PIL import Image, ImageTk
import time
import os
import uuid
import board
import busio
import spidev
from datetime import datetime
from adafruit_ads1x15.analog_in import AnalogIn
from adafruit_ads1x15.ads1115 import ADS
from math import pi

# Initialize camera
camera = Picamera2()
camera.configure(camera.create_still_configuration())  # Use still configuration for faster capture
camera.start()

# Initialize I2C and ADS1115 for Hall sensor
i2c = busio.I2C(board.SCL, board.SDA)
ads = ADS.ADS1115(i2c)

# Use A0 channel for Hall sensor
hall_sensor = AnalogIn(ads, ADS.P0)

# Sensitivity factor for the Hall sensor (example for a typical sensor, adjust based on your sensor)
SENSITIVITY_V_PER_TESLA = 0.0004  # Voltage per Tesla (e.g., 0.0004 V/T for a typical sensor)
# Convert the reading to milliTesla (mT)
SENSITIVITY_V_PER_MILLITESLA = SENSITIVITY_V_PER_TESLA * 1000  # 1 T = 1000 mT

# Idle voltage (baseline) for your Hall sensor
IDLE_VOLTAGE = 1.7  # Adjust this based on your actual idle voltage reading

# Initialize SPI for LDC1101
spi = spidev.SpiDev()
spi.open(0, 0)  # Bus 0, Device 0 (Pin 19, GPIO 10)
spi.max_speed_hz = 50000  # Set the SPI clock speed

# Function to read inductance from LDC1101
def read_inductance():
    # Send a command to the LDC1101 to get the measurement result (this is just an example)
    # You might need to adjust the command depending on your specific LDC1101 setup
    result = spi.xfer2([0x00, 0x00])  # Sending dummy data to read the sensor (Replace with correct command)
    # Assuming LDC1101 sends back 2 bytes of data, combine them:
    raw_value = (result[0] << 8) + result[1]
    # Convert raw value to inductance (you will need the correct formula for your LDC1101)
    inductance = raw_value * 0.01  # Example scaling factor (adjust based on your LDC1101 datasheet)
    return inductance

# Function to update inductance display on GUI
def update_inductance():
    inductance_value = read_inductance()
    inductance_label.config(text=f"Inductance: {inductance_value:.2f} µH")
    window.after(100, update_inductance)  # Update every 100ms for live readings

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
    magnetism_mT = adjusted_voltage / SENSITIVITY_V_PER_MILLITESLA  # Convert to mT

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


# Create main window
window = tk.Tk()
window.title("Camera Feed with Magnetism and Inductance Measurement")

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

# Create label to display inductance value
inductance_label = tk.Label(controls_frame, text="Inductance: 0.00 µH", font=("Helvetica", 14))
inductance_label.grid(row=2, column=0)

# Create a larger button to capture the photo
capture_button = tk.Button(controls_frame, text="Capture Photo", command=capture_photo, height=3, width=20,
                           font=("Helvetica", 14))
capture_button.grid(row=3, column=0, pady=10)


# Function to update the camera feed in the GUI
def update_camera_feed():
    frame = camera.capture_array()  # Capture a single frame
    img = Image.fromarray(frame)  # Open the captured image
    img = img.resize((640, 480))  # Resize the image to fit the screen
    img_tk = ImageTk.PhotoImage(img)
    camera_label.img_tk = img_tk
    camera_label.configure(image=img_tk)
    window.after(60, update_camera_feed)  # Update every 60ms for better performance


# Start the camera feed, magnetism, and inductance measurement updates
update_camera_feed()
update_inductance()

# Run the GUI loop
window.mainloop()

# Stop the camera when the GUI is closed
camera.close()
