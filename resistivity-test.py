import tkinter as tk
from picamera2 import Picamera2
from PIL import Image, ImageTk
import time
import os
import uuid
import board
import busio
import spidev  # For SPI communication
from adafruit_ads1x15.analog_in import AnalogIn
from datetime import datetime

# Initialize camera
camera = Picamera2()
camera.configure(camera.create_still_configuration())  # Use still configuration for faster capture
camera.start()

# SPI Setup for LDC1101 (LHR mode for inductance measurement)
spi = spidev.SpiDev()
spi.open(0, 0)  # Use SPI bus 0, device 0 (CE0)
spi.max_speed_hz = 1000000  # 1 MHz clock speed
spi.mode = 0b01  # SPI Mode 1 (CPOL = 0, CPHA = 1)

# LDC1101 register addresses (just example addresses, please refer to your datasheet)
LDC1101_REG_CONFIG = 0x00  # Register for configuration (example)
LDC1101_REG_MEASURE = 0x01  # Register to start measurement (example)
LDC1101_REG_DATA = 0x02  # Register to read the result (example)

# LDC1101 Initialization (start the measurement in LHR mode)
def initialize_ldc1101():
    # Send command to configure LDC1101 into LHR mode
    # Refer to LDC1101 datasheet for exact register settings
    spi.xfer2([LDC1101_REG_CONFIG, 0x00])  # Write to configuration register (adjust for LHR mode)
    time.sleep(0.1)  # Allow time for the sensor to stabilize

# Function to read inductance from LDC1101
def read_inductance():
    # Send a command to trigger the measurement (this depends on the register structure)
    spi.xfer2([LDC1101_REG_MEASURE, 0x01])  # Start a measurement (example)

    # Wait for the measurement to complete (adjust timing based on datasheet)
    time.sleep(0.5)  # Sleep to allow the LDC1101 to finish the measurement (adjust timing)

    # Now read the data (inductance measurement)
    result = spi.xfer2([LDC1101_REG_DATA, 0x00])  # Read the result register
    raw_value = (result[0] << 8) | result[1]  # Combine the bytes into a 16-bit value
    
    # Convert the raw value into an inductance (use scaling factor from datasheet)
    inductance = raw_value * 0.1  # This scaling factor will vary depending on your setup
    return inductance


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
inductance_label = tk.Label(controls_frame, text="Inductance: 0.00 μH", font=("Helvetica", 14))
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

    # Update every 60ms for better performance
    window.after(60, update_magnetism)


# Function to update inductance measurement
def update_inductance():
    inductance = read_inductance()  # Read inductance from the LDC1101
    inductance_label.config(text=f"Inductance: {inductance:.2f} μH")  # Update the label

    # Update every 500ms (or adjust as necessary for your system's speed)
    window.after(500, update_inductance)


# Initialize LDC1101
initialize_ldc1101()

# Start the camera feed, magnetism, and inductance measurement updates
update_camera_feed()
update_magnetism()
update_inductance()

# Run the GUI loop
window.mainloop()

# Stop the camera when the GUI is closed
camera.close()
