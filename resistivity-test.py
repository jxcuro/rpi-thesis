import tkinter as tk
from picamera2 import Picamera2
from PIL import Image, ImageTk
import time
import os
import uuid
import board
import busio
import spidev  # Import SPI library for communication with LDC1101
from datetime import datetime

# Initialize camera
camera = Picamera2()
camera.configure(camera.create_still_configuration())  # Use still configuration for faster capture
camera.start()

# Initialize SPI for LDC1101
spi = spidev.SpiDev()
spi.open(0, 0)  # Use SPI bus 0, device 0 (CE0)
spi.max_speed_hz = 500000  # Set the SPI clock speed
spi.mode = 0b00  # Set SPI mode to CPOL = 0, CPHA = 0

# LDC1101 Registers (using datasheet values for SPI communication)
LDC1101_MODE_REGISTER = 0x00
LDC1101_DATA_REGISTER = 0x01
LDC1101_STATUS_REGISTER = 0x02
LDC1101_INTERRUPT_MASK_REGISTER = 0x03

LDC1101_LHR_MODE = 0x01  # Set LHR mode

# Read data from LDC1101 (16-bit data)
def read_ldc1101_register(register):
    # Send register address with a read command (0x80) to the device
    response = spi.xfer2([register | 0x80, 0x00])  # 8-bit address + dummy byte to read
    # Read the 2-byte response from the LDC1101
    data = spi.xfer2([0x00, 0x00])  # Send dummy bytes to receive data
    return (data[0] << 8) | data[1]  # Combine high and low bytes

# Set LDC1101 to LHR mode
def set_ldc1101_to_LHR():
    # Write the LHR mode configuration to the mode register
    spi.xfer2([LDC1101_MODE_REGISTER, LDC1101_LHR_MODE])

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
capture_button = tk.Button(controls_frame, text="Capture Photo", command=capture_photo, height=3, width=20,
                           font=("Helvetica", 14))
capture_button.grid(row=2, column=0, pady=10)


# Function to update the camera feed in the GUI
def update_camera_feed():
    frame = camera.capture_array()  # Capture a single frame
    img = Image.fromarray(frame)  # Open the captured image
    img = img.resize((640, 480))  # Resize the image to fit the screen
    img_tk = ImageTk.PhotoImage(img)
    camera_label.img_tk = img_tk
    camera_label.configure(image=img_tk)
    window.after(60, update_camera_feed)  # Update every 60ms for better performance


# Function to update inductance (LHR mode) measurement
def update_inductance():
    # Set LDC1101 to LHR mode
    set_ldc1101_to_LHR()

    # Read inductance data from LDC1101
    inductance = read_ldc1101_register(LDC1101_DATA_REGISTER)

    # Update the GUI with the inductance value
    magnetism_label.config(text=f"Inductance: {inductance} μH")  # Assuming the LDC1101 returns values in μH

    # Update every 100ms for better performance
    window.after(100, update_inductance)


# Start the camera feed and inductance measurement updates
update_camera_feed()
update_inductance()

# Run the GUI loop
window.mainloop()

# Stop the camera when the GUI is closed
camera.close()

# Close the SPI connection when done
spi.close()
