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
import spidev

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
# Convert the reading to milliTesla (mT)
SENSITIVITY_V_PER_MILLITESLA = SENSITIVITY_V_PER_TESLA * 1000  # 1 T = 1000 mT

# Idle voltage (baseline) for your Hall sensor
IDLE_VOLTAGE = 1.7  # Adjust this based on your actual idle voltage reading

# LDC1101 SPI configuration
LDC1101_CS_PIN = 5  # Chip select (GPIO 5)
spi = spidev.SpiDev()
spi.open(0, 0)  # Open SPI bus 0, device 0
spi.max_speed_hz = 50000  # Set SPI speed (adjust as needed)
spi.mode = 0b00  # SPI mode 0 (CPOL=0, CPHA=0)

# LDC1101 registers (refer to the LDC1101 datasheet for more details)
LDC1101_MEASUREMENT_REGISTER = 0x0E  # Register to start measurement
LDC1101_STATUS_REGISTER = 0x0A      # Register for status
LDC1101_CONTROL_REGISTER = 0x00     # Register for control configuration
LDC1101_CHANNELS = [0x01, 0x02]     # Channels for inductance measurement (refer to datasheet)

# Function to initialize LDC1101 for LHR mode (Low-Resolution)
def init_ldc1101_lhr_mode():
    # Configure the LDC1101 for LHR mode (24-bit inductance measurement)
    # Check datasheet to see the exact control register values to enable LHR mode.
    
    # Enable the clock signal on the PWM pin to enable LHR mode
    # Ensure the clock signal is sent to CLKIN pin for LHR mode
    spi.xfer([LDC1101_CONTROL_REGISTER, 0x80])  # Example: writing control register value to enable LHR mode
    
    # Set the device to LHR mode by configuring the appropriate registers
    spi.xfer([0x01, 0x40])  # Example: configure for LHR mode (check datasheet for exact values)
    
    print("LDC1101 initialized in LHR mode.")

# Function to check if the measurement is ready
def check_measurement_ready():
    # Read the status register to check if measurement is ready
    status = spi.xfer([LDC1101_STATUS_REGISTER, 0x00])
    print(f"Status Register: {status}")
    if status[1] & 0x01:  # RDY bit, measurement ready?
        return True
    return False

# Function to read LDC1101 data (inductance) in LHR mode
def read_ldc1101_inductance(channel):
    # Trigger the measurement and wait for completion (waiting until ready)
    if check_measurement_ready():
        result = spi.xfer([channel, 0x00])  # Read inductance value from the channel register

        # Combine MSB and LSB to get the full 24-bit result (use 24-bit register for LHR mode)
        inductance_raw = (result[0] << 16) | (result[1] << 8) | result[2]  # Combine MSB, 8-bit, LSB
        inductance_value = inductance_raw / 1000.0  # Convert to µH (adjust as needed)
        print(f"Inductance Raw Value: {inductance_raw}, Converted Inductance: {inductance_value} µH")
        return inductance_value
    return None

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
magnetism_label.grid(row=1, column=0, padx=10, pady=10)

# Create capture button
capture_button = tk.Button(controls_frame, text="Capture Photo", font=("Helvetica", 14), command=capture_photo)
capture_button.grid(row=2, column=0, padx=10, pady=10)

# Initialize LDC1101 for LHR mode
init_ldc1101_lhr_mode()

# Main loop to display camera feed
while True:
    # Capture the current frame from the camera
    frame = camera.capture_array()

    # Convert the frame to an Image object for display in Tkinter
    img = Image.fromarray(frame)
    img = img.resize((640, 480))  # Resize the image to match display size

    # Update the Tkinter window with the current frame
    photo = ImageTk.PhotoImage(img)
    camera_label.config(image=photo)
    camera_label.image = photo

    # Update the Tkinter window
    window.update_idletasks()
    window.update()

    # Optional: Read inductance and update it in the UI
    inductance_value = read_ldc1101_inductance(LDC1101_CHANNELS[0])
    if inductance_value is not None:
        magnetism_label.config(text=f"Inductance: {inductance_value:.2f} µH")

# Close the window on exit
window.mainloop()
