import tkinter as tk
from picamera2 import Picamera2
from PIL import Image, ImageTk
import time
import os

# Initialize camera
camera = Picamera2()
camera.configure(camera.create_still_configuration())  # Configure for still images
camera.start()

# Create main window
window = tk.Tk()
window.title("Camera Feed with MobileNetV3 Resolution")

# Create label for displaying the camera feed
camera_label = tk.Label(window)
camera_label.pack()

# Function to update the camera feed in the GUI
def update_camera_feed():
    # Capture image from the camera
    frame = camera.capture_array()  # Capture a frame from the camera

    # Convert the captured frame to a PIL image
    img = Image.fromarray(frame)

    # Resize the image to 224x224 for MobileNetV3
    img = img.resize((224, 224))

    # Convert image to PhotoImage format for displaying in Tkinter
    img_tk = ImageTk.PhotoImage(img)

    # Update the label with the new image
    camera_label.img_tk = img_tk
    camera_label.configure(image=img_tk)

    # Call update_camera_feed again after 100ms to continuously update the feed
    window.after(100, update_camera_feed)

# Function to capture and save the image
def capture_photo():
    # Capture image from the camera
    frame = camera.capture_array()

    # Convert the captured frame to a PIL image
    img = Image.fromarray(frame)

    # Resize the image to 224x224 for MobileNetV3
    img = img.resize((224, 224))

    # Get the path to save the image
    save_path = os.path.expanduser('~') + "/Pictures/Thesis/"
    os.makedirs(save_path, exist_ok=True)  # Ensure the directory exists

    # Define the file path for saving
    file_path = os.path.join(save_path, "captured_image.jpg")

    # Save the image
    img.save(file_path)
    print(f"Image saved at {file_path}")

# Create a button to capture the photo
capture_button = tk.Button(window, text="Capture Photo", command=capture_photo)
capture_button.pack()

# Run the GUI loop
window.mainloop()

# Stop the camera when the GUI is closed
camera.close()
