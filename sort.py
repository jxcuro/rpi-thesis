import RPi.GPIO as GPIO
import time
import tkinter as tk
from tkinter import ttk

# Define the GPIO pins connected to the Arduino
data_pin_lsb = 16  # Changed to GPIO 16
data_pin_msb = 26  # Changed to GPIO 26

# Set the GPIO mode to BCM numbering
GPIO.setmode(GPIO.BCM)

# Set the GPIO pins as outputs
GPIO.setup(data_pin_lsb, GPIO.OUT)
GPIO.setup(data_pin_msb, GPIO.OUT)

def send_data(data_value):
    """
    Sends data to the Arduino via GPIO pins.

    Args:
        data_value: The data value to send (1, 2, 3, or 4).
    """
    if data_value == 1:
        GPIO.output(data_pin_msb, GPIO.LOW)
        GPIO.output(data_pin_lsb, GPIO.LOW)
        print("Sending: 1 (Aluminum - 00)")
    elif data_value == 2:
        GPIO.output(data_pin_msb, GPIO.LOW)
        GPIO.output(data_pin_lsb, GPIO.HIGH)
        print("Sending: 2 (Copper - 01)")
    elif data_value == 3:
        GPIO.output(data_pin_msb, GPIO.HIGH)
        GPIO.output(data_pin_lsb, GPIO.LOW)
        print("Sending: 3 (Steel - 10)")
    elif data_value == 4:
        GPIO.output(data_pin_msb, GPIO.HIGH)
        GPIO.output(data_pin_lsb, GPIO.HIGH)
        print("Sending: 4 (Others - 11)")
    else:
        print("Invalid data value. Please send 1, 2, 3, or 4.")

def create_gui():
    """
    Creates the main application window with buttons.
    """
    root = tk.Tk()
    root.title("Material Selection")
    root.geometry("300x200")  # Increased size for better button spacing

    # Create and style the buttons
    aluminum_button = ttk.Button(root, text="1 - Aluminum", command=lambda: send_data(1))
    copper_button = ttk.Button(root, text="2 - Copper", command=lambda: send_data(2))
    steel_button = ttk.Button(root, text="3 - Steel", command=lambda: send_data(3))
    others_button = ttk.Button(root, text="4 - Others", command=lambda: send_data(4))

    # Use grid layout for better control over button placement and spacing
    aluminum_button.grid(row=0, column=0, padx=20, pady=20, sticky="ew")
    copper_button.grid(row=0, column=1, padx=20, pady=20, sticky="ew")
    steel_button.grid(row=1, column=0, padx=20, pady=20, sticky="ew")
    others_button.grid(row=1, column=1, padx=20, pady=20, sticky="ew")

    # Configure column weights to make buttons expand horizontally
    root.grid_columnconfigure(0, weight=1)
    root.grid_columnconfigure(1, weight=1)
    
    # Ensure proper cleanup on window close
    root.protocol("WM_DELETE_WINDOW", on_closing)

    root.mainloop()
    
def on_closing():
    """
    This function is called when the window is closed.  It's important
    to clean up any GPIO resources that we're using.
    """
    GPIO.cleanup()
    print("GPIO cleanup complete. Exiting application.")
    # No need to explicitly destroy the root window; Tkinter handles that.

if __name__ == "__main__":
    try:
        create_gui()
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        GPIO.cleanup() # Ensure cleanup happens even if there's an error
