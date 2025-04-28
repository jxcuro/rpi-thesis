import RPi.GPIO as GPIO
import time
import tkinter as tk
from tkinter import ttk

# Define the GPIO pins connected to the Arduino
data_pin_lsb = 16
data_pin_mid = 6
data_ready_pin = 26  # Changed to be the data ready pin

# Set the GPIO mode to BCM numbering
GPIO.setmode(GPIO.BCM)

# Set the GPIO pins as outputs
GPIO.setup(data_pin_lsb, GPIO.OUT)
GPIO.setup(data_pin_mid, GPIO.OUT)
GPIO.setup(data_ready_pin, GPIO.OUT)  # Set data_ready_pin as output

def send_data(data_value):
    """
    Sends data to the Arduino via GPIO pins, now with 5 values and 3 pins, using
    GPIO 26 as the data ready pin, and 1XX encoding for materials.

    Args:
        data_value: The data value to send (0, 1, 2, 3, or 4).
    """
    if data_value == 0:
        GPIO.output(data_ready_pin, GPIO.LOW)
        GPIO.output(data_pin_mid, GPIO.LOW)
        GPIO.output(data_pin_lsb, GPIO.LOW)
        GPIO.output(data_ready_pin, GPIO.HIGH)
        print("Sending: 0 (None - 000)")
    elif data_value == 1:
        GPIO.output(data_ready_pin, GPIO.LOW)
        GPIO.output(data_pin_mid, GPIO.LOW)
        GPIO.output(data_pin_lsb, GPIO.HIGH)
        GPIO.output(data_ready_pin, GPIO.HIGH)
        print("Sending: 1 (Aluminum - 101)")
    elif data_value == 2:
        GPIO.output(data_ready_pin, GPIO.LOW)
        GPIO.output(data_pin_mid, GPIO.HIGH)
        GPIO.output(data_pin_lsb, GPIO.LOW)
        GPIO.output(data_ready_pin, GPIO.HIGH)
        print("Sending: 2 (Copper - 110)")
    elif data_value == 3:
        GPIO.output(data_ready_pin, GPIO.LOW)
        GPIO.output(data_pin_mid, GPIO.HIGH)
        GPIO.output(data_pin_lsb, GPIO.HIGH)
        GPIO.output(data_ready_pin, GPIO.HIGH)
        print("Sending: 3 (Steel - 111)")
    elif data_value == 4:
        GPIO.output(data_ready_pin, GPIO.LOW)
        GPIO.output(data_pin_mid, GPIO.LOW)
        GPIO.output(data_pin_lsb, GPIO.LOW)
        GPIO.output(data_ready_pin, GPIO.HIGH)
        print("Sending: 4 (Others - 100)")
    else:
        print("Invalid data value. Please send 0, 1, 2, 3, or 4.")
    GPIO.output(data_ready_pin, GPIO.LOW)

def create_gui():
    """
    Creates the main application window with buttons.
    """
    root = tk.Tk()
    root.title("Material Selection")
    root.geometry("300x250")

    none_button = ttk.Button(root, text="0 - None", command=lambda: send_data(0))
    aluminum_button = ttk.Button(root, text="1 - Aluminum", command=lambda: send_data(1))
    copper_button = ttk.Button(root, text="2 - Copper", command=lambda: send_data(2))
    steel_button = ttk.Button(root, text="3 - Steel", command=lambda: send_data(3))
    others_button = ttk.Button(root, text="4 - Others", command=lambda: send_data(4))

    none_button.grid(row=0, column=0, columnspan=2, padx=20, pady=10, sticky="ew")
    aluminum_button.grid(row=1, column=0, padx=20, pady=10, sticky="ew")
    copper_button.grid(row=1, column=1, padx=20, pady=10, sticky="ew")
    steel_button.grid(row=2, column=0, padx=20, pady=10, sticky="ew")
    others_button.grid(row=2, column=1, padx=20, pady=10, sticky="ew")

    root.grid_columnconfigure(0, weight=1)
    root.grid_columnconfigure(1, weight=1)
    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.mainloop()

def on_closing():
    """
    This function is called when the window is closed.
    """
    GPIO.cleanup()
    print("GPIO cleanup complete. Exiting application.")

if __name__ == "__main__":
    try:
        create_gui()
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        GPIO.cleanup()
