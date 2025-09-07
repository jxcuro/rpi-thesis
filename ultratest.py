import RPi.GPIO as GPIO
import time
import sys

# Define GPIO pins using BCM numbering
INPUT_PIN = 23      # This is physical pin 16
RESTART_PIN = 24    # This is physical pin 18

def setup_gpio():
    """Sets up the GPIO pins."""
    # Use Broadcom (BCM) GPIO numbering
    GPIO.setmode(GPIO.BCM)
    # Suppress warnings about channels being in use
    GPIO.setwarnings(False)

    # Set up INPUT_PIN as an input with a pull-down resistor.
    # The pull-down resistor ensures the pin reads LOW (0) by default
    # when nothing is connected to it.
    GPIO.setup(INPUT_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    # Set up RESTART_PIN as an input with a pull-down resistor.
    GPIO.setup(RESTART_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    print("GPIO pins have been configured.")
    print(f"Monitoring GPIO {INPUT_PIN} (Pin 16) for input.")
    print(f"Press the button on GPIO {RESTART_PIN} (Pin 18) to restart.")
    print("-" * 30)

def main_loop():
    """The main logic loop for the program."""
    while True:
        # --- Calibration State ---
        print("Waiting for LOW signal to start calibration...")
        # This inner loop handles the "Calibrate" phase.
        # It will continue as long as the input pin is LOW.
        while GPIO.input(INPUT_PIN) == GPIO.LOW:
            print("Calibrate")
            time.sleep(0.5)

        # If the code reaches here, it means the input pin went HIGH.
        print("\n" + "=" * 30)
        print("HIGH signal detected!")
        print("=" * 30 + "\n")

        # --- Classification State ---
        # This loop handles the "Classify" phase.
        # It will continue as long as the restart button is NOT pressed.
        while GPIO.input(RESTART_PIN) == GPIO.LOW:
            print("Classify")
            # We can sleep for a shorter time here just to prevent the
            # console from being flooded too quickly.
            time.sleep(0.2)

        # If the code reaches here, the restart button was pressed.
        print("\n" + "=" * 30)
        print("Restart button pressed! Resetting process...")
        print("=" * 30 + "\n")
        # The main 'while True' loop will now restart the process from the top.


if __name__ == '__main__':
    try:
        setup_gpio()
        main_loop()
    except KeyboardInterrupt:
        # This block runs when the user presses CTRL+C
        print("\nExiting program. Cleaning up GPIO pins.")
    except Exception as e:
        # This will catch other potential errors
        print(f"An error occurred: {e}")
    finally:
        # This ensures that GPIO.cleanup() is called no matter how the
        # script exits, resetting the state of the GPIO pins.
        GPIO.cleanup()
        print("GPIO cleanup complete. Program terminated.")
        sys.exit(0)
