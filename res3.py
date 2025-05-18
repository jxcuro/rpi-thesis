import tkinter as tk
from tkinter import ttk
from tkinter import messagebox
import spidev
import RPi.GPIO as GPIO
import time
import threading
import queue # For thread-safe communication

# --- LDC1101 Constants and SPI Setup ---
# SPI settings
SPI_BUS = 0
SPI_DEVICE = 0
SPI_SPEED = 1000000  # 1 MHz clock speed
SPI_MODE = 0b00

# LDC1101 register addresses
RP_SET_REG = 0x01
TC1_REG = 0x02
TC2_REG = 0x03
DIG_CONFIG_REG = 0x04
ALT_CONFIG_REG = 0x05
START_CONFIG_REG = 0x0B
D_CONF_REG = 0x0C
STATUS_REG = 0x20
RP_DATA_LSB_REG = 0x21
RP_DATA_MSB_REG = 0x22
L_DATA_LSB_REG = 0x23  # L Conversion Result Data Output - bits 7:0
L_DATA_MSB_REG = 0x24  # L Conversion Result Data Output - bits 15:8
CHIP_ID_REG = 0x3F

# GPIO settings
CS_PIN = 8    # Chip Select pin (BCM numbering)

# Power Modes
ACTIVE_CONVERSION_MODE = 0x00
SLEEP_MODE = 0x01

# Estimated Initial Values (from context)
ESTIMATED_RP_SET = "0x25"
ESTIMATED_TC1 = "0x9B"
ESTIMATED_TC2 = "0xFB"
ESTIMATED_DIG_CONF = "0xE4" # This (CONV_MODE=11) enables RP+L continuous

# --- LDC1101 Communication Class ---
class LDC1101_Driver:
    def __init__(self):
        self.spi = None
        self.is_initialized = False
        self.lock = threading.Lock() # Lock for SPI access

    def initialize_spi_gpio(self):
        """Initializes SPI bus and GPIO."""
        try:
            # Initialize GPIO
            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(CS_PIN, GPIO.OUT)
            GPIO.output(CS_PIN, GPIO.HIGH) # Deselect device initially

            # Initialize SPI
            self.spi = spidev.SpiDev()
            self.spi.open(SPI_BUS, SPI_DEVICE)
            self.spi.max_speed_hz = SPI_SPEED
            self.spi.mode = SPI_MODE
            print("SPI and GPIO initialized.")
            return True
        except Exception as e:
            messagebox.showerror("Hardware Error", f"Failed to initialize SPI/GPIO:\n{e}\n\nCheck SPI enabled (raspi-config) and wiring.")
            return False

    def _transfer(self, data):
        """Performs SPI transfer with CS control and delay."""
        with self.lock: # Ensure only one thread accesses SPI at a time
            GPIO.output(CS_PIN, GPIO.LOW)
            time.sleep(0.00001) # Short delay after CS low
            result = self.spi.xfer2(data)
            time.sleep(0.00001) # Short delay before CS high
            GPIO.output(CS_PIN, GPIO.HIGH)
            time.sleep(0.0001) # Ensure CS high time (t_IAG)
        return result

    def write_register(self, reg_addr, value):
        """Writes a byte value to the specified register."""
        if not self.spi: return False
        try:
            self._transfer([reg_addr & 0x7F, value])
            return True
        except Exception as e:
            print(f"Error writing register 0x{reg_addr:02X}: {e}")
            return False

    def read_register(self, reg_addr):
        """Reads a byte value from the specified register."""
        if not self.spi: return 0xFF # Indicate error
        try:
            result = self._transfer([reg_addr | 0x80, 0x00])
            return result[1]
        except Exception as e:
            print(f"Error reading register 0x{reg_addr:02X}: {e}")
            return 0xFF # Indicate error

    def check_chip_id(self):
        """Reads and verifies the Chip ID."""
        chip_id = self.read_register(CHIP_ID_REG)
        if chip_id == 0xD4:
            print("Chip ID OK (0xD4)")
            self.is_initialized = True
            return True
        else:
            messagebox.showerror("Chip ID Error", f"Incorrect Chip ID: 0x{chip_id:02X}. Expected 0xD4.\nCheck wiring and power.")
            self.is_initialized = False
            return False

    def configure_device(self, rp_set, tc1, tc2, dig_conf):
        """Writes the main configuration registers."""
        if not self.is_initialized:
            messagebox.showwarning("Not Initialized", "Device not initialized or Chip ID wrong.")
            return False

        print(f"Writing Config: RP_SET=0x{rp_set:02X}, TC1=0x{tc1:02X}, TC2=0x{tc2:02X}, DIG_CONF=0x{dig_conf:02X}")

        # Go to sleep mode to configure
        if not self.write_register(START_CONFIG_REG, SLEEP_MODE): return False
        time.sleep(0.01)

        # Write the registers
        success = True
        success &= self.write_register(RP_SET_REG, rp_set)
        success &= self.write_register(TC1_REG, tc1)
        success &= self.write_register(TC2_REG, tc2)
        success &= self.write_register(DIG_CONFIG_REG, dig_conf)
        # Ensure other necessary defaults for RP+L mode
        success &= self.write_register(ALT_CONFIG_REG, 0x00) # Ensure L-measurement path enabled if DIG_CONF allows
        success &= self.write_register(D_CONF_REG, 0x00)

        if not success:
            messagebox.showerror("Write Error", "Failed to write one or more configuration registers.")
            return False

        print("Configuration written.")
        return True

    def set_active_mode(self, active=True):
        """Sets the device to Active or Sleep mode."""
        if not self.is_initialized: return False
        mode = ACTIVE_CONVERSION_MODE if active else SLEEP_MODE
        print(f"Setting mode to {'Active' if active else 'Sleep'}...")
        if not self.write_register(START_CONFIG_REG, mode):
            messagebox.showerror("Mode Error", f"Failed to set {'Active' if active else 'Sleep'} mode.")
            return False
        time.sleep(0.01) # Allow mode transition
        return True

    def get_rp_data(self):
        """Reads the 16-bit RP data correctly."""
        if not self.is_initialized: return -1 # Error indicator
        # IMPORTANT: Read LSB (0x21) before MSB (0x22)
        lsb = self.read_register(RP_DATA_LSB_REG)
        if lsb == 0xFF: return -1 # Check for read error
        msb = self.read_register(RP_DATA_MSB_REG)
        if msb == 0xFF: return -1 # Check for read error
        value = (msb << 8) | lsb
        return value

    def get_l_data(self):
        """Reads the 16-bit L data correctly."""
        if not self.is_initialized: return -1 # Error indicator
        # IMPORTANT: Read LSB (0x23) before MSB (0x24)
        lsb = self.read_register(L_DATA_LSB_REG)
        if lsb == 0xFF: return -1 # Check for read error
        msb = self.read_register(L_DATA_MSB_REG)
        if msb == 0xFF: return -1 # Check for read error
        value = (msb << 8) | lsb
        return value

    def get_status(self):
        """Reads the STATUS register."""
        if not self.is_initialized: return 0xFF # Error indicator
        return self.read_register(STATUS_REG)

    def cleanup(self):
        """Cleans up SPI and GPIO resources."""
        print("Cleaning up...")
        if self.spi:
            try:
                # Try to put device to sleep before closing
                if self.is_initialized : self.set_active_mode(False) # Check if initialized before setting mode
            except Exception as e:
                print(f"Could not set sleep mode during cleanup: {e}")
            try:
                self.spi.close()
                self.spi = None
                print("SPI closed.")
            except Exception as e:
                print(f"Error closing SPI: {e}")
        try:
            GPIO.cleanup()
            print("GPIO cleaned up.")
        except Exception as e:
            # Might happen if cleanup already occurred
            print(f"Error cleaning up GPIO: {e}")

# --- GUI Application Class ---
class LdcTunerApp:
    def __init__(self, root):
        self.root = root
        self.root.title("LDC1101 RP & L Tuner")
        # self.root.geometry("400x400") # Adjust size as needed

        self.ldc = LDC1101_Driver()
        self.data_queue = queue.Queue() # Queue for RP/L/Status data from thread
        self.reading_thread = None
        self.is_reading = threading.Event() # Event to signal thread to stop

        # Initialize hardware
        if not self.ldc.initialize_spi_gpio():
            # Error already shown by the method
            self.root.destroy()
            return
        if not self.ldc.check_chip_id():
            # Error already shown
            self.root.destroy()
            return

        # --- GUI Elements ---
        main_frame = ttk.Frame(root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        # Register Configuration Frame
        config_frame = ttk.LabelFrame(main_frame, text="Configuration Registers (Hex)", padding="10")
        config_frame.grid(row=0, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=5)

        ttk.Label(config_frame, text="RP_SET (0x01):").grid(row=0, column=0, sticky=tk.W, padx=5)
        self.rp_set_var = tk.StringVar(value=ESTIMATED_RP_SET)
        self.rp_set_entry = ttk.Entry(config_frame, textvariable=self.rp_set_var, width=8)
        self.rp_set_entry.grid(row=0, column=1, padx=5)

        ttk.Label(config_frame, text="TC1 (0x02):").grid(row=1, column=0, sticky=tk.W, padx=5)
        self.tc1_var = tk.StringVar(value=ESTIMATED_TC1)
        self.tc1_entry = ttk.Entry(config_frame, textvariable=self.tc1_var, width=8)
        self.tc1_entry.grid(row=1, column=1, padx=5)

        ttk.Label(config_frame, text="TC2 (0x03):").grid(row=0, column=2, sticky=tk.W, padx=5)
        self.tc2_var = tk.StringVar(value=ESTIMATED_TC2)
        self.tc2_entry = ttk.Entry(config_frame, textvariable=self.tc2_var, width=8)
        self.tc2_entry.grid(row=0, column=3, padx=5)

        ttk.Label(config_frame, text="DIG_CONF (0x04):").grid(row=1, column=2, sticky=tk.W, padx=5)
        self.dig_conf_var = tk.StringVar(value=ESTIMATED_DIG_CONF)
        self.dig_conf_entry = ttk.Entry(config_frame, textvariable=self.dig_conf_var, width=8)
        self.dig_conf_entry.grid(row=1, column=3, padx=5)

        # Register Action Buttons
        btn_frame = ttk.Frame(main_frame)
        btn_frame.grid(row=1, column=0, columnspan=3, pady=5)
        self.write_btn = ttk.Button(btn_frame, text="Write Registers", command=self.write_config)
        self.write_btn.pack(side=tk.LEFT, padx=5)
        self.read_btn = ttk.Button(btn_frame, text="Read Registers", command=self.read_config)
        self.read_btn.pack(side=tk.LEFT, padx=5)

        # Measurement Display Frame
        measure_frame = ttk.LabelFrame(main_frame, text="Live Measurements", padding="10")
        measure_frame.grid(row=2, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=5)

        ttk.Label(measure_frame, text="RP Data:").grid(row=0, column=0, sticky=tk.W, padx=5, pady=2)
        self.rp_data_var = tk.StringVar(value="---")
        self.rp_data_label = ttk.Label(measure_frame, textvariable=self.rp_data_var, font=("Consolas", 12), width=8, anchor=tk.E)
        self.rp_data_label.grid(row=0, column=1, sticky=tk.E, padx=5, pady=2)

        ttk.Label(measure_frame, text="L Data:").grid(row=1, column=0, sticky=tk.W, padx=5, pady=2)
        self.l_data_var = tk.StringVar(value="---")
        self.l_data_label = ttk.Label(measure_frame, textvariable=self.l_data_var, font=("Consolas", 12), width=8, anchor=tk.E)
        self.l_data_label.grid(row=1, column=1, sticky=tk.E, padx=5, pady=2)

        ttk.Label(measure_frame, text="Status (0x20):").grid(row=2, column=0, sticky=tk.W, padx=5, pady=2)
        self.status_var = tk.StringVar(value="0x--")
        self.status_label = ttk.Label(measure_frame, textvariable=self.status_var, font=("Consolas", 12), width=8, anchor=tk.E)
        self.status_label.grid(row=2, column=1, sticky=tk.E, padx=5, pady=2)

        # Start/Stop Buttons
        control_frame = ttk.Frame(main_frame)
        control_frame.grid(row=3, column=0, columnspan=3, pady=10)
        self.start_btn = ttk.Button(control_frame, text="Start Reading", command=self.start_reading)
        self.start_btn.pack(side=tk.LEFT, padx=5)
        self.stop_btn = ttk.Button(control_frame, text="Stop Reading", command=self.stop_reading, state=tk.DISABLED)
        self.stop_btn.pack(side=tk.LEFT, padx=5)

        # Status Bar
        self.status_bar_var = tk.StringVar(value="Initialized. Ready.")
        status_bar = ttk.Label(main_frame, textvariable=self.status_bar_var, relief=tk.SUNKEN, anchor=tk.W)
        status_bar.grid(row=4, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=(5,0))

        # Set protocol for window close
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

        # Start the update loop for the GUI
        self.update_gui()

    def parse_hex_input(self, hex_string):
        """Tries to parse hex input (0x...), returns integer or None."""
        try:
            return int(hex_string, 16)
        except ValueError:
            return None

    def write_config(self):
        """Reads values from entry fields and writes to LDC1101."""
        self.status_bar_var.set("Writing configuration...")
        self.root.update_idletasks()

        rp_set = self.parse_hex_input(self.rp_set_var.get())
        tc1 = self.parse_hex_input(self.tc1_var.get())
        tc2 = self.parse_hex_input(self.tc2_var.get())
        dig_conf = self.parse_hex_input(self.dig_conf_var.get())

        if None in [rp_set, tc1, tc2, dig_conf]:
            messagebox.showerror("Input Error", "Invalid hex value entered. Use '0x' prefix (e.g., 0x25).")
            self.status_bar_var.set("Write failed: Invalid input.")
            return

        was_reading = self.is_reading.is_set()
        if was_reading:
            self.stop_reading() # Stop reading before writing config
            time.sleep(0.1) # Give thread time to stop

        if self.ldc.configure_device(rp_set, tc1, tc2, dig_conf):
            self.status_bar_var.set("Configuration written successfully.")
            # Optionally restart reading if it was running before
            if was_reading:
                self.start_reading()
        else:
            self.status_bar_var.set("Write failed: SPI error.")


    def read_config(self):
        """Reads current config from LDC1101 and updates entry fields."""
        self.status_bar_var.set("Reading configuration...")
        self.root.update_idletasks()

        if not self.ldc.is_initialized:
            messagebox.showwarning("Not Initialized", "Device not initialized or Chip ID wrong.")
            self.status_bar_var.set("Read failed: Not initialized.")
            return

        was_reading = self.is_reading.is_set()
        if was_reading:
            self.stop_reading() # Stop reading before reading config
            time.sleep(0.1)

        # Put device in sleep to ensure stable read (optional but safe)
        self.ldc.set_active_mode(False)

        rp_set = self.ldc.read_register(RP_SET_REG)
        tc1 = self.ldc.read_register(TC1_REG)
        tc2 = self.ldc.read_register(TC2_REG)
        dig_conf = self.ldc.read_register(DIG_CONFIG_REG)

        if 0xFF in [rp_set, tc1, tc2, dig_conf]:
            messagebox.showerror("Read Error", "Failed to read one or more registers.")
            self.status_bar_var.set("Read failed: SPI error.")
        else:
            self.rp_set_var.set(f"0x{rp_set:02X}")
            self.tc1_var.set(f"0x{tc1:02X}")
            self.tc2_var.set(f"0x{tc2:02X}")
            self.dig_conf_var.set(f"0x{dig_conf:02X}")
            self.status_bar_var.set("Configuration read successfully.")

        # Restore previous mode if needed
        if was_reading:
            self.start_reading()
        else:
            # Leave in sleep or set active if preferred after read
            # self.ldc.set_active_mode(True)
            pass


    def start_reading(self):
        """Starts the background thread for continuous reading."""
        if self.reading_thread and self.reading_thread.is_alive():
            print("Reading thread already running.")
            return

        if not self.ldc.set_active_mode(True):
            self.status_bar_var.set("Failed to start: Could not set Active mode.")
            return

        self.is_reading.set() # Signal thread to run
        self.reading_thread = threading.Thread(target=self.read_data_loop, daemon=True)
        self.reading_thread.start()

        self.start_btn.config(state=tk.DISABLED)
        self.stop_btn.config(state=tk.NORMAL)
        self.write_btn.config(state=tk.DISABLED) # Disable writing while reading
        self.read_btn.config(state=tk.DISABLED)
        self.status_bar_var.set("Reading started...")

    def stop_reading(self):
        """Stops the background reading thread."""
        self.is_reading.clear() # Signal thread to stop
        if self.reading_thread and self.reading_thread.is_alive():
            print("Waiting for reading thread to stop...")
            # Give a bit more time for the thread to join if it's in a sleep
            self.reading_thread.join(timeout=0.2)
            if self.reading_thread.is_alive():
                print("Warning: Reading thread did not stop gracefully.")

        self.ldc.set_active_mode(False) # Put device back to sleep

        self.start_btn.config(state=tk.NORMAL)
        self.stop_btn.config(state=tk.DISABLED)
        self.write_btn.config(state=tk.NORMAL)
        self.read_btn.config(state=tk.NORMAL)
        self.status_bar_var.set("Reading stopped.")
        # Clear display when stopped (optional)
        # self.rp_data_var.set("---")
        # self.l_data_var.set("---")
        # self.status_var.set("0x--")

    def read_data_loop(self):
        """Background loop to read data and put it in the queue."""
        print("Reading thread started.")
        while self.is_reading.is_set():
            status = self.ldc.get_status()
            rp_data = self.ldc.get_rp_data()
            l_data = self.ldc.get_l_data()

            # Check for errors from read functions
            if status == 0xFF or rp_data == -1 or l_data == -1:
                print("Read error in loop, stopping.")
                self.data_queue.put(("Error", "Error", "Error")) # Signal error for all values
                self.is_reading.clear() # Signal stop
                break # Exit thread loop on error

            # Put valid data into the queue for the GUI thread
            self.data_queue.put((rp_data, l_data, status))

            # Check status register for device errors
            if status & 0b10000000: # NO_SENSOR_OSC
                print("!!! WARNING: Sensor Oscillation Error detected (STATUS bit 7 = 1) !!!")
            # Add other status bit checks if needed (e.g., DRDY_L, DRDY_RP)

            time.sleep(0.05) # ~20 Hz read rate - adjust as needed

        print("Reading thread finished.")

    def update_gui(self):
        """Periodically checks the queue and updates GUI labels."""
        try:
            while not self.data_queue.empty():
                rp_data, l_data, status = self.data_queue.get_nowait() # Expect three values
                if rp_data == "Error": # Check if error was signaled
                    self.rp_data_var.set("ERROR")
                    self.l_data_var.set("ERROR")
                    self.status_var.set("ERROR")
                    self.status_bar_var.set("Read error occurred. Stopping.")
                    self.stop_reading() # Force stop GUI elements
                else:
                    self.rp_data_var.set(f"{rp_data}")
                    self.l_data_var.set(f"{l_data}")
                    self.status_var.set(f"0x{status:02X}")
                    if status & 0b10000000: # NO_SENSOR_OSC flag
                        self.status_bar_var.set("Reading... WARNING: Oscillation Error!")
                    elif self.is_reading.is_set():
                        self.status_bar_var.set("Reading...")


        except queue.Empty:
            pass # No new data, just continue
        except ValueError: # Catch if queue did not contain 3 items as expected (e.g. during shutdown)
            print("Queue format error during update_gui.")
            pass


        # Schedule the next update
        self.root.after(100, self.update_gui) # Update GUI every 100ms

    def on_closing(self):
        """Handles window close event."""
        print("Close button clicked.")
        if self.is_reading.is_set():
            self.is_reading.clear() # Signal thread to stop
            if self.reading_thread and self.reading_thread.is_alive():
                print("Waiting for reading thread to terminate before closing...")
                self.reading_thread.join(timeout=0.5) # Wait a bit
        self.ldc.cleanup()
        self.root.destroy()


# --- Main Execution ---
if __name__ == "__main__":
    root = tk.Tk()
    app = LdcTunerApp(root)
    # Only run mainloop if initialization was successful
    # The app constructor now handles destroy on init fail
    if hasattr(app, 'ldc') and app.ldc.is_initialized : # Check if app and ldc were fully initialized
        try:
            root.mainloop()
        except Exception as e:
            print(f"Error in mainloop: {e}")
        finally:
            if app.is_reading.is_set(): # Ensure cleanup if mainloop crashes while reading
                app.stop_reading()
            app.ldc.cleanup() # Redundant if on_closing works, but good for safety
    print("Application exited.")
