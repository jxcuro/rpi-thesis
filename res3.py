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

# GPIO settings
CS_PIN = 8    # Chip Select pin (BCM numbering)

# Power Modes
ACTIVE_CONVERSION_MODE = 0x00
SLEEP_MODE = 0x01

# --- LDC1101 Register Definitions ---
LDC1101_REGISTERS = {
    "RP_SET":           (0x01, "0x07", True, "RP Measurement Dynamic Range"),
    "TC1":              (0x02, "0x90", True, "Internal Time Constant 1"),
    "TC2":              (0x03, "0xA0", True, "Internal Time Constant 2"),
    "DIG_CONFIG":       (0x04, "0xE4", True, "Conv. Interval & Op Mode (0xE4=RP+L cont.)"),
    "ALT_CONFIG":       (0x05, "0x00", True, "Additional device settings"),
    "RP_THRESH_H_LSB":  (0x06, "0x00", True, "RP_THRESHOLD High Setting – LSB"),
    "RP_THRESH_H_MSB":  (0x07, "0x00", True, "RP_THRESHOLD High Setting – MSB"),
    "RP_THRESH_L_LSB":  (0x08, "0x00", True, "RP_THRESHOLD Low Setting – LSB"),
    "RP_THRESH_L_MSB":  (0x09, "0x00", True, "RP_THRESHOLD Low Setting – MSB"),
    "INTB_MODE":        (0x0A, "0x00", True, "Configure INTB reporting on SDO pin"),
    "START_CONFIG":     (0x0B, "0x01", True, "Configure Power State (Default Sleep)"),
    "D_CONF":           (0x0C, "0x00", True, "Sensor Amplitude Control Requirement"),
    "L_THRESH_HI_LSB":  (0x16, "0x00", True, "L_THRESHOLD High Setting – LSB"),
    "L_THRESH_HI_MSB":  (0x17, "0x00", True, "L_THRESHOLD High Setting – MSB"),
    "L_THRESH_LO_LSB":  (0x18, "0x00", True, "L_THRESHOLD Low Setting – LSB"),
    "L_THRESH_LO_MSB":  (0x19, "0x00", True, "L_THRESHOLD Low Setting – MSB"),
    "STATUS":           (0x20, "0x00", False, "Report RP+L measurement status"),
    "RP_DATA_LSB":      (0x21, "0x00", False, "RP Conversion Result Data Output - LSB"),
    "RP_DATA_MSB":      (0x22, "0x00", False, "RP Conversion Result Data Output - MSB"),
    "L_DATA_LSB":       (0x23, "0x00", False, "L Conversion Result Data Output - LSB"),
    "L_DATA_MSB":       (0x24, "0x00", False, "L Conversion Result Data Output - MSB"),
    "LHR_RCOUNT_LSB":   (0x30, "0x00", True, "High Res L Reference Count – LSB"),
    "LHR_RCOUNT_MSB":   (0x31, "0x00", True, "High Res L Reference Count – MSB"),
    "LHR_OFFSET_LSB":   (0x32, "0x00", True, "High Resolution L Offset – LSB"),
    "LHR_OFFSET_MSB":   (0x33, "0x00", True, "High Resolution L Offset – MSB"),
    "LHR_CONFIG":       (0x34, "0x00", True, "High Resolution L Configuration"),
    "LHR_DATA_LSB":     (0x38, "0x00", False, "High Res L Conversion Result - LSB"),
    "LHR_DATA_MID":     (0x39, "0x00", False, "High Res L Conversion Result - MID"),
    "LHR_DATA_MSB":     (0x3A, "0x00", False, "High Res L Conversion Result - MSB"),
    "LHR_STATUS":       (0x3B, "0x00", False, "High Resolution L Measurement Status"),
    "RID":              (0x3E, "0x02", False, "Device RID value"),
    "CHIP_ID":          (0x3F, "0xD4", False, "Device ID value (Expected 0xD4)"),
}
ORDERED_REG_NAMES = sorted(LDC1101_REGISTERS.keys(), key=lambda k: LDC1101_REGISTERS[k][0])

# --- LDC1101 Communication Class ---
class LDC1101_Driver:
    def __init__(self):
        self.spi = None
        self.is_initialized = False
        self.lock = threading.Lock()

    def initialize_spi_gpio(self):
        try:
            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(CS_PIN, GPIO.OUT)
            GPIO.output(CS_PIN, GPIO.HIGH)
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
        with self.lock:
            GPIO.output(CS_PIN, GPIO.LOW)
            time.sleep(0.00001)
            result = self.spi.xfer2(data)
            time.sleep(0.00001)
            GPIO.output(CS_PIN, GPIO.HIGH)
            time.sleep(0.0001)
        return result

    def write_register(self, reg_addr, value):
        if not self.spi: return False
        try:
            self._transfer([reg_addr & 0x7F, value])
            return True
        except Exception as e:
            print(f"Error writing register 0x{reg_addr:02X}: {e}")
            return False

    def read_register(self, reg_addr):
        if not self.spi: return 0xFF
        try:
            result = self._transfer([reg_addr | 0x80, 0x00])
            return result[1]
        except Exception as e:
            print(f"Error reading register 0x{reg_addr:02X}: {e}")
            return 0xFF

    def check_chip_id(self):
        chip_id_addr = LDC1101_REGISTERS["CHIP_ID"][0]
        expected_chip_id = int(LDC1101_REGISTERS["CHIP_ID"][1], 16)
        chip_id = self.read_register(chip_id_addr)
        if chip_id == expected_chip_id:
            print(f"Chip ID OK (0x{chip_id:02X})")
            self.is_initialized = True
            return True
        else:
            messagebox.showerror("Chip ID Error", f"Incorrect Chip ID: 0x{chip_id:02X}. Expected 0x{expected_chip_id:02X}.\nCheck wiring and power.")
            self.is_initialized = False
            return False

    def set_active_mode(self, active=True):
        if not self.is_initialized: return False
        mode = ACTIVE_CONVERSION_MODE if active else SLEEP_MODE
        start_config_addr = LDC1101_REGISTERS["START_CONFIG"][0]
        print(f"Setting mode to {'Active' if active else 'Sleep'} (Reg 0x{start_config_addr:02X} = 0x{mode:02X})...")
        if not self.write_register(start_config_addr, mode):
            messagebox.showerror("Mode Error", f"Failed to set {'Active' if active else 'Sleep'} mode.")
            return False
        time.sleep(0.01) # Allow mode transition. Datasheet: "Wait 1 ms for mode transition"
        return True

    def get_rp_data(self):
        if not self.is_initialized: return -1
        lsb = self.read_register(LDC1101_REGISTERS["RP_DATA_LSB"][0])
        if lsb == 0xFF: return -1
        msb = self.read_register(LDC1101_REGISTERS["RP_DATA_MSB"][0])
        if msb == 0xFF: return -1
        return (msb << 8) | lsb

    def get_l_data(self):
        if not self.is_initialized: return -1
        lsb = self.read_register(LDC1101_REGISTERS["L_DATA_LSB"][0])
        if lsb == 0xFF: return -1
        msb = self.read_register(LDC1101_REGISTERS["L_DATA_MSB"][0])
        if msb == 0xFF: return -1
        return (msb << 8) | lsb

    def get_lhr_data(self):
        """Reads the 24-bit LHR data."""
        if not self.is_initialized: return -1
        lsb = self.read_register(LDC1101_REGISTERS["LHR_DATA_LSB"][0])
        if lsb == 0xFF: return -1
        mid = self.read_register(LDC1101_REGISTERS["LHR_DATA_MID"][0])
        if mid == 0xFF: return -1
        msb = self.read_register(LDC1101_REGISTERS["LHR_DATA_MSB"][0])
        if msb == 0xFF: return -1
        # LHR data is 24-bit, MSB contains bits 23:16
        value = (msb << 16) | (mid << 8) | lsb
        return value

    def get_status(self):
        if not self.is_initialized: return 0xFF
        return self.read_register(LDC1101_REGISTERS["STATUS"][0])
        
    def get_lhr_status(self):
        if not self.is_initialized: return 0xFF
        return self.read_register(LDC1101_REGISTERS["LHR_STATUS"][0])


    def cleanup(self):
        print("Cleaning up...")
        if self.spi:
            try:
                if self.is_initialized: self.set_active_mode(False)
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
            print(f"Error cleaning up GPIO (this might be normal if already cleaned): {e}")

# --- Scrollable Frame Class (Helper) ---
class ScrollableFrame(ttk.Frame):
    def __init__(self, container, *args, **kwargs):
        super().__init__(container, *args, **kwargs)
        canvas = tk.Canvas(self)
        scrollbar = ttk.Scrollbar(self, orient="vertical", command=canvas.yview)
        self.scrollable_frame = ttk.Frame(canvas)

        self.scrollable_frame.bind("<Configure>", lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.create_window((0, 0), window=self.scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)

        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

# --- GUI Application Class ---
class LdcTunerApp:
    def __init__(self, root):
        self.root = root
        self.root.title("LDC1101 Universal Register Tuner")
        # self.root.geometry("700x750") # Increased height for LHR data

        self.ldc = LDC1101_Driver()
        self.data_queue = queue.Queue()
        self.reading_thread = None
        self.is_reading = threading.Event()

        self.register_vars = {}
        self.register_entries = {}

        if not self.ldc.initialize_spi_gpio():
            self.root.destroy()
            return
        if not self.ldc.check_chip_id():
            self.root.destroy()
            return

        self._create_gui_elements()
        self.populate_default_register_values()

        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.update_gui_from_queue()

    def _create_gui_elements(self):
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1) # Allow main_frame to expand

        # --- Register Configuration Frame (Scrollable) ---
        reg_outer_frame = ttk.LabelFrame(main_frame, text="Device Registers", padding="10")
        reg_outer_frame.grid(row=0, column=0, columnspan=2, sticky=(tk.W, tk.E, tk.N, tk.S), pady=5)
        # Allow row 0 of main_frame (where reg_outer_frame is) to take up vertical space
        main_frame.rowconfigure(0, weight=1) 

        scrollable_reg_frame = ScrollableFrame(reg_outer_frame)
        scrollable_reg_frame.pack(fill="both", expand=True)
        
        reg_header_frame = scrollable_reg_frame.scrollable_frame
        ttk.Label(reg_header_frame, text="Name (Addr)", font=('TkDefaultFont', 9, 'bold')).grid(row=0, column=0, sticky=tk.W, padx=5, pady=2)
        ttk.Label(reg_header_frame, text="Value (Hex)", font=('TkDefaultFont', 9, 'bold')).grid(row=0, column=1, sticky=tk.W, padx=5, pady=2)
        ttk.Label(reg_header_frame, text="Description", font=('TkDefaultFont', 9, 'bold')).grid(row=0, column=2, sticky=tk.W, padx=5, pady=2)
        current_row = 1
        for reg_name in ORDERED_REG_NAMES:
            addr, default_val, is_writable, desc = LDC1101_REGISTERS[reg_name]
            label_text = f"{reg_name} (0x{addr:02X})"
            ttk.Label(reg_header_frame, text=label_text).grid(row=current_row, column=0, sticky=tk.W, padx=5, pady=2)
            var = tk.StringVar(value=default_val)
            self.register_vars[addr] = var
            entry_state = tk.NORMAL if is_writable else tk.DISABLED
            entry = ttk.Entry(reg_header_frame, textvariable=var, width=10, state=entry_state)
            entry.grid(row=current_row, column=1, padx=5, pady=2)
            self.register_entries[addr] = entry
            ttk.Label(reg_header_frame, text=desc, wraplength=300).grid(row=current_row, column=2, sticky=tk.W, padx=5, pady=2)
            current_row += 1
        
        reg_btn_frame = ttk.Frame(main_frame)
        reg_btn_frame.grid(row=1, column=0, columnspan=2, pady=5, sticky=tk.EW)
        self.write_all_btn = ttk.Button(reg_btn_frame, text="Write All Writable Registers", command=self.write_registers_from_gui)
        self.write_all_btn.pack(side=tk.LEFT, padx=5)
        self.read_all_btn = ttk.Button(reg_btn_frame, text="Read All Registers", command=self.read_registers_to_gui)
        self.read_all_btn.pack(side=tk.LEFT, padx=5)

        # --- Live Measurement Display Frame ---
        measure_frame = ttk.LabelFrame(main_frame, text="Live Measurements", padding="10")
        measure_frame.grid(row=2, column=0, sticky=(tk.W, tk.E, tk.N), pady=5) # tk.N to align top
        # Allow row 2 of main_frame to have some weight but not expand like registers
        main_frame.rowconfigure(2, weight=0) 


        row_idx = 0
        ttk.Label(measure_frame, text="RP Data:").grid(row=row_idx, column=0, sticky=tk.W, padx=5, pady=2)
        self.rp_data_var = tk.StringVar(value="---")
        ttk.Label(measure_frame, textvariable=self.rp_data_var, font=("Consolas", 12), width=10, anchor=tk.E).grid(row=row_idx, column=1, sticky=tk.E, padx=5, pady=2)
        row_idx += 1

        ttk.Label(measure_frame, text="L Data:").grid(row=row_idx, column=0, sticky=tk.W, padx=5, pady=2)
        self.l_data_var = tk.StringVar(value="---")
        ttk.Label(measure_frame, textvariable=self.l_data_var, font=("Consolas", 12), width=10, anchor=tk.E).grid(row=row_idx, column=1, sticky=tk.E, padx=5, pady=2)
        row_idx += 1

        ttk.Label(measure_frame, text="LHR Data (24b):").grid(row=row_idx, column=0, sticky=tk.W, padx=5, pady=2)
        self.lhr_data_var = tk.StringVar(value="---")
        ttk.Label(measure_frame, textvariable=self.lhr_data_var, font=("Consolas", 12), width=10, anchor=tk.E).grid(row=row_idx, column=1, sticky=tk.E, padx=5, pady=2)
        row_idx += 1
        
        ttk.Label(measure_frame, text="Status (0x20):").grid(row=row_idx, column=0, sticky=tk.W, padx=5, pady=2)
        self.status_var = tk.StringVar(value="0x--")
        ttk.Label(measure_frame, textvariable=self.status_var, font=("Consolas", 12), width=10, anchor=tk.E).grid(row=row_idx, column=1, sticky=tk.E, padx=5, pady=2)
        row_idx += 1

        ttk.Label(measure_frame, text="LHR Status (0x3B):").grid(row=row_idx, column=0, sticky=tk.W, padx=5, pady=2)
        self.lhr_status_var = tk.StringVar(value="0x--")
        ttk.Label(measure_frame, textvariable=self.lhr_status_var, font=("Consolas", 12), width=10, anchor=tk.E).grid(row=row_idx, column=1, sticky=tk.E, padx=5, pady=2)


        live_control_frame = ttk.Frame(main_frame)
        live_control_frame.grid(row=2, column=1, sticky=(tk.W, tk.E, tk.N), pady=5, padx=10)
        self.start_live_btn = ttk.Button(live_control_frame, text="Start Live Reading", command=self.start_live_reading)
        self.start_live_btn.pack(side=tk.TOP, padx=5, pady=(0,5), fill=tk.X) # Adjusted y-padding
        self.stop_live_btn = ttk.Button(live_control_frame, text="Stop Live Reading", command=self.stop_live_reading, state=tk.DISABLED)
        self.stop_live_btn.pack(side=tk.TOP, padx=5, pady=5, fill=tk.X)

        self.status_bar_var = tk.StringVar(value="Initialized. Ready.")
        status_bar = ttk.Label(main_frame, textvariable=self.status_bar_var, relief=tk.SUNKEN, anchor=tk.W)
        status_bar.grid(row=3, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(5,0))
        main_frame.rowconfigure(3, weight=0) # Status bar should not expand

    def populate_default_register_values(self):
        for reg_name in ORDERED_REG_NAMES:
            addr, default_val, _, _ = LDC1101_REGISTERS[reg_name]
            if addr in self.register_vars:
                self.register_vars[addr].set(default_val)
        self.status_bar_var.set("Default register values loaded into GUI.")

    def parse_hex_input(self, hex_string):
        try:
            return int(hex_string, 16)
        except ValueError:
            return None

    def write_registers_from_gui(self):
        self.status_bar_var.set("Writing registers from GUI...")
        self.root.update_idletasks()
        was_reading = self.is_reading.is_set()
        if was_reading:
            self.stop_live_reading()
            time.sleep(0.1)

        if not self.ldc.set_active_mode(False):
            self.status_bar_var.set("Write failed: Could not set Sleep mode.")
            if was_reading: self.start_live_reading()
            return
        
        success_count, fail_count = 0, 0
        first_fail_addr = None
        for reg_name in ORDERED_REG_NAMES:
            addr, _, is_writable, _ = LDC1101_REGISTERS[reg_name]
            if is_writable and addr in self.register_vars:
                val_str = self.register_vars[addr].get()
                val_int = self.parse_hex_input(val_str)
                if val_int is not None and 0x00 <= val_int <= 0xFF:
                    if self.ldc.write_register(addr, val_int): success_count += 1
                    else:
                        fail_count += 1
                        if first_fail_addr is None: first_fail_addr = addr
                elif val_int is None:
                    fail_count +=1; first_fail_addr = addr if first_fail_addr is None else first_fail_addr
                    messagebox.showwarning("Input Error", f"Invalid hex '{val_str}' for {reg_name} (0x{addr:02X}). Skipped.")
                else:
                    fail_count += 1; first_fail_addr = addr if first_fail_addr is None else first_fail_addr
                    messagebox.showwarning("Input Error", f"Value 0x{val_int:02X} out of range for {reg_name} (0x{addr:02X}). Skipped.")

        if fail_count > 0:
            self.status_bar_var.set(f"Wrote {success_count} regs. Failed {fail_count} (first at 0x{first_fail_addr if first_fail_addr is not None else '--':02X}).")
        else:
            self.status_bar_var.set(f"All {success_count} writable registers from GUI written successfully.")
        
        if was_reading: self.start_live_reading()

    def read_registers_to_gui(self):
        self.status_bar_var.set("Reading all registers to GUI...")
        self.root.update_idletasks()
        if not self.ldc.is_initialized:
            messagebox.showwarning("Not Initialized", "Device not initialized.")
            self.status_bar_var.set("Read failed: Not initialized.")
            return

        was_reading = self.is_reading.is_set()
        if was_reading:
            self.stop_live_reading()
            time.sleep(0.1)

        original_mode_active = False
        # It's good practice to read START_CONFIG first if we plan to restore it
        start_config_addr = LDC1101_REGISTERS["START_CONFIG"][0]
        current_start_config_val = self.ldc.read_register(start_config_addr)

        if current_start_config_val == ACTIVE_CONVERSION_MODE: # Check if device thinks it's active
            original_mode_active = True
            self.ldc.set_active_mode(False) # Go to sleep for safer read

        read_success_count, read_fail_count = 0,0
        for reg_name in ORDERED_REG_NAMES:
            addr, _, _, _ = LDC1101_REGISTERS[reg_name]
            if addr in self.register_vars:
                val = self.ldc.read_register(addr)
                if val != 0xFF : # Assuming 0xFF indicates driver read error
                    self.register_vars[addr].set(f"0x{val:02X}")
                    read_success_count += 1
                else:
                    self.register_vars[addr].set("ERR")
                    read_fail_count +=1
        
        if read_fail_count > 0: self.status_bar_var.set(f"Read {read_success_count} regs. Failed for {read_fail_count}.")
        else: self.status_bar_var.set("All registers read to GUI successfully.")

        # Restore previous active state ONLY if we explicitly put it to sleep for this read operation
        if original_mode_active and current_start_config_val == ACTIVE_CONVERSION_MODE :
             self.ldc.set_active_mode(True)

        if was_reading: self.start_live_reading()


    def start_live_reading(self):
        if self.reading_thread and self.reading_thread.is_alive(): return

        # Check DIG_CONFIG from GUI. User needs to set it for the desired mode (RP+L, LHR, etc.)
        dig_conf_addr = LDC1101_REGISTERS["DIG_CONFIG"][0]
        dig_conf_val_str = self.register_vars[dig_conf_addr].get()
        dig_conf_val = self.parse_hex_input(dig_conf_val_str)
        
        op_mode = (dig_conf_val & 0x03) if dig_conf_val is not None else -1 # Extract OP_MODE bits (bits 1:0)
        power_state_bits = (dig_conf_val >> 6) if dig_conf_val is not None else -1 # Extract Power mode bits (bits 7:6)

        # Warn if DIG_CONFIG suggests sleep or is unconfigured
        if dig_conf_val is None or power_state_bits == 0b00: # 00 = Sleep Mode in DIG_CONFIG
             messagebox.showwarning("Config Warning",
                                   f"DIG_CONFIG (0x04) is '{dig_conf_val_str}'. Its Power Mode bits (7:6) might be set to Sleep (00), or value is invalid. "
                                   f"Ensure DIG_CONFIG is set for an active measurement mode (e.g., 0xE4 for RP+L, 0xCA for LHR continuous).")
        
        # Regardless of DIG_CONFIG's power bits, START_CONFIG (0x0B) ultimately controls active/sleep for conversions
        if not self.ldc.set_active_mode(True): # This sets START_CONFIG to 0x00
            self.status_bar_var.set("Failed to start live reading: Could not set Active mode via START_CONFIG.")
            return

        self.is_reading.set()
        self.reading_thread = threading.Thread(target=self.live_read_data_loop, daemon=True)
        self.reading_thread.start()

        self.start_live_btn.config(state=tk.DISABLED)
        self.stop_live_btn.config(state=tk.NORMAL)
        self.write_all_btn.config(state=tk.DISABLED)
        self.read_all_btn.config(state=tk.DISABLED)
        self.status_bar_var.set(f"Live reading started (DIG_CONFIG OP_MODE: {op_mode if op_mode !=-1 else '??'})...")

    def stop_live_reading(self):
        self.is_reading.clear()
        if self.reading_thread and self.reading_thread.is_alive():
            self.reading_thread.join(timeout=0.2)
        
        self.ldc.set_active_mode(False) 

        self.start_live_btn.config(state=tk.NORMAL)
        self.stop_live_btn.config(state=tk.DISABLED)
        self.write_all_btn.config(state=tk.NORMAL)
        self.read_all_btn.config(state=tk.NORMAL)
        self.status_bar_var.set("Live reading stopped.")

    def live_read_data_loop(self):
        print("Live reading thread started.")
        while self.is_reading.is_set():
            status = self.ldc.get_status()
            rp_data = self.ldc.get_rp_data()
            l_data = self.ldc.get_l_data()
            lhr_data = self.ldc.get_lhr_data()
            lhr_status = self.ldc.get_lhr_status()


            if status == 0xFF or rp_data == -1 or l_data == -1 or lhr_data == -1 or lhr_status == 0xFF:
                print("Read error in live loop, stopping.")
                self.data_queue.put(("Error", "Error", "Error", "Error", "Error"))
                self.is_reading.clear() 
                break 
            
            self.data_queue.put((rp_data, l_data, lhr_data, status, lhr_status))

            if status & 0b10000000: print("!!! WARNING: Sensor Oscillation Error (STATUS bit 7) !!!")
            # Add LHR status checks if desired, e.g., LHR_STATUS & 0x01 (DRDY_LHR)
            
            time.sleep(0.05) # ~20 Hz

        print("Live reading thread finished.")

    def update_gui_from_queue(self):
        try:
            while not self.data_queue.empty():
                data_tuple = self.data_queue.get_nowait()
                # Expecting (rp_data, l_data, lhr_data, status_val, lhr_status_val)
                if data_tuple[0] == "Error": # Check first element for error signal
                    self.rp_data_var.set("ERROR")
                    self.l_data_var.set("ERROR")
                    self.lhr_data_var.set("ERROR")
                    self.status_var.set("ERROR")
                    self.lhr_status_var.set("ERROR")
                    self.status_bar_var.set("Live read error occurred. Stopping.")
                    self.stop_live_reading() 
                else:
                    rp_data, l_data, lhr_data, status_val, lhr_status_val = data_tuple
                    self.rp_data_var.set(f"{rp_data}")
                    self.l_data_var.set(f"{l_data}")
                    self.lhr_data_var.set(f"{lhr_data}") # Display raw 24-bit LHR data
                    self.status_var.set(f"0x{status_val:02X}")
                    self.lhr_status_var.set(f"0x{lhr_status_val:02X}")
                    
                    # Update the read-only status registers in the main GUI list as well
                    for reg_name, addr_val in [("STATUS", status_val), ("LHR_STATUS", lhr_status_val)]:
                        addr = LDC1101_REGISTERS[reg_name][0]
                        if addr in self.register_vars:
                             self.register_vars[addr].set(f"0x{addr_val:02X}")

                    if status_val & 0b10000000:
                        self.status_bar_var.set("Live Reading... WARNING: Oscillation Error!")
                    elif self.is_reading.is_set():
                        current_op_mode = self.parse_hex_input(self.register_vars[LDC1101_REGISTERS["DIG_CONFIG"][0]].get())
                        op_mode_bits = (current_op_mode & 0x03) if current_op_mode is not None else -1
                        self.status_bar_var.set(f"Live Reading (DIG_OP_MODE: {op_mode_bits if op_mode_bits != -1 else '??'})...")

        except queue.Empty: pass
        except (ValueError, IndexError) as e: # Catch if queue item not as expected
            print(f"Queue format error during update_gui: {e}, Data: {data_tuple if 'data_tuple' in locals() else 'Unknown'}")
            pass
        self.root.after(100, self.update_gui_from_queue)

    def on_closing(self):
        print("Close button clicked.")
        if self.is_reading.is_set():
            self.stop_live_reading()
        self.ldc.cleanup()
        self.root.destroy()

# --- Main Execution ---
if __name__ == "__main__":
    root = tk.Tk()
    try: GPIO.cleanup() 
    except Exception: pass

    app = LdcTunerApp(root)
    if hasattr(app, 'ldc') and app.ldc.is_initialized:
        try: root.mainloop()
        except Exception as e: print(f"Error in mainloop: {e}")
    print("Application exited.")
