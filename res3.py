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
    "RP_SET":           (0x01, "0x07", True, "RP Dyn. Range"), # Shorter desc
    "TC1":              (0x02, "0x90", True, "Time Constant 1"),
    "TC2":              (0x03, "0xA0", True, "Time Constant 2"),
    "DIG_CONFIG":       (0x04, "0xE4", True, "Conv.Int/OpMode (E4=RP+L)"),
    "ALT_CONFIG":       (0x05, "0x00", True, "Additional Settings"),
    "RP_THRESH_H_LSB":  (0x06, "0x00", True, "RP Thr-H LSB"),
    "RP_THRESH_H_MSB":  (0x07, "0x00", True, "RP Thr-H MSB"),
    "RP_THRESH_L_LSB":  (0x08, "0x00", True, "RP Thr-L LSB"),
    "RP_THRESH_L_MSB":  (0x09, "0x00", True, "RP Thr-L MSB"),
    "INTB_MODE":        (0x0A, "0x00", True, "INTB Pin Report"),
    "START_CONFIG":     (0x0B, "0x01", True, "Power State (01=Sleep)"),
    "D_CONF":           (0x0C, "0x00", True, "Sensor Amp Ctrl"),
    "L_THRESH_HI_LSB":  (0x16, "0x00", True, "L Thr-HI LSB"),
    "L_THRESH_HI_MSB":  (0x17, "0x00", True, "L Thr-HI MSB"),
    "L_THRESH_LO_LSB":  (0x18, "0x00", True, "L Thr-LO LSB"),
    "L_THRESH_LO_MSB":  (0x19, "0x00", True, "L Thr-LO MSB"),
    "STATUS":           (0x20, "0x00", False, "RP+L Meas. Status"),
    "RP_DATA_LSB":      (0x21, "0x00", False, "RP Data LSB"),
    "RP_DATA_MSB":      (0x22, "0x00", False, "RP Data MSB"),
    "L_DATA_LSB":       (0x23, "0x00", False, "L Data LSB"),
    "L_DATA_MSB":       (0x24, "0x00", False, "L Data MSB"),
    "LHR_RCOUNT_LSB":   (0x30, "0x00", True, "LHR Ref.Cnt LSB"),
    "LHR_RCOUNT_MSB":   (0x31, "0x00", True, "LHR Ref.Cnt MSB"),
    "LHR_OFFSET_LSB":   (0x32, "0x00", True, "LHR Offset LSB"),
    "LHR_OFFSET_MSB":   (0x33, "0x00", True, "LHR Offset MSB"),
    "LHR_CONFIG":       (0x34, "0x00", True, "LHR Config"),
    "LHR_DATA_LSB":     (0x38, "0x00", False, "LHR Data LSB"),
    "LHR_DATA_MID":     (0x39, "0x00", False, "LHR Data MID"),
    "LHR_DATA_MSB":     (0x3A, "0x00", False, "LHR Data MSB"),
    "LHR_STATUS":       (0x3B, "0x00", False, "LHR Meas. Status"),
    "RID":              (0x3E, "0x02", False, "Device RID"),
    "CHIP_ID":          (0x3F, "0xD4", False, "Device ID (D4)"),
}
ORDERED_REG_NAMES = sorted(LDC1101_REGISTERS.keys(), key=lambda k: LDC1101_REGISTERS[k][0])

# --- LDC1101 Communication Class (Identical to previous, not shown for brevity) ---
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
            # print("SPI and GPIO initialized.") # Less verbose
            return True
        except Exception as e:
            messagebox.showerror("Hardware Error", f"Failed to initialize SPI/GPIO:\n{e}\n\nCheck SPI enabled (raspi-config) and wiring.")
            return False

    def _transfer(self, data):
        with self.lock:
            GPIO.output(CS_PIN, GPIO.LOW)
            time.sleep(0.00001) # 10us
            result = self.spi.xfer2(data)
            time.sleep(0.00001) # 10us
            GPIO.output(CS_PIN, GPIO.HIGH)
            time.sleep(0.0001) # 100us CS high time
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
        if not self.spi: return 0xFF # Error
        try:
            result = self._transfer([reg_addr | 0x80, 0x00]) # Read command
            return result[1]
        except Exception as e:
            print(f"Error reading register 0x{reg_addr:02X}: {e}")
            return 0xFF # Error

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
        # print(f"Setting mode to {'Active' if active else 'Sleep'} (Reg 0x{start_config_addr:02X} = 0x{mode:02X})...")
        if not self.write_register(start_config_addr, mode):
            messagebox.showerror("Mode Error", f"Failed to set {'Active' if active else 'Sleep'} mode.")
            return False
        time.sleep(0.01) 
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
        if not self.is_initialized: return -1
        lsb = self.read_register(LDC1101_REGISTERS["LHR_DATA_LSB"][0])
        if lsb == 0xFF: return -1
        mid = self.read_register(LDC1101_REGISTERS["LHR_DATA_MID"][0])
        if mid == 0xFF: return -1
        msb = self.read_register(LDC1101_REGISTERS["LHR_DATA_MSB"][0])
        if msb == 0xFF: return -1
        value = (msb << 16) | (mid << 8) | lsb
        return value

    def get_status(self):
        if not self.is_initialized: return 0xFF
        return self.read_register(LDC1101_REGISTERS["STATUS"][0])
        
    def get_lhr_status(self):
        if not self.is_initialized: return 0xFF
        return self.read_register(LDC1101_REGISTERS["LHR_STATUS"][0])

    def cleanup(self):
        # print("Cleaning up...") # Less verbose
        if self.spi:
            try:
                if self.is_initialized: self.set_active_mode(False)
            except Exception: pass # Ignore errors during cleanup
            try:
                self.spi.close()
                self.spi = None
            except Exception: pass
        try:
            GPIO.cleanup()
        except Exception: pass


# --- Scrollable Frame Class (Helper) ---
class ScrollableFrame(ttk.Frame):
    def __init__(self, container, *args, **kwargs):
        super().__init__(container, *args, **kwargs)
        canvas = tk.Canvas(self, highlightthickness=0) # Remove canvas border
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
        self.root.title("LDC1101 Tuner") # Shorter title
        self.root.geometry("640x480") # Smaller default size

        self.ldc = LDC1101_Driver()
        self.data_queue = queue.Queue()
        self.reading_thread = None
        self.is_reading = threading.Event()

        self.register_vars = {}
        self.register_entries = {}

        # Define fonts - reducing size slightly
        self.small_font = ('TkDefaultFont', 8)
        self.data_font = ("Consolas", 10) # Smaller data font

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
        main_frame = ttk.Frame(self.root, padding="2") # Reduced padding
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)

        # Configure row weights in main_frame to control vertical space distribution
        # Row 0: Registers (takes most space, but scrollable)
        # Row 1: Register buttons (minimal space)
        # Row 2: Live measurements & controls (fixed, visible portion)
        # Row 3: Status bar (minimal space)
        main_frame.rowconfigure(0, weight=3) # Registers get more weight but scrollable
        main_frame.rowconfigure(1, weight=0)
        main_frame.rowconfigure(2, weight=1) # Live measurements get some fixed space
        main_frame.rowconfigure(3, weight=0)


        # --- Register Configuration Frame (Scrollable) ---
        reg_outer_frame = ttk.LabelFrame(main_frame, text="Device Registers", padding="2")
        reg_outer_frame.grid(row=0, column=0, columnspan=2, sticky=(tk.W, tk.E, tk.N, tk.S), pady=2)
        
        scrollable_reg_frame = ScrollableFrame(reg_outer_frame)
        scrollable_reg_frame.pack(fill="both", expand=True)
        
        reg_header_frame = scrollable_reg_frame.scrollable_frame
        ttk.Label(reg_header_frame, text="Name (Addr)", font=self.small_font).grid(row=0, column=0, sticky=tk.W, padx=2, pady=1)
        ttk.Label(reg_header_frame, text="Value", font=self.small_font).grid(row=0, column=1, sticky=tk.W, padx=2, pady=1) # Shorter "Value (Hex)"
        ttk.Label(reg_header_frame, text="Desc.", font=self.small_font).grid(row=0, column=2, sticky=tk.W, padx=2, pady=1) # Shorter "Description"
        current_row = 1
        for reg_name in ORDERED_REG_NAMES:
            addr, default_val, is_writable, desc = LDC1101_REGISTERS[reg_name]
            label_text = f"{reg_name}({addr:02X})" # Compacted
            ttk.Label(reg_header_frame, text=label_text, font=self.small_font).grid(row=current_row, column=0, sticky=tk.W, padx=2, pady=1) # Reduced pady
            var = tk.StringVar(value=default_val)
            self.register_vars[addr] = var
            entry_state = tk.NORMAL if is_writable else tk.DISABLED
            # Entry width can be smaller if values are just 0xXX
            entry = ttk.Entry(reg_header_frame, textvariable=var, width=6, font=self.small_font, state=entry_state) 
            entry.grid(row=current_row, column=1, padx=2, pady=1)
            self.register_entries[addr] = entry
            # Reduced wraplength and smaller font for description
            ttk.Label(reg_header_frame, text=desc, font=self.small_font, wraplength=150).grid(row=current_row, column=2, sticky=tk.W, padx=2, pady=1) 
            current_row += 1
        
        reg_btn_frame = ttk.Frame(main_frame) # No specific padding, rely on widget padx/pady
        reg_btn_frame.grid(row=1, column=0, columnspan=2, pady=2, sticky=tk.EW)
        self.write_all_btn = ttk.Button(reg_btn_frame, text="Write All", command=self.write_registers_from_gui) # Shorter button text
        self.write_all_btn.pack(side=tk.LEFT, padx=2)
        self.read_all_btn = ttk.Button(reg_btn_frame, text="Read All", command=self.read_registers_to_gui) # Shorter button text
        self.read_all_btn.pack(side=tk.LEFT, padx=2)

        # --- Live Measurement Display Frame ("Dashboard") ---
        measure_frame = ttk.LabelFrame(main_frame, text="Live Measurements", padding="2")
        measure_frame.grid(row=2, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), pady=2)
        measure_frame.columnconfigure(1, weight=1) # Allow data labels to expand if needed

        row_idx = 0
        def add_live_data_row(text, var):
            nonlocal row_idx
            ttk.Label(measure_frame, text=text, font=self.small_font).grid(row=row_idx, column=0, sticky=tk.W, padx=2, pady=1)
            ttk.Label(measure_frame, textvariable=var, font=self.data_font, width=8, anchor=tk.E).grid(row=row_idx, column=1, sticky=tk.EW, padx=2, pady=1)
            row_idx+=1

        self.rp_data_var = tk.StringVar(value="---")
        add_live_data_row("RP Data:", self.rp_data_var)
        self.l_data_var = tk.StringVar(value="---")
        add_live_data_row("L Data:", self.l_data_var)
        self.lhr_data_var = tk.StringVar(value="---")
        add_live_data_row("LHR Data:", self.lhr_data_var) # Removed (24b) for space
        self.status_var = tk.StringVar(value="0x--")
        add_live_data_row("Status:", self.status_var) # Removed (0x20)
        self.lhr_status_var = tk.StringVar(value="0x--")
        add_live_data_row("LHR Stat:", self.lhr_status_var) # Removed (0x3B)


        live_control_frame = ttk.Frame(main_frame) # No specific padding
        live_control_frame.grid(row=2, column=1, sticky=(tk.W, tk.E, tk.N, tk.S), pady=2, padx=2) # Align with measure_frame
        # Configure live_control_frame to center buttons vertically if possible, or just stack them
        live_control_frame.rowconfigure(0, weight=1)
        live_control_frame.rowconfigure(1, weight=1)


        self.start_live_btn = ttk.Button(live_control_frame, text="Start Live", command=self.start_live_reading)
        self.start_live_btn.grid(row=0, column=0, sticky=tk.EW, padx=2, pady=1)
        self.stop_live_btn = ttk.Button(live_control_frame, text="Stop Live", command=self.stop_live_reading, state=tk.DISABLED)
        self.stop_live_btn.grid(row=1, column=0, sticky=tk.EW, padx=2, pady=1)


        self.status_bar_var = tk.StringVar(value="Init. Ready.") # Shorter
        status_bar = ttk.Label(main_frame, textvariable=self.status_bar_var, relief=tk.SUNKEN, anchor=tk.W, font=self.small_font)
        status_bar.grid(row=3, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(2,0))


    def populate_default_register_values(self):
        for reg_name in ORDERED_REG_NAMES:
            addr, default_val, _, _ = LDC1101_REGISTERS[reg_name]
            if addr in self.register_vars:
                self.register_vars[addr].set(default_val)
        self.status_bar_var.set("Defaults loaded.") # Shorter

    def parse_hex_input(self, hex_string):
        try:
            return int(hex_string, 16)
        except ValueError:
            return None

    def write_registers_from_gui(self):
        self.status_bar_var.set("Writing regs...")
        self.root.update_idletasks()
        was_reading = self.is_reading.is_set()
        if was_reading:
            self.stop_live_reading()
            time.sleep(0.1) # Ensure thread stops

        if not self.ldc.set_active_mode(False): # Sleep for config
            self.status_bar_var.set("Write fail: Sleep mode.")
            if was_reading: self.start_live_reading() # Try restore
            return
        
        success_count, fail_count = 0, 0
        first_fail_addr = None
        for reg_name in ORDERED_REG_NAMES: # Iterate over defined order
            addr, _, is_writable, _ = LDC1101_REGISTERS[reg_name]
            if is_writable and addr in self.register_vars:
                val_str = self.register_vars[addr].get()
                val_int = self.parse_hex_input(val_str)
                if val_int is not None and 0x00 <= val_int <= 0xFF:
                    if self.ldc.write_register(addr, val_int): success_count += 1
                    else:
                        fail_count += 1
                        if first_fail_addr is None: first_fail_addr = addr
                # Be less verbose with message boxes for invalid input during bulk write to save popups
                elif val_int is None and val_str.strip() != "" and not val_str.lower().startswith("0x"): # Non-empty, not 0x means likely error
                    fail_count +=1; first_fail_addr = addr if first_fail_addr is None else first_fail_addr
                    print(f"Warn: Invalid hex '{val_str}' for {reg_name}({addr:02X}). Skipped.")
                elif val_int is not None: # Out of range
                    fail_count += 1; first_fail_addr = addr if first_fail_addr is None else first_fail_addr
                    print(f"Warn: Value 0x{val_int:02X} out of range for {reg_name}({addr:02X}). Skipped.")


        if fail_count > 0:
            self.status_bar_var.set(f"Wrote {success_count}. Fail {fail_count} (1st: {first_fail_addr if first_fail_addr is not None else '--':02X}).")
            # messagebox.showerror("Write Error", f"Failed to write {fail_count} register(s). Check console. First failure at 0x{first_fail_addr if first_fail_addr is not None else '--':02X}.")
        else:
            self.status_bar_var.set(f"All {success_count} writable regs written.")
        
        if was_reading: self.start_live_reading()


    def read_registers_to_gui(self):
        self.status_bar_var.set("Reading regs...")
        self.root.update_idletasks()
        if not self.ldc.is_initialized:
            # messagebox.showwarning("Not Initialized", "Device not initialized.")
            self.status_bar_var.set("Read fail: Not init.")
            return

        was_reading = self.is_reading.is_set()
        if was_reading:
            self.stop_live_reading()
            time.sleep(0.1)

        original_mode_active = False
        start_config_addr = LDC1101_REGISTERS["START_CONFIG"][0]
        current_start_config_val = self.ldc.read_register(start_config_addr)

        if current_start_config_val == ACTIVE_CONVERSION_MODE:
            original_mode_active = True
            self.ldc.set_active_mode(False) 

        read_success_count, read_fail_count = 0,0
        for reg_name in ORDERED_REG_NAMES: # Iterate over defined order
            addr, _, _, _ = LDC1101_REGISTERS[reg_name]
            if addr in self.register_vars:
                val = self.ldc.read_register(addr)
                if val != 0xFF : 
                    self.register_vars[addr].set(f"0x{val:02X}")
                    read_success_count += 1
                else:
                    self.register_vars[addr].set("ERR")
                    read_fail_count +=1
        
        if read_fail_count > 0: self.status_bar_var.set(f"Read {read_success_count}. Fail {read_fail_count}.")
        else: self.status_bar_var.set("All regs read to GUI.")

        if original_mode_active and current_start_config_val == ACTIVE_CONVERSION_MODE :
             self.ldc.set_active_mode(True)

        if was_reading: self.start_live_reading()


    def start_live_reading(self):
        if self.reading_thread and self.reading_thread.is_alive(): return

        dig_conf_addr = LDC1101_REGISTERS["DIG_CONFIG"][0]
        dig_conf_val_str = self.register_vars[dig_conf_addr].get()
        dig_conf_val = self.parse_hex_input(dig_conf_val_str)
        
        op_mode = (dig_conf_val & 0x03) if dig_conf_val is not None else -1 
        power_state_bits = (dig_conf_val >> 6) if dig_conf_val is not None else -1 

        if dig_conf_val is None or power_state_bits == 0b00: 
             # messagebox.showwarning("Config Warning", # Less popups
             print(f"Warning: DIG_CONFIG (0x04) is '{dig_conf_val_str}'. May indicate Sleep Mode. Ensure active mode is configured.")
        
        if not self.ldc.set_active_mode(True): 
            self.status_bar_var.set("Start fail: Active mode.")
            return

        self.is_reading.set()
        self.reading_thread = threading.Thread(target=self.live_read_data_loop, daemon=True)
        self.reading_thread.start()

        self.start_live_btn.config(state=tk.DISABLED)
        self.stop_live_btn.config(state=tk.NORMAL)
        self.write_all_btn.config(state=tk.DISABLED)
        self.read_all_btn.config(state=tk.DISABLED)
        self.status_bar_var.set(f"Live (DIG.OP_MODE:{op_mode if op_mode !=-1 else '?'})...") # Shorter

    def stop_live_reading(self):
        self.is_reading.clear()
        if self.reading_thread and self.reading_thread.is_alive():
            self.reading_thread.join(timeout=0.2) # Wait for thread
        
        self.ldc.set_active_mode(False) # Go to sleep

        self.start_live_btn.config(state=tk.NORMAL)
        self.stop_live_btn.config(state=tk.DISABLED)
        self.write_all_btn.config(state=tk.NORMAL)
        self.read_all_btn.config(state=tk.NORMAL)
        self.status_bar_var.set("Live stopped.") # Shorter

    def live_read_data_loop(self):
        # print("Live reading thread started.") # Less verbose
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
            # if status & 0b10000000: print("! OSC ERR (STAT) !") # Very short warning
            time.sleep(0.05) # ~20 Hz
        # print("Live reading thread finished.") # Less verbose


    def update_gui_from_queue(self):
        try:
            while not self.data_queue.empty():
                data_tuple = self.data_queue.get_nowait()
                if data_tuple[0] == "Error": 
                    self.rp_data_var.set("ERR") # Shorter
                    self.l_data_var.set("ERR")
                    self.lhr_data_var.set("ERR")
                    self.status_var.set("ERR")
                    self.lhr_status_var.set("ERR")
                    self.status_bar_var.set("Live read ERR. Stop.") # Shorter
                    self.stop_live_reading() 
                else:
                    rp_data, l_data, lhr_data, status_val, lhr_status_val = data_tuple
                    self.rp_data_var.set(f"{rp_data}")
                    self.l_data_var.set(f"{l_data}")
                    self.lhr_data_var.set(f"{lhr_data}") 
                    self.status_var.set(f"{status_val:02X}") # Keep 0x for status
                    self.lhr_status_var.set(f"{lhr_status_val:02X}")
                    
                    for reg_name, addr_val in [("STATUS", status_val), ("LHR_STATUS", lhr_status_val)]:
                        addr = LDC1101_REGISTERS[reg_name][0]
                        if addr in self.register_vars:
                             self.register_vars[addr].set(f"0x{addr_val:02X}")

                    if status_val & 0b10000000: # NO_SENSOR_OSC
                        self.status_bar_var.set("Live... OSC ERR!") # Shorter
                    elif self.is_reading.is_set():
                        current_op_mode = self.parse_hex_input(self.register_vars[LDC1101_REGISTERS["DIG_CONFIG"][0]].get())
                        op_mode_bits = (current_op_mode & 0x03) if current_op_mode is not None else -1
                        self.status_bar_var.set(f"Live (OP:{op_mode_bits if op_mode_bits != -1 else '?'})...") # Shorter
        except queue.Empty: pass
        except (ValueError, IndexError) as e: 
            print(f"Queue err: {e}, Data: {data_tuple if 'data_tuple' in locals() else '?'}")
            pass
        self.root.after(100, self.update_gui_from_queue) # Poll queue at 10Hz

    def on_closing(self):
        # print("Close button clicked.") # Less verbose
        if self.is_reading.is_set():
            self.stop_live_reading()
        self.ldc.cleanup()
        self.root.destroy()

# --- Main Execution ---
if __name__ == "__main__":
    root = tk.Tk()
    try: GPIO.cleanup() 
    except Exception: pass # Ignore if already clean or not setup

    app = LdcTunerApp(root)
    if hasattr(app, 'ldc') and app.ldc.is_initialized: # Check app and ldc fully initialized
        try: root.mainloop()
        except Exception as e: print(f"Mainloop error: {e}")
        finally: # Ensure cleanup even on mainloop crash
            if hasattr(app, 'is_reading') and app.is_reading.is_set(): app.stop_live_reading()
            if hasattr(app, 'ldc'): app.ldc.cleanup()
    # print("Application exited.") # Less verbose
