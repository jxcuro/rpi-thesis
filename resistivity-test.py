import spidev  # For SPI communication with the LDC1101

# Initialize SPI for LDC1101
spi = spidev.SpiDev()
spi.open(0, 0)  # Open SPI bus 0, chip select 0
spi.max_speed_hz = 50000  # Adjust speed as needed

# Define LDC1101 registers (you can refer to the datasheet for exact registers)
LDC1101_MEASUREMENT_REG = 0x00  # Example register for reading measurement
LDC1101_STATUS_REG = 0x01      # Example register for checking status

# Function to read inductance value from LDC1101
def read_inductance():
    # Send a command to start measurement (depending on the sensor's datasheet)
    # For example, we're reading a 16-bit measurement value from the LDC1101
    raw_data = spi.xfer2([LDC1101_MEASUREMENT_REG])  # Adjust command as per datasheet
    inductance_raw = (raw_data[0] << 8) | raw_data[1]  # Combine the 2-byte result
    
    # You can scale the raw value to actual inductance based on sensor calibration
    inductance_value = inductance_raw  # Example, you may need to apply scaling here
    
    return inductance_value

# Function to update inductance measurement on the GUI
def update_inductance():
    inductance_value = read_inductance()  # Get the raw inductance measurement
    inductance_label.config(text=f"Inductance: {inductance_value} mH")  # Display the value in mH or other units
    window.after(60, update_inductance)  # Update every 60ms for better performance

# Create label to display inductance measurement below magnetism
inductance_label = tk.Label(controls_frame, text="Inductance: 0.00 mH", font=("Helvetica", 14))
inductance_label.grid(row=2, column=0)

# Start the inductance measurement update
update_inductance()
