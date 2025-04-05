# Wait for LHR conversion to complete
while True:
    lhr_status = read_register(0x3B)
    if lhr_status & 0x01:  # Check if bit 0 is set (conversion complete)
        break
    time.sleep(0.01)  # Delay to avoid busy-waiting too much

# Now read the LHR data
lhr_data_lsb = read_register(0x38)
lhr_data_mid = read_register(0x39)
lhr_data_msb = read_register(0x3A)

# Combine the data bytes into a 24-bit value (LHR result)
lhr_data = (lhr_data_msb << 16) | (lhr_data_mid << 8) | lhr_data_lsb
print(f"High-Resolution L Inductance Data: 0x{lhr_data:06X}")
