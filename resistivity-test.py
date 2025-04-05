# Step 6: Set LHR reference count and offset to valid values for testing
write_register(0x30, 0x01)  # LHR_RCOUNT_LSB = 0x01 (set a non-zero reference count)
write_register(0x31, 0x00)  # LHR_RCOUNT_MSB = 0x00
write_register(0x32, 0x00)  # LHR_OFFSET_LSB = 0 (offset is fine as 0 for now)
write_register(0x33, 0x00)  # LHR_OFFSET_MSB = 0
time.sleep(0.01)

# Step 7: Check and print contents of LHR-related registers again
print("Checking LHR Configuration Registers:")
lhr_config = read_register(0x34)
print(f"LHR_CONFIG: 0x{lhr_config:02X}")

lhr_rcount_lsb = read_register(0x30)
lhr_rcount_msb = read_register(0x31)
print(f"LHR_RCOUNT_LSB: 0x{lhr_rcount_lsb:02X}")
print(f"LHR_RCOUNT_MSB: 0x{lhr_rcount_msb:02X}")

lhr_offset_lsb = read_register(0x32)
lhr_offset_msb = read_register(0x33)
print(f"LHR_OFFSET_LSB: 0x{lhr_offset_lsb:02X}")
print(f"LHR_OFFSET_MSB: 0x{lhr_offset_msb:02X}")

# Step 8: Check LHR measurement status
lhr_status = read_register(0x3B)
print(f"LHR_STATUS: 0x{lhr_status:02X}")
