# --- Corrected Block ---
    current_mag_mT = None # Initialize default
    mag_display_text = "N/A" # Default display text

    if avg_voltage is not None:
        # Voltage reading was successful, proceed with calculation attempt
        try:
            # Indented block for the try statement
            idle_v = IDLE_VOLTAGE if IDLE_VOLTAGE != 0 else avg_voltage
            current_mag_mT = (avg_voltage - idle_v) / SENSITIVITY_V_PER_MILLITESLA
            # Format text only on successful calculation
            mag_display_text = f"{current_mag_mT:+.3f} mT"
        except ZeroDivisionError:
            # Indented block for except ZeroDivisionError
            mag_display_text = "Cal Error?" # Handle division by zero if sensitivity is 0
            current_mag_mT = None
        except Exception as e:
            # Indented block for general except Exception
            # print(f"Debug: Magnetism calc error: {e}") # Optional debug
            mag_display_text = "Calc Error"
            current_mag_mT = None

        # Check calibration status *after* try-except block
        # Append "(No Cal)" only if calculation didn't fail
        # This 'if' is indented under 'if avg_voltage is not None:'
        if IDLE_VOLTAGE == 0 and "Error" not in mag_display_text:
             mag_display_text += " (No Cal)"
    # --- End of Corrected Block ---

    # Sensor reading for LDC (already corrected in previous version)
    current_rp_val_avg=get_averaged_rp_data(NUM_SAMPLES_CALIBRATION)
    current_rp_val, delta_rp, ldc_display_text = None, None, "N/A"
    # ... (rest of LDC calculation remains the same) ...
