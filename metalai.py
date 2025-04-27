# ... (inside preprocess_input)
    # Check if we successfully found both indices via primary method
    if image_input_index == -1 or numerical_input_index == -1:
         # This block executes if primary identification failed
         print("Warning: Could not reliably determine input tensor indices from shape. Trying fallback.")

         # This 'if' is correctly indented under the outer 'if'
         if len(input_details) == 2:
              # This block is correctly indented under 'if len(input_details) == 2:'
              print("Attempting fallback identification based only on number of dimensions...")
              try:
                  # This block is correctly indented under 'try:'
                  i0_shape, i1_shape = input_details[0]['shape'], input_details[1]['shape']
                  i0_idx, i1_idx = input_details[0]['index'], input_details[1]['index']
                  # ... (rest of try block correctly indented) ...
              except Exception as e:
                    # This block is correctly indented under 'except:'
                    print(f"Fallback failed with exception: {e}")
                    return None
         else:
             # This block is correctly indented under 'else:' (matching the inner 'if')
             print("Cannot use fallback, model does not have exactly 2 inputs.")
             return None

         # This 'if' is correctly indented under the *outer* 'if' (same level as the inner 'if' and 'else')
         if image_input_index == -1 or numerical_input_index == -1:
              print("ERROR: Fallback failed to identify input indices.") # Message corrected
              return None
    # ... rest of function
