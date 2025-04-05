#include "ldc1101.h"

// Function to initialize LDC1101 and configure it for LHR mode
void ldc1101_init_and_configure_lhr(ldc1101_t *ctx, ldc1101_cfg_t *cfg) {
    // Initialize the LDC1101
    if (ldc1101_init(ctx, cfg) != LDC1101_OK) {
        printf("Error initializing LDC1101\n");
        return;
    }
    
    // Configure default settings
    ldc1101_default_cfg(ctx);
    
    // Activate LHR mode
    ldc1101_go_to_l_mode(ctx);
    
    // Set the device to active conversion mode
    ldc1101_set_power_mode(ctx, LDC1101_FUNC_MODE_ACTIVE_CONVERSION_MODE);
    
    printf("LDC1101 configured for LHR mode\n");
}

// Function to retrieve L data from LDC1101
uint16_t ldc1101_get_l_data_value(ldc1101_t *ctx) {
    // Retrieve L data
    uint16_t l_data = ldc1101_get_l_data(ctx);
    
    // Print the L data value
    printf("L data value: %d\n", l_data);
    
    return l_data;
}

int main(void) {
    ldc1101_t ldc1101;
    ldc1101_cfg_t ldc1101_cfg;

    // Setup communication pins
    ldc1101_cfg_setup(&ldc1101_cfg);

    // Initialize and configure LDC1101 for LHR mode
    ldc1101_init_and_configure_lhr(&ldc1101, &ldc1101_cfg);

    // Give some time for the measurement to complete
    // You might need to add a delay here if required
    sleep(1); // Adjust as needed

    // Retrieve L data
    uint16_t l_data = ldc1101_get_l_data_value(&ldc1101);
    
    if (l_data == 0) {
        printf("Error: Received 0 L data, check the configuration.\n");
    }
    
    return 0;
}
