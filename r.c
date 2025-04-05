#include "__ldc1101_driver.h"
#include "__ldc1101_hal.c"

/* Constants for LHR Configuration */
#define LHR_MODE_ENABLE      0x01
#define LHR_MODE_DISABLE     0x00

/* Function to configure LDC1101 into LHR Mode */
uint8_t ldc1101_configure_LHR_mode()
{
    uint8_t chip_id;
    
    // Read Chip ID to verify LDC1101 is present
    chip_id = ldc1101_readByte(_LDC1101_REG_CHIP_ID);
    if (chip_id != 0xD4)  // Check if the chip ID matches the expected value
    {
        return DEVICE_ERROR;  // Return error if chip ID doesn't match
    }
    
    // Enter High Resolution L (LHR) mode
    ldc1101_writeByte(_LDC1101_REG_CFG_LHR, LHR_MODE_ENABLE);  // Enable LHR mode
    
    // Configure LHR RCOUNT (Resolution Count)
    ldc1101_writeByte(_LDC1101_REG_LHR_RCOUNT_LSB, 0x00);  // Set lower byte of RCOUNT
    ldc1101_writeByte(_LDC1101_REG_LHR_RCOUNT_MSB, 0x00);  // Set higher byte of RCOUNT
    
    // Configure LHR Offset
    ldc1101_writeByte(_LDC1101_REG_LHR_OFFSET_LSB, 0x00);  // Set lower byte of Offset
    ldc1101_writeByte(_LDC1101_REG_LHR_OFFSET_MSB, 0x00);  // Set higher byte of Offset
    
    // Set the frequency divider for LHR mode (e.g., divide by 2 for higher resolution)
    ldc1101_writeByte(_LDC1101_REG_LHR_CFG_FREQUENCY_DIVIDED_BY_2, 0x01);
    
    // Optionally, configure other LHR-related settings as needed (Thresholds, etc.)
    ldc1101_writeByte(_LDC1101_REG_L_THRESH_HI_LSB, 0x00);  // Set high threshold
    ldc1101_writeByte(_LDC1101_REG_L_THRESH_HI_MSB, 0x00);  // Set high threshold
    ldc1101_writeByte(_LDC1101_REG_L_THRESH_LO_LSB, 0x00);  // Set low threshold
    ldc1101_writeByte(_LDC1101_REG_L_THRESH_LO_MSB, 0x00);  // Set low threshold
    
    // Finalize by enabling measurement or changing settings as required
    ldc1101_goTo_Lmode();  // Enter L measurement mode for LHR
    
    Delay_100ms();  // Allow some time for the configuration to take effect

    return DEVICE_OK;  // Return success
}

int main()
{
    uint8_t status;
    
    // Initialize the LDC1101
    status = ldc1101_init();
    if (status != DEVICE_OK)
    {
        // Handle initialization failure
        return -1;
    }

    // Configure LDC1101 to LHR Mode
    status = ldc1101_configure_LHR_mode();
    if (status != DEVICE_OK)
    {
        // Handle error configuring LHR mode
        return -1;
    }

    // Additional code for processing or reading data...
    
    return 0;  // Return success
}
