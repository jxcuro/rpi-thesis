import spidev
import time

# Initialize SPI
spi = spidev.SpiDev()
spi.open(0, 0)  # Bus 0, Device 0 (CE0)
spi.max_speed_hz = 50000  # You can adjust this if needed
spi.mode = 0b00  # SPI mode

def read_register(address):
    """Reads a register from LDC1101 using SPI."""
    # Adjust the address format (shift left and OR with read bit)
    # The read bit (bit 0) must be set for a read operation (0x01)
    address |= 0x01  # Set read bit
    
    # Send the address (read command) and a dummy byte (0x00)
    response = spi.xfer2([address, 0x00])  # Send address and dummy data
    return response[1]  # Return the received data

def read_chip_id():
    """Reads the chip ID (register 0x3F) from LDC1101."""
    chip_id = read_register(0x3F)
    print(f"LDC1101 Chip ID Register (0x3F): 0x{chip_id:02X}")

# Read and print chip ID
read_chip_id()
