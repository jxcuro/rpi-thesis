import spidev
import time

# Initialize SPI
spi = spidev.SpiDev()
spi.open(0, 0)  # Bus 0, Device 0 (CE0)
spi.max_speed_hz = 50000  # You can adjust this if needed
spi.mode = 0b00  # SPI mode

def read_register(address):
    """Reads a register from LDC1101 using SPI."""
    # SPI transaction format: [Address, Read Command]
    # Bit 7: Read command (1 for read)
    # Address is shifted left by 1 (because it's 8-bit address, we use the 7 most significant bits)
    address = address << 1 | 0x01  # Set read bit
    response = spi.xfer2([address, 0x00])  # Send address and read (0x00 is the dummy data)
    return response[1]  # Return the received data

def read_registers(start_addr, end_addr):
    """Reads multiple registers from LDC1101."""
    for addr in range(start_addr, end_addr + 1):
        value = read_register(addr)
        print(f"Register 0x{addr:02X}: 0x{value:02X}")

def read_chip_id():
    """Reads the chip ID (register 0x3F) from LDC1101."""
    chip_id = read_register(0x3F)
    print(f"LDC1101 Chip ID Register (0x3F): 0x{chip_id:02X}")
    
# Read and print chip ID
read_chip_id()

# Read and print a range of registers from 0x00 to 0x10 (adjust as necessary)
read_registers(0x00, 0x10)
