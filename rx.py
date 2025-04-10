import pigpio
import time
import struct

# Initialize pigpio
pi = pigpio.pi()

if not pi.connected:
    print("Failed to connect to pigpio daemon.")
    exit()

# Define CE and CSN pins
CE = 17  # GPIO17
CSN = 8  # GPIO8

# Set CE as output
pi.set_mode(CE, pigpio.OUTPUT)

# Initialize SPI
spi = pi.spi_open(0, 500000)  # SPI channel 0, 500 kHz speed

# Function to send SPI command
def spi_command(cmd, data=None):
    if data is None:
        data = []
    count, response = pi.spi_xfer(spi, [cmd] + data)
    return response

# Function to read a register
def read_register(reg):
    return spi_command(reg & 0x1F, [0])[1]

# Function to write to a register
def write_register(reg, value):
    spi_command(0x20 | (reg & 0x1F), [value])

# Function to set RX address
def set_rx_address(pipe, address):
    spi_command(0x20 | (0x0A + pipe), list(address))  # RX_ADDR_P0 register

# Function to read data
def read_data():
    pi.write(CE, 0)  # Disable CE
    status = spi_command(0xFF, [0, 0, 0, 0])  # Read payload (4 bytes for an integer)
    pi.write(CE, 1)  # Enable CE
    return status[1:]  # Return payload (ignore status byte)

# Configure nRF24L01 for RX mode
write_register(0x00, 0x0F)  # CONFIG: Power up, CRC enabled, Prim_RX = 1 (RX mode)
write_register(0x05, 0x4C)  # RF_CH: Set channel to 76
write_register(0x06, 0x07)  # RF_SETUP: 1 Mbps, 0 dBm
write_register(0x11, 0x04)  # RX_PW_P0: Set payload width to 4 bytes (for an integer)

# Set RX address
rx_address = b'LASER'
set_rx_address(0, rx_address)

# Enable RX mode
pi.write(CE, 1)

# Listen for data
print("Listening for data...")
try:
    while True:
        status = read_register(0x07)  # Read STATUS register
        if status & 0x40:  # Check if data is available (RX_DR bit)
            data = read_data()  # Read the payload
            value = struct.unpack('<I', bytes(data))[0]  # Unpack integer from bytes
            print(f"Received: {value}")
            spi_command(0xE2)  # Flush RX FIFO
        time.sleep(0.1)  # Small delay to avoid busy-waiting
except KeyboardInterrupt:
    print("Exiting...")
finally:
    pi.write(CE, 0)
    pi.spi_close(spi)
    pi.stop()