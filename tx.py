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

# Function to write to a register
def write_register(reg, value):
    spi_command(0x20 | (reg & 0x1F), [value])

# Function to set TX address
def set_tx_address(address):
    spi_command(0x20 | 0x10, list(address))  # TX_ADDR register

# Function to send data
def send_data(data):
    pi.write(CE, 0)  # Disable CE
    spi_command(0xE1)  # Flush TX FIFO
    spi_command(0xA0, list(data))  # Convert bytes to list and write payload
    pi.write(CE, 1)  # Enable CE
    time.sleep(0.00001)  # Wait for transmission to start
    pi.write(CE, 0)  # Disable CE

# Configure nRF24L01 for TX mode
write_register(0x00, 0x0E)  # CONFIG: Power up, CRC enabled, Prim_RX = 0 (TX mode)
write_register(0x05, 0x4C)  # RF_CH: Set channel to 76
write_register(0x06, 0x07)  # RF_SETUP: 1 Mbps, 0 dBm
write_register(0x11, 0x20)  # RX_PW_P0: Set payload width to 32 bytes

# Set TX address
tx_address = b'LASER'
set_tx_address(tx_address)

counter = 1

# Send incrementing integer continuously
while True:
    # Pack the integer into a 4-byte structure
    data = struct.pack('<I', counter) 
    print(f"Sending: {counter}")
    send_data(data)  # Send the packed data
    counter += 1  # Increment the counter
    time.sleep(1)  # Wait for 1 second before sending the next number

pi.spi_close(spi)
pi.stop()