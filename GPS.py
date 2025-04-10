#!/usr/bin/env python3

import serial
import struct
import time
import board
import digitalio

# Set up LED on GPIO D17
led = digitalio.DigitalInOut(board.D17)
led.direction = digitalio.Direction.OUTPUT

# Initialize GPS fix status
has_fix = False

# Record the start time of the program
start_time = time.time()

# Time when the LED should turn off
led_off_time = None

# Function to convert latitude and longitude to decimal degrees
def convert_to_decimal(degrees_minutes, direction):
    try:
        degrees = int(degrees_minutes[:len(degrees_minutes)-7])  # Extract degrees
        minutes = float(degrees_minutes[len(degrees_minutes)-7:])  # Extract minutes
        decimal = degrees + (minutes / 60)  # Convert to decimal degrees
        if direction in ["S", "W"]:
            decimal *= -1  # Apply negative for South and West
        return decimal
    except ValueError as e:
        print(f"Error in converting to decimal: {e}")
        return 0.0

# Function to handle serial port opening and error handling
def open_serial_port():
    try:
        uart = serial.Serial("/dev/serial0", baudrate=9600, timeout=1)
        print("Serial port opened successfully")
        return uart
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        return None

# Function to handle writing data to binary file
def write_to_bin_file(data, binfile):
    try:
        binfile.write(data)
        binfile.flush()
        print("Data written to binary file.")
    except IOError as e:
        print(f"Error writing to file: {e}")

# Main logic to handle GPS data reading and processing
def main():
    global has_fix, led_off_time
    
    uart = open_serial_port()
    if uart is None:
        print("Failed to open serial port. Exiting.")
        return

    # Open binary file for writing
    with open('gps_data.bin', 'wb') as binfile:
        while True:
            try:
                if uart.in_waiting > 0:
                    data = uart.readline().decode('ascii', errors='ignore').strip()  # Read received data
                    print(f"Received Data: {data}")  # Print received data

                    # Check for Recommended Minimum Navigation Information (GPRMC)
                    if data.startswith("$GPRMC"):
                        fields = data.split(",")
                        status = fields[2]
                        
                        if status == "A":  # GPS has a valid fix
                            has_fix = True  # Mark GPS has fix

                            # Parse data
                            time_str = fields[1]
                            latitude = convert_to_decimal(fields[3], fields[4])  # Convert to decimal
                            longitude = convert_to_decimal(fields[5], fields[6])  # Convert to decimal
                            speed = float(fields[7])
                            course = float(fields[8])
                            date = fields[9]

                            # Calculate program run time
                            program_time = time.time() - start_time

                            # Print parsed data
                            print(f"\nTime: {time_str}, Status: {status}")
                            print(f"Latitude: {latitude}, Longitude: {longitude}")
                            print(f"Speed: {speed} knots, Course: {course}°")
                            print(f"Date: {date}")
                            print(f"Program Run Time: {program_time:.2f} seconds\n")

                            # Pack data to binary format
                            packed_data = struct.pack('!6s 1s f f f f 6s f',
                                                      time_str.encode('ascii'),
                                                      status.encode('ascii'),
                                                      latitude,
                                                      longitude,
                                                      speed,
                                                      course,
                                                      date.encode('ascii'),
                                                      program_time)

                            # Write packed data to binary file
                            write_to_bin_file(packed_data, binfile)

                        elif status == "V":  # GPS is not valid
                            has_fix = False
                            print("GPS fix is not valid. Waiting for valid fix...")

                    # Check if GPS has a fix
                    if not has_fix:
                        print("Waiting for GPS fix...")
                        time.sleep(1)

                    # LED logic without blocking
                    if has_fix and led_off_time is None:
                        led.value = True
                        led_off_time = time.time() + 0.1  # Turn off LED after 0.1s

                    if led_off_time is not None and time.time() >= led_off_time:
                        led.value = False
                        led_off_time = None

                else:
                    print("No data available, retrying...")
                    time.sleep(1)  # Delay to avoid tight loop

            except Exception as e:
                print(f"Unexpected error occurred: {e}")
                time.sleep(1)  # Sleep for a bit before retrying the loop

if __name__ == "__main__":
    main()
