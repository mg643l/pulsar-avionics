#!/usr/bin/env python3

import serial
import struct
import os
import time
import pigpio
from nrf24 import *

# Initialize GPS fix status
has_fix = False

# Record the start time of the program
start_time = time.time()

# Function to convert latitude and longitude to decimal degrees
def convert_to_decimal(degrees_minutes, direction):
    degrees = int(degrees_minutes[:len(degrees_minutes)-7])  # Extract degrees
    minutes = float(degrees_minutes[len(degrees_minutes)-7:])  # Extract minutes
    decimal = degrees + (minutes / 60)  # Convert to decimal degrees
    if direction in ["S", "W"]:
        decimal *= -1  # Apply negative for South and West
    return decimal

# Initialize NRF24L01
pi = pigpio.pi()
nrf = NRF24(pi, ce=25, payload_size=RF24_PAYLOAD.DYNAMIC, channel=100, data_rate=RF24_DATA_RATE.RATE_250KBPS, pa_level=RF24_PA.LOW)
nrf.set_address_bytes(5)
nrf.open_writing_pipe("1SNSR")
nrf.show_registers()

# Open the serial port
try:
    uart = serial.Serial("/dev/serial0", baudrate=9600, timeout=1)
    print("Serial port opened successfully")

    while True:
        if uart.in_waiting > 0:
            data = uart.readline().decode('ascii', errors='ignore').strip()
            print(data)

            if data.startswith("$GPRMC"):
                fields = data.split(",")
                status = fields[2]
                if status == "A":
                    has_fix = True

                    # Parse data
                    time_str = fields[1]
                    latitude = convert_to_decimal(fields[3], fields[4])
                    longitude = convert_to_decimal(fields[5], fields[6])
                    speed = float(fields[7])
                    course = float(fields[8])
                    date = fields[9]
                    program_time = time.time() - start_time

                    print(f"\nTime: {time_str}, Status: {status}")
                    print(f"Latitude: {latitude}, Longitude: {longitude}")
                    print(f"Speed: {speed} knots, Course: {course}°")
                    print(f"Date: {date}")
                    print(f"Program Run Time: {program_time:.2f} seconds\n")

                    # Pack and send data via NRF24L01
                    try:
                        payload = struct.pack("<Bff", 0x02, latitude, longitude)
                        nrf.reset_packages_lost()
                        nrf.send(payload)
                        try:
                            nrf.wait_until_sent()
                            print("Data sent successfully via NRF24L01")
                        except TimeoutError:
                            print("Timeout waiting for transmission to complete.")
                    except Exception as e:
                        print(f"Error packing or sending data: {e}")

                elif status == "V":
                    has_fix = False

            if not has_fix:
                print("Waiting for GPS fix...")
                time.sleep(1)

            time.sleep(0.01)

except Exception as e:
    print(f"Error: {e}")
finally:
    nrf.power_down()
    pi.stop()
