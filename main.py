import subprocess
import time
import signal
import sys
import board
import threading
import struct
import numpy as np
import adafruit_bmp3xx
import adafruit_lis331
import adafruit_mpu6050
import pickle
import serial
import os
import busio
import digitalio
import subprocess

try:
    subprocess.run(["sudo", "chmod", "666", "/dev/ttyS0"], check=True)
except subprocess.CalledProcessError:
    print("Failed to chmod /dev/ttyS0 — make sure user has sudo access.")

# Initialize I2C bus
i2c = busio.I2C(board.SCL, board.SDA)

led = digitalio.DigitalInOut(board.D27)
led.direction = digitalio.Direction.OUTPUT

# Initialize sensors
bmp = adafruit_bmp3xx.BMP3XX_I2C(i2c)
mpu = adafruit_mpu6050.MPU6050(i2c)

sensor = adafruit_lis331.H3LIS331(i2c)
sensor.range = adafruit_lis331.H3LIS331Range.RANGE_100G

# Set a practical resolution for video recording (1080p)
width = 1920  # Width for 1080p
height = 1080  # Height for 1080p

# Set framerate to 60fps
framerate = 60

# Function to handle the interrupt signal
def cameraInterrupt(sig, frame):
    print("Recording stopped.")
    process.terminate()  # Terminate the recording process
    process.wait()  # Wait for the process to finish
    print("Video Successfully Recorded in H.264 format!")
    sys.exit(0)

# Register the signal handler for graceful exit
signal.signal(signal.SIGINT, cameraInterrupt)

# Start recording video using libcamera-vid at 1080p resolution and specified framerate, output as H.264
process = subprocess.Popen(
    ['libcamera-vid', '--framerate', str(framerate), '--width', str(width), '--height', str(height), '-o', 'test60.h264'],
    stdout=subprocess.DEVNULL,  # Suppress standard output
    stderr=subprocess.DEVNULL   # Suppress standard error
)

print("Recording video... Press Ctrl+C to stop.")

# Load calibration offsets
def load_offsets(filename="offsets.bin"):
    try:
        with open(filename, "rb") as file:
            return pickle.load(file)
    except FileNotFoundError:
        led.value = True
        print("Error: Calibration file not found. Run calibration first.")
        exit(1)

x_offset, y_offset, z_offset = load_offsets()
print(f"Loaded Offsets: X: {x_offset}, Y: {y_offset}, Z: {z_offset}")

# Function to apply offsets
def apply_offsets(x, y, z, x_offset, y_offset, z_offset):
    return x - x_offset, y - y_offset, z - z_offset

# Function to convert latitude and longitude to decimal degrees
def convert_to_decimal(degrees_minutes, direction):
    degrees = int(degrees_minutes[:len(degrees_minutes)-7])  # Extract degrees
    minutes = float(degrees_minutes[len(degrees_minutes)-7:])  # Extract minutes
    decimal = degrees + (minutes / 60)  # Convert to decimal degrees
    if direction in ["S", "W"]:
        decimal *= -1  # Apply negative for South and West
    return decimal

# Open the serial port for GPS
try:
    uart = serial.Serial("/dev/serial0", baudrate=9600, timeout=1)
    print("Serial port opened successfully")
except Exception as e:
    print(f"Error: {e}")
    led.value = True
    exit(1)

# Open binary file for writing
with open("data.bin", "wb") as bin_file:
    start_time = time.time()  # Timestamp start
    gps_last_data = {'latitude': 0.0, 'longitude': 0.0, 'speed': 0.0, 'course': 0.0, 'time': '00:00:00', 'date': '00/00/00'}

    while True:
        led.value = False
        current_time = time.time() - start_time  # Elapsed time

        # Read sensors
        try:
            led.value = False  
            x, y, z = sensor.acceleration
            x_cal, y_cal, z_cal = apply_offsets(x, y, z, x_offset, y_offset, z_offset)
        except:
            led.value = True
            x_cal, y_cal, z_cal = 0.0, 0.0, 0.0

        # Read BMP390 data
        try:
            led.value = False  
            pressure, altitude, temperature = bmp.pressure, bmp.altitude, bmp.temperature
        except:
            led.value = True
            pressure, altitude, temperature = 0.0, 0.0, 0.0
        try:
            led.value = False  
            IMU_accel_x, IMU_accel_y, IMU_accel_z = mpu.acceleration
            IMU_gyro_x, IMU_gyro_y, IMU_gyro_z = mpu.gyro
        except:
            led.value = True
            IMU_accel_x, IMU_accel_y, IMU_accel_z = 0.0, 0.0, 0.0
            IMU_gyro_x, IMU_gyro_y, IMU_gyro_z = 0.0, 0.0, 0.0

        # Read GPS data
        gps_data_received = False
        if uart.in_waiting > 0:
            data = uart.readline().decode('ascii', errors='ignore').strip()  # Read received data

            # Check for Recommended Minimum Navigation Information 
            if data.startswith("$GPRMC"):
                fields = data.split(",")
                status = fields[2]
                if status == "A":  # Check if data is valid
                    gps_data_received = True
                    # Parse data
                    gps_last_data['time'] = fields[1]
                    gps_last_data['latitude'] = convert_to_decimal(fields[3], fields[4])  # Convert to decimal
                    gps_last_data['longitude'] = convert_to_decimal(fields[5], fields[6])  # Convert to decimal
                    gps_last_data['speed'] = float(fields[7])
                    gps_last_data['course'] = float(fields[8])
                    gps_last_data['date'] = fields[9]
        
        if not gps_data_received:
            # If no GPS fix, use the previous values (zero or last known values)
            gps_last_data = {key: 0.0 if key != 'time' and key != 'date' else '00:00:00' for key in gps_last_data}

        # Print the sensor values as they are read
        print(f"Time: {current_time:.6f}s")
        print(f"H3LIS Accel - X: {x_cal:.3f} Y: {y_cal:.3f} Z: {z_cal:.3f} m/s^2")
        print(f"MPU6050 Accel - X: {IMU_accel_x:.3f} Y: {IMU_accel_y:.3f} Z: {IMU_accel_z:.3f} m/s^2")
        print(f"MPU6050 Gyro - X: {IMU_gyro_x:.3f} Y: {IMU_gyro_y:.3f} Z: {IMU_gyro_z:.3f}")
        print(f"Pressure: {pressure:.3f} hPa Altitude: {altitude:.3f} meters Temperature: {temperature:.3f} °C")
        print(f"GPS - Time: {gps_last_data['time']}, Latitude: {gps_last_data['latitude']}, Longitude: {gps_last_data['longitude']}, Speed: {gps_last_data['speed']} knots, Course: {gps_last_data['course']}°")

        # Pack data for binary write
        data_packet = struct.pack(
            "f 3f 3f 3f 3f f f f f",
            current_time,       # Timestamp
            x_cal, y_cal, z_cal,  # H3LIS331 Calibrated Acceleration
            IMU_accel_x, IMU_accel_y, IMU_accel_z,  # MPU6050 Acceleration
            IMU_gyro_x, IMU_gyro_y, IMU_gyro_z,  # MPU6050 Gyroscope
            pressure, altitude, temperature,  # BMP390
            gps_last_data['latitude'], gps_last_data['longitude'], gps_last_data['speed'], gps_last_data['course']  # GPS Data
        )

        # Write the packed data to the binary file
        bin_file.write(data_packet)

        # Sleep for 20ms (50 Hz logging)
        led.value = False 
        time.sleep(0.05)
