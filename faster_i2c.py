import subprocess
import time
import signal
import sys
import board
import threading
import struct
import pickle
import serial
import os
import busio
import digitalio
from concurrent.futures import ThreadPoolExecutor

# Constants
BUFFER_SIZE = 50  # Number of packets to buffer before file write
TARGET_HZ = 50    # Target logging frequency
SAMPLE_INTERVAL = 1.0 / TARGET_HZ

# Initialize I2C bus at higher speed
i2c = busio.I2C(board.SCL, board.SDA)

# Initialize LED (disable if not needed)
led = digitalio.DigitalInOut(board.D27)
led.direction = digitalio.Direction.OUTPUT
led.value = False

# Initialize sensors
bmp = adafruit_bmp3xx.BMP3XX_I2C(i2c)
mpu = adafruit_mpu6050.MPU6050(i2c)
sensor = adafruit_lis331.H3LIS331(i2c)
sensor.range = adafruit_lis331.H3LIS331Range.RANGE_100G

# Video recording settings
width = 1280
height = 720
framerate = 30

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

# GPS functions
def convert_to_decimal(degrees_minutes, direction):
    degrees = int(degrees_minutes[:len(degrees_minutes)-7])
    minutes = float(degrees_minutes[len(degrees_minutes)-7:])
    decimal = degrees + (minutes / 60)
    if direction in ["S", "W"]:
        decimal *= -1
    return decimal

# Initialize GPS
try:
    uart = serial.Serial("/dev/serial0", baudrate=9600, timeout=0.1)
except Exception as e:
    print(f"Error opening serial port: {e}")
    led.value = True
    exit(1)

# Sensor reading functions for parallel execution
def read_accel():
    try:
        return sensor.acceleration
    except:
        return (0.0, 0.0, 0.0)

def read_bmp():
    try:
        return (bmp.pressure, bmp.altitude, bmp.temperature)
    except:
        return (0.0, 0.0, 0.0)

def read_mpu():
    try:
        return (mpu.acceleration, mpu.gyro)
    except:
        return ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))

def read_gps():
    gps_data = {'latitude': 0.0, 'longitude': 0.0, 
                'speed': 0.0, 'course': 0.0, 
                'time': '00:00:00', 'date': '00/00/00'}
    
    data = b''
    while uart.in_waiting > 0:
        data += uart.read(uart.in_waiting)
        time.sleep(0.001)
    
    if b'$GPRMC' in data:
        lines = data.split(b'\n')
        for line in lines:
            if line.startswith(b'$GPRMC'):
                try:
                    fields = line.decode('ascii', errors='ignore').strip().split(',')
                    if fields[2] == 'A':  # Valid data
                        gps_data.update({
                            'time': fields[1],
                            'latitude': convert_to_decimal(fields[3], fields[4]),
                            'longitude': convert_to_decimal(fields[5], fields[6]),
                            'speed': float(fields[7]) if fields[7] else 0.0,
                            'course': float(fields[8]) if fields[8] else 0.0,
                            'date': fields[9] if len(fields) > 9 else '00/00/00'
                        })
                except:
                    pass
    return gps_data

# Video recording
def start_recording():
    return subprocess.Popen(
        ['libcamera-vid', '--framerate', str(framerate), 
         '--width', str(width), '--height', str(height),
         '--rotation', '180', '--timeout', '0', 
         '-o', 'FlightVideo.h264'],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE
    )

# Signal handler
def camera_interrupt(sig, frame):
    print("\nRecording stopped.")
    video_process.terminate()
    video_process.wait()
    print("Video successfully recorded in H.264 format!")
    sys.exit(0)

signal.signal(signal.SIGINT, camera_interrupt)

# Main program
if __name__ == "__main__":
    print("Starting data logger...")
    video_process = start_recording()
    print(f"Recording video at {width}x{height} {framerate}fps...")
    
    packet_buffer = []
    start_time = time.time()
    last_print = time.time()
    
    with open("data.bin", "wb") as bin_file, ThreadPoolExecutor(max_workers=4) as executor:
        while True:
            loop_start = time.time()
            
            # Read sensors in parallel
            accel_future = executor.submit(read_accel)
            bmp_future = executor.submit(read_bmp)
            mpu_future = executor.submit(read_mpu)
            gps_future = executor.submit(read_gps)
            
            # Get results
            x, y, z = accel_future.result()
            x_cal, y_cal, z_cal = x - x_offset, y - y_offset, z - z_offset
            pressure, altitude, temperature = bmp_future.result()
            (IMU_accel_x, IMU_accel_y, IMU_accel_z), (IMU_gyro_x, IMU_gyro_y, IMU_gyro_z) = mpu_future.result()
            gps_data = gps_future.result()
            
            # Calculate timestamp
            current_time = time.time() - start_time
            
            # Pack data
            data_packet = struct.pack(
                "f 3f 3f 3f 3f f f f f",
                current_time,
                x_cal, y_cal, z_cal,
                IMU_accel_x, IMU_accel_y, IMU_accel_z,
                IMU_gyro_x, IMU_gyro_y, IMU_gyro_z,
                pressure, altitude, temperature,
                gps_data['latitude'], gps_data['longitude'], 
                gps_data['speed'], gps_data['course']
            )
            
            # Buffer and write
            packet_buffer.append(data_packet)
            if len(packet_buffer) >= BUFFER_SIZE:
                bin_file.write(b''.join(packet_buffer))
                packet_buffer = []
            
            # Occasional status print
            if time.time() - last_print >= 1.0:
                print(f"Logging at {len(packet_buffer)/BUFFER_SIZE*TARGET_HZ:.1f} Hz")
                last_print = time.time()
            
            # Sleep for consistent rate
            elapsed = time.time() - loop_start
            if elapsed < SAMPLE_INTERVAL:
                time.sleep(SAMPLE_INTERVAL - elapsed)