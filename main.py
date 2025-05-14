import subprocess
import time
import signal
import sys
import board
import struct
import pickle
import serial
import adafruit_bmp3xx
import adafruit_lis331
import adafruit_mpu6050
import busio
import digitalio
import os

# Constants
BUFFER_SIZE = 50
TARGET_HZ = 50
SAMPLE_INTERVAL = 1.0 / TARGET_HZ
AIR_PRESSURE = 1013.25

# Calculate Altitude
def calculate_altitude(pressure, ground_pressure=AIR_PRESSURE):
    """
    Calculate altitude based on pressure and sea-level pressure using the barometric formula.
    """
    return 44330.0 * (1.0 - (pressure / ground_pressure) ** (1 / 5.255))

# Initialize I2C
i2c = busio.I2C(board.SCL, board.SDA)

# Setup status LED
led = digitalio.DigitalInOut(board.D27)
led.direction = digitalio.Direction.OUTPUT
led.value = False

# Initialize sensors
bmp = adafruit_bmp3xx.BMP3XX_I2C(i2c)
mpu = adafruit_mpu6050.MPU6050(i2c)
sensor = adafruit_lis331.H3LIS331(i2c)
sensor.range = adafruit_lis331.H3LIS331Range.RANGE_100G

# Load calibration
def load_offsets(filename="offsets.bin"):
    try:
        with open(filename, "rb") as f:
            return pickle.load(f)
    except FileNotFoundError:
        led.value = True
        print("Error: Calibration file not found. Run calibration first.")
        sys.exit(1)

x_offset, y_offset, z_offset = load_offsets()
print(f"Loaded Offsets: X={x_offset}, Y={y_offset}, Z={z_offset}")

# Convert GPS data
def convert_to_decimal(degrees_minutes, direction):
    degrees = int(degrees_minutes[:len(degrees_minutes) - 7])
    minutes = float(degrees_minutes[len(degrees_minutes) - 7:])
    decimal = degrees + (minutes / 60)
    return -decimal if direction in ["S", "W"] else decimal

# Open GPS serial
try:
    uart = serial.Serial("/dev/serial0", baudrate=9600, timeout=1)
    print("Serial port opened successfully")
except Exception as e:
    print(f"Error opening serial port: {e}")
    led.value = True
    sys.exit(1)

# Video settings
width, height, framerate = 1920, 1080, 30

# Start video recording
video_process = subprocess.Popen(
    ['libcamera-vid', '--framerate', str(framerate),
     '--width', str(width), '--height', str(height),
     '--rotation', '180', '--timeout', '0',
     '-o', 'TestFlightVideo.h264'],
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE
)

print("Recording video... Press Ctrl+C to stop.")

# Handle Ctrl+C
def camera_interrupt(sig, frame):
    print("\nInterrupt received, stopping recording...")
    video_process.terminate()
    video_process.wait()
    print("Video successfully recorded in H.264 format!")
    sys.exit(0)

signal.signal(signal.SIGINT, camera_interrupt)

# Prepare GPS default
gps_last_data = {
    'latitude': 0.0, 'longitude': 0.0,
    'speed': 0.0, 'course': 0.0,
    'time': '00:00:00', 'date': '00/00/00'
}

# Open binary file for logging
packet_buffer = []
start_time = time.time()

with open("data.bin", "wb") as bin_file:
    while True:
        loop_start = time.time()
        current_time = time.time() - start_time

        # Read H3LIS accelerometer
        try:
            x, y, z = sensor.acceleration
            x_cal, y_cal, z_cal = x - x_offset, y - y_offset, z - z_offset
        except:
            led.value = True
            x_cal = y_cal = z_cal = 0.0

        # Read BMP390
        try:
            pressure = bmp.pressure
            altitude = calculate_altitude(pressure)
            temperature = bmp.temperature
        except:
            led.value = True
            pressure = altitude = temperature = 0.0

        # Read MPU6050
        try:
            ax, ay, az = mpu.acceleration
            gx, gy, gz = mpu.gyro
        except:
            led.value = True
            ax = ay = az = gx = gy = gz = 0.0

        # Read GPS if available
        gps_data_received = False
        if uart.in_waiting > 0:
            line = uart.readline().decode('ascii', errors='ignore').strip()
            if line.startswith("$GPRMC"):
                fields = line.split(",")
                if fields[2] == "A":
                    gps_data_received = True
                    gps_last_data['time'] = fields[1]
                    gps_last_data['latitude'] = convert_to_decimal(fields[3], fields[4])
                    gps_last_data['longitude'] = convert_to_decimal(fields[5], fields[6])
                    gps_last_data['speed'] = float(fields[7]) if fields[7] else 0.0
                    gps_last_data['course'] = float(fields[8]) if fields[8] else 0.0
                    gps_last_data['date'] = fields[9]

        if not gps_data_received:
            gps_last_data = {key: 0.0 if key not in ('time', 'date') else '00:00:00' for key in gps_last_data}

        # Pack binary
        data_packet = struct.pack(
            "f 3f 3f 3f 3f f f f f",
            current_time,
            x_cal, y_cal, z_cal,
            ax, ay, az,
            gx, gy, gz,
            pressure, altitude, temperature,
            gps_last_data['latitude'], gps_last_data['longitude'],
            gps_last_data['speed'], gps_last_data['course']
        )

        packet_buffer.append(data_packet)

        # Flush buffer
        if len(packet_buffer) >= BUFFER_SIZE:
            bin_file.write(b''.join(packet_buffer))
            packet_buffer = []

        # Print live status
        print(f"Time: {current_time:.3f}s")
        print(f"H3LIS Accel: X={x_cal:.2f} Y={y_cal:.2f} Z={z_cal:.2f}")
        print(f"MPU6050 Accel: X={ax:.2f} Y={ay:.2f} Z={az:.2f}")
        print(f"MPU6050 Gyro: X={gx:.2f} Y={gy:.2f} Z={gz:.2f}")
        print(f"Pressure: {pressure:.2f} hPa, Altitude: {altitude:.2f} m, Temp: {temperature:.2f} °C")
        print(f"GPS: Lat={gps_last_data['latitude']}, Lon={gps_last_data['longitude']}, "
              f"Speed={gps_last_data['speed']} knots, Course={gps_last_data['course']}°")

        # Timing control
        elapsed = time.time() - loop_start
        if elapsed < SAMPLE_INTERVAL:
            time.sleep(SAMPLE_INTERVAL - elapsed)
