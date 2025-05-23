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
from datetime import datetime
import psutil 
import pigpio
from nrf24 import *

# Constants
BUFFER_SIZE = 50
TARGET_FREQ = 30
SAMPLE_INTERVAL = 1.0 / TARGET_FREQ
AIR_PRESSURE = 1013.25
LAUNCH_THRESHOLD = 50.0
BURNOUT_THRESHOLD = 20.0
BURNOUT_SAMPLE_COUNT = 3
APOGEE_SAMPLE_COUNT = 5
LANDING_SAMPLE_COUNT = 10
LANDING_THRESHOLD = 2.0
ALTITUDE_AVG_WINDOW = 10 
altitude_buffer = []

# Flight milestone boolean flags
on_launchpad = True
launched = False
motor_burnout = False
apogee_detected = False
landed = False

# Counters for consecutive samples
burnout_counter = 0
apogee_counter = 0
landing_counter = 0

# --- NRF24L01 Setup and Send Functions ---
def setup_nrf24():
    pi = pigpio.pi()
    nrf = NRF24(
        pi, ce=25, payload_size=RF24_PAYLOAD.DYNAMIC, channel=100,
        data_rate=RF24_DATA_RATE.RATE_250KBPS, pa_level=RF24_PA.LOW
    )
    nrf.set_address_bytes(5)
    nrf.open_writing_pipe("1SNSR")
    nrf.show_registers()
    return pi, nrf

def send_telemetry(nrf, program_time, vertical_speed, altitude, phase):
    try:
        payload = struct.pack(
            "<Bffff",
            0x10,
            float(program_time),
            float(vertical_speed),
            float(altitude),
            float(phase)
        )
        nrf.reset_packages_lost()
        nrf.send(payload)
        try:
            nrf.wait_until_sent()
            print("Telemetry sent via NRF24L01")
        except TimeoutError:
            print("Timeout sending telemetry via NRF24L01")
    except Exception as e:
        print(f"NRF24L01 telemetry send error: {e}")

def send_gps(nrf, gps_data):
    try:
        payload = struct.pack(
            "<Bffff",
            0x11,
            float(gps_data['latitude']),
            float(gps_data['longitude']),
            float(gps_data['speed']),
            float(gps_data['course'])
        )
        nrf.reset_packages_lost()
        nrf.send(payload)
        try:
            nrf.wait_until_sent()
            print("GPS sent via NRF24L01")
        except TimeoutError:
            print("Timeout sending GPS via NRF24L01")
    except Exception as e:
        print(f"NRF24L01 GPS send error: {e}")

# Calculate altitude from pressure using the barometric formula
def calculate_altitude(pressure, ground_pressure=AIR_PRESSURE):
    return 44330.0 * (1.0 - (pressure / ground_pressure) ** (1 / 5.255))

# Calculate vertical speed based on the change in altitude and time.
def calculate_vertical_speed(current_altitude, previous_altitude, current_time, previous_time):
    delta_time = current_time - previous_time
    if delta_time > 0:  # Avoid division by zero
        vertical_speed = (current_altitude - previous_altitude) / delta_time
    else:
        vertical_speed = 0.0
    return vertical_speed

# Detect flight milestones based on acceleration and altitude
def flight_milestone(y_accel, current_altitude, previous_altitude):
    global on_launchpad, launched, motor_burnout, burnout_counter, apogee_detected, apogee_counter, landed, landing_counter

    # Detect launch
    if on_launchpad and not launched and y_accel > LAUNCH_THRESHOLD:
        on_launchpad = False
        launched = True
        print("Launch detected! on_launchpad=False, launched=True")

    # Detect motor burnout
    if launched and not motor_burnout:
        # Check if the acceleration is below the threshold for a certain number of samples
        if y_accel < BURNOUT_THRESHOLD:
            burnout_counter += 1  # Increment counter if condition is met
            if burnout_counter >= BURNOUT_SAMPLE_COUNT:
                motor_burnout = True
                print("Motor burnout detected! motor_burnout=True")
        else:
            burnout_counter = 0  # Reset counter if condition is not met

    # Detect apogee
    if launched and not apogee_detected:
        if current_altitude < previous_altitude:
            # Check if the rocket is descending for a certain number of samples
            apogee_counter += 1  # Increment counter if condition is met
            if apogee_counter >= APOGEE_SAMPLE_COUNT:
                apogee_detected = True
                print("Apogee detected! apogee_detected=True")
        else:
            apogee_counter = 0  # Reset counter if condition is not met

    # Detect landing
    if launched and apogee_detected and not landed:
        if abs(current_altitude - previous_altitude) <= LANDING_THRESHOLD:
            landing_counter += 1
            if landing_counter >= LANDING_SAMPLE_COUNT:
                landed = True
                print("Landing detected! landed=True")
        else:
            landing_counter = 0

# Determine phase number
# -1: Unknown, 0: on launchpad, 1: launched, 2: motor burnout, 3: apogee, 4: landing
def flight_phase():
    if on_launchpad:
        current_phase = 0
    elif launched and not motor_burnout:
        current_phase = 1
    elif motor_burnout and not apogee_detected:
        current_phase = 2
    elif apogee_detected and not landed:
        current_phase = 3
    elif landed:
        current_phase = 4
    else:
        current_phase = -1  # Unknown phase

    return current_phase

# Load calibration offsets from a file
def load_offsets(filename="offsets.bin"):
    try:
        with open(filename, "rb") as f:
            return pickle.load(f)
    except FileNotFoundError:
        led.value = True
        print("Error: Calibration file not found. Run calibration first.")
        sys.exit(1)

# Convert GPS coordinates from degrees and minutes to decimal format
def convert_to_decimal(degrees_minutes, direction):
    degrees = int(degrees_minutes[:len(degrees_minutes) - 7])
    minutes = float(degrees_minutes[len(degrees_minutes) - 7:])
    decimal = degrees + (minutes / 60)
    return -decimal if direction in ["S", "W"] else decimal

# Handle Ctrl+C to stop video recording
def camera_interrupt(sig, frame):
    print("\nInterrupt received, stopping recording...")
    video_process.terminate()
    video_process.wait()
    print("Video successfully recorded in H.264 format!")
    
    # Turn off pi
    #os.system("sudo shutdown -h now")
    sys.exit(0)  

# Handles post-landing operations
def auto_shutdown():
    global landed, landing_counter, packet_buffer

    print("Landing detected! Entering post-landing procedure.")
    landing_timer_start = time.time()

    # Stop writing to the file and flush the buffer
    if packet_buffer:
        with open(data_filename, "ab") as bin_file:
            bin_file.write(b''.join(packet_buffer))
        packet_buffer = []

    # Stop video recording
    camera_interrupt(signal.SIGINT, None)

# Initialise I2C
i2c = busio.I2C(board.SCL, board.SDA)

# Setup status LED
led = digitalio.DigitalInOut(board.D27)
led.direction = digitalio.Direction.OUTPUT
led.value = False

# Initialise sensors
bmp = adafruit_bmp3xx.BMP3XX_I2C(i2c)
mpu = adafruit_mpu6050.MPU6050(i2c)
sensor = adafruit_lis331.H3LIS331(i2c)
sensor.range = adafruit_lis331.H3LIS331Range.RANGE_100G

# Load calibration
x_offset, y_offset, z_offset = load_offsets()
print(f"Loaded Offsets: X={x_offset}, Y={y_offset}, Z={z_offset}")

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
video_filename = f"video_{datetime.now().strftime('%Y%m%d_%H%M%S')}.h264"

# Start video recording
video_process = subprocess.Popen(
    ['libcamera-vid', '--framerate', str(framerate),
     '--width', str(width), '--height', str(height),
     '--rotation', '180', '--timeout', '0',
     '-o', video_filename],
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE
)

print("Recording video... Press Ctrl+C to stop.")

# Handle Ctrl+C
signal.signal(signal.SIGINT, camera_interrupt)

# Prepare GPS default
gps_last_data = {
    'latitude': 0.0, 'longitude': 0.0,
    'speed': 0.0, 'course': 0.0,
    'time': '00:00:00', 'date': '00/00/00'
}

# For vertical speed calculation
previous_altitude = 0.0
previous_time = time.time()

phase = -1  # Unknown phase

# Open binary file for logging
packet_buffer = []
start_time = time.time()

# Generate unique filename for each run
data_filename = f"data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.bin"

# --- NRF24L01 Setup ---
pi, nrf = setup_nrf24()
telemetry_send_counter = 0
last_gps_sent = {'latitude': 0.0, 'longitude': 0.0, 'speed': 0.0, 'course': 0.0}

# Main loop
with open(data_filename, "wb") as bin_file:
    while True:
        loop_start = time.time()
        program_time = time.time() - start_time

        # Calculate current absolute time
        current_time = time.time()

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

            # --- Moving average filter for altitude ---
            altitude_buffer.append(altitude)
            if len(altitude_buffer) > ALTITUDE_AVG_WINDOW:
                altitude_buffer.pop(0)
            smoothed_altitude = sum(altitude_buffer) / len(altitude_buffer)

            vertical_speed = calculate_vertical_speed(
                smoothed_altitude, 
                previous_altitude, 
                current_time, 
                previous_time
            )
        except:
            led.value = True
            pressure = altitude = temperature = vertical_speed = 0.0
            smoothed_altitude = altitude  # fallback

        # Read MPU6050
        try:
            ax, ay, az = mpu.acceleration
            gx, gy, gz = mpu.gyro
        except:
            led.value = True
            ax = ay = az = gx = gy = gz = 0.0

        # Update flight milestone
        flight_milestone(y_cal, altitude, previous_altitude)

        # Update phase
        phase = flight_phase()

        # Use original altitude for logging and storage, but vertical_speed is now smoothed
        previous_altitude = smoothed_altitude
        previous_time = current_time

        # Track how long we've been in phase 4 (landed)
        if phase == 4:
            if landed_time is None:
                landed_time = time.time()
            elif time.time() - landed_time >= 10:
                auto_shutdown()
        else:
            landed_time = None  # Reset if not in phase 4

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

        # Get system metrics
        try:
            # CPU/Memory/Disk Usage
            with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
                cpu_temp = int(f.read()) / 1000.0
            cpu_usage = psutil.cpu_percent(interval=None)
            memory = psutil.virtual_memory()
            memory_usage = memory.percent
            disk = psutil.disk_usage("/")
            disk_usage = disk.percent
        except Exception as e:
            # Set default values if there's an error
            cpu_temp = cpu_usage = memory_usage = disk_usage = 0.0

        # Pack binary data
        data_packet = struct.pack(
            "f 3f 3f 3f 4f 4f f 4f",  
            program_time,
            x_cal, y_cal, z_cal,
            ax, ay, az,
            gx, gy, gz,
            pressure, altitude, temperature,
            vertical_speed,
            gps_last_data['latitude'], gps_last_data['longitude'],
            gps_last_data['speed'], gps_last_data['course'],
            phase,
            cpu_temp, cpu_usage, memory_usage, disk_usage  # Added system metrics
        )

        packet_buffer.append(data_packet)

        # Flush buffer
        if len(packet_buffer) >= BUFFER_SIZE:
            bin_file.write(b''.join(packet_buffer))
            packet_buffer = []

        # NRF24L01 telemetry transmission
        telemetry_send_counter += 1
        if telemetry_send_counter >= 5:
            send_telemetry(nrf, program_time, vertical_speed, altitude, phase)
            telemetry_send_counter = 0

        # Send GPS as soon as a new fix is received
        if (
            gps_last_data['latitude'] != 0.0 or gps_last_data['longitude'] != 0.0
        ) and (
            gps_last_data != last_gps_sent
        ):
            send_gps(nrf, gps_last_data)
            last_gps_sent = gps_last_data.copy()

        # Print variables
        print(f"Time: {program_time:.3f}s")
        print(f"H3LIS Accel: X={x_cal:.2f} Y={y_cal:.2f} Z={z_cal:.2f}")
        print(f"MPU6050 Accel: X={ax:.2f} Y={ay:.2f} Z={az:.2f}")
        print(f"MPU6050 Gyro: X={gx:.2f} Y={gy:.2f} Z={gz:.2f}")
        print(f"Pressure: {pressure:.2f} hPa, Altitude: {altitude:.2f} m, Temp: {temperature:.2f} °C")
        print(f"Vertical Speed: {vertical_speed:.2f} m/s")
        print(f"GPS: Lat={gps_last_data['latitude']}, Lon={gps_last_data['longitude']}, "
              f"Speed={gps_last_data['speed']} knots, Course={gps_last_data['course']}°")
        print(f"Phase: {phase}")
        print(f"CPU Temp: {cpu_temp:.1f}°C | CPU Usage: {cpu_usage:.1f}% | "
              f"Memory Usage: {memory_usage:.1f}% | Disk Usage: {disk_usage:.1f}%")

        # Timing control
        elapsed = time.time() - loop_start
        if elapsed < SAMPLE_INTERVAL:
            time.sleep(SAMPLE_INTERVAL - elapsed)

# Stop daemon cleanly
nrf.power_down()
pi.stop()
