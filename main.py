import subprocess
import time
import signal
import sys
import board  # type: ignore
import struct
import pickle
import serial  # type: ignore
import adafruit_bmp3xx  # type: ignore
import adafruit_lis331  # type: ignore
import adafruit_mpu6050  # type: ignore
import busio  # type: ignore
import digitalio  # type: ignore
import os
from datetime import datetime
import psutil # type: ignore
import threading

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

PHASE_NAMES = [
    "On Launchpad",
    "Launched",
    "Motor Burnout",
    "Apogee",
    "Landed"
]

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

# For manual phase advancement
phase_lock = threading.Lock()
manual_phase = 0

def advance_phase_on_keypress():
    global manual_phase
    while manual_phase < 4:
        input("Press Enter to advance to the next phase...")
        with phase_lock:
            manual_phase += 1

# Override flight_phase to use manual_phase
def flight_phase():
    with phase_lock:
        return manual_phase

# Override flight_milestone to do nothing
def flight_milestone(y_accel, current_altitude, previous_altitude):
    pass

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
    sys.exit(0)

# Handles post-landing operations
def auto_shutdown():
    global landed, landing_counter, packet_buffer

    print("Landing detected! Entering post-landing procedure.")
    landing_timer_start = time.time()

    while True:
        # Wait for 60 seconds while checking if landing is still true
        if not landed:
            print("Landing condition no longer true. Resetting timer.")
            return  # Exit the function

        elapsed_time = time.time() - landing_timer_start
        if elapsed_time >= 60:
            print("Landing confirmed after 60 seconds. Proceeding with shutdown.")
            break

        time.sleep(1)  # Check every second

    # Stop writing to the file and flush the buffer
    if packet_buffer:
        with open(data_filename, "ab") as bin_file:
            bin_file.write(b''.join(packet_buffer))
        packet_buffer = []

    # Stop video recording
    camera_interrupt(signal.SIGINT, None)
    print("Video recording stopped and data saved successfully.")

    # Turn off pi
    os.system("sudo shutdown -h now")

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

# Start the keypress thread
keypress_thread = threading.Thread(target=advance_phase_on_keypress, daemon=True)
keypress_thread.start()

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

            vertical_speed = calculate_vertical_speed(altitude, previous_altitude, current_time, previous_time)

        except:
            led.value = True
            pressure = altitude = temperature = vertical_speed = 0.0

        # Read MPU6050
        try:
            ax, ay, az = mpu.acceleration
            gx, gy, gz = mpu.gyro
        except:
            led.value = True
            ax = ay = az = gx = gy = gz = 0.0

        # Update phase
        phase = flight_phase()

        previous_altitude = altitude
        previous_time = current_time

        # If landing is detected, call the handle_landing function
        if phase == 4:
            auto_shutdown()

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

        # Pack binary - Updated to include system metrics
        # We changed the format string to add 4 more float values (4f) at the end
        data_packet = struct.pack(
            "f 3f 3f 3f 4f 4f f 4f",  # Added 4f for the system metrics
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

        # Print only the current phase
        print(f"Current Phase: {PHASE_NAMES[phase]}")

        # Timing control
        elapsed = time.time() - loop_start
        if elapsed < SAMPLE_INTERVAL:
            time.sleep(SAMPLE_INTERVAL - elapsed)
