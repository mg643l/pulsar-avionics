import time
import board
import struct
import adafruit_bmp3xx
import adafruit_lis331
import adafruit_mpu6050
import pickle
import busio
import digitalio


# Initialize I2C bus
i2c = busio.I2C(board.SCL, board.SDA)

# Initialize sensors
bmp = adafruit_bmp3xx.BMP3XX_I2C(i2c)
mpu = adafruit_mpu6050.MPU6050(i2c)

sensor = adafruit_lis331.H3LIS331(i2c)
sensor.range = adafruit_lis331.H3LIS331Range.RANGE_100G

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

# Open binary file for writing
with open("data.bin", "wb") as bin_file:
    start_time = time.time()  # Timestamp start

    while True:
        current_time = time.time() - start_time  # Elapsed time

        # Read sensors
        try:
            x, y, z = sensor.acceleration
            x_cal, y_cal, z_cal = apply_offsets(x, y, z, x_offset, y_offset, z_offset)
        except:
            x_cal, y_cal, z_cal = 0.0, 0.0, 0.0

        # Read BMP390 data
        try:
            pressure, altitude, temperature = bmp.pressure, bmp.altitude, bmp.temperature
        except:
            pressure, altitude, temperature = 0.0, 0.0, 0.0
        
        # Read MPU6050 data
        try:
            IMU_accel_x, IMU_accel_y, IMU_accel_z = mpu.acceleration
            IMU_gyro_x, IMU_gyro_y, IMU_gyro_z = mpu.gyro
        except:
            IMU_accel_x, IMU_accel_y, IMU_accel_z = 0.0, 0.0, 0.0
            IMU_gyro_x, IMU_gyro_y, IMU_gyro_z = 0.0, 0.0, 0.0

        # Print the sensor values as they are read
        print(f"Time: {current_time:.6f}s")
        print(f"H3LIS Accel - X: {x_cal:.3f} Y: {y_cal:.3f} Z: {z_cal:.3f} m/s^2")
        print(f"MPU6050 Accel - X: {IMU_accel_x:.3f} Y: {IMU_accel_y:.3f} Z: {IMU_accel_z:.3f} m/s^2")
        print(f"MPU6050 Gyro - X: {IMU_gyro_x:.3f} Y: {IMU_gyro_y:.3f} Z: {IMU_gyro_z:.3f}")
        print(f"Pressure: {pressure:.3f} hPa Altitude: {altitude:.3f} meters Temperature: {temperature:.3f} °C")

        # Pack data for binary write
        data_packet = struct.pack(
            "f 3f 3f 3f 3f",
            current_time,       # Timestamp
            x_cal, y_cal, z_cal,  # H3LIS331 Calibrated Acceleration
            IMU_accel_x, IMU_accel_y, IMU_accel_z,  # MPU6050 Acceleration
            IMU_gyro_x, IMU_gyro_y, IMU_gyro_z,  # MPU6050 Gyroscope
            pressure, altitude, temperature,  # BMP390
        )

        # Write the packed data to the binary file
        bin_file.write(data_packet)

        # Sleep for 20ms (50 Hz logging)
        time.sleep(0.05)
