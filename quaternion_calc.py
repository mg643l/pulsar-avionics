import time
import board
import struct
import adafruit_bmp3xx
import adafruit_lis331
import adafruit_mpu6050
import pickle
import busio
import digitalio
import math

# Madgwick filter implementation (Quaternion-based sensor fusion)
class MadgwickFilter:
    def __init__(self, beta=0.1):
        self.beta = beta  # filter gain
        self.q = [1.0, 0.0, 0.0, 0.0]  # quaternion representing the current orientation

    def update(self, ax, ay, az, gx, gy, gz, dt):
        q1, q2, q3, q4 = self.q
        norm = math.sqrt(ax * ax + ay * ay + az * az)
        if norm == 0:
            return  # invalid accelerometer data
        
        ax /= norm
        ay /= norm
        az /= norm

        # Gyroscope data (rad/s)
        gx = math.radians(gx)
        gy = math.radians(gy)
        gz = math.radians(gz)

        # Step 1: Compute the gradient descent algorithm to minimize the error
        f1 = 2.0 * (q2 * q4 - q1 * q3) - ax
        f2 = 2.0 * (q1 * q2 + q3 * q4) - ay
        f3 = 1.0 - 2.0 * (q2 * q2 + q3 * q3) - az
        J_11or24 = 2.0 * q3
        J_12or23 = 2.0 * q4
        J_13or22 = 2.0 * q1
        J_14or21 = 2.0 * q2
        J_32 = 2.0 * q2
        J_33 = 2.0 * q1

        # Step 2: Apply feedback step to adjust quaternion
        J_11 = -2.0 * q4
        J_22 = -2.0 * q3

        # Step 3: Use gyroscope integration to update quaternion
        q1 += (-q2 * gx - q3 * gy - q4 * gz) * dt
        q2 += (q1 * gx + q3 * gz - q4 * gy) * dt
        q3 += (q1 * gy - q2 * gz + q4 * gx) * dt
        q4 += (q1 * gz + q2 * gy - q3 * gx) * dt

        # Normalize quaternion
        norm = math.sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4)
        self.q = [q1 / norm, q2 / norm, q3 / norm, q4 / norm]

    def get_orientation(self):
        return self.q


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

# Initialize Madgwick filter
madgwick_filter = MadgwickFilter()

# Convert quaternion to roll, pitch, yaw (Euler Angles)
def quaternion_to_euler(q):
    w, x, y, z = q
    
    # Roll (rotation around X-axis)
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    
    # Pitch (rotation around Y-axis)
    pitch = math.asin(2.0 * (w * y - z * x))
    
    # Yaw (rotation around Z-axis)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    
    # Convert from radians to degrees for easier interpretation
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


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

        # Read MPU6050 gyroscope data
        try:
            IMU_accel_x, IMU_accel_y, IMU_accel_z = mpu.acceleration
            IMU_gyro_x, IMU_gyro_y, IMU_gyro_z = mpu.gyro
        except:
            IMU_accel_x, IMU_accel_y, IMU_accel_z = 0.0, 0.0, 0.0
            IMU_gyro_x, IMU_gyro_y, IMU_gyro_z = 0.0, 0.0, 0.0

        # Update Madgwick filter with sensor data
        madgwick_filter.update(x_cal, y_cal, z_cal, IMU_gyro_x, IMU_gyro_y, IMU_gyro_z, 0.05)

        # Get quaternion representing the orientation
        q = madgwick_filter.get_orientation()

        # Convert quaternion to roll, pitch, yaw
        roll, pitch, yaw = quaternion_to_euler(q)

        # Print the sensor values and Euler angles
        print(f"Time: {current_time:.6f}s")
        print(f"H3LIS Accel - X: {x_cal:.3f} Y: {y_cal:.3f} Z: {z_cal:.3f} m/s^2")
        print(f"MPU6050 Accel - X: {IMU_accel_x:.3f} Y: {IMU_accel_y:.3f} Z: {IMU_accel_z:.3f} m/s^2")
        print(f"MPU6050 Gyro - X: {IMU_gyro_x:.3f} Y: {IMU_gyro_y:.3f} Z: {IMU_gyro_z:.3f}")
        print(f"Euler Angles: Roll: {roll:.3f}° Pitch: {pitch:.3f}° Yaw: {yaw:.3f}°")

        # Pack data for binary write
        data_packet = struct.pack(
            "f 3f 3f 3f f f f f f",
            current_time,       # Timestamp
            x_cal, y_cal, z_cal,  # H3LIS331 Calibrated Acceleration
            IMU_accel_x, IMU_accel_y, IMU_accel_z,  # MPU6050 Acceleration
            IMU_gyro_x, IMU_gyro_y, IMU_gyro_z,  # MPU6050 Gyroscope
            q[0], q[1], q[2], q[3],  # Quaternion (w, x, y, z)
            roll, pitch, yaw  # Euler Angles (Roll, Pitch, Yaw)
        )

        # Write the packed data to the binary file
        bin_file.write(data_packet)

        # Sleep for 20ms (50 Hz logging)
        time.sleep(0.05)
