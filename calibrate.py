# Program: Calibrate_H3LIS331_Orientation.py
# Author: T.Kalvin
# Project: PULSAR LIFTS H3LIS331 Calibration with Orientation
# Sub-system Team: Electronics And Programming
# Team: UnityRise

import time
import board
import adafruit_lis331
import numpy as np
import pickle  # Import pickle to store offsets in a binary file

# Initialise I2C and sensor
i2c = board.I2C()
sensor = adafruit_lis331.H3LIS331(i2c)
sensor.range = adafruit_lis331.H3LIS331Range.RANGE_100G

# Function to collect sample data for calibration in each orientation
def collect_samples(orientation_name, samples=500):
    print(f"\nPlace the sensor in the {orientation_name} orientation.")
    input(f"Press Enter to begin collecting {samples} samples...")
    x_samples, y_samples, z_samples = [], [], []
    
    for _ in range(samples):
        x, y, z = sensor.acceleration
        x_samples.append(x)
        y_samples.append(y)
        z_samples.append(z)
        time.sleep(0.01)  # 10ms delay between samples for stability

    # Calculate the average offset for each axis
    x_offset = np.mean(x_samples)
    y_offset = np.mean(y_samples)
    z_offset = np.mean(z_samples)

    print(f"Offsets for {orientation_name}: X: {x_offset:.3f} g, Y: {y_offset:.3f} g, Z: {z_offset:.3f} g")
    return x_offset, y_offset, z_offset

# Function to show orientation instructions with a delay
def show_orientation_instruction(orientation_name, delay=5):
    print(f"\nYou need to place the sensor in the {orientation_name} orientation.")
    print(f"Hold the sensor still in this position for {delay} seconds...")
    time.sleep(delay)

# Perform calibration in each orientation and get the offsets
def calibrate_sensor():
    orientations = [
        ("Z-axis pointing up", 5),
        ("Z-axis pointing down", 5),
        ("X-axis pointing up", 5),
        ("X-axis pointing down", 5),
        ("Y-axis pointing up", 5),
        ("Y-axis pointing down", 5)
    ]
    
    offsets = {}
    for orientation, delay in orientations:
        show_orientation_instruction(orientation, delay)
        offsets[orientation] = collect_samples(orientation)

    # Calculate average offsets for each axis
    x_offset = np.mean([offsets[o][0] for o in offsets])
    y_offset = np.mean([offsets[o][1] for o in offsets])
    z_offset = np.mean([offsets[o][2] for o in offsets])

    print(f"\nOverall Calibration Offsets: X: {x_offset:.3f} g, Y: {y_offset:.3f} g, Z: {z_offset:.3f} g")
    return x_offset, y_offset, z_offset

# Save offsets to a binary file
def save_offsets_to_bin(x_offset, y_offset, z_offset, filename='offsets.bin'):
    with open(filename, 'wb') as file:
        pickle.dump((x_offset, y_offset, z_offset), file)
    print(f"Calibration offsets saved to {filename}")

# Perform the calibration and save offsets
x_offset, y_offset, z_offset = calibrate_sensor()

# Debugging: Confirm offsets before saving
print(f"Offsets saved: X: {x_offset}, Y: {y_offset}, Z: {z_offset}")

# Save to binary file
save_offsets_to_bin(x_offset, y_offset, z_offset)
