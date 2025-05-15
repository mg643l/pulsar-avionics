import struct
import csv
import os

# Define updated binary format
binary_format = "f 3f 3f 3f 4f 4f f"
data_size = struct.calcsize(binary_format)

# Ask user for the filename
bin_filename = input("Enter the .bin filename to decode (e.g., data_YYYYMMDD_HHMMSS.bin): ").strip()

# Check if file exists
if not os.path.isfile(bin_filename):
    print(f"Error: File '{bin_filename}' not found.")
    exit(1)

csv_filename = bin_filename.replace('.bin', '.csv')

# Open binary file and convert to CSV
with open(bin_filename, "rb") as bin_file, open(csv_filename, "w", newline="") as csv_file:
    writer = csv.writer(csv_file)

    # Write header row to CSV, including GPS data fields
    writer.writerow([
        "Time", 
        "H3LIS_X", "H3LIS_Y", "H3LIS_Z",
        "MPU6050_Accel_X", "MPU6050_Accel_Y", "MPU6050_Accel_Z",
        "MPU6050_Gyro_X", "MPU6050_Gyro_Y", "MPU6050_Gyro_Z",
        "Pressure", "Altitude", "Temperature", "Vertical_Speed",
        "GPS_Latitude", "GPS_Longitude", "GPS_Speed", "GPS_Course",
        "Phase"
    ])

    # Read binary data and unpack into CSV format
    while True:
        data = bin_file.read(data_size)
        if not data:
            break  # End of file

        # Unpack the binary data
        unpacked_data = struct.unpack(binary_format, data)

        # Write the unpacked data to the CSV file
        writer.writerow(unpacked_data)

print(f"Data successfully converted to {csv_filename}")
