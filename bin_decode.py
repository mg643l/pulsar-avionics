import struct
import csv

# Define updated binary format
binary_format = "f 3f 3f 3f 3f f f f f f"
data_size = struct.calcsize(binary_format)

# Open binary file and convert to CSV
with open("data.bin", "rb") as bin_file, open("data.csv", "w", newline="") as csv_file:
    writer = csv.writer(csv_file)

    # Write header row to CSV, including GPS data fields
    writer.writerow([
        "Time", 
        "H3LIS_X", "H3LIS_Y", "H3LIS_Z",
        "MPU6050_Accel_X", "MPU6050_Accel_Y", "MPU6050_Accel_Z",
        "MPU6050_Gyro_X", "MPU6050_Gyro_Y", "MPU6050_Gyro_Z",
        "Pressure", "Altitude", "Temperature", "Vertical_Speed",
        "GPS_Latitude", "GPS_Longitude", "GPS_Speed", "GPS_Course"
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

print("Data successfully converted to data.csv")
