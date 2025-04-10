import struct
import csv

# Define binay format
binary_format = "f 3f 4f 3f 3f f f f 3f"
data_size = struct.calcsize(binary_format)

# Open binary file and convert to CSV
with open("data.bin", "rb") as bin_file, open("data.csv", "w", newline="") as csv_file:
    writer = csv.writer(csv_file)

    # Write header row to CSV, including GPS data fields
    writer.writerow([
        "Time", 
        "Accel_X", "Accel_Y", "Accel_Z", 
        "Quat_I", "Quat_J", "Quat_K", "Quat_Real", 
        "Gyro_X", "Gyro_Y", "Gyro_Z", 
        "Mag_X", "Mag_Y", "Mag_Z", 
        "Pressure", "Altitude", "Temperature", 
        "H3LIS_X", "H3LIS_Y", "H3LIS_Z",
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
