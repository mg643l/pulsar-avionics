import struct
import csv

# Define binay format
binary_format = "f 3f 3f 3f"
data_size = struct.calcsize(binary_format)

# Open binary file and convert to CSV
with open("data.bin", "rb") as bin_file, open("data.csv", "w", newline="") as csv_file:
    writer = csv.writer(csv_file)

    # Write header row to CSV, including GPS data fields
    writer.writerow([
        "Time", 
        "H3LIS_X", "H3LIS_Y", "H3LIS_Z",
        "Accel_X", "Accel_Y", "Accel_Z", 
        "Gyro_X", "Gyro_Y", "Gyro_Z", 
        "Pressure", "Altitude", "Temperature", 
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
