import serial
import csv
from datetime import datetime

# Serial port configuration
ser = serial.Serial('/dev/cu.usbmodem131412701', 115200)  # Change to match your Arduino's serial port

# Create a CSV file
csv_filename = 'rpm_data_ppc.csv'
with open(csv_filename, 'w', newline='') as csvfile:
    csv_writer = csv.writer(csvfile)
    csv_writer.writerow(['Time (s)', 'RPM'])

    # Read data from serial port and write to CSV
    try:
        while True:
            serial_data = ser.readline().decode().strip()  # Read a line from serial port
            if serial_data:
                data = serial_data.split(',')  # Split the data by comma
                timestamp = float(data[0])  # Convert time to float
                rpm = float(data[1])  # Convert RPM to float
                csv_writer.writerow([timestamp, rpm])  # Write timestamp and RPM to CSV
                print(f"Time: {timestamp}, RPM: {rpm}")  # Print data to console
    except KeyboardInterrupt:
        print("Recording stopped by user.")

# Close serial port
ser.close()
