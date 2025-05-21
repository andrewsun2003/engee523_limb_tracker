import serial 
import time
import csv

SERIAL_PORT = "COM6"
BAUD_RATE = 115200
OUTPUT_FILE = "imu_data.csv"

ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
time.sleep(2)

with open(OUTPUT_FILE, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["Timestamp", "ACCEL_ROLL", "ACCEL_PITCH"])
    
    print(f"Logging data to {OUTPUT_FILE}... Press Ctrl+C to stop.")

    try:
        while True:
            line = ser.readline().decode().strip()
            if line:
                print(line)
                data = line.split("\t")

                if len(data) == 3:
                    writer.writerow(data)
    
    except KeyboardInterrupt:
        print("\nData logging stopped.")
        ser.close()
