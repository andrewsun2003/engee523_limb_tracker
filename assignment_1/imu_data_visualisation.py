import serial 
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import time
import csv


SERIAL_PORT = "COM19"
BAUD_RATE = 115200
OUTPUT_FILE = "imu_data.csv"

max_len = 200
time_vals = deque(maxlen=max_len)
pitch_vals = deque(maxlen=max_len)
roll_vals = deque(maxlen=max_len)
yaw_vals = deque(maxlen=max_len)

ser = serial.Serial(SERIAL_PORT, BAUD_RATE)

fig, ax = plt.subplots()
line_pitch, = ax.plot([], [], label='Pitch')
line_roll, = ax.plot([], [], label='Roll')
line_yaw, = ax.plot([], [], label='Yaw')

ax.set_ylim(-180, 180)
ax.set_xlim(0, max_len)
ax.set_xlabel('Time (s)')
ax.set_ylabel('Measurement (°)')
ax.set_title('Raw Data')
ax.legend()
ax.grid()


def update(frame):
    while ser.in_waiting:
        
        line = ser.readline().decode().strip()
        parts = line.split('\t')

        if len(parts) == 4:
            try:

                t = float(parts[0])
                pitch = float(parts[1])
                roll = float(parts[2])
                yaw = float(parts[3])
                
                time_vals.append(t)
                pitch_vals.append(pitch)
                roll_vals.append(roll)
                yaw_vals.append(yaw)

                line_pitch.set_data(range(len(pitch_vals)), pitch_vals)
                line_roll.set_data(range(len(roll_vals)), roll_vals)
                line_yaw.set_data(range(len(yaw_vals)), yaw_vals)

            except ValueError:
                pass
    
    return line_pitch, line_roll, line_yaw

ani = animation.FuncAnimation(fig, update, interval=50)
plt.show()



# with open(OUTPUT_FILE, mode='w', newline='') as file:
#     writer = csv.writer(file)
#     writer.writerow(["Timestamp", "ACCEL_ROLL", "ACCEL_PITCH"])
    
#     print(f"Logging data to {OUTPUT_FILE}... Press Ctrl+C to stop.")

#     try:
#         while True:
#             line = ser.readline().decode().strip()
#             if line:
#                 print(line)
#                 data = line.split("\t")

#                 if len(data) == 3:
#                     writer.writerow(data)
    
#     except KeyboardInterrupt:
#         print("\nData logging stopped.")
#         ser.close()
