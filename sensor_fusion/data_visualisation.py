import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import csv

# === Configuration ===
SERIAL_PORT = "/dev/cu.usbserial-130"
BAUD_RATE = 115200
OUTPUT_FILE = "imu_data.csv"
MAX_LEN = 200  # Maximum number of data points to display in plots

# === Setup serial ===
ser = serial.Serial(SERIAL_PORT, BAUD_RATE)

# === Data storage ===
time_vals = deque(maxlen=MAX_LEN)

# Raw sensor data
raw_roll = deque(maxlen=MAX_LEN)
raw_pitch = deque(maxlen=MAX_LEN)
raw_yaw = deque(maxlen=MAX_LEN)

# EKF data
ekf_roll = deque(maxlen=MAX_LEN)
ekf_pitch = deque(maxlen=MAX_LEN)
ekf_yaw = deque(maxlen=MAX_LEN)

# DMP data
dmp_roll = deque(maxlen=MAX_LEN)
dmp_pitch = deque(maxlen=MAX_LEN)
dmp_yaw = deque(maxlen=MAX_LEN)

# === Plot setup ===
fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

# Labels
titles = ["Raw Sensor Data", "EKF Sensor Fusion Output", "DMP Output"]
colors = ['r', 'g', 'b']

# Plot lines
lines = []
for ax, title in zip(axs, titles):
    ax.set_title(title)
    ax.set_ylim(-180, 180)
    ax.set_ylabel("Degrees")
    ax.grid(True)
    roll_line, = ax.plot([], [], color='r', label='Roll')
    pitch_line, = ax.plot([], [], color='g', label='Pitch')
    yaw_line, = ax.plot([], [], color='b', label='Yaw')
    lines.append((roll_line, pitch_line, yaw_line))
axs[-1].set_xlabel("Time (s)")
axs[0].legend(loc='upper right')


def init():
    for line_group in lines:
        for line in line_group:
            line.set_data([], [])
    return [line for group in lines for line in group]


def update(frame):
    try:
        line = ser.readline().decode('utf-8').strip()
        parts = line.split('\t')

        if len(parts) == 10:
            t = float(parts[0])
            raw_r, raw_p, raw_y = map(float, parts[1:4])
            ekf_r, ekf_p, ekf_y = map(float, parts[4:7])
            dmp_r, dmp_p, dmp_y = map(float, parts[7:10])

            # Append to deques
            time_vals.append(t)
            raw_roll.append(raw_r)
            raw_pitch.append(raw_p)
            raw_yaw.append(raw_y)

            ekf_roll.append(ekf_r)
            ekf_pitch.append(ekf_p)
            ekf_yaw.append(ekf_y)

            dmp_roll.append(dmp_r)
            dmp_pitch.append(dmp_p)
            dmp_yaw.append(dmp_y)

            # Update plots
            data_sets = [
                (raw_roll, raw_pitch, raw_yaw),
                (ekf_roll, ekf_pitch, ekf_yaw),
                (dmp_roll, dmp_pitch, dmp_yaw)
            ]

            for i in range(3):
                lines[i][0].set_data(time_vals, data_sets[i][0])  # Roll
                lines[i][1].set_data(time_vals, data_sets[i][1])  # Pitch
                lines[i][2].set_data(time_vals, data_sets[i][2])  # Yaw
                axs[i].set_xlim(max(0, time_vals[0]), time_vals[-1])

    except Exception as e:
        print("Error:", e)

    return [line for group in lines for line in group]


ani = animation.FuncAnimation(fig, update, init_func=init, interval=50, blit=True)
plt.tight_layout()
plt.show()
