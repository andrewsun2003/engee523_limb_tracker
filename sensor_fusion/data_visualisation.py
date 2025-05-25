import serial 
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque


SERIAL_PORT = "COM6"
BAUD_RATE = 115200
OUTPUT_FILE = "imu_data.csv"


ser = serial.Serial(SERIAL_PORT, BAUD_RATE)

max_len = 200

time_vals = deque(maxlen=max_len)

raw_roll = deque(maxlen=max_len)
raw_pitch = deque(maxlen=max_len)
raw_yaw = deque(maxlen=max_len)

ekf_roll = deque(maxlen=max_len)
ekf_pitch = deque(maxlen=max_len)
ekf_yaw = deque(maxlen=max_len)

dmp_roll = deque(maxlen=max_len)
dmp_pitch = deque(maxlen=max_len)
dmp_yaw = deque(maxlen=max_len)

fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 9), sharex=True)


def setup_axis(ax, label):
    ax.set_ylim(-180, 180)
    ax.set_xlim(0, max_len)
    ax.set_ylabel("°")
    ax.set_title(label)
    ax.grid()
    ax.legend()

# Plot lines
lines = {
    'raw': {
        'roll': ax1.plot([], [], 'r-', label='Raw Roll')[0],
        'pitch': ax1.plot([], [], 'g-', label='Raw Pitch')[0],
        'yaw': ax1.plot([], [], 'b-', label='Raw Yaw')[0],
    },
    'ekf': {
        'roll': ax2.plot([], [], 'r-', label='EKF Roll')[0],
        'pitch': ax2.plot([], [], 'g-', label='EKF Pitch')[0],
        'yaw': ax2.plot([], [], 'b-', label='EKF Yaw')[0],
    },
    'dmp': {
        'roll': ax3.plot([], [], 'r-', label='DMP Roll')[0],
        'pitch': ax3.plot([], [], 'g-', label='DMP Pitch')[0],
        'yaw': ax3.plot([], [], 'b-', label='DMP Yaw')[0],
    }
}
 
setup_axis(ax1, "Raw Data")
setup_axis(ax2, "Sensor Fusion Data")
setup_axis(ax3, "DMP Data")
ax3.set_xlabel("Samples")

def update(frame):
    while ser.in_waiting:
        line = ser.readline().decode(errors='ignore').strip()
        parts = line.split('\t')

        if len(parts) == 10:
            try:
                t = float(parts[0])

                rroll = float(parts[1])
                rpitch = float(parts[2])
                ryaw = float(parts[3])

                eroll = float(parts[4])
                epitch = float(parts[5])
                eyaw = float(parts[6])

                droll = float(parts[7])
                dpitch= float(parts[8])
                dyaw = float(parts[9])

                time_vals.append(t)

                raw_roll.append(rroll)
                raw_pitch.append(rpitch)
                raw_yaw.append(ryaw)

                ekf_roll.append(eroll)
                ekf_pitch.append(epitch)
                ekf_yaw.append(eyaw)

                dmp_roll.append(droll)
                dmp_pitch.append(dpitch)
                dmp_yaw.append(dyaw)

                # Update plots
                x_vals = range(len(time_vals))
                lines['raw']['roll'].set_data(x_vals, raw_roll)
                lines['raw']['pitch'].set_data(x_vals, raw_pitch)
                lines['raw']['yaw'].set_data(x_vals, raw_yaw)

                lines['ekf']['roll'].set_data(x_vals, ekf_roll)
                lines['ekf']['pitch'].set_data(x_vals, ekf_pitch)
                lines['ekf']['yaw'].set_data(x_vals, ekf_yaw)

                lines['dmp']['roll'].set_data(x_vals, dmp_roll)
                lines['dmp']['pitch'].set_data(x_vals, dmp_pitch)
                lines['dmp']['yaw'].set_data(x_vals, dmp_yaw)

            except ValueError:
                continue

    return (
        lines['raw']['roll'], lines['raw']['pitch'], lines['raw']['yaw'],
        lines['ekf']['roll'], lines['ekf']['pitch'], lines['ekf']['yaw'],
        lines['dmp']['roll'], lines['dmp']['pitch'], lines['dmp']['yaw']
    )

# Animate
ani = animation.FuncAnimation(fig, update, interval=50)

# Show plot
try:
    plt.tight_layout()
    plt.show()
except KeyboardInterrupt:
    print("Plot closed.")
