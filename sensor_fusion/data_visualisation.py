import serial 
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque


SERIAL_PORT = "/dev/cu.usbmodem1301"
BAUD_RATE = 115200
OUTPUT_FILE = "imu_data.csv"


ser = serial.Serial(SERIAL_PORT, BAUD_RATE)

max_len = 100

time_vals = deque(maxlen=max_len)

accel_x = deque(maxlen=max_len)
accel_y = deque(maxlen=max_len)
accel_z = deque(maxlen=max_len)

gyro_x = deque(maxlen=max_len)
gyro_y = deque(maxlen=max_len)
gyro_z = deque(maxlen=max_len)

mag_x = deque(maxlen=max_len)
mag_y = deque(maxlen=max_len)
mag_z = deque(maxlen=max_len)

ekf_roll = deque(maxlen=max_len)
ekf_pitch = deque(maxlen=max_len)
ekf_yaw = deque(maxlen=max_len)

dmp_roll = deque(maxlen=max_len)
dmp_pitch = deque(maxlen=max_len)
dmp_yaw = deque(maxlen=max_len)

fig, (ax1, ax2, ax3, ax4, ax5) = plt.subplots(5, 1, figsize=(12, 9), sharex=True)


def setup_axis(ax, label, y_label, y_limits):
    ax.set_ylim(*y_limits)
    ax.set_xlim(0, max_len)
    ax.set_ylabel(y_label)
    ax.set_title(label)
    ax.grid()
    ax.legend(loc='lower left', bbox_to_anchor=(0, 0))

# Plot lines
lines = {
    'accel': {
        'x': ax1.plot([], [], 'r-', label='Accel X')[0],
        'y': ax1.plot([], [], 'g-', label='Accel Y')[0],
        'z': ax1.plot([], [], 'b-', label='Accel Z')[0],
    },
    'gyro': {
        'x': ax2.plot([], [], 'r-', label='Gyro X')[0],
        'y': ax2.plot([], [], 'g-', label='Gyro Y')[0],
        'z': ax2.plot([], [], 'b-', label='Gyro Z')[0],
    },
    'mag': {
        'x': ax3.plot([], [], 'r-', label='Mag X')[0],
        'y': ax3.plot([], [], 'g-', label='Mag Y')[0],
        'z': ax3.plot([], [], 'b-', label='Mag Z')[0],
    },
    'ekf': {
        'roll': ax4.plot([], [], 'r-', label='EKF Roll')[0],
        'pitch': ax4.plot([], [], 'g-', label='EKF Pitch')[0],
        'yaw': ax4.plot([], [], 'b-', label='EKF Yaw')[0],
    },
    'dmp': {
        'roll': ax5.plot([], [], 'r-', label='DMP Roll')[0],
        'pitch': ax5.plot([], [], 'g-', label='DMP Pitch')[0],
        'yaw': ax5.plot([], [], 'b-', label='DMP Yaw')[0],
    }
}
 
setup_axis(ax1, "Raw Accelerometer", "m/s²", (-20, 20))
setup_axis(ax2, "Raw Gyroscope", "°/s", (-20, 20))
setup_axis(ax3, "Raw Magnetometer", "µT", (-100, 100))
setup_axis(ax4, "Sensor Fusion Data", "°", (-180, 180))
setup_axis(ax5, "DMP Data", "°", (-180, 180))
ax5.set_xlabel("Samples")

def update(frame):
    while ser.in_waiting:
        line = ser.readline().decode(errors='ignore').strip()
        parts = line.split('\t')

        if len(parts) == 16:
            try:
                t = float(parts[0])

                ax = float(parts[1])
                ay = float(parts[2])
                az = float(parts[3])

                gx = float(parts[4])
                gy = float(parts[5])
                gz = float(parts[6])

                mx = float(parts[7])
                my = float(parts[8])
                mz = float(parts[9])

                eroll = float(parts[10])
                epitch = float(parts[11])
                eyaw = float(parts[12])

                droll = float(parts[13])
                dpitch= float(parts[14])
                dyaw = float(parts[15])

                time_vals.append(t)

                accel_x.append(ax)
                accel_y.append(ay)
                accel_z.append(az)

                gyro_x.append(gx)
                gyro_y.append(gy)
                gyro_z.append(gz)

                mag_x.append(mx)
                mag_y.append(my)
                mag_z.append(mz)

                ekf_roll.append(eroll)
                ekf_pitch.append(epitch)
                ekf_yaw.append(eyaw)

                dmp_roll.append(droll)
                dmp_pitch.append(dpitch)
                dmp_yaw.append(dyaw)

                # Update plots
                x_vals = range(len(time_vals))

                lines['accel']['x'].set_data(x_vals, accel_x)
                lines['accel']['y'].set_data(x_vals, accel_y)
                lines['accel']['z'].set_data(x_vals, accel_z)

                lines['gyro']['x'].set_data(x_vals, gyro_x)
                lines['gyro']['y'].set_data(x_vals, gyro_y)
                lines['gyro']['z'].set_data(x_vals, gyro_z)

                lines['mag']['x'].set_data(x_vals, mag_x)
                lines['mag']['y'].set_data(x_vals, mag_y)
                lines['mag']['z'].set_data(x_vals, mag_z)

                lines['ekf']['roll'].set_data(x_vals, ekf_roll)
                lines['ekf']['pitch'].set_data(x_vals, ekf_pitch)
                lines['ekf']['yaw'].set_data(x_vals, ekf_yaw)

                lines['dmp']['roll'].set_data(x_vals, dmp_roll)
                lines['dmp']['pitch'].set_data(x_vals, dmp_pitch)
                lines['dmp']['yaw'].set_data(x_vals, dmp_yaw)

            except ValueError:
                continue

    return (
        lines['accel']['x'], lines['accel']['y'], lines['accel']['z'],
        lines['gyro']['x'], lines['gyro']['y'], lines['gyro']['z'],
        lines['mag']['x'], lines['mag']['y'], lines['mag']['z'],
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
