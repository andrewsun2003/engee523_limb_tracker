import serial 
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import csv


SERIAL_PORT = "/dev/cu.usbmodem11301"
BAUD_RATE = 115200
OUTPUT_FILE = "/Users/andrewsun/Downloads/engee523_limb_tracker-main/sensor_fusion/figure4.csv"
csv_file = open(OUTPUT_FILE, mode='w', newline='')
csv_writer = csv.writer(csv_file)

csv_writer.writerow([
    "Time",
    # "Accel_X", "Accel_Y", "Accel_Z",
    # "Gyro_X", "Gyro_Y", "Gyro_Z",
    # "Mag_X", "Mag_Y", "Mag_Z",
    "Raw_Roll", "Raw_Pitch", "Raw_Yaw",
    "EKF_Roll", "EKF_Pitch", "EKF_Yaw",
    "DMP_Roll", "DMP_Pitch", "DMP_Yaw"
])

ser = serial.Serial(SERIAL_PORT, BAUD_RATE)

max_len = 200

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

raw_roll = deque(maxlen=max_len)
raw_pitch = deque(maxlen=max_len)
raw_yaw = deque(maxlen=max_len)

ekf_roll = deque(maxlen=max_len)
ekf_pitch = deque(maxlen=max_len)
ekf_yaw = deque(maxlen=max_len)

dmp_roll = deque(maxlen=max_len)
dmp_pitch = deque(maxlen=max_len)
dmp_yaw = deque(maxlen=max_len)

fig, (ax1, ax2, ax3, ax4, ax5, ax6) = plt.subplots(6, 1, figsize=(12, 9), sharex=True)


def setup_axis(ax, label, y_label, y_limits):
    ax.set_ylim(*y_limits)
    ax.set_xlim(0, max_len)
    ax.set_ylabel(y_label)
    ax.set_title(label)
    ax.grid()
    # ax.legend(loc='lower left', bbox_to_anchor=(0, 0))

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
    'raw': {
        'roll': ax4.plot([], [], 'r-', label='Raw Roll')[0],
        'pitch': ax4.plot([], [], 'g-', label='Raw Pitch')[0],
        'yaw': ax4.plot([], [], 'b-', label='Raw Yaw')[0],
    },
    'ekf': {
        'roll': ax5.plot([], [], 'r-', label='EKF Roll')[0],
        'pitch': ax5.plot([], [], 'g-', label='EKF Pitch')[0],
        'yaw': ax5.plot([], [], 'b-', label='EKF Yaw')[0],
    },
    'dmp': {
        'roll': ax6.plot([], [], 'r-', label='DMP Roll')[0],
        'pitch': ax6.plot([], [], 'g-', label='DMP Pitch')[0],
        'yaw': ax6.plot([], [], 'b-', label='DMP Yaw')[0],
    }
}
 
setup_axis(ax1, "Raw Accelerometer", "m/s²", (-20, 20))
setup_axis(ax2, "Raw Gyroscope", "°/s", (-20, 20))
setup_axis(ax3, "Raw Magnetometer", "µT", (-100, 100))
setup_axis(ax4, "Raw Data", "°", (-180, 180))
setup_axis(ax5, "Sensor Fusion Data", "°", (-180, 180))
setup_axis(ax6, "DMP Data", "°", (-180, 180))
ax5.set_xlabel("Samples")

def update(frame):
    while ser.in_waiting:
        line = ser.readline().decode(errors='ignore').strip()
        parts = line.split('\t')

        if len(parts) == 19:
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

                rroll = float(parts[10])
                rpitch = float(parts[11])
                ryaw = float(parts[12])

                eroll = float(parts[13])
                epitch = float(parts[14])
                eyaw = float(parts[15])

                droll = float(parts[16])
                dpitch= float(parts[17])
                dyaw = float(parts[18])
                
                # Write row to CSV
                csv_writer.writerow([
                    t,
                    # ax, ay, az,
                    # gx, gy, gz,
                    # mx, my, mz,
                    rroll, rpitch, ryaw,
                    eroll, epitch, eyaw,
                    droll, dpitch, dyaw
                ])

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

                lines['accel']['x'].set_data(x_vals, accel_x)
                lines['accel']['y'].set_data(x_vals, accel_y)
                lines['accel']['z'].set_data(x_vals, accel_z)

                lines['gyro']['x'].set_data(x_vals, gyro_x)
                lines['gyro']['y'].set_data(x_vals, gyro_y)
                lines['gyro']['z'].set_data(x_vals, gyro_z)

                lines['mag']['x'].set_data(x_vals, mag_x)
                lines['mag']['y'].set_data(x_vals, mag_y)
                lines['mag']['z'].set_data(x_vals, mag_z)

                lines['raw']['roll'].set_data(x_vals, raw_roll)
                lines['raw']['pitch'].set_data(x_vals, raw_pitch)
                lines['raw']['yaw'].set_data(x_vals, raw_yaw)

                lines['ekf']['roll'].set_data(x_vals, ekf_roll)
                lines['ekf']['pitch'].set_data(x_vals, ekf_pitch)
                lines['ekf']['yaw'].set_data(x_vals, ekf_yaw)

                lines['dmp']['roll'].set_data(x_vals, dmp_roll)
                lines['dmp']['pitch'].set_data(x_vals, dmp_pitch)
                lines['dmp']['yaw'].set_data(x_vals, dmp_yaw)
                
                # Update all legends with latest values
                lines['accel']['x'].set_label(f"Accel X: {ax:.2f}")
                lines['accel']['y'].set_label(f"Accel Y: {ay:.2f}")
                lines['accel']['z'].set_label(f"Accel Z: {az:.2f}")
                ax1.legend(loc='lower left', bbox_to_anchor=(0, 0))

                lines['gyro']['x'].set_label(f"Gyro X: {gx:.2f}")
                lines['gyro']['y'].set_label(f"Gyro Y: {gy:.2f}")
                lines['gyro']['z'].set_label(f"Gyro Z: {gz:.2f}")
                ax2.legend(loc='lower left', bbox_to_anchor=(0, 0))

                lines['mag']['x'].set_label(f"Mag X: {mx:.2f}")
                lines['mag']['y'].set_label(f"Mag Y: {my:.2f}")
                lines['mag']['z'].set_label(f"Mag Z: {mz:.2f}")
                ax3.legend(loc='lower left', bbox_to_anchor=(0, 0))

                lines['raw']['roll'].set_label(f"Raw Roll: {rroll:.1f}°")
                lines['raw']['pitch'].set_label(f"Raw Pitch: {rpitch:.1f}°")
                lines['raw']['yaw'].set_label(f"Raw Yaw: {ryaw:.1f}°")
                ax4.legend(loc='lower left', bbox_to_anchor=(0, 0))

                lines['ekf']['roll'].set_label(f"EKF Roll: {eroll:.1f}°")
                lines['ekf']['pitch'].set_label(f"EKF Pitch: {epitch:.1f}°")
                lines['ekf']['yaw'].set_label(f"EKF Yaw: {eyaw:.1f}°")
                ax5.legend(loc='lower left', bbox_to_anchor=(0, 0))

                lines['dmp']['roll'].set_label(f"DMP Roll: {droll:.1f}°")
                lines['dmp']['pitch'].set_label(f"DMP Pitch: {dpitch:.1f}°")
                lines['dmp']['yaw'].set_label(f"DMP Yaw: {dyaw:.1f}°")
                ax6.legend(loc='lower left', bbox_to_anchor=(0, 0))

            except ValueError:
                continue

    return (
        lines['accel']['x'], lines['accel']['y'], lines['accel']['z'],
        lines['gyro']['x'], lines['gyro']['y'], lines['gyro']['z'],
        lines['mag']['x'], lines['mag']['y'], lines['mag']['z'],
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
    plt.ion()
    
    csv_file.close()
    print("CSV file saved.")

except KeyboardInterrupt:
    print("Plot closed.")
