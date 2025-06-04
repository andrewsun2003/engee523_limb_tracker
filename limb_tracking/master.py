import socket
import threading
from vpython import *
import numpy as np


# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Bind to the local IP and port that ESPs send to
UDP_IP = "0.0.0.0"  # Accept from any IP
UDP_PORT = 1234           # Must match the port in your ESP32 code
sock.bind((UDP_IP, UDP_PORT))

print(f"Listening for UDP packets on {UDP_IP}:{UDP_PORT}...\n")

scene.title = "Limb Tracker"
scene.width = 800
scene.height = 600
scene.autoscale = True

axis_len = 1
chest_hip_length = 4
hip_knee_length = 5
knee_shin_length = 5.5

hip_pos = vector(0, 0, 0)
chest_pos = hip_pos + vector(0, chest_hip_length, 0)
knee_pos = hip_pos + vector(0, -hip_knee_length, 0)
shin_pos = knee_pos + vector(0, -knee_shin_length, 0)

chest_hip = cylinder(pos=chest_pos, axis=hip_pos - chest_pos, radius=0.1, color=color.orange)
hip_knee = cylinder(pos=hip_pos, axis=knee_pos - hip_pos, radius=0.1, color=color.blue)
knee_shin = cylinder(pos=knee_pos, axis=shin_pos - knee_pos, radius=0.1, color=color.green)

def draw_axes(origin):
    x_axis = arrow(pos=origin, axis=vector(axis_len, 0, 0), color=color.red)
    y_axis = arrow(pos=origin, axis=vector(0, axis_len, 0), color=color.green)
    z_axis = arrow(pos=origin, axis=vector(0, 0, axis_len), color=color.blue)
    return [x_axis, y_axis, z_axis]

axes_chest = draw_axes(chest_pos)
axes_hip = draw_axes(hip_pos)
axes_knee = draw_axes(knee_pos)
axes_shin = draw_axes(shin_pos)

latest_data = {
    "IMU 1": (0.0, 0.0, 0.0),
    "IMU 2": (0.0, 0.0, 0.0),
    "IMU 3": (0.0, 0.0, 0.0)
}

def parse_packet(data):
    try:
        msg = data.decode()
        print(f"{msg}")
        if msg.startswith("IMU 1:") or msg.startswith("IMU 2:") or msg.startswith("IMU 3:"):
            label, values = msg.split(":")
            roll, pitch, yaw = map(float, values.strip().split(","))
            latest_data[label.strip()] = (roll, pitch, yaw)
    except Exception as e:
        print("Error parsing:", e)
        
def udp_listener():
    while True:
        data, addr = sock.recvfrom(1024)
        parse_packet(data)
      
            
threading.Thread(target=udp_listener, daemon=True).start()


def euler_to_vector(length, roll, pitch, yaw):
    roll = np.radians(roll)
    pitch = np.radians(pitch)
    yaw = np.radians(yaw)
    # Assuming pitch (Y) is forward axis
    x = length * np.cos(pitch) * np.sin(yaw)
    y = length * np.sin(pitch)
    z = length * np.cos(pitch) * np.cos(yaw)
    return vector(x, y, z)


def rotation_matrix(roll, pitch, yaw):
    r, p, y = np.radians([roll, pitch, yaw])

    Rz = np.array([[np.cos(y), -np.sin(y), 0],
                   [np.sin(y),  np.cos(y), 0],
                   [0, 0, 1]])

    Ry = np.array([[np.cos(p), 0, np.sin(p)],
                   [0, 1, 0],
                   [-np.sin(p), 0, np.cos(p)]])

    Rx = np.array([[1, 0, 0],
                   [0, np.cos(r), -np.sin(r)],
                   [0, np.sin(r),  np.cos(r)]])

    return Rz @ Ry @ Rx


def update_limb_orientation():
    pitch1, roll1, yaw1 = latest_data["IMU 1"]  # Chest to Hip
    pitch2, roll2, yaw2 = latest_data["IMU 2"]  # Hip to Knee
    pitch3, roll3, yaw3 = latest_data["IMU 3"]  # Knee to Shin

    #     # Get matrices
    # R1 = rotation_matrix(roll1, pitch1, yaw1)
    # R2 = rotation_matrix(roll2, pitch2, yaw2)
    # R3 = rotation_matrix(roll3, pitch3, yaw3)

    # # Get direction vectors using full rotation
    # dir1 = vector(*R1 @ np.array([0, -chest_hip_length, 0]))
    # dir2 = vector(*R2 @ np.array([0, -hip_knee_length, 0]))
    # dir3 = vector(*R3 @ np.array([0, -knee_shin_length, 0]))

    # # Update joint positions
    # chest_pos_new = hip_pos + dir1
    # knee_pos_new = hip_pos + dir2
    # shin_pos_new = knee_pos_new + dir3

    # # Update cylinders
    # chest_hip.pos = chest_pos_new
    # chest_hip.axis = hip_pos - chest_pos_new

    # hip_knee.pos = hip_pos
    # hip_knee.axis = knee_pos_new - hip_pos

    # knee_shin.pos = knee_pos_new
    # knee_shin.axis = shin_pos_new - knee_pos_new

    # # Update arrows using respective rotation matrices
    # for arrows, origin, R in zip(
    #     [axes_chest, axes_hip, axes_knee, axes_shin],
    #     [chest_pos_new, hip_pos, knee_pos_new, shin_pos_new],
    #     [R1, np.eye(3), R2, R3]
    # ):
    #     x_arrow, y_arrow, z_arrow = arrows
    #     x_arrow.pos = origin
    #     y_arrow.pos = origin
    #     z_arrow.pos = origin

    #     x_arrow.axis = vector(*R @ np.array([axis_len, 0, 0]))
    #     y_arrow.axis = vector(*R @ np.array([0, axis_len, 0]))
    #     z_arrow.axis = vector(*R @ np.array([0, 0, axis_len]))


    #test
    # def update_limb_orientation():
    # roll1, pitch1, yaw1 = latest_data["IMU 1"]  # Chest to Hip
    # roll2, pitch2, yaw2 = latest_data["IMU 2"]  # Hip to Knee
    # roll3, pitch3, yaw3 = latest_data["IMU 3"]  # Knee to Shin

    # # Step 1: Compute rotation matrices
    # R1 = rotation_matrix(roll1, pitch1, yaw1)  # chest to hip
    # R2_local = rotation_matrix(roll2, pitch2, yaw2)  # relative: hip to knee
    # R3_local = rotation_matrix(roll3, pitch3, yaw3)  # relative: knee to shin

    # # Step 2: Chain rotations
    # R2 = R1 @ R2_local  # global orientation of hip to knee
    # R3 = R2 @ R3_local  # global orientation of knee to shin

    # # Step 3: Compute joint positions
    # chest_pos_new = hip_pos + vector(*R1 @ np.array([0, chest_hip_length, 0]))
    # knee_pos_new = hip_pos + vector(*R2 @ np.array([0, -hip_knee_length, 0]))
    # shin_pos_new = knee_pos_new + vector(*R3 @ np.array([0, -knee_shin_length, 0]))

    # # Step 4: Update limb cylinders
    # chest_hip.pos = chest_pos_new
    # chest_hip.axis = hip_pos - chest_pos_new

    # hip_knee.pos = hip_pos
    # hip_knee.axis = knee_pos_new - hip_pos

    # knee_shin.pos = knee_pos_new
    # knee_shin.axis = shin_pos_new - knee_pos_new

    # # Step 5: Update arrows (axes) for visual orientation
    # for arrows, origin, R in zip(
    #     [axes_chest, axes_hip, axes_knee, axes_shin],
    #     [chest_pos_new, hip_pos, knee_pos_new, shin_pos_new],
    #     [R1, np.eye(3), R2, R3]
    # ):
    #     x_arrow, y_arrow, z_arrow = arrows
    #     x_arrow.pos = y_arrow.pos = z_arrow.pos = origin
    #     x_arrow.axis = vector(*R @ np.array([axis_len, 0, 0]))
    #     y_arrow.axis = vector(*R @ np.array([0, axis_len, 0]))
    #     z_arrow.axis = vector(*R @ np.array([0, 0, axis_len]))
    # Compute direction vectors
    dir1 = euler_to_vector(chest_hip_length, roll1, pitch1, yaw1)
    dir2 = euler_to_vector(hip_knee_length, roll2, pitch2, yaw2)
    dir3 = euler_to_vector(knee_shin_length, roll3, pitch3, yaw3)

    # Update positions and orientations
    chest_pos_new = hip_pos + dir1
    knee_pos_new = hip_pos + dir2
    shin_pos_new = knee_pos_new + dir3

    # Update cylinders
    chest_hip.pos = chest_pos_new
    chest_hip.axis = hip_pos - chest_pos_new

    hip_knee.pos = hip_pos
    hip_knee.axis = knee_pos_new - hip_pos

    knee_shin.pos = knee_pos_new
    knee_shin.axis = shin_pos_new - knee_pos_new

    # Update axes
    for arrows, origin, roll, pitch, yaw in zip(
        [axes_chest, axes_hip, axes_knee, axes_shin],
        [chest_pos_new, hip_pos, knee_pos_new, shin_pos_new],
        [roll1, 0, roll2, roll3],
        [pitch1, 0, pitch2, pitch3],
        [yaw1, 0, yaw2, yaw3]
    ):
        x_arrow, y_arrow, z_arrow = arrows
        x_arrow.pos = origin
        y_arrow.pos = origin
        z_arrow.pos = origin
        
    R = rotation_matrix(roll, pitch, yaw)
    x_arrow.axis = vector(*R @ np.array([axis_len, 0, 0]))
    y_arrow.axis = vector(*R @ np.array([0, axis_len, 0]))
    z_arrow.axis = vector(*R @ np.array([0, 0, axis_len]))


try:
    while True:
        rate(30)
        update_limb_orientation()
except KeyboardInterrupt:
    print("\nStopped.")
finally:
    sock.close()
