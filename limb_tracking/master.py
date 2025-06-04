from vpython import *
import socket
import threading
import numpy as np

# VPython setup
scene = canvas(title="3-IMU Leg Visualisation")

hip = vector(0, 0, 0)
thigh_length = 2.0
shin_length = 2.0
foot_length = 1.5

# Limb segments
thigh = cylinder(pos=hip, axis=vector(0, thigh_length, 0), radius=0.2, color=color.red)
knee_pos = thigh.pos + thigh.axis
shin = cylinder(pos=knee_pos, axis=vector(0, shin_length, 0), radius=0.15, color=color.blue)
ankle_pos = shin.pos + shin.axis
foot = cylinder(pos=ankle_pos, axis=vector(0, foot_length, 0), radius=0.12, color=color.green)

# Quaternion-to-rotation utility
def quaternion_to_rotation_matrix(w, x, y, z):
    r00 = 1 - 2*y*y - 2*z*z
    r01 = 2*x*y - 2*z*w
    r02 = 2*x*z + 2*y*w
    r10 = 2*x*y + 2*z*w
    r11 = 1 - 2*x*x - 2*z*z
    r12 = 2*y*z - 2*x*w
    r20 = 2*x*z - 2*y*w
    r21 = 2*y*z + 2*x*w
    r22 = 1 - 2*x*x - 2*y*y
    return np.array([[r00, r01, r02],
                     [r10, r11, r12],
                     [r20, r21, r22]])

# UDP receiver
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", 1234))

# Shared quaternion states
quat1 = None  # Thigh
quat2 = None  # Shin
quat3 = None  # Foot (IMU3)

def listen_udp():
    global quat1, quat2, quat3
    while True:
        data, _ = sock.recvfrom(1024)
        msg = data.decode().strip()

        if "IMU 1 Quaternion:" in msg:
            parts = msg.split(":")[1].strip().split(",")
            quat1 = [float(p.strip()) for p in parts]
            print(f"Received IMU1: {quat1}")

        elif "IMU 2 Quaternion:" in msg:
            parts = msg.split(":")[1].strip().split(",")
            quat2 = [float(p.strip()) for p in parts]
            print(f"Received IMU2: {quat2}")

        elif "IMU 3 Quaternion:" in msg:
            parts = msg.split(":")[1].strip().split(",")
            quat3 = [float(p.strip()) for p in parts]
            print(f"Received IMU3: {quat3}")

threading.Thread(target=listen_udp, daemon=True).start()

# Main animation loop
while True:
    rate(60)

    if quat1:
        R1 = quaternion_to_rotation_matrix(*quat1)
        thigh_axis = R1 @ np.array([0, thigh_length, 0])
        thigh.axis = vector(*thigh_axis)
        knee_pos = hip + vector(*thigh_axis)
        shin.pos = knee_pos

    if quat2:
        R2 = quaternion_to_rotation_matrix(*quat2)
        shin_axis = R2 @ np.array([0, shin_length, 0])
        shin.axis = vector(*shin_axis)
        ankle_pos = shin.pos + shin.axis
        foot.pos = ankle_pos

    if quat3:
        R3 = quaternion_to_rotation_matrix(*quat3)
        foot_axis = R3 @ np.array([0, foot_length, 0])
        foot.axis = vector(*foot_axis)
