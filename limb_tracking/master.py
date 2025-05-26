import socket
import threading
from vpython import cylinder, vector, rate, scene, color
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

thigh = cylinder(pos=vector(0, 0, 0), axis=vector(0, 2, 0), radius=0.1, color=color.red)
shin = cylinder(pos=thigh.pos + thigh.axis, axis=vector(0, 2, 0), radius=0.1, color=color.green)

latest_data = {
    "IMU 1": (0.0, 0.0, 0.0),
    "IMU 2": (0.0, 0.0, 0.0)
}

def parse_packet(data):
    try:
        msg = data.decode()
        print(f"{msg}")
        if msg.startswith("IMU 1:") or msg.startswith("IMU 2:"):
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

def euler_to_vector(roll, pitch, yaw):
    roll = np.radians(roll)
    pitch = np.radians(pitch)
    yaw = np.radians(yaw)
    # Assuming pitch (Y) is forward axis
    x = np.cos(pitch) * np.sin(yaw)
    y = np.sin(pitch)
    z = np.cos(pitch) * np.cos(yaw)
    return vector(x, y, z)

def update_limb_orientation():
    roll1, pitch1, yaw1 = latest_data["IMU 1"]
    roll2, pitch2, yaw2 = latest_data["IMU 2"]

    # Compute direction vectors
    dir1 = euler_to_vector(roll1, pitch1, yaw1).norm()
    dir2 = euler_to_vector(roll2, pitch2, yaw2).norm()

    # Update thigh
    thigh.axis = dir1 * 2
    # Update shin, attached to thigh
    shin.pos = thigh.pos + thigh.axis
    shin.axis = dir2 * 2
    
try:
    while True:
        rate(30)
        update_limb_orientation()
except KeyboardInterrupt:
    print("\nStopped.")
finally:
    sock.close()
