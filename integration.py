from pymavlink import mavutil
import socket

def connect_pixhawk(connection_string='/dev/serial0', baud=57600):
    vehicle = mavutil.mavlink_connection(connection_string, baud=baud)
    vehicle.wait_heartbeat()
    print("Heartbeat from vehicle (system %u component %u)" % (vehicle.target_system, vehicle.target_component))
    return vehicle

def takeoff(vehicle, altitude=10):
    print("Takeoff command received")
    vehicle.mav.command_long_send(
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0, 0, 0, altitude)
    print("Takeoff command sent")

def land(vehicle):
    print("Land command received")
    vehicle.mav.command_long_send(
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0,
        0, 0, 0, 0, 0, 0, 0)
    print("Land command sent")

def yaw(vehicle, direction='clockwise', degree=45):
    print(f"Yaw command received: {direction}, {degree}")
    radian = degree * (3.14159 / 180.0)
    if direction == 'counterclockwise':
        radian = -radian
    vehicle.mav.command_long_send(
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,
        abs(radian), 0, 1 if direction == 'clockwise' else -1, 1, 0, 0, 0)
    print("Yaw command sent")

# Define host and port
HOST = '0.0.0.0'  # Listen on all network interfaces
PORT = 5000

# Create a socket object
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Bind the socket to the address and port
server_socket.bind((HOST, PORT))

# Listen for incoming connections
server_socket.listen()

print("Waiting for connection...")

client_socket, client_address = server_socket.accept()
print(f"Connection established with {client_address}")


# Connect to Pixhawk
vehicle = connect_pixhawk()

# Accept incoming connections and process them
try:
    while True:
        client_socket, client_address = server_socket.accept()
        print(f"Connection established with {client_address}")

        try:
            while True:
                data = client_socket.recv(1024).decode().strip()
                if not data:
                    print("Client disconnected")
                    break
                print("Received:", data)

                # Process received data
                if data.lower() == 'takeoff':
                    takeoff(vehicle)
                elif data.lower() == 'land':
                    land(vehicle)
                elif data.lower().startswith('yaw'):
                    _, direction, degree = data.split()
                    yaw(vehicle, direction, float(degree))
                else:
                    print(f"Unknown command: {data}")

        finally:
            client_socket.close()
finally:
    server_socket.close()