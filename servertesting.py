import socket

# Define host and port to listen on
HOST = '127.0.0.1'  # Listen on all available interfaces
PORT = 5000  # Same port number used in the client script

# Create a socket object
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Bind the socket to the address and port
server_socket.bind((HOST, PORT))

# Listen for incoming connections
server_socket.listen()

print("Waiting for connection...")

# Accept incoming connections
client_socket, client_address = server_socket.accept()
print(f"Connection established with {client_address}")

# Receive data from the client in a loop
while True:
    data = client_socket.recv(1024)
    if not data:
        print("Client disconnected")
        break
    print("Received:", data.decode())

# Close the connection
client_socket.close()
