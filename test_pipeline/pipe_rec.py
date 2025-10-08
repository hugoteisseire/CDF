# client.py
import socket
import struct
import time

path = "/tmp/robot.sock"

# Attendre un peu que le serveur C soit prêt
time.sleep(0.5)

sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
sock.connect(path)
print("Connecté au serveur C.")

while True:
    data = sock.recv(12)  # 3 floats = 12 octets
    if not data:
        break
    x, y, z = struct.unpack('fff', data)
    print(f"Reçu: x={x:f}, y={y:f}, z={z:f}")
