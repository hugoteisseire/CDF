# server.py
import socket, struct, os

path = "/tmp/robot.sock"
if os.path.exists(path):
    os.remove(path)

sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
sock.bind(path)
sock.listen(1)
print("En attente du client C...")
conn, _ = sock.accept()
print("Client connecté !")

while True:
    data = conn.recv(12)  # 3 floats = 12 octets
    if not data:
        break
    x, y, z = struct.unpack('fff', data)
    print(f"Reçu: x={x:.2f}, y={y:.2f}, z={z:.2f}")
