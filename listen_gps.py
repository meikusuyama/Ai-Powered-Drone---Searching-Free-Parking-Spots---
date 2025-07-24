import socket

LISTEN_PORT = 3000
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", LISTEN_PORT))

print(f"Listening for GPS packets on UDP port {LISTEN_PORT}…")
while True:
    data, addr = sock.recvfrom(1024)
    lat, lon, alt = data.decode("utf-8").split(",")
    print(f"Drone at lat={lat}, lon={lon}, alt={alt} m (from {addr[0]})")
