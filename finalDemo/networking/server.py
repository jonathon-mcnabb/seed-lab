import socket
import requests

UDP_PORT = 4210

ip = requests.get('https://checkip.amazonaws.com').text.strip()
print(ip)

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.bind((ip, UDP_PORT))

i=0
while True:
    data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
    print("received message: %s" % data, "      ", i)
    i = i + 1
