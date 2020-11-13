import socket

msgFromClient = "A-11.938"

bytesToSend = str.encode(msgFromClient)

serverAddressPort = ("138.67.201.108", 4210)

bufferSize = 1024

# Create a UDP socket at client side

UDPClientSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

# Send to server using created UDP socket
while True:

    print("sending")

    UDPClientSocket.sendto(bytesToSend, serverAddressPort)

    #msgFromServer = UDPClientSocket.recvfrom(bufferSize)

    #msg = "Message from Server {}".format(msgFromServer[0])

    #print(msg)
