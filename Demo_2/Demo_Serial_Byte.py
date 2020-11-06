import serial
import time

# Set serial address
ser = serial.Serial('/dev/ttyACM0', 115200)

# Wait for connection to complete
time.sleep(3)

# Function to read serial
def ReadfromArduino():
    while (ser.in_waiting > 0):
        try:
            line = ser.readline().decode('utf-8').rstrip()
            line = int(line)
            print("Arduino: Hey RPI, I am sending you the digit ", line)
        except:
            print("Communication Error")

# Prompt user for input and convert to int
var = input("Enter a Number: ")
var = int(var)
ser.write(var.to_bytes(1, byteorder="big"))
print("RPI: Hi Arduino, I sent you ", var)

# wait for Arduino to set up response and receive the response
time.sleep(1)
number = ReadfromArduino()
