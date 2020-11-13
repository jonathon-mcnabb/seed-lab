import smbus
import time
import board
import busio

# Create the bus connection (might need multiple?)
bus = smbus.SMBus(1)

# Addresses for each Pi
address1 = 0x04
address2 = 0x08
address3 = 0x0C

# Initialise I2C bus.
i2c = busio.I2C(board.SCL, board.SDA)

# function to write value to the arduino
def writeArduino(value):
    bus.write_byte(address1, value)
    return -1

# function to read value from arduino
def readArduino():
    number = bus.read_byte(address1)
    return number
    
# function to write value to Pi2
def writePi2(value):
    bus.write_byte(address2, value)
    return -1

# function to read value from Pi2
def readPi2():
    number = bus.read_byte(address2)
    return number
    
# function to write value to Pi3
def writePi3(value):
    bus.write_byte(address3, value)
    return -1

# function to read value from Pi3
def readPi3():
    number = bus.read_byte(address3)
    return number

# prompt user for input and turn it into an int to write to arduino
var = input("Enter a Number: ")
var = int(var)
writeArduino(var)

var = input("Enter a Number: ")
var = int(var)
writePi2(var)

var = input("Enter a Number: ")
var = int(var)
writePi3(var)
time.sleep(1)

# print out the returned number to the console and LCD
number = readArduino()
number = readPi2()
number = readPi3()
