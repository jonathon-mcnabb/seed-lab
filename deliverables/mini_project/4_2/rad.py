# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
from cv2 import aruco
import smbus
import board
import busio
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

# Create the bus connection
bus = smbus.SMBus(1)

# This is the address we setup in the Arduino Program
address = 0x04

# Character LCD size
lcd_columns = 16
lcd_rows = 2

# Initialise I2C bus.
i2c = busio.I2C(board.SCL, board.SDA)

# Initialise the LCD class
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
lcd.clear()

# Set LCD color to white
lcd.color = [10, 10, 10]
time.sleep(1)

# function to write value to the arduino
def writeNumber(value):
    bus.write_byte(address, value)
    return -1

def white_balance(img):
    result = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    avg_a = np.average(result[:, :, 1])
    avg_b = np.average(result[:, :, 2])
    result[:, :, 1] = result[:, :, 1] - ((avg_a - 128) * (result[:, :, 0] / 255.0) * 1.1)
    result[:, :, 2] = result[:, :, 2] - ((avg_b - 128) * (result[:, :, 0] / 255.0) * 1.1)
    result = cv2.cvtColor(result, cv2.COLOR_LAB2BGR)
    return result

def aruco_detection():    
    # start video capture for distance
    cap = cv2.VideoCapture(0)
	
    prevQuadrant = 0
    sendStuff = False
    while(True):
        # Capture frame-by-frame
        ret, frame = cap.read()
        #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        gray = white_balance(frame)
		
		
		
        # gray = cv2.flip(gray, 0) 
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        parameters =  aruco.DetectorParameters_create()

        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        #if ids != None:
           #print("Found ids:")
           #print(ids)
        #else:
            #print("Aruco Marker not found")

        gray = aruco.drawDetectedMarkers(gray, corners, ids)
        
        height, width, _ = gray.shape
        deltaX = 0
        delyaY = 0
        
        absX = int(width/2)
        absY = int(height/2)
        
        #print("Corner: ", corners)
        if len(corners):    #returns no of arucos
            #print (len(corners)
            aruco_list = {}
            #print (len(ids))
            markerOne = corners[0][0]
            
            cornerOne = markerOne[0]
            cornerTwo = markerOne[1]
            cornerThree = markerOne[2]
            cornerFour = markerOne[3]
            
            centerX1 = int((cornerTwo[0] + cornerFour[0]) / 2)
            centerY1 = int((cornerTwo[1] + cornerFour[1]) / 2)
            centerX2 = int((cornerOne[0] + cornerThree[0]) / 2)
            centerY2 = int((cornerOne[1] + cornerThree[1]) / 2)
        
            centerX = (centerX1+centerX2) / 2
            centerY = (centerY1+centerY2) / 2
            
            quadrant = 0

            if centerX > absX:
                if centerY > absY:
                    quadrant = 3
                else:
                    quadrant = 0
            else:
                if centerY > absY:
                    quadrant = 2
                else:
                    quadrant = 1
			
            if quadrant != prevQuadrant:
                prevQuadrant = quadrant
                sendStuff = True
                print(quadrant)

            # GREG ADD HERE
            if sendStuff == True:
                writeNumber(quadrant)
                lcd.message = "Setpoint: " + str(quadrant)
                sendStuff = False
            #
        cv2.imshow('frame',gray)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
aruco_detection()
