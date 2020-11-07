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

def find_angles():    
    # start video capture for distance
    cap = cv2.VideoCapture(0)
    # init "global" vars
    prevQuadrant = 0
    sendStuff = False
    i = 0
    while(True):
        # Capture frame-by-frame
        ret, frame = cap.read()
        gray = frame
        #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
		
		
        # gray = cv2.flip(gray, 0) 
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        parameters =  aruco.DetectorParameters_create()

        # grab corners from frame
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        #if ids != None:
           #print("Found ids:")
           #print(ids)
        #else:
            #print("Aruco Marker not found")

	# grab frame with aruco markers
        gray = aruco.drawDetectedMarkers(gray, corners, ids)
        
        height, width, _ = gray.shape
        deltaX = 0
        delyaY = 0
        
        # get middle pixels
        
        absX = int(width/2)
        absY = int(height/2)
        
        #print(absX)
        # = 320
        
        #print("Corner: ", corners)
        # if corners exists
        if len(corners):    #returns no of arucos
            #print (len(corners)
            aruco_list = {}
            #print (len(ids))
            markerOne = corners[0][0]
            
            # grab corners
            cornerOne = markerOne[0]
            cornerTwo = markerOne[1]
            cornerThree = markerOne[2]
            cornerFour = markerOne[3]
            
            centerX1 = ((cornerTwo[0] + cornerFour[0]) / 2)
            #centerY1 = int((cornerTwo[1] + cornerFour[1]) / 2)
            centerX2 = ((cornerOne[0] + cornerThree[0]) / 2)
            #centerY2 = int((cornerOne[1] + cornerThree[1]) / 2)
            
            #centerX3 = ((cornerTwo[0] + cornerOne[0]) / 2)
            #centerX4 = ((cornerThree[0] + cornerFour[0]) / 2)
         
            # calcualte center X coordinate of marker
            centerX = (centerX1+centerX2) / 2
            
            #centerX_2 = (centerX3 + centerX4) / 2
            #centerY = (centerY1+centerY2) / 2
            
            #centerX_4 = (centerX+centerX_2) / 2
            
            #print("CENTER: ", centerX)
            #print("CENTER2: ", centerX_2)
            #print("CENTER4: ", centerX_2)
	    # find out how off centered we are
            deltaX = abs(absX-centerX)
            #deltaY = abs(absY-centerY)
            
            # get angle   
            xFOV = (deltaX/width) * 54
            #yFOV = (deltaY/height) * 41

            angle = xFOV
            #print("ANGLE: ",angle)
            
            # if we are left of screen center, apply left polynomial fix
            if(centerX < 320):
                error = 0.0000487516774389672*(centerX**2)-0.0137673927681325*centerX-0.425377206030661
                angle = angle - error
                angle = 0 - angle
            # if we are right of screen center, apply right polynominal fix
            else:
                error = 0.000032254210770952*(centerX**2)-0.03289220867007*centerX+7.53540624358558
                angle = angle - error
            if (centerX > 319 and centerX < 321):
                # if we are centered, set angle to 0
                angle = 0
            
            # print info every 10th frame a marker is found
            if(i % 10 == 0):
                print("ANGLE: ",angle)
                print("CENTER: ", centerX)
                lcd.message = "Angle: " + str(angle) + "\nCenterX: " + str(centerX)
                
            #if angle != prevAngle:
                #prevQuadrant = angle
                #sendStuff = True
                #print(quadrant)

            # When a new quadrant is seen, send that info to the Arduino to move the wheel to the correct position
            #if sendStuff == True:
                #writeNumber(quadrant)
                #lcd.message = "Setpoint: " + str(quadrant)
                #sendStuff = False
        
        #angle = np.sqrt(xFOV*xFOV+yFOV*yFOV)
		    
        # show frame
        cv2.imshow('frame',gray)
        i = i+1
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
find_angles()
