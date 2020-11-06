# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
from cv2 import aruco

def aruco_detection():    
    # start video capture for distance
    cap = cv2.VideoCapture(0)

    while(True):
        # Capture frame-by-frame
        ret, frame = cap.read()
        #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        gray = frame
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
            
            #deltaX1 = abs(cornerTwo[0] - cornerOne[0])
            #deltaX2 = abs(cornerThree[0] - cornerFour[0])  
            
            #deltaY1 = abs(cornerFour[1] - cornerOne[1])
            #deltaY2 = abs(cornerThree[1] - cornerTwo[1])
            deltaX1 = cornerTwo[0] - cornerOne[0]
            deltaX2 = cornerThree[0] - cornerFour[0]
            
            deltaY1 = cornerFour[1] - cornerOne[1]
            deltaY2 = cornerThree[1] - cornerTwo[1]
            
            centerX1 = int((cornerTwo[0] + cornerFour[0]) / 2)
            centerY1 = int((cornerTwo[1] + cornerFour[1]) / 2)
            centerX2 = int((cornerOne[0] + cornerThree[0]) / 2)
            centerY2 = int((cornerOne[1] + cornerThree[1]) / 2)
        
            centerX = (centerX1+centerX2) / 2
            centerY = (centerY1+centerY2) / 2
            
            deltaX = abs(absX-centerX)
            deltaY = abs(absY-centerY)
        
           
            
            arucoWidth = (deltaX1+deltaX2) / 2
            arucoHeight = (deltaY1+deltaY2) / 2
            
            #figure out width of screen
            screenWidth = 0.492126 / (arucoWidth / width)
            screenHeight = 0.492126 / (arucoHeight / height)
            
            #figure out real distance from center
            xDistance = (screenWidth / 2)*(deltaX / (width/2))
            yDistance = (screenHeight / 2)*(deltaY / (height/2))
            
            # figure out how many pixels correlates t
            f = 0.0036
            x = 0.0038
            y = 0.0029
            Z1 = f*(screenWidth/x)
            Z2 = f*(screenHeight/y)
            
            zDistance = abs((Z1+Z2)/2)
            
            
            XYplaneDistance = np.sqrt(xDistance*xDistance + zDistance*zDistance)
            
            
            
            totalDistance = np.sqrt(xDistance*xDistance + yDistance*yDistance + zDistance*zDistance)
            
            
            print("xOffset", xDistance)
            print("yOffset", yDistance)
            print("zOffset: ", zDistance)
            print("Plane Distance: ",  XYplaneDistance)
            print("Total Distance: ", totalDistance)
            


    
        

        cv2.imshow('frame',gray)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
aruco_detection()