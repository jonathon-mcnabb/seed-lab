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
	
    prevQuadrant = 0
    sendStuff = False
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
			
            print(quadrant)
            if quadrant != prevQuadrant:
                prevQuadrant = quadrant
                sendStuff = True

            # GREG ADD HERE
            
            
            #
        cv2.imshow('frame',gray)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
aruco_detection()


#I HATE GIT