# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
from cv2 import aruco

image = cv2.imread('exercise.png')

clickRGB = False

def onMouse(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        small = cv2.resize(image, (0,0), fx=0.5, fy=0.5)
        colors = small[y,x]
        if clickRGB:
            # draw circle here (etc...)
            colorsB = small[y,x,0]
            colorsG = small[y,x,1]
            colorsR = small[y,x,2]
            print("Red: ", colorsR)
            print("Green: ", colorsG)
            print("Blue: ", colorsB)
            print("BGR Format: ", colors)
            print("Coordinates of pixel: X ", x, "Y: ", y)
        else:
            print('pixel')
            print(colors)
            colors = np.uint8([[colors]])
            hsv = cv2.cvtColor(colors, cv2.COLOR_BGR2HSV)
            print('hsv')
            print(hsv)

def cv_exercise1():
    cv2.imshow('large', image)
    small = cv2.resize(image, (0,0), fx=0.5, fy=0.5)
    cv2.imshow('small', small)
def cv_exercise2():
    cv2.setMouseCallback('small', onMouse)
def cv_exercise3():
    small = cv2.resize(image, (0,0), fx=0.5, fy=0.5)
    gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
    cv2.imshow('gray', gray)
def cv_exercise4():
    # yellow
    yellow = cv2.imread('yellow.png')
    small = cv2.resize(yellow, (0,0), fx=0.5, fy=0.5)
   
    boundary = ([20, 100, 100], [30, 255, 255])
   
    lower = np.array(boundary[0], dtype = "uint8")    
    upper = np.array(boundary[1], dtype = "uint8")
   
    hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)
   
    # Apply the cv2.inrange method to create a mask
    mask = cv2.inRange(hsv, lower, upper)
    # Apply the mask on the image to extract the original color
    frame = cv2.bitwise_and(small, small, mask=mask)
   
    cv2.imshow("yellow_mask", np.hstack([small, frame]))
def cv_exercise5():
    #aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    #nx = 2
    #ny = 1
    #white = [255,255,255]
    #plots = []
    #for i in range(1, nx*ny+1):
    #    img = aruco.drawMarker(aruco_dict,i, 700)
    #    img= cv2.copyMakeBorder(img,20,20,20,20,cv2.BORDER_CONSTANT,value=white)
    #    plots.append(img)
    #cv2.imshow("aruco_markers", np.hstack(plots))
    #cv2.imwrite("markers.png", np.hstack(plots))
    
    #break

    frame = cv2.imread('markers.png')

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    parameters =  aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    if ids!=None:
        print(ids)
    else:
        print("No markers found")
    frame_markers = aruco.drawDetectedMarkers(frame.copy(), corners, ids)

    cv2.imshow("images", np.hstack([frame, frame_markers]))
        
    # see single aruco script for capture


def cv_exercise6():
    cap = cv2.VideoCapture(0)

    while(True):
        # Capture frame-by-frame
        ret, frame = cap.read()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        parameters =  aruco.DetectorParameters_create()

        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        if ids != None:
           print("Found ids:")
           print(ids)
        else:
            print("Aruco Marker not found")

        gray = aruco.drawDetectedMarkers(gray, corners, ids)

        cv2.imshow('frame',gray)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
def cv_exercise7():
    frame = cv2.imread('markers.png')

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    parameters =  aruco.DetectorParameters_create()
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    height, width = gray.shape
    deltaX = 0
    delyaY = 0
    
    absX = int(width/2)
    absY = int(height/2)
    
    print("Corner: ", corners)
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
        
        deltaX = abs(absX-centerX)
        deltaY = abs(absY-centerY)
        
        xFOV = (deltaX/width) * 54
        yFOV = (deltaY/height) * 41
        
        
        angle = np.sqrt(xFOV*xFOV+yFOV*yFOV)
        
        print("Angle: ", angle)
        print("CenterX: ", centerX, "CenterY: ", centerY)

    cv2.imshow("angles", frame)
    #print("test")
    
    # start video capture for distance
    cap = cv2.VideoCapture(0)

    while(True):
        # Capture frame-by-frame
        ret, frame = cap.read()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        parameters =  aruco.DetectorParameters_create()

        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        #if ids != None:
           #print("Found ids:")
           #print(ids)
        #else:
            #print("Aruco Marker not found")

        gray = aruco.drawDetectedMarkers(gray, corners, ids)
        
        height, width = gray.shape
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
            
            deltaX1 = abs(cornerTwo[0] - cornerOne[0])
            deltaX2 = abs(cornerThree[0] - cornerFour[0])  
            
            deltaY1 = abs(cornerFour[1] - cornerOne[1])
            deltaY2 = abs(cornerThree[1] - cornerTwo[1])
            
           
            
            arucoWidth = (deltaX1+deltaX2) / 2
            arucoHeight = (deltaY1+deltaY2) / 2
            
            #figure out width of screen
            screenWidth = 0.15 / (arucoWidth / width)
            screenHeight = 0.15 / (arucoHeight / height)
           
            # figure out how many pixels correlates t
            f = 0.0036
            x = 0.0038
            y = 0.0029
            Z1 = f*(screenWidth/x)
            Z2 = f*(screenHeight/y)
            
            finalDistance = (Z1+Z2)/2
            
            print("Distance: ", finalDistance)
            


    
        

        cv2.imshow('frame',gray)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

#cv_exercise1()
#cv_exercise2()
#cv_exercise3()
#cv_exercise4()
#cv_exercise5()
# only run 6 or 7. do not run both
cv_exercise6()
#cv_exercise7()
    
cv2.waitKey(0)


