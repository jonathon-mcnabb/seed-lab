# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
from cv2 import aruco

import serial

# Set serial address

# VERIFY SERIAL PORT THAT ARDUINO IS CONNECTED TO
ser = serial.Serial('/dev/ttyACM0', 115200)
# Wait for connection to complete
time.sleep(3)


cap = cv2.VideoCapture(0)

cap.set(cv2.CAP_PROP_FPS, 150)
#cap.set(cv2.CAP_PROP_FRAME_WIDTH,2592)
#cap.set(cv2.CAP_PROP_FRAME_HEIGHT,1944)
cap.set(cv2.CAP_PROP_EXPOSURE, 0.01)

detecting = True

def write_to_serial(value):
    try:
        ser.write(str(value).encode('utf-8'))
    except:
        print("Communication Error")

def set_high_res():
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,2592)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT,1944)
    
def set_low_res():
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT,380)


def share_points():
    
    print("test")
    
    # start video capture for distance
    
    #frameCount = 0
    
    #set_low_res()




    # Capture frame-by-frame
    start = time.time()
    ret, frame = cap.read()
    gray = frame
    #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)


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
    deltaY = 0

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



    # ANGLE
        centerX1 = ((cornerTwo[0] + cornerFour[0]) / 2)
        #centerY1 = int((cornerTwo[1] + cornerFour[1]) / 2)
        centerX2 = ((cornerOne[0] + cornerThree[0]) / 2)
        #centerY2 = int((cornerOne[1] + cornerThree[1]) / 2)

        #centerX3 = ((cornerTwo[0] + cornerOne[0]) / 2)
        #centerX4 = ((cornerThree[0] + cornerFour[0]) / 2)

        # calcualte center X coordinate of marker
        centerX = (centerX1+centerX2) / 2

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

    # ANGLE DONE

    # DISTANCE

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
        #Z1 = f*(screenWidth/x)
        Z2 = f*(screenHeight/y)

        #finalDistance = (Z1+Z2)/2
        finalDistance = Z2

    # DISTANCE DONE

    # POINTS

        boxOffset = 0.1524

        # Are all boxes the same size?

        x = 0
        y = 1

        # figure out points to move to

        point1 = [finalDistance - boxOffset, 0]
        point2 = [point1[x] , point1[y] + 2*boxOffset]
        point3 = [point2[x] + 4*boxOffset, point2[y]]
        point4 = [point3[x], point3[y] - 4*boxOffset]
        point5 = [point4[x] - 4*boxOffset, point4[y]]
        point6 = [point5[x], point5[y] + 2*boxOffset]

        points = [point1, point2, point3, point4, point5, point6]

    # POINTS DONE

    # FINAL ADJUSTMENTS
        angle = round(angle - 3.8059460516, 3) # need to verify that adjustment is correct
    # ADJUSTMENTS DONE

        print("Distance: ", finalDistance)
        print("ANGLE: ", angle)
        print("POINTS: ", points)


    #print(frameCount)
    #frameCount = frameCount + 1

    cv2.imshow('frame',frame)




def share_angle():
    # Capture frame-by-frame
    ret, frame = cap.read()
    gray = frame
    # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    parameters = aruco.DetectorParameters_create()

    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    # if ids != None:
    # print("Found ids:")
    # print(ids)
    # else:
    # print("Aruco Marker not found")

    gray = aruco.drawDetectedMarkers(gray, corners, ids)

    height, width, _ = gray.shape
    delta_x = 0
    deltaY = 0

    absX = int(width / 2)
    absY = int(height / 2)


    if len(corners):  # returns no of arucos

        marker_one = corners[0][0]

        corner_one = marker_one[0]
        corner_two = marker_one[1]
        corner_three = marker_one[2]
        corner_four = marker_one[3]

        # ANGLE
        center_x1 = ((corner_two[0] + corner_four[0]) / 2)
        # centerY1 = int((corner_two[1] + cornerFour[1]) / 2)
        center_x2 = ((corner_one[0] + corner_three[0]) / 2)
        # centerY2 = int((corner_one[1] + corner_three[1]) / 2)

        # centerX3 = ((corner_two[0] + corner_one[0]) / 2)
        # centerX4 = ((corner_three[0] + cornerFour[0]) / 2)

        # calcualte center X coordinate of marker
        center_x = (center_x1 + center_x2) / 2

        # find out how off centered we are
        delta_x = abs(absX - center_x)
        # deltaY = abs(absY-centerY)

        # get angle
        x_fov = (delta_x / width) * 54
        # yFOV = (deltaY/height) * 41

        angle = x_fov
        # print("ANGLE: ",angle)

        # if we are left of screen center, apply left polynomial fix
        if center_x < 320:
            error = 0.0000487516774389672 * (center_x ** 2) - 0.0137673927681325 * center_x - 0.425377206030661
            angle = angle - error
            angle = 0 - angle
        # if we are right of screen center, apply right polynominal fix
        else:
            error = 0.000032254210770952 * (center_x ** 2) - 0.03289220867007 * center_x + 7.53540624358558
            angle = angle - error
        if 319 < center_x < 321:
            # if we are centered, set angle to 0
            angle = 0

        # ANGLE DONE

        # FINAL ADJUSTMENTS
        angle = round(angle - 3.8059460516, 4)  # need to verify that adjustment is correct
        # ADJUSTMENTS DONE
        value_to_send = 'A' + angle
        write_to_serial(value_to_send)

    #print(frameCount)
    #frameCount = frameCount + 1

    cv2.imshow('frame', frame)

#cv_exercise1()
#cv_exercise2()
#cv_exercise3()
#cv_exercise4()
#cv_exercise5()
# only run 6 or 7. do not run both
#cv_exercise6()
while True:
    share_angle()

cv2.waitKey(0)


