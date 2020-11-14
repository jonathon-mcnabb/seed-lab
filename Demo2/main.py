# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import threading
from time import sleep
import cv2
import numpy as np
from cv2 import aruco
import array
import serial
from math import radians
from smbus import SMBus

# Set busial address
try:
    addr = 0x8 # bus address
    bus = SMBus(1) # indicates /dev/ic2-1
except:
    print("port not availble")
# Wait for connection to complete
time.sleep(3)

detecting = True

def write_then_read(bus, value):
    write_to_i2c(bus, value)
    read_from_arduino(bus)

def write_to_i2c(bus, value):
    print(value)
    try:
        b = value.encode('ascii')
        for byte in b:
            bus.write_byte(addr, byte) # switch it on
    except Exception as e:
        print(e)
        print("WRITE Error")


def set_high_res():
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,2592)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT,1944)


def set_low_res():
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT,380)


def share_points():
    print("starting up")
    cap = cv2.VideoCapture(0)

    cap.set(cv2.CAP_PROP_FPS, 150)
    cap.set(cv2.CAP_PROP_EXPOSURE, 0.01)

    while True:
        print("capturing a frame")

        # Capture frame-by-frame
        start = time.time()
        ret, frame = cap.read()
        gray = frame

        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        parameters =  aruco.DetectorParameters_create()

        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        gray = aruco.drawDetectedMarkers(gray, corners, ids)

        height, width, _ = gray.shape

        print(height, width)
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
            centerX2 = ((cornerOne[0] + cornerThree[0]) / 2)

            # calcualte center X coordinate of marker
            centerX = (centerX1+centerX2) / 2

            # find out how off centered we are
            deltaX = abs(absX-centerX)

            # get angle
            xFOV = (deltaX/width) * 54

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
            screenWidth = 0.10 / (arucoWidth / width)
            screenHeight = 0.10 / (arucoHeight / height)

            # figure out how many pixels correlates t
            f = 0.0036
            x = 0.0038
            y = 0.0029
            #Z1 = f*(screenWidth/x)
            Z2 = f*(screenHeight/y)
            finalDistance = Z2

            # DISTANCE DONE

            # POINTS
            boxOffset = 0.2 #0.1524

            # Are all boxes the same size?
            x = 0
            y = 1

            # figure out points to move to
            point1 = [finalDistance - boxOffset, 0]
            point2 = [0 , 2*boxOffset]
            point3 = [4*boxOffset, 0]
            point4 = [0, -4*boxOffset]
            point5 = [-4*boxOffset, point4[y]]
            point6 = [0, 2*boxOffset]

            point1[x] = round(point1[y], 3)
            point1[x] = round(point1[y], 3)

            point2[x] = round(point1[y], 3)
            point2[x] = round(point1[y], 3)

            point3[x] = round(point1[y], 3)
            point3[x] = round(point1[y], 3)

            point4[x] = round(point1[y], 3)
            point4[x] = round(point1[y], 3)

            point5[x] = round(point1[y], 3)
            point5[x] = round(point1[y], 3)

            point6[x] = round(point1[y], 3)
            point6[x] = round(point1[y], 3)


            points = [point1, point2, point3, point4, point5, point6]


            value_to_send = '(' + str(point1[x]) + ',' + str(point1[y]) + ')' + '(' + str(point2[x]) + ',' + str(point2[y]) + ')' + '(' + str(point3[x]) + ',' + str(point3[y]) + ')'+ '(' + str(point4[x]) + ',' + str(point4[y]) + ')'
            #write_to_i2c(value_to_send)

            sleep(0.01)

            value_to_send = '(' + str(point5[x]) + ',' + str(point5[y]) + ')' + '(' + str(point6[x]) + ',' + str(point6[y]) + ')'

            # POINTS DONE

            # FINAL ADJUSTMENTS
            angle = round(radians(angle - 3.8059460516), 4) # need to verify that adjustment is correct

            value_to_send = 'A' + str(angle) + 'S'

            print("Distance: ", finalDistance)
            value_to_send += 'D' + str(round(finalDistance - boxOffset , 4)) + 'S'
            #print("ANGLE: ", angle)
            #print("POINTS: ", points)

            # Use multithreading to send information to the Arduino
            thread_list = []
            for thread in threading.enumerate():
                thread_list.append(thread.getName())

            if "send" not in thread_list:
                t1 = threading.Thread(target=write_to_i2c, name="send",  args=(bus, value_to_send,))
                t1.start()

share_points()

cv2.waitKey(0)
