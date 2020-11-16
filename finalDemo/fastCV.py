# USAGE
# python picamera_fps_demo.py
# python picamera_fps_demo.py --display 1 --log 2

# import the necessary packages
from imutils.video.pivideostream import PiVideoStream
from imutils.video import FPS
from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy as np
import argparse
import imutils
import time
import cv2
from cv2 import aruco
import threading
import socket
import requests
from smbus import SMBus

try:
    addr = 0x8 # bus address
    bus = SMBus(1)
except Exception as e:
    print("port not available: ", e)
    # Wait for connection to complete

time.sleep(3)

last_sent = 0
state = "a"

# constants
box_radius = 0.3048 # 1 foot in meters

def write_to_i2c(bus, value, frame_number):
    global this_pi
    global middle
    if this_pi != middle:
        return
    global last_sent
    if frame_number < last_sent:
        #print("[INFO] skipping frame in write_to_i2c")
        return
    try:
        b = value.encode('ascii')
        for byte in b:
            bus.write_byte(addr, byte)
        last_sent = frame_number
        print("sent")
    except Exception as e:
        print("Write Error: ", e)
        pass
    last_sent = frame_number

def read_from_i2c(bus):
    pass

bufferSize = 1024
UDPClientSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
def send_to_server(msg):
    bytesToSend = str.encode(msg)
    global serverAddressPort
    UDPClientSocket.sendto(bytesToSend, serverAddressPort)

def read_from_client():
    messages = []
    leftAngle = None
    rightAngle = None
    leftDistance = None
    rightDistance = None
    for i in range(3):
        data, addr = socket2.recvfrom(1024)
        data = data.decode("utf-8")
        index = data.find("S")
        if data[0:2] == "AL":
            leftAngle = data[2:index]
        elif data[0:2] == "AR":
            rightAngle = data[2:index]
        if data[index+1:index+3] == "DL":
            leftDistance = data[index+3:len(data)-2]
        elif data[index+1:index+3] == "DR":
            rightDistance = data[index+3:len(data)-2]
    return leftAngle, rightAngle, leftDistance, rightDistance

aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
parameters = aruco.DetectorParameters_create()

def process_frame(frame, frame_number):
    global last_sent
    if frame_number < last_sent:
        print("[INFO] skipping frame in process_frame")
        return
    corners, ids, _ = aruco.detectMarkers(frame, aruco_dict, parameters=parameters)
    x_angle = None
    finalDistance = None
    global this_pi
    if len(corners):
        height, width, _ = frame.shape
        value_to_send = ""
        global request
        middle_x = int(width / 2)
        middle_y = int(height / 2)
        marker_one = corners[0][0]

        corner_one = marker_one[0]
        corner_two = marker_one[1]
        corner_three = marker_one[2]
        corner_four = marker_one[3]
        if "a" in request:
            center_x1 = ((corner_two[0] + corner_four[0]) / 2)
            center_x2 = ((corner_one[0] + corner_three[0]) / 2)

            center_x  = (center_x1 + center_x2) / 2
            delta_x = abs(middle_x - center_x)

            x_angle = (delta_x / width) * 54

            if center_x < 320:
                error = 0.0000487516774389672 * (center_x ** 2) - 0.0137673927681325 * center_x - 0.425377206030661
                x_angle = 0 - (x_angle - error)
            elif center_x > 321:
                error = 0.000032254210770952 * (center_x **2) - 0.03289220867007 * center_x + 7.53540624358558
                x_angle = x_angle - error
            else:
                x_angle = 0

            # depending on what pi is running, change the angle accordingly
            left_offset = 34.5085
            middle_offset = 3.9232
            right_offset = 38.6598
            if this_pi == "left":
                x_angle = left_offset + x_angle
                value_to_send = 'AL' + str(round(x_angle, 4)) + 'S'
            elif this_pi == "middle":
                x_angle = x_angle - middle_offset
            elif this_pi == "right":
                x_angle = -1*(right_offset - x_angle)
                value_to_send = 'AR' + str(round(x_angle, 4)) + 'S'
        if "d" in request:
            deltaY1 = abs(corner_four[1] - corner_one[1])
            deltaY2 = abs(corner_three[1] - corner_two[1])

            arucoHeight = (deltaY1 + deltaY2) / 2

            screenHeight = 0.15 / (arucoHeight / height)
            f = 0.0036
            y = 0.0038
            finalDistance = f*(screenHeight / y)
            finalDistance = round(finalDistance, 4)
            if this_pi == "left":
                value_to_send = value_to_send + 'DL' + str(finalDistance) + 'S'
            elif this_pi == "right":
                value_to_send = value_to_send +  'DR' + str(finalDistance) + 'S'
        if value_to_send is not "":
            print("Value to send: ", value_to_send)
            if this_pi != "middle" and frame_number % 3 == 0:
                t1 = threading.Thread(target=send_to_server, name="send", args=(value_to_send,))
                t1.start()
    print(frame_number)
    if frame_number % 3 == 0:
        if this_pi == "middle":
            leftAngle, rightAngle, leftDistance, rightDistance = read_from_client()
            middleAngle = x_angle
            middleDistance = finalDistance

            print("left: ", leftAngle, "right: ", rightAngle)
            # parse though read_from_client()
            EXTRA_ANGLE = np.radians(0)
            angle_to_spin_while_finding = 1.5708 + EXTRA_ANGLE # right turn 90 deg in radians + EXTRA_ANGLE

            # this is where we actually are going to handle states

            # i is the state to find the first beacon
            if state == "i":
                # initial marker

                # 1) spin clockwise until initial marker is found
                # 2) move to P1 (offset by angle)
                # P1 == (theta_i = tan^-1(r/d), distance = sqrt(r^2+d^2)
                # 3) Spin right so that full FOV can be seen at once
                # rotate so edge of camera FOV aligns with a 90 deg angle
                # theta_s -> theta search
                # theta_s = 30 deg - theta_i + EXTRA_ANGLE
                # if we need to add more range to the FOV after turning, modify extra angle

                # if an angle exists
                if (leftAngle is not None) or (rightAngle is not None) or (middleAngle is not None):
                    # get P1
                    # we want to take the biggest point
                    angleArray = [leftAngle, rightAngle, middleAngle]
                    distanceArray = [leftDistance, rightDistance, middleDistance]
                    biggestAngleIndex = np.argmax(angleArray) # finds index of maximum angle

                    # find the angle and distance to move towards the 1st beacon
                    theta = round(np.arctan(box_offset/distanceArray[biggestAngleIndex]) + angleArray[biggestAngleIndex], 4) # finds theta_offset to move to correct point. Made up of angle to box + offset angle
                    distance_to_move = round(np.sqrt(distanceArray[biggestAngleIndex]**2 + box_offset**2), 4)

                    theta2 = round(1.5708 - theta ,4) # after we move to P1, reset angle to 90 deg (1.5708)
                    # send the
                    value_to_send = 'A' + theta + 'SD' + distance_to_move+ 'S' + 'A' + theta2 + 'SD0S'  # send P1 as well as next angle to turn
                    t1 = threading.Thread(target=write_to_i2c, name="send", args=(bus, value_to_send, frame_number))
                    t1.start()
                else:
                    value_to_send = 'A' + angle_to_spin_while_finding + 'SD0S' # send the angle and also send 0 distance
                    t1 = threading.Thread(target=write_to_i2c, name="send", args=(bus, value_to_send, frame_number))
                    t1.start()


            # b state is to find all other beacons
            elif state == "b":
                pass

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-n", "--num-frames", type=int, default=100,
	help="# of frames to loop over for FPS test")
ap.add_argument("-d", "--display", type=int, default=-1,
	help="Whether or not frames should be displayed")
ap.add_argument("-l", "--log", type=int, default=-1,
    help="Whether or not debug messages should be displayed")
ap.add_argument("-t", "--test", type=int, default=0,
    help="set to 1 if testing frames")
args = vars(ap.parse_args())

left = "left"
middle = "middle"
right = "right"
# figure out what pi this is

this_pi = socket.gethostname()

if this_pi != middle:
    print("[CONFIG] Enter middle pi ip address (format -> 138.67.xxx.xxx) :")
    middle_ip = input()
    serverAddressPort = (middle_ip, 4210)
if this_pi == middle:
    ip = requests.get('https://checkip.amazonaws.com').text.strip()
    client_address = (ip, 4210)
    socket2 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    socket2.bind(client_address)
    print("Middle IP: ", ip)

if args["log"] > 0:
    print("This is the " + this_pi + " camera pi")
# initialize the camera and stream
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))
stream = camera.capture_continuous(rawCapture, format="bgr",
	use_video_port=True)

# do a bit of cleanup
cv2.destroyAllWindows()
stream.close()
rawCapture.close()
camera.close()

# created a *threaded *video stream, allow the camera sensor to warmup,
# and start the FPS counter

if args["log"] > 0:
    print("[INFO] sampling frames...")
vs = PiVideoStream().start()
time.sleep(1.0)
fps = FPS().start()

# loop over some frames...this time using the threaded stream

frame_number = 0
state = "i"
request = "ad"
if args["test"] is 1:
    while fps._numFrames < args["num_frames"]:
	    # grab the frame from the threaded video stream and resize it
	    # to have a maximum width of 400 pixels
        frame = vs.read()
        t1 = threading.Thread(target=process_frame, name="process_frame", args=(frame,frame_number ))
        t1.start()
        small_frame = imutils.resize(frame, width=640)
        # check to see if the frame should be displayed to our screen
        if args["display"] > 0:
            cv2.imshow("Frame", small_frame)
            key = cv2.waitKey(1) & 0xFF

        # update the FPS counter
        fps.update()
        frame_number = frame_number + 1
else:
    while True:
        frame = vs.read()
        frame = imutils.resize(frame, width=640)
        try:
            t1 = threading.Thread(target=process_frame, name="process_frame", args=(frame, frame_number))
            t1.start()
        except:
            pass
        small_frame = imutils.resize(frame, width=400)
        if args["display"] > 0:
            cv2.imshow("Frame", small_frame)
            key = cv2.waitKey(1) & 0xFF
        frame_number = frame_number + 1

# stop the timer and display FPS information
fps.stop()
if args["log"] > 0:
    print("[INFO] elasped time: {:.2f}".format(fps.elapsed()))
    print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))

# do a bit of cleanup
cv2.destroyAllWindows()
vs.stop()


# Notes



# All other markers

#1) check if current marker can be seen
    # if a marker can be seen, choose the most right marker

# if marker cannot be seen, we may need to move around the box
#2) if marker cannot be seen
    # spin 90 deg
