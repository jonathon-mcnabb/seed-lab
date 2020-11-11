# USAGE
# python picamera_fps_demo.py
# python picamera_fps_demo.py --display 1 --log 2

# import the necessary packages
from imutils.video.pivideostream import PiVideoStream
from imutils.video import FPS
from picamera.array import PiRGBArray
from picamera import PiCamera
import argparse
import imutils
import time
import cv2
from cv2 import aruco
import threading

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

def write_to_i2c(bus, value, frame_number):
    global last_sent
    if frame_number < last_sent:
        return
    try:
        b = value.encode('ascii')
        for byte in b:
            bus.write_byte(addr, byte)
        last_sent = frame_number
    except Exception as e:
        print("Write Error: ", e)
    last_sent = frame_number

def read_from_i2c(bus):
    pass


aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
parameters = aruco.DetectorParameters_create()

def process_frame(frame, frame_number):
    global last_sent
    if frame_number < last_sent:
        return
    corners, ids, _ = aruco.detectMarkers(frame, aruco_dict, parameters=parameters)
    if len(corners):
        height, width, _ = frame.shape
        value_to_send = ""
        global state
        if state is "a":
            middle_x = int(width / 2)
            middle_y = int(height / 2)
            marker_one = corners[0][0]

            corner_one = marker_one[0]
            corner_two = marker_one[1]
            corner_three = marker_one[2]
            corner_four = marker_one[3]

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

            x_angle  = round(x_angle - 3.80594, 4)

            value_to_send = 'A' + str(x_angle)
        elif state is "d":
            deltaY1 = abs(cornerFour[1] - cornerOne[1])
            deltaY2 = abs(cornerThree[1] - cornerTwo[1])

            arucoHeight = (deltaY1 + deltaY2) / 2

            screenHeight = 0.15 / (arucoHeight / height)
            f = 0.0036
            y = 0.0038
            finalDistance = f*(screenHeight / y)
            finalDistance = round(finalDistance, 4)

            value_to_send = 'D' + str(finalDistance)
        if value_to_send is not "":
            t1 = threading.Thread(target=write_to_i2c, name="send", args=(bus, value_to_send,frame_number))
            t1 .start()


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

# initialize the camera and stream
camera = PiCamera()
camera.resolution = (1296, 730)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(1296, 730))
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
    print("[INFO] sampling THREADED frames from `picamera` module in...")
    print("3")
vs = PiVideoStream().start()
time.sleep(1.0)
if args["log"] > 0:
    print("2")

time.sleep(1.0)

if args["log"] > 0:
    print("1") 
fps = FPS().start()

# loop over some frames...this time using the threaded stream

frame_number = 0
state = "a"
if args["test"] is 1:
    while fps._numFrames < args["num_frames"]:
	    # grab the frame from the threaded video stream and resize it
	    # to have a maximum width of 400 pixels
        frame = vs.read()
        t1 = threading.Thread(target=process_frame, name="process_frame", args=(frame,frame_number ))
        t1.start()
        frame = imutils.resize(frame, width=400)
        # check to see if the frame should be displayed to our screen
        if args["display"] > 0:
            cv2.imshow("Frame", frame)
            key = cv2.waitKey(1) & 0xFF

        # update the FPS counter
        fps.update()
        frame_number = frame_number + 1
else:
    while True:
        frame = vs.read()
        t1 = threading.Thread(target=process_frame, name="process_frame", args=(frame, frame_number))
        t1.start()
        frame = imutils.resize(frame, width=400)
        
        if args["display"] > 0:
            cv2.imshow("Frame", frame)
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
