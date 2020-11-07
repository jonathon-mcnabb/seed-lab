
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
from cv2 import aruco


cap = cv2.VideoCapture(0)
#cap.set(cv2.CAP_PROP_FRAME_WIDTH,640)
#cap.set(cv2.CAP_PROP_FRAME_HEIGHT,480)
cap.set(cv2.CAP_PROP_FPS, 150)
start = time.time()
for i in range(400):
    ret, img = cap.read()
print("Time for {0} frames: {1} seconds", time.time() - start)