import numpy as np
import cv2
from cv2 import aruco

frame = cv2.imread('test.png')


gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
parameters =  aruco.DetectorParameters_create()
corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
print(ids)
frame_markers = aruco.drawDetectedMarkers(frame.copy(), corners, ids)

cv2.imshow("images", np.hstack([frame, frame_markers]))
cv2.waitKey(0)