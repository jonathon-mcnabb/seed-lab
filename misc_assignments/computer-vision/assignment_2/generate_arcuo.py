import numpy as np
import cv2
from cv2 import aruco


aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)

nx = 2
ny = 1
white = [255,255,255]

plots = []
for i in range(1, nx*ny+1):
    img = aruco.drawMarker(aruco_dict,i, 700)
    img= cv2.copyMakeBorder(img,20,20,20,20,cv2.BORDER_CONSTANT,value=white)
    plots.append(img)
print(plots)
cv2.imshow("images", np.hstack(plots))
cv2.imwrite("markers.jpg", np.hstack(plots))
cv2.waitKey(0)
