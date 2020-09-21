# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np


    
 
        
fileName = input("File Name:")

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
rawCapture = PiRGBArray(camera)
   
 
# allow the camera to warmup
time.sleep(0.1)
 
# grab an image from the camera
print("Capturing Image...")
try:
    camera.capture(rawCapture, format="bgr")
      
    image = rawCapture.array
    image = cv2.cvtColor( image, cv2.COLOR_BGR2RGB )
    # image = cv2.cvtColor( image, cv2.COLOR_BGR2RGB )
except:
    print("Failed to capture")
      
    
# save the image to the disk
# print("Saving image "+fileName)
try:

    cv2.imwrite(fileName, image)
      
      
except:
    # print("Couldn't save "+fileName)
    pass
    
 
# display the image on screen and wait for a keypress
# display the image on screen and wait for a keypress
cv2.imshow('Image', image)

cv2.waitKey(0)
cv2.destroyAllWindows()




