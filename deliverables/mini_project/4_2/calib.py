# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np

def autoAdjustments_with_convertScaleAbs(img):
    alow = img.min()
    ahigh = img.max()
    amax = 255
    amin = 0

    # calculate alpha, beta
    alpha = ((amax - amin) / (ahigh - alow))
    beta = amin - alow * alpha
    # perform the operation g(x,y)= α * f(x,y)+ β
    new_img = cv2.convertScaleAbs(img, alpha=alpha, beta=beta)

    return [new_img, alpha, beta]

def colorBalance(img, percent):

    img = img / 255
    
    lower_q = percent / 2
    upper_q = 1-lower_q
    
    b, g, r    = img[:, :, 0], img[:, :, 1], img[:, :, 2] # For RGB image
    
    b = b.ravel()
    g = g.ravel()
    r = r.ravel()
    
    lower_b = np.percentile(b, lower_q)
    lower_g = np.percentile(g, lower_q)
    lower_r = np.percentile(r, lower_q)
    higher_b = np.percentile(b, upper_q)
    higher_g = np.percentile(g, upper_q)
    higher_r = np.percentile(r, upper_q)
    
    print(lower_b)
    print(higher_b)


   # initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
rawCapture = PiRGBArray(camera)

time.sleep(0.2)

   # initialize the camera and grab a reference to the raw camera capture

 
   # grab an image from the camera
print("Capturing Image...")
try:
    camera.capture(rawCapture, format="bgr")
      
    image = rawCapture.array
    image = cv2.cvtColor( image, cv2.COLOR_BGR2RGB )
    small = cv2.resize(image, (0,0), fx=0.5, fy=0.5) 
    print(image.size)
      # image = cv2.cvtColor( image, cv2.COLOR_BGR2RGB )
except:
    print("Failed to capture")


new_image, alpha, beta = autoAdjustments_with_convertScaleAbs(small)
new_image_2 = colorBalance(small,1)
cv2.imshow('Image', small)
cv2.imshow('newImage', new_image)
cv2.imshow('newImage2', new_image_2)



cv2.waitKey(0)
   
