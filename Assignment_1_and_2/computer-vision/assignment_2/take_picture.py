# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np

if __name__ == '__main__':
    
   def onMouse(event, x, y, flags, param):
      if event == cv2.EVENT_LBUTTONDOWN:
         # draw circle here (etc...)
         b,g,r = image[y, x]
         b = int(b)
         g = int(g)
         r = int(r)
         print(r)
         print(g)
         print(b)
         #print('pixel')
         #print(pixel)
         #pixel = np.uint8([[pixel]])
         #hsv = cv2.cvtColor(pixel, cv2.COLOR_BGR2HSV)
         #print('hsv')
         #print(hsv)
   def grayscale():
      gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
      cv2.imshow('gray', gray)
        
 
   # fileName = input("File Name:")

   # initialize the camera and grab a reference to the raw camera capture
   camera = PiCamera()
   rawCapture = PiRGBArray(camera)
   
 
   # allow the camera to warmup
   time.sleep(0.2)
 
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
      
    
   # save the image to the disk
   # print("Saving image "+fileName)

 
   # display the image on screen and wait for a keypress
   # display the image on screen and wait for a keypress
   cv2.imshow('Image', small)
   
   grayscale()

   cv2.setMouseCallback('Image', onMouse)
   
   # yellow
   
   boundary = ([20, 100, 100], [30, 255, 255])
   
   print(boundary[0])
   
   lower = np.array(boundary[0], dtype = "uint8")    
   upper = np.array(boundary[1], dtype = "uint8")
   
   hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)
   
   # Apply the cv2.inrange method to create a mask
   mask = cv2.inRange(hsv, lower, upper)
   # Apply the mask on the image to extract the original color
   frame = cv2.bitwise_and(small, small, mask=mask)
   
   cv2.imshow("images", np.hstack([small, frame]))
   cv2.waitKey(0)
   


   #try:
   #   cv2.imshow("Image", image)
   #except Exception, e:
   #   print e

   cv2.waitKey(0)
   cv2.destroyAllWindows()




