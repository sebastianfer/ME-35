from picamera2 import Picamera2
import cv2 as cv
import numpy as np
from libcamera import controls
import time
import os
from os import path

picam2 = Picamera2()

# configure camera

picam2.set_controls({"AfMode": controls.AfModeEnum.Continuous}) #sets auto focus mode

picam2.start() #must start the camera before taking any images
time.sleep(0.1)

values = range(1000)

while(True):
    for i in values:
        image = picam2.capture_array("main")
        img_name = 'image{0:04d}.jpg'.format(i)
        time.sleep(1)
        picam2.capture_file(img_name) #take image 
        #img = cv.imread("image.jpg") #read image with open cv, to get the bgr value of one pixel index using print(img[row][col])
        address = "/home/tuftsrobot/Object_1"
        imgSaveDir = path.join(address, img_name)
        img = cv.imwrite(imgSaveDir , image)

    if (cv.waitKey(1) & 0xFF == ord('q')) or ():
        picam2.stop() #stop the picam    
        break
