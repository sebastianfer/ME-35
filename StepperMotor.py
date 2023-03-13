# Import required libraries

import sys

import time

import RPi.GPIO as GPIO

 

# Use BCM GPIO references

# instead of physical pin numbers

GPIO.setmode(GPIO.BOARD)

 

# Define GPIO signals to use

# Physical pins 11,15,16,18

# GPIO17,GPIO22,GPIO23,GPIO24

ControlPin = [12,11,13,15]

 

# Set all pins as output

for pin in ControlPin:

  GPIO.setup(pin,GPIO.OUT)

  GPIO.output(pin, 0)

 

ccw =  [[1,1,0,0],

       [0,1,1,0],

       [0,0,1,1],

       [1,0,0,1],

]

 

cw = [[1,0,0,1],

       [0,0,1,1],

       [0,1,1,0],

       [1,1,0,0],

]

rotationNeeded = 0

rotationCount = 0

while(1):

    rotationNeeded == 0

    print("\n")

    userInput = input("Press e-exit, Number of Rotations: ")

    print("\n")

    orientation = input("What orientation (cw or ccw): ")

    print("\n")

    if userInput == 'e':

        break;

   

    rotationNeeded = int(userInput)

    rotationCount = 50 * rotationNeeded

    

    if orientation == 'cw':

        for i in range(rotationCount):

            for fullStep in range(4):

                for pin in range(4):

                    GPIO.output(ControlPin[pin], cw[fullStep][pin])

                    time.sleep(0.001)


    if orientation == 'ccw':

        for i in range(rotationCount):

            for fullStep in range(4):

                for pin in range(4):

                    GPIO.output(ControlPin[pin], ccw[fullStep][pin])

                    time.sleep(0.001)


GPIO.cleanup()

