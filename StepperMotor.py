import sys
import time
import RPi.GPIO as GPIO

# Use BCM GPIO references instead of physical pin numbers
GPIO.setmode(GPIO.BOARD)

# Define GPIO signals to use
ControlPin = [12,11,13,15]

# Set all pins as output
for pin in ControlPin:
  GPIO.setup(pin,GPIO.OUT)
  GPIO.output(pin, 0)

# Counter-clockwise
ccw =  [[1,1,0,0],
       [0,1,1,0],
       [0,0,1,1],
       [1,0,0,1],
]

# Clockwise
cw = [[1,0,0,1],
       [1,1,0,0],
       [0,1,1,0],
       [0,0,1,1],
]

def MoveMotor(rotationCount, orientation):
    if orientation == 'cw':
       sequence = cw
    elif orientation == 'ccw':
       sequence = ccw
    else:
       print("Invalid orientation")
       return

    # Rotate Motor in a full Step
    for i in range(rotationCount):
       for fullStep in range(4):
          for pin in range(4):
             GPIO.output(ControlPin[pin], sequence[fullStep][pin])
             time.sleep(0.001)

while True:
   userInput = input("Press e for exit or insert number of Rotations: ")
   if userInput == 'e':
      break
   
   # Spin the Motor
   rotationCount = int(userInput) * 50
   
   # Define the orientation
   orientation = input("Orientation (cw or ccw): ")

   MoveMotor(rotationCount, orientation)

GPIO.cleanup()
