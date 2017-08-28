#!/usr/bin/python
# coding: utf-8


#----------------------------------------------------------------
# Adapted from work done by:
#Autor: Saymon C. A. Oliveira
# Email: saymowan@gmail.com
# Descrição: este algoritmo descreve a sexta implementação de OpenCV
# Funções: Imagem digital -> Transformação HSV -> Imagem binária -> Erosão binária -> Encontrar área -> Encontrar coordenadas

# Tecnologias: OpenCV, Python e NumPy
# Url: http://www.instructables.com/id/The-RROP-RaspRobot-OpenCV-Project/
#---------------------------------------------------------------
# Glen Willis

class CameraStream:

    def __init__(self):
        print("Initialising Camera")
        ret, unfilteredImage = capture.read()
        imgHSV = cv2.cvtColor(unfilteredImage,cv2.cv.CV_BGR2HSV)    
        imgThreshold = cv2.inRange(imgHSV, rangeMin, rangeMax)
        imgErosion = cv2.erode(imgThreshold, None, iterations = 3)
        self._moments = cv2.moments(imgErosion, True)
        self._xPos = 0
        
    def start(self):
        print("Starting Camera Thread")
        t = Thread(target=self.update,args=())
        t.daemon = True
        t.start()
        return self
    
    def update(self):
        while True:    
            ret, unfilteredImage = capture.read()
            imgHSV = cv2.cvtColor(unfilteredImage,cv2.cv.CV_BGR2HSV)    
            imgThreshold = cv2.inRange(imgHSV, rangeMin, rangeMax)
            imgErosion = cv2.erode(imgThreshold, None, iterations = 3)
            self._moments = cv2.moments(imgErosion, True)
            if self._moments['m00'] > 0:
                self._xPos = self._moments['m10'] / self._moments['m00']               
                
        #sleep(1)

    def returnMonments(self):
        return self._moments

    def returnXPos(self):
        return self._xPos

    def stopSteam(self):
        capture.release()
        t.stop()

import cv2.cv as cv
import cv2 as cv2
import time
import numpy as np
import os
from time import sleep
import RPi.GPIO as GPIO
from threading import Thread

from random import randint

forwardMotorLeftActionPin = 9
forwardMotorRightActionPin = 10
reverseMotorLeftActionPin = 22
reverseMotorRightActionPin = 27

startScanPin = 17

servoKick = 18

scanDonePin = 11
scanDonePinVal = 0

lookingForBall = False
GPIO.setmode(GPIO.BCM)
GPIO.setup(forwardMotorLeftActionPin,GPIO.OUT)
GPIO.setup(forwardMotorRightActionPin,GPIO.OUT)
GPIO.setup(reverseMotorLeftActionPin,GPIO.OUT)
GPIO.setup(reverseMotorRightActionPin,GPIO.OUT)
GPIO.setup(servoKick,GPIO.OUT)
GPIO.setup(startScanPin, GPIO.OUT)
GPIO.setup(scanDonePin, GPIO.IN)

servoKickPwm = GPIO.PWM(servoKick,100)
servoKickPwm.start(0)
#servoKickPwm.ChangeDutyCycle(0)

# The HSV range used to detect the coloured object
# Green ball
Hmin = 42
Hmax = 92
Smin = 62
Smax = 255
Vmin = 63
Vmax = 235


width = 500 #160 * 2
centrePoint = width / 2
height = 500 # Default 120
#delay = (0.0001 / 1000.0)
delay = (1/1000.0)
ballKicked = False
directionChoice = 0
directionSet = False
maxTime = 2
bufferCt = 0
bufferMax = 10
ballFoundToRight = False


 # Creates a HSV array based on the min and maxium values (as an unsigned int)
rangeMin = np.array([Hmin, Smin, Vmin], np.uint8)
rangeMax = np.array([Hmax, Smax, Vmax], np.uint8)

# Minimum area to be detected
#minArea = 50 Default area
minArea = 25

# Max area to be detected to activate kick
#maxArea = 70000 # Workable value
maxArea = 65000

#Uncomment below to display unfiltered image in window
#cv.NamedWindow("Unfiltered Image")

#cv.NamedWindow("HSV")
#cv.NamedWindow("Threshold")
#cv.NamedWindow("Filtered Image")


capture = cv2.VideoCapture(0)



# Image capture window parameters
width = 500 #Default value
height = 500 #Default value

def RotateClockwise():
    GPIO.output(forwardMotorLeftActionPin, 1)
    GPIO.output(forwardMotorRightActionPin, 0)

def RotateAntiClockwise():
    GPIO.output(forwardMotorLeftActionPin, 0)
    GPIO.output(forwardMotorRightActionPin, 1)

def ForwardFull():
    GPIO.output(forwardMotorLeftActionPin, 1)
    GPIO.output(forwardMotorRightActionPin, 1)

def StopMotors():
    GPIO.output(forwardMotorLeftActionPin, 0)
    GPIO.output(reverseMotorLeftActionPin, 0)
    GPIO.output(forwardMotorRightActionPin, 0)
    GPIO.output(reverseMotorRightActionPin, 0)

def findBall(directionChoice):
    if(directionChoice == 0): #Turn Left
        RotateClockwise()
        print "Looking to the Left"
    else:
        RotateAntiClockwise() #Turn Right
    print "Looking to the Right"
        
def scanFindBall():
    startScan = True
    GPIO.output(startScanPin,1)
    sleep(0.01)
    GPIO.output(startScanPin,0)
    
def kickBall():
    print("Trying to kick the ball")
    for dcUp in range(0, 10, 1):
        servoKickPwm.ChangeDutyCycle(dcUp)
        time.sleep(0.1)
    sleep(0.5)
    for dcDown in range(10, 0, 1):
        servoKickPwm.ChangeDutyCycle(dcDown)
        time.sleep(0.1)
    sleep(0.01)
    servoKickPwm.start(0)
    sleep(0.1)
    print("Ball kicked")
    
    
    # 12.5 is 180
    # 7.5 is neutral
    # 2.5 is 0

# Set a size the for the frames
if capture.isOpened():
    capture.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, width)
    capture.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, height)

print("Soccerbot initialised. Starting ball search now")

StopMotors()
GPIO.output(startScanPin,0)
startScan = False
cam = CameraStream().start()
time.sleep(1.0)

if __name__== '__main__':
    while True:
        ret, unfilteredImage = capture.read()
        # Uncomment below to display unfiltered image as window
        #cv2.imshow("Unfiltered Image",unfilteredImage)
        moments = cam.returnMonments()
        if bufferCt <= bufferMax:
            bufferCt += 1 
        if moments['m00'] >= minArea:
            if moments['m00'] <= maxArea:
                x = moments['m10'] / moments['m00']
                lookingForBall = False
                if(int(x) < (centrePoint - 50)): #Default is 50
                    print("To the right")
                    RotateClockwise()
                    directionChoice = 1
                elif(int(x) > (centrePoint + 50)): #Default is 50
                    print("To the Left")
                    directionChoice = 0
                    RotateAntiClockwise()                 
                else:
                    print("Centered")
                    ForwardFull()
            else:
                if bufferCt >= bufferMax:
                    print("Looking to kick the ball")
                    #kickBall()
                    bufferCt = 0
        else:
            if lookingForBall == False:
                findBall(directionChoice)
                lookingForBall = True           
        if cv.WaitKey(10) == 27:
            print("Exiting Soccerbot")
            break      
    print("Cleaning up Soccerbot")
    StopMotors()
    GPIO.cleanup()
    cv.DestroyAllWindows()
    cam = CameraStream().stop()
