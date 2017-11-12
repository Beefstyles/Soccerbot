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
            # Looking for the green ball
            if lookingForGoal == False:
                imgThreshold = cv2.inRange(imgHSV, rangeMinGreen, rangeMaxGreen)
            # Looking for the blue goal
            else:
                imgThreshold = cv2.inRange(imgHSV, rangeMinBlue, rangeMaxBlue)
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
import signal

from random import randint

forwardMotorLeftActionPin = 9
forwardMotorRightActionPin = 10
reverseMotorLeftActionPin = 22
reverseMotorRightActionPin = 27
kickBallPin = 17

ballIsTrappedPin = 11

lookingForBall = False
GPIO.setmode(GPIO.BCM)
GPIO.setup(forwardMotorLeftActionPin,GPIO.OUT)
GPIO.setup(forwardMotorRightActionPin,GPIO.OUT)
GPIO.setup(reverseMotorLeftActionPin,GPIO.OUT)
GPIO.setup(reverseMotorRightActionPin,GPIO.OUT)
GPIO.setup(kickBallPin, GPIO.OUT)
GPIO.setup(ballIsTrappedPin, GPIO.IN)

# The HSV range used to detect the coloured object
# Green ball
HminGreen = 42
HmaxGreen = 92
SminGreen = 62
SmaxGreen = 255
VminGreen = 63
VmaxGreen = 235


# Blue goal
HminBlue = 88
HmaxBlue = 113
SminBlue = 62
SmaxBlue = 255
VminBlue = 63
VmaxBlue = 235

# Image capture window parameters
width = 1000 #Default value is 500
height = 500 #Default value

#centrePoint = width / 2
centrePoint = 440 #Based on a width of 1000 -> observational value
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

lookingForGoal = False


 # Creates a HSV array based on the min and maxium values (as an unsigned int) - For the Green Ball
rangeMinGreen = np.array([HminGreen, SminGreen, VminGreen], np.uint8)
rangeMaxGreen = np.array([HmaxGreen, SmaxGreen, VmaxGreen], np.uint8)

 # Creates a HSV array based on the min and maxium values (as an unsigned int) - For the Blue Goal
rangeMinBlue = np.array([HminBlue, SminBlue, VminBlue], np.uint8)
rangeMaxBlue = np.array([HmaxBlue, SmaxBlue, VmaxBlue], np.uint8)

# Minimum area to be detected
minArea = 25 #Default area = 50

# Maximum area to be detected - Determines distance from goal
maxArea = 1500 

capture = cv2.VideoCapture(0)
    
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
        print "Looking for the ball to the Left"
    else:
        RotateAntiClockwise() #Turn Right
        print "Looking for the ball to the Right"
        
def scanFindBall():
    startScan = True
    GPIO.output(startScanPin,1)
    sleep(0.01)
    GPIO.output(startScanPin,0)

def KickBall():
    StopMotors()
    print("Found goal and taking a shot!")
    GPIO.output(kickBallPin, 1)
    print("Kicked ball. Did I hit it?")
    GPIO.output(kickBallPin, 0)
    lookingForGoal = False
    sleep(2)

print("Soccerbot initialised. Starting ball search now")

StopMotors()
GPIO.output(startScanPin,0)
startScan = False
cam = CameraStream().start()
time.sleep(1.0)

try:
    if __name__== '__main__':
        while True:
            moments = cam.returnMonments()
            if (GPIO.input(ballIsTrappedPin) and lookingForGoal == False):
                print("Ball is trapped, now looking for the goal")
                lookingForGoal = True
            if moments['m00'] >= minArea:
                x = moments['m10'] / moments['m00']
                lookingForBall = False
                if(int(x) < (centrePoint - 100)): #Default is 50
                    print("To the right")
                    directionChoice = 1
                    RotateClockwise()
                elif(int(x) > (centrePoint + 50)): #Default is 50
                    print("To the Left")
                    directionChoice = 0
                    RotateAntiClockwise()                 
                else:
                    print("Centered")
                    ForwardFull()
                if (moments['m00'] >= maxArea and lookingForGoal == True):
                    print("Goal close enough")
                    KickBall()
            else:
                if lookingForBall == False:
                    findBall(directionChoice)
                    lookingForBall = True           
            if cv.WaitKey(10) == 27:
                print("Exiting Soccerbot")
                break      
except KeyboardInterrupt:
    print("Soccerbot interrupted")
finally:
    print("Exiting soccerbot")
    StopMotors()
    GPIO.cleanup()
    cv.DestroyAllWindows()
    capture.release()
