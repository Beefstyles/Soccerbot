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
import signal

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
GPIO.setup(startScanPin, GPIO.OUT)
GPIO.setup(scanDonePin, GPIO.IN)

# The HSV range used to detect the coloured object
# Green ball
Hmin = 42
Hmax = 92
Smin = 62
Smax = 255
Vmin = 63
Vmax = 235


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


 # Creates a HSV array based on the min and maxium values (as an unsigned int)
rangeMin = np.array([Hmin, Smin, Vmin], np.uint8)
rangeMax = np.array([Hmax, Smax, Vmax], np.uint8)

# Minimum area to be detected
minArea = 25 #Default area = 50

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
            if moments['m00'] >= minArea:
                x = moments['m10'] / moments['m00']
                #y = moments['m01'] / moments['m00']
                lookingForBall = False
                #print(int(x))
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
