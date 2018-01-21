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
# Altered by Glen Willis 2017 to include Arduino code and further movement
# Email: glenwillis@gmail.com
# Description: Open CV algorithm to transform an unfiltered image into one where a specific colour is highlighted (object in question)
# Used along with an Arduino helper program that runs the motors.

import cv2.cv as cv
import cv2 as cv2
import time
import numpy as np
import os
from time import sleep
import RPi.GPIO as GPIO
from threading import Thread

class CameraStream:

    def __init__(self):
        print("Initialising Camera")
        ret, unfilteredImage = capture.read()
        imgHSV = cv2.cvtColor(unfilteredImage,cv2.cv.CV_BGR2HSV)    
        imgThreshold = cv2.inRange(imgHSV, rangeMinBall, rangeMaxBall)
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
            # Looking for the ball - assumed green
            if lookingForGoal == False:
                imgThreshold = cv2.inRange(imgHSV, rangeMinBall, rangeMaxBall)
            # Looking for the goal - assumed blue
            elif lookingForGoal == True:
                imgThreshold = cv2.inRange(imgHSV, rangeMinGoal, rangeMaxGoal)
            imgErosion = cv2.erode(imgThreshold, None, iterations = 3)
            self._moments = cv2.moments(imgErosion, True)
            if self._moments['m00'] > 0:
                self._xPos = self._moments['m10'] / self._moments['m00']

            # Uncomment one or more of the below to display the results in these windows. Need to run in a GUI application in Raspbian or similar or will throw an error
            # cv2.imshow("Unfiltered Image", unfilteredImage)
            # cv2.imshow("HSV", imgHSV)
            # cv2.imshow("Threshold", imgThreshold)
            # cv2.imshow("Filtered Image", imgErosion)
                
        #sleep(1)

    def returnMonments(self):
        return self._moments

    def returnXPos(self):
        return self._xPos

    def stopSteam(self):
        capture.release()
        t.stop()

forwardMotorLeftActionPin = 9
forwardMotorRightActionPin = 10

kickBallPin = 27

ballIsTrappedPin = 22

lookingForBall = False
GPIO.setmode(GPIO.BCM)
GPIO.setup(forwardMotorLeftActionPin,GPIO.OUT)
GPIO.setup(forwardMotorRightActionPin,GPIO.OUT)

GPIO.setup(kickBallPin, GPIO.OUT)
GPIO.setup(ballIsTrappedPin, GPIO.IN)

# The HSV range used to detect the coloured object
# Green ball
HminBall = 42
HmaxBall = 92
SminBall = 62
SmaxBall = 255
VminBall = 63
VmaxBall = 235

"""
# Light red goal
HminGoal = 165
HmaxGoal = 174
SminGoal = 100
SmaxGoal = 255
VminGoal = 205
VmaxGoal = 255
"""

# Blue goal
HminGoal = 95
HmaxGoal = 113
SminGoal = 65
SmaxGoal = 255
VminGoal = 205
VmaxGoal = 255


# Image capture window parameters
width = 1000 #Default value is 500
height = 500 #Default value

centrePoint = 317.5 #Based on a width of 1000 at max x of 635 -> observational value
height = 500 # Default 120
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
rangeMinBall = np.array([HminBall, SminBall, VminBall], np.uint8)
rangeMaxBall = np.array([HmaxBall, SmaxBall, VmaxBall], np.uint8)

 # Creates a HSV array based on the min and maxium values (as an unsigned int) - For the Blue Goal
rangeMinGoal = np.array([HminGoal, SminGoal, VminGoal], np.uint8)
rangeMaxGoal = np.array([HmaxGoal, SmaxGoal, VmaxGoal], np.uint8)

# Minimum area to be detected
minArea = 25 # Default area = 50

# Maximum area to be detected - Determines distance from goal
maxArea = 5000 


# Uncomment the below to display the windows that shows what the camera is seeing and the transformation of the images
#cv.NamedWindow("Unfiltered Image")
#cv.NamedWindow("HSV")
#cv.NamedWindow("Threshold")
#cv.NamedWindow("Filtered Image")


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
    GPIO.output(forwardMotorRightActionPin, 0)

def findBall(directionChoice):
    if(directionChoice == 0): #Turn Left
        RotateClockwise()
        print "Looking for the ball to the Left"
    else:
        RotateAntiClockwise() #Turn Right
        print "Looking for the ball to the Right"

def KickBall():
    StopMotors()
    print("Found goal and taking a shot!")
    GPIO.output(kickBallPin, 1)
    sleep(1)
    print("Kicked ball. Did I hit it?")
    GPIO.output(kickBallPin, 0)
    lookingForGoal = False
    sleep(2)

print("Soccerbot initialised. Starting ball search now")

StopMotors()
cam = CameraStream().start()
time.sleep(1.0)

try:
    if __name__== '__main__':
        while True:
            moments = cam.returnMonments()
            if GPIO.input(ballIsTrappedPin) and lookingForGoal == False:
                print("Ball is trapped, now looking for the goal")
                lookingForGoal = True
                time.sleep(5)
                moments = cam.returnMonments()
            if moments['m00'] >= minArea:
                x = moments['m10'] / moments['m00']
                lookingForBall = False
                # Print x to manually calibrate centre point
                # print(x)
                if(int(x) < (centrePoint - 135)): #Default is 50
                    print("To the right")
                    directionChoice = 1
                    RotateClockwise()
                elif(int(x) > (centrePoint + 100)): #Default is 50
                    print("To the Left")
                    directionChoice = 0
                    RotateAntiClockwise()                 
                else:
                    print("Centered")
                    ForwardFull()
                if lookingForGoal == True:
                    if moments['m00'] >= maxArea:
                        lookingForGoal = False
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
    GPIO.output(kickBallPin, 0)
    GPIO.cleanup()
    cv.DestroyAllWindows()
    capture.release()
    
