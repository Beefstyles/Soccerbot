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

import cv2.cv as cv
import cv2 as cv2
import time
import numpy as np
import os
from time import sleep
import RPi.GPIO as GPIO

from random import randint

forwardMotorLeftActionPin = 9
forwardMotorRightActionPin = 10
reverseMotorLeftActionPin = 22
reverseMotorRightActionPin = 27

startScanPin = 17

servoKick = 18

scanDonePin = 11
scanDonePinVal = 0

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
# This example is for a green ball
Hmin = 42
Hmax = 92
Smin = 62
Smax = 255
Vmin = 63
Vmax = 235


#Standard Red example
#Hmin = 0
#Hmax = 179 
#Smin = 131
#Smax = 255
#Vmin = 126
#Vmax = 255

width = 160 * 2
centrePoint = width / 2
height = 120 * 2
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
minArea = 500
# Max area to be detected to activate kick
#maxArea = 70000
maxArea = 65000

#cv.NamedWindow("Unfiltered Image")
#cv.NamedWindow("HSV")
#cv.NamedWindow("Threshold")
#cv.NamedWindow("Filtered Image")


capture = cv2.VideoCapture(0)

# Image capture window parameters
#width = 160 #Default value
#height = 120 #Default value

def RotateClockwise():
    GPIO.output(forwardMotorLeftActionPin, 1)
    GPIO.output(reverseMotorLeftActionPin, 0)
    GPIO.output(forwardMotorRightActionPin, 0)
    GPIO.output(reverseMotorRightActionPin, 0)

def RotateAntiClockwise():
    GPIO.output(forwardMotorRightActionPin, 1)
    GPIO.output(forwardMotorLeftActionPin, 0)
    GPIO.output(reverseMotorLeftActionPin, 0)
    GPIO.output(forwardMotorRightActionPin, 0)

def ForwardFull():
    GPIO.output(forwardMotorRightActionPin, 1)
    GPIO.output(forwardMotorLeftActionPin, 1)
    GPIO.output(reverseMotorLeftActionPin, 0)
    GPIO.output(forwardMotorRightActionPin, 0)

def findBall(directionChoice):
    if(directionChoice == 0): #Turn Left
        RotateClockwise()
        #backwardsMotor1(delay)
    else:
        RotateAntiClockwise() #Turn Right
        #backwardsMotor2(delay)
        
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
    #print("Ball kicked")
    
    
    # 12.5 is 180
    # 7.5 is neutral
    # 2.5 is 0

# Set a size the for the frames (discarding the Pyramid Down)
if capture.isOpened():
  capture.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, width)
  capture.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, height)

print("Soccerbot initialised. Starting ball search now")

GPIO.output(startScanPin,0)
startScan = False
while True:
    ret, unfilteredImage = capture.read()
    imgHSV = cv2.cvtColor(unfilteredImage,cv2.cv.CV_BGR2HSV)    
    imgThreshold = cv2.inRange(imgHSV, rangeMin, rangeMax)
    imgErosion = cv2.erode(imgThreshold, None, iterations = 3)
    moments = cv2.moments(imgErosion, True)
    #print(moments['m00'])
    if startScan == True and directionSet == False:
        #print(GPIO.input(scanDonePin))
        if GPIO.input(scanDonePin):
            directionSet = True
            directionChoice = 0
            print("To the right")        
    if bufferCt <= bufferMax:
        bufferCt += 1 
    xCalc = moments['m10']
    yCalc = moments['m01']
    if moments['m00'] >= minArea:
        if moments['m00'] <= maxArea:
            if startScan == True:
                print("Found to the left")
                directionChoice = 1
                startScan = False
                directionSet = True
            x = moments['m10'] / moments['m00']
            y = moments['m01'] / moments['m00']
            #cv2.circle(unfilteredImage, (int(x), int(y)), 5, (0, 0, 255), -1)
            if(int(x) < (centrePoint - 50)):
                print("To the left")
                RotateClockwise()
            elif(int(x) > (centrePoint + 50)):
                print("To the Right")
                RotateAntiClockwise()
            else:
                print("Centered")
                ForwardFull()
        else:
            if bufferCt >= bufferMax:
                kickBall()
                bufferCt = 0
                #sleep(1)
    else:
        #findBall(directionChoice)
        if startScan == False and directionSet == False:
            startScan = True
            scanFindBall()
        else:
            if directionSet == True:
                findBall(directionChoice)            
    # cv2.imshow("Unfiltered Image",unfilteredImage)
    # cv2.imshow("HSV", imgHSV)
    # cv2.imshow("Threshold", imgThreshold)
    # cv2.imshow("Filtered Image", imgErosion)
    if cv.WaitKey(10) == 27:
        break

cv.DestroyAllWindows()
