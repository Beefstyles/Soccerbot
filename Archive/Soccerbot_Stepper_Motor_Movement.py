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

sMotor1Step = 2
sMotor1Dir = 3
sMotor2Step = 17
sMotor2Dir = 27
servoKick = 18

GPIO.setmode(GPIO.BCM)
GPIO.setup(sMotor1Step,GPIO.OUT)
GPIO.setup(sMotor1Dir,GPIO.OUT)
GPIO.setup(sMotor2Step,GPIO.OUT)
GPIO.setup(sMotor2Dir,GPIO.OUT)
GPIO.setup(servoKick,GPIO.OUT)
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
delay = (1 / 1000.0)
ballKicked = False
directionChoice = 0
directionSet = False
maxTime = 2
bufferCt = 0
bufferMax = 10


 # Creates a HSV array based on the min and maxium values (as an unsigned int)
rangeMin = np.array([Hmin, Smin, Vmin], np.uint8)
rangeMax = np.array([Hmax, Smax, Vmax], np.uint8)

# Minimum area to be detected
minArea = 50

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

def forwardsMotor1(delay):
    GPIO.output(sMotor1Dir, 0)
    GPIO.output(sMotor1Step,1)
    sleep(delay)
    GPIO.output(sMotor1Step,0)
    sleep(delay)

def backwardsMotor1(delay):
    GPIO.output(sMotor1Dir, 1)
    GPIO.output(sMotor1Step,1)
    sleep(delay)
    GPIO.output(sMotor1Step,0)
    sleep(delay)

def forwardsMotor2(delay):
    GPIO.output(sMotor2Dir, 1)
    GPIO.output(sMotor2Step,1)
    sleep(delay)
    GPIO.output(sMotor2Step,0)
    sleep(delay)

def backwardsMotor2(delay):
    GPIO.output(sMotor2Dir, 0)
    GPIO.output(sMotor2Step,1)
    sleep(delay)
    GPIO.output(sMotor2Step, 0)
    sleep(delay)

def findBall(directionChoice):
    if(directionChoice == 0):
        forwardsMotor2(delay)
        #backwardsMotor1(delay)
    else:
        forwardsMotor1(delay)
        #backwardsMotor2(delay)

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

while True:
    ret, unfilteredImage = capture.read()
    imgHSV = cv2.cvtColor(unfilteredImage,cv2.cv.CV_BGR2HSV)	
    imgThreshold = cv2.inRange(imgHSV, rangeMin, rangeMax)
    imgErosion = cv2.erode(imgThreshold, None, iterations = 3)
    moments = cv2.moments(imgErosion, True)
    if bufferCt <= bufferMax:
        bufferCt += 1
    xCalc = moments['m10']
    yCalc = moments['m01']
    if moments['m00'] >= minArea:
        if moments['m00'] <= maxArea:
            #print("moments is currently %d" % moments['m00'])
            x = moments['m10'] / moments['m00']
            y = moments['m01'] / moments['m00']
            #x = xCalc / areaCalc
            #y = yCalc / areaCalc
            if directionSet == True:
                directionSet = False
            #print(x, ", ", y)
            #print(int(x))
            #cv2.circle(unfilteredImage, (int(x), int(y)), 5, (0, 0, 255), -1)
            if(int(x) < (centrePoint - 50)):
                forwardsMotor2(delay)
                #backwardsMotor1(delay)
                #print("Turning to the right")
            elif(int(x) > (centrePoint + 50)):
                forwardsMotor1(delay)
                #backwardsMotor2(delay)
                #print("Turning to the left")
            else:
                forwardsMotor1(delay)
                forwardsMotor2(delay)
                #print("Ball is centred")
        else:
            #print("Would kick now")
            #print("moments is currently %d" % moments['m00'])
	    if bufferCt >= bufferMax:
                print("Buffer ct is now %d" % bufferCt)
                kickBall()
                bufferCt = 0
                #print("Sleeping 1s")
                sleep(1)
                #print("Back to it")
    else:
        if directionSet == False:
            directionChoice = randint(0,1)
            #print("moments is currently %d" % moments['m00'])
            #if(directionChoice==0):
                #print("Cannot find ball, rotating clockwise")
            #else:
                #print("Cannot find ball, rotating anti-clockwise")
            directionSet = True
        findBall(directionChoice)
    # cv2.imshow("Unfiltered Image",unfilteredImage)
    # cv2.imshow("HSV", imgHSV)
    # cv2.imshow("Threshold", imgThreshold)
    # cv2.imshow("Filtered Image", imgErosion)
    sleep(0.01)
    if cv.WaitKey(10) == 27:
        break

cv.DestroyAllWindows()
