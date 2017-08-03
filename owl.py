# Simple demo of of the PCA9685 PWM servo/LED controller library.
# This will move channel 0 from min to max position repeatedly.
# Author: Tony DiCola
# License: Public Domain
from __future__ import division
from thread import start_new_thread
import time
import subprocess
# Import the PCA9685 module.
import Adafruit_PCA9685
import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy as np
camera = PiCamera()
camera.resolution = (240, 180)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(240, 180))
face_cascade =  cv2.CascadeClassifier('/home/pi/Adafruit/examples/face.xml')
time.sleep(0.1)
eye_min = 425
eye_max = 580
head_min = 150
head_max = 650
left_wing_min = 250
left_wing_max = 375
right_wing_min = 275
right_wing_max = 375
# Uncomment to enable debug output.
#import logging
#logging.basicConfig(level=logging.DEBUG)

# Initialise the PCA9685 using the default address (0x40).
pwm = Adafruit_PCA9685.PCA9685()
flapping = False
faceThread = False
blinking = False
faces = []
def faceDetect(img):
    global faces
    global faceThread
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray, 1.3, 5)
    faceThread = False
def blink():
    global pwm
    global eye_max
    global eye_min
    global blinking
    while True:
        if blinking:
            print "blink"
            try:
                pwm.set_pwm(4,0,eye_max)
                time.sleep(.3)
                pwm.set_pwm(4,0,eye_min)
            except IOError:
                subprocess.call(['i2cdetect','-y','1'])
            blinking = False
def flapWings():
    global pwm
    global flapping
    # Configure min and max servo pulse lengths
    back_min = 250
    back_max = 475
    while True:
        while flapping:
            try:
                pwm.set_pwm(2, 0, back_min)
                time.sleep(.4)
                pwm.set_pwm(2, 0, back_max)
                time.sleep(.4)
            except IOError:
                subprocess.call(['i2cdetect','-y','1'])
# Alternatively specify a different address and/or bus:
#pwm = Adafruit_PCA9685.PCA9685(address=0x41, busnum=2)

# Helper function to make setting a servo pulse width simpler.
def set_servo_pulse(channel, pulse):
    pulse_length = 1000000    # 1,000,000 us per second
    pulse_length //= 60       # 60 Hz
    print('{0}us per period'.format(pulse_length))
    pulse_length //= 4096     # 12 bits of resolution
    print('{0}us per bit'.format(pulse_length))
    pulse *= 1000
    pulse //= pulse_length
    pwm.set_pwm(channel, 0, pulse)

# Set frequency to 60hz, good for servos.
pwm.set_pwm_freq(60)
start_new_thread(flapWings,())
targetEyes = time.time()
targetWings = time.time()
for frame in camera.capture_continuous(rawCapture, format="bgr",  use_video_port=True):
    print "main loop"
    curTime = time.time()
    if not faceThread:
        faceThread = True
        img=np.asarray(frame.array)
        start_new_thread(faceDetect,(img,))
    try:
        if len(faces)>0 and faces[0][2]+faces[0][3] > 100:
            flapping = True
            print "face found"
            pwm.set_pwm(4,0,eye_min)
            centerX = float(faces[0][0]+faces[0][2]/float(2))
            pwm.set_pwm(0,0,left_wing_max)
            pwm.set_pwm(1,0,right_wing_max)
            pwm.set_pwm(3,0,int(((centerX/240.0)*500)+150))
            targetEyes = curTime+15
            targetWings = curTime+2
        elif len(faces) > 0:
            print "face found"
            pwm.set_pwm(4,0,eye_min)
            centerX = float(faces[0][0]+faces[0][2]/float(2))
            pwm.set_pwm(3,0,int(((centerX/240.0)*500)+150))
            if targetWings - curTime<=0:
                pwm.set_pwm(0,0,left_wing_min)
                pwm.set_pwm(1,0,right_wing_min)
                flapping = False
            targetEyes = curTime+15
            #if curTime%3==0 and not blinking:
            #   print "blink time"
            #   blinking = True
        else:
            if targetWings - curTime<= 0:
                pwm.set_pwm(0,0,left_wing_min)
                pwm.set_pwm(1,0,right_wing_min)
                flapping = False
            if targetEyes - curTime <=0:
                pwm.set_pwm(4,0,eye_max)
            #elif curTime%3==0 and not blinking:
            #   print "blink time"
            #   blinking = True
    except IOError:
        subprocess.call(['i2cdetect','-y','1'])
    except Exception as e:
        print e
    rawCapture.truncate(0)
