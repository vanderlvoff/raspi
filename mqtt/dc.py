# Import the PCA9685 module. Available in the bundle and here:
# https://github.com/adafruit/Adafruit_CircuitPython_PCA9685
from adafruit_pca9685 import PCA9685
from adafruit_motor import motor
from board import SCL, SDA
import busio
import time
import json
import threading
import math
from magneto import SL_MPU9250

class DC:
    
    controlCourse = 0 
    controlCourseFlag = True
    getDirectionFlag = True
    
    LE_MAX = 0.80
    LE_CORRECTION = 0.7
    
    RE_MAX = 1
    RE_CORRECTION = 0.8
    
    i2c = busio.I2C(SCL, SDA)
    # Create a simple PCA9685 class instance.
    pca = PCA9685(i2c, address=0x40)
    motor_hl = motor.DCMotor(pca.channels[2], pca.channels[1])
    motor_hr = motor.DCMotor(pca.channels[3], pca.channels[4])
    
    #Magnetometer
    sensor = SL_MPU9250(0x68,1)
    sensor.resetRegister()
    sensor.powerWakeUp()
    sensor.setMagRegister('100Hz','16bit')

    def __init__(self):
        self.pca.frequency = 100

    def forward(self):
        self.getDirectionFlag = True
        self.motor_hr.throttle = 1
        self.motor_hl.throttle = self.LE_MAX
        x = threading.Thread(target=self.getDirection).start()

    def getDirection(self):
        while self.getDirectionFlag:
            now = time.time()
            mag = self.sensor.getMag()
            print ("%+8.7f" % mag[0] + " ")
            print ("%+8.7f" % mag[1] + " ")
            print ("%+8.7f" % mag[2])
            deg = round(math.atan2(mag[1], mag[0]) * 180/math.pi)+180
            print (str(deg))
            if (self.controlCourseFlag):
                self.controlCourse = deg
                self.controlCourseFlag = False
            
            if (deg < self.controlCourse):
                self.courseCorrection(direction = "right")
            elif (deg > self.controlCourse):
                self.courseCorrection(direction = "left")
            else:
                self.courseCorrection(direction = "ahead")
                
            sleepTime = 1 - (time.time() - now)
            if sleepTime < 0.0:
                continue
            time.sleep(sleepTime)

    def courseCorrection(self, direction):
        
        if (direction == "left"):
            hr = self.RE_MAX
            hl = self.LE_CORRECTION
        elif (direction == "right"):
            hr = self.RE_CORRECTION
            hl = self.LE_MAX
        else:
            hr = self.RE_MAX
            hl = self.LE_MAX
        
        if (self.getDirectionFlag == False):
            return
        
        self.motor_hr.throttle = hr
        self.motor_hl.throttle = hl
        
    def stop(self):
        self.getDirectionFlag = False
        self.motor_hr.throttle = 0
        self.motor_hl.throttle = 0
    
    def right(self):
        self.getDirectionFlag = False
        self.motor_hr.throttle = -1
        self.motor_hl.throttle = self.LE_MAX

    def left(self):
        self.getDirectionFlag = False
        self.motor_hr.throttle = 1
        self.motor_hl.throttle = self.LE_MAX * -1
    
    def back(self):
        self.getDirectionFlag = False
        self.motor_hr.throttle = -1
        self.motor_hl.throttle = self.LE_MAX * -1
        