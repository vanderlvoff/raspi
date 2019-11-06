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
    getDirectionFlag = False 
    
    LE_MAX = 1
    LE_MAX_NEGATIVE = -1
    LE_CORRECTION = 0.7
    right_index = 100
    left_index = 100
    
    RE_MAX = 1
    RE_MAX_NEGATIVE = -1
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

    def setPower(self, rightEngine, leftEngine):
        self.right_index = rightEngine/100
        self.left_index = leftEngine/100
        print(self.right_index)

    def forward(self, speed):
        self.getDirectionFlag = True
        self.controlCourseFlag = True
        self.motor_hr.throttle = self.right_index * self.RE_MAX * speed /10
        self.motor_hl.throttle = self.left_index * self.LE_MAX * speed /10
      #  x = threading.Thread(target=self.getDirection).start()

    def getDirection(self):
        print(str(self.getDirectionFlag))
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
            
            if (deg < self.controlCourse - 1):
                self.courseCorrection(direction = "right")
            elif (deg > self.controlCourse + 1):
                self.courseCorrection(direction = "left")
            #else:
                #self.courseCorrection(direction = "ahead")
                #self.getDirectionFlag = False
                
            sleepTime = 1 - (time.time() - now)
            if sleepTime < 0.0:
                continue
            time.sleep(sleepTime)

    def courseCorrection(self, direction):
        
        if (direction == "left"):
            hr = 0.3 #self.RE_MAX
            hl = 0 #self.LE_CORRECTION
        elif (direction == "right"):
            hr = 0 # self.RE_CORRECTION
            hl = 0.3 #self.LE_MAX
        else:
            return
        
        if (self.getDirectionFlag == False):
            return
        
        self.motor_hr.throttle = hr
        self.motor_hl.throttle = hl
        return
        
    def stop(self):
        self.getDirectionFlag = False
        self.motor_hr.throttle = 0
        self.motor_hl.throttle = 0
    
    def right(self, speed):
        self.getDirectionFlag = False
        self.motor_hr.throttle = self.right_index * self.RE_MAX_NEGATIVE*speed/10
        self.motor_hl.throttle = self.left_index * self.LE_MAX*speed/10

    def left(self, speed):
        self.getDirectionFlag = False
        self.motor_hr.throttle = self.right_index * self.RE_MAX*speed/10
        self.motor_hl.throttle = self.left_index * self.LE_MAX_NEGATIVE*speed/10
    
    def back(self, speed):
        self.getDirectionFlag = False
        self.motor_hr.throttle = self.right_index * self.RE_MAX_NEGATIVE*speed/10
        self.motor_hl.throttle = self.left_index * self.LE_MAX_NEGATIVE*speed/10
        