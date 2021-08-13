# Import the PCA9685 module. Available in the bundle and here:
# https://github.com/adafruit/Adafruit_CircuitPython_PCA9685
from adafruit_servokit import ServoKit
import time
import json

class ServoMotors:

    #Constants
    nbPCAServo=16
    
    local_angle = 90
    current_servo_angle = 90
    
    #Objects
    pca = ServoKit(channels=16, address=0x41)
    
    # Create a simple PCA9685 class instance.
    DEFAULT_NUMBER = 0
    servoMotor = pca.servo[DEFAULT_NUMBER]
    
    def __init__(self, channel):
        self.servoMotor = self.pca.servo[channel]

    def moveForward(self):
        print("Move Forward")
        self.local_angle = self.current_servo_angle + 1
        self.servoMotor.angle = self.local_angle
        self.current_servo_angle = self.local_angle
        print(self.local_angle)

    def moveBack(self):
        self.local_angle = self.current_servo_angle - 1
        self.servoMotor.angle = self.local_angle
        self.current_servo_angle = self.local_angle
        print(self.local_angle)

    def init(self, angle = 72):
        self.servoMotor.angle = angle
        self.current_servo_angle = angle
        
        
#sv = ServoMotors(channel = 0)
#sv.moveForward()
#sv.moveForward()