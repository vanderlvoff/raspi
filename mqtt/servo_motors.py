# Import the PCA9685 module. Available in the bundle and here:
# https://github.com/adafruit/Adafruit_CircuitPython_PCA9685
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
from board import SCL, SDA
import busio
import time
import json

class ServoMotors:

    local_angle = 90
    current_servo_angle = 90
    i2c = busio.I2C(SCL, SDA)
    # Create a simple PCA9685 class instance.
    pca = PCA9685(i2c, address=0x41)
    DEFAULT_NUMBER = 11
    servoMotor = servo.Servo(pca.channels[DEFAULT_NUMBER])
    
    def __init__(self, channel):
        self.pca.frequency = 50
        self.servoMotor = servo.Servo(self.pca.channels[channel])

    def moveForward(self):
        self.local_angle = self.current_servo_angle + 1
        self.servoMotor.angle = self.local_angle
        self.current_servo_angle = self.local_angle

    def moveBack(self):
        self.local_angle = self.current_servo_angle - 1
        self.servoMotor.angle = self.local_angle
        self.current_servo_angle = self.local_angle

    def init(self, angle = 72):
        self.servoMotor.angle = angle
        self.current_servo_angle = angle