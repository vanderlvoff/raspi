# Import the PCA9685 module. Available in the bundle and here:
# https://github.com/adafruit/Adafruit_CircuitPython_PCA9685
from adafruit_pca9685 import PCA9685
from adafruit_motor import motor
from board import SCL, SDA
import busio
import time
import json

class DC:
    i2c = busio.I2C(SCL, SDA)
    # Create a simple PCA9685 class instance.
    pca = PCA9685(i2c)
    motor_rl = motor.DCMotor(pca.channels[10], pca.channels[9])
    motor_rr = motor.DCMotor(pca.channels[7], pca.channels[8])
    motor_hl = motor.DCMotor(pca.channels[2], pca.channels[1])
    motor_hr = motor.DCMotor(pca.channels[3], pca.channels[4])
    
    def __init__(self):
        self.pca.frequency = 100

    def forward(self):
        self.motor_rr.throttle = 1
        self.motor_hr.throttle = 1
        self.motor_rl.throttle = 1
        self.motor_hl.throttle = 1

    def stop(self):
        self.motor_rr.throttle = 0
        self.motor_hr.throttle = 0
        self.motor_rl.throttle = 0
        self.motor_hl.throttle = 0
    
    def right(self):
        self.motor_rr.throttle = -1
        self.motor_hr.throttle = -1
        self.motor_rl.throttle = 1
        self.motor_hl.throttle = 1

    def left(self):
        self.motor_rr.throttle = 1
        self.motor_hr.throttle = 1
        self.motor_rl.throttle = -1
        self.motor_hl.throttle = -1
    
    def back(self):
        self.motor_rr.throttle = -1
        self.motor_hr.throttle = -1
        self.motor_rl.throttle = -1
        self.motor_hl.throttle = -1