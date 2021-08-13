# Import the PCA9685 module. Available in the bundle and here:
# https://github.com/adafruit/Adafruit_CircuitPython_PCA9685
from adafruit_pca9685 import PCA9685
from board import SCL, SDA
import busio
import time
import json

class DC4WD:
    i2c = busio.I2C(SCL, SDA)
    # Create a simple PCA9685 class instance.
    pca = PCA9685(i2c, address=0x40)
    speed = 64000
    
    def __init__(self):
        self.pca.frequency = 1500
        
    def forward(self):
        self.pca.channels[12].duty_cycle = 0xffff
        self.pca.channels[13].duty_cycle = 0
        self.pca.channels[14].duty_cycle = self.speed
        
        #left - rear
        self.pca.channels[2].duty_cycle = 0xffff
        self.pca.channels[1].duty_cycle = 0
        self.pca.channels[0].duty_cycle = self.speed
        
        #right - rear
        self.pca.channels[6].duty_cycle = 0xffff
        self.pca.channels[5].duty_cycle = 0
        self.pca.channels[7].duty_cycle = self.speed
        
        #right - head
        self.pca.channels[9].duty_cycle = 0xffff
        self.pca.channels[10].duty_cycle = 0
        self.pca.channels[8].duty_cycle = self.speed

    def stop(self):
        self.pca.channels[12].duty_cycle = 0
        self.pca.channels[13].duty_cycle = 0
        
        #left - rear
        self.pca.channels[2].duty_cycle = 0
        self.pca.channels[1].duty_cycle = 0

        
        #right - rear
        self.pca.channels[6].duty_cycle = 0
        self.pca.channels[5].duty_cycle = 0
                
        #right - head
        self.pca.channels[9].duty_cycle = 0
        self.pca.channels[10].duty_cycle = 0

    
    def right(self):
        #left - head
        self.pca.channels[12].duty_cycle = 0xffff
        self.pca.channels[13].duty_cycle = 0
        self.pca.channels[14].duty_cycle = self.speed
        
        #left - rear
        self.pca.channels[2].duty_cycle = 0xffff
        self.pca.channels[1].duty_cycle = 0
        self.pca.channels[0].duty_cycle = self.speed
        
        #right - rear
        self.pca.channels[6].duty_cycle = 0
        self.pca.channels[5].duty_cycle = 0xffff
        self.pca.channels[7].duty_cycle = self.speed
        
        #right - head
        self.pca.channels[9].duty_cycle = 0
        self.pca.channels[10].duty_cycle = 0xffff
        self.pca.channels[8].duty_cycle = self.speed
        
        
    def move_right(self):
        #left - head
        self.pca.channels[12].duty_cycle = 0xffff
        self.pca.channels[13].duty_cycle = 0
        self.pca.channels[14].duty_cycle = self.speed
        
        #left - rear
        self.pca.channels[2].duty_cycle = 0
        self.pca.channels[1].duty_cycle = 0xffff
        self.pca.channels[0].duty_cycle = self.speed
        
        #right - rear
        self.pca.channels[6].duty_cycle = 0xffff
        self.pca.channels[5].duty_cycle = 0
        self.pca.channels[7].duty_cycle = self.speed
        
        #right - head
        self.pca.channels[9].duty_cycle = 0
        self.pca.channels[10].duty_cycle = 0xffff
        self.pca.channels[8].duty_cycle = self.speed
        
    def move_left(self):
        
        #left - head
        self.pca.channels[12].duty_cycle = 0
        self.pca.channels[13].duty_cycle = 0xffff
        self.pca.channels[14].duty_cycle = self.speed
        
        #left - rear
        self.pca.channels[2].duty_cycle = 0xffff
        self.pca.channels[1].duty_cycle = 0
        self.pca.channels[0].duty_cycle = self.speed
        
        #right - rear
        self.pca.channels[6].duty_cycle = 0
        self.pca.channels[5].duty_cycle = 0xffff
        self.pca.channels[7].duty_cycle = self.speed
        
        #right - head
        self.pca.channels[9].duty_cycle = 0xffff
        self.pca.channels[10].duty_cycle = 0
        self.pca.channels[8].duty_cycle = self.speed    
        

    def left(self):
        #left - head
        self.pca.channels[12].duty_cycle = 0
        self.pca.channels[13].duty_cycle = 0xffff
        self.pca.channels[14].duty_cycle = self.speed
        
        #left - rear
        self.pca.channels[2].duty_cycle = 0
        self.pca.channels[1].duty_cycle = 0xffff
        self.pca.channels[0].duty_cycle = self.speed
        
        #right - rear
        self.pca.channels[6].duty_cycle = 0xffff
        self.pca.channels[5].duty_cycle = 0
        self.pca.channels[7].duty_cycle = self.speed
        
        #right - head
        self.pca.channels[9].duty_cycle = 0xffff
        self.pca.channels[10].duty_cycle = 0
        self.pca.channels[8].duty_cycle = self.speed
    
    def back(self):
        #left - head
        self.pca.channels[12].duty_cycle = 0
        self.pca.channels[13].duty_cycle = 0xffff
        self.pca.channels[14].duty_cycle = self.speed
        
        #left - rear
        self.pca.channels[2].duty_cycle = 0
        self.pca.channels[1].duty_cycle = 0xffff
        self.pca.channels[0].duty_cycle = self.speed
        
        #right - rear
        self.pca.channels[6].duty_cycle = 0
        self.pca.channels[5].duty_cycle = 0xffff
        self.pca.channels[7].duty_cycle = self.speed
        
        #right - head
        self.pca.channels[9].duty_cycle = 0
        self.pca.channels[10].duty_cycle = 0xffff
        self.pca.channels[8].duty_cycle = self.speed
        
drive = DC4WD()    
drive.back()
drive.stop()