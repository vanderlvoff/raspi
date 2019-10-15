import RPi.GPIO as GPIO
import time

class Led:
    LED_1 = 20
    LED_2 = 21
    
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.LED_1,GPIO.OUT)
        GPIO.setup(self.LED_2,GPIO.OUT)

    def lightsOn(self):
        GPIO.output(self.LED_1,GPIO.HIGH)
        GPIO.output(self.LED_2,GPIO.HIGH)

    def lightsOff(self):
        GPIO.output(self.LED_1,GPIO.LOW)
        GPIO.output(self.LED_2,GPIO.LOW)


