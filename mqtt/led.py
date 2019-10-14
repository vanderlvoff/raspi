import RPi.GPIO as GPIO
import time

class led:
    LED_1 = 20
    LED_2 = 21
    
    def __init__():
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.LED_1,GPIO.OUT)
        GPIO.setup(self.LED_2,GPIO.OUT)

    def lightsOn:
        GPIO.output(LED_1,GPIO.HIGH)
        GPIO.output(LED_2,GPIO.HIGH)

    def lightsOff:
        GPIO.output(LED_1,GPIO.LOW)
        GPIO.output(LED_2,GPIO.LOW)


