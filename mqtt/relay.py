import RPi.GPIO as GPIO

class Relay:
    RELAYS_1_GPIO = 24
    
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.RELAYS_1_GPIO, GPIO.OUT)

    def switchRelay(self, status):
        if status == 0:
            GPIO.output(self.RELAYS_1_GPIO, GPIO.LOW)
        else:
            GPIO.output(self.RELAYS_1_GPIO, GPIO.HIGH)