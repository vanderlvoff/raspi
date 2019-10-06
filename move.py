import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

SENSOR_PIN = 11

GPIO.setup(SENSOR_PIN,GPIO.IN)

def myCallback(self):
    print("there was a movement")
    
try:
    GPIO.add_event_detect(SENSOR_PIN, GPIO.RISING, callback=myCallback)
    while True:
        time.sleep(1)

except KeyboardInterrupt:
    print("finish")
    GPIO.cleanup()
        