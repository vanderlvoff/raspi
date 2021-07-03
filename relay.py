import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM) # GPIO Numbers instead of board numbers
 
RELAIS_1_GPIO = 24
GPIO.setup(RELAIS_1_GPIO, GPIO.OUT) # GPIO Assign mode
GPIO.output(RELAIS_1_GPIO, GPIO.LOW) # out
GPIO.output(RELAIS_1_GPIO, GPIO.HIGH) # on
#time.sleep(3)
#PIO.output(RELAIS_1_GPIO, GPIO.LOW) # on
#PIO.cleanup()

