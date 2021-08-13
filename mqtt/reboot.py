import RPi.GPIO as GPIO
import os
import time

class RebootButton:
    
    rebootButton = 17
    
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.rebootButton, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    def execute(self):
        while True:
            input_state = GPIO.input(self.rebootButton)
            if input_state == False:
                print("Button pressed")
                time.sleep(0.4)
                #os.system('sudo shutdown -r now')
                
run = RebootButton()
run.execute()