import RPi.GPIO as GPIO
import time
import sys
import os

class HardButtons:
    
    resetButtonPin = 16
    poweroffButtonPin = 17

    button_status = 0
    
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.resetButtonPin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(self.poweroffButtonPin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(
                self.resetButtonPin,
                GPIO.RISING
            )
        GPIO.add_event_detect(
            self.poweroffButtonPin,
            GPIO.FALLING,
            callback=self.poweroff
        )
        print("Initialized")
    def poweroff(self, channel):
        os.system('sudo shutdown now')

    def execute(self):
        print("Process started")
        while True:
            if GPIO.event_detected(self.resetButtonPin):
                print("Detected")
                self.reset_on_press(self.resetButtonPin)

    def reset_on_press(self, channel):
        print("pressed")
        start_time = time.time()
        state = True
        
        while state:
            button_time = time.time() - start_time
            status = GPIO.input(self.resetButtonPin)
            if 0 < button_time <= 5 and status == 1:
                print("Waiting: ", button_time, "; ", status)

            elif button_time > 5 and status == 1:
                print("Step two: ", button_time, "; ", status)
                GPIO.cleanup()
                sys.exit()
            
            elif status == 0:
                print("exit")
                state = False
            
            time.sleep(0.5)            
    
run = HardButtons()
run.execute()