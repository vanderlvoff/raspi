import RPi.GPIO as GPIO
import threading
from dc import DC
from relay import Relay
from ultrasonic_sensor import UltrasonicSensor

class Robot:
    dcmotor = DC()
    relay = Relay()
    sensor = UltrasonicSensor()
    status = 0

    def startProgram(self):
        self.relay.switchRelay(1)
        self.dcmotor.forward()
        self.status = 1
        x = threading.Thread(target=get_distance, args=(1,))
        x.start()
    
    def get_distance(self):
        while True:
            time.sleep(0.5)
            dist = sensor.distance()
            if dist < 10:
                self.motorStop()
                self.status = 0
            else if dist > 10 and self.status = 0:
                self.dcmotor.forward()
                

    def motorStop(self):
        self.dcmotor.stop()
        self.relay.switchRelay(0)