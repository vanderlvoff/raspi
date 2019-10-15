import RPi.GPIO as GPIO
import threading
from dc import DC
import time
from relay import Relay
from ultrasonic_sensor import UltrasonicSensor

class Robot:
    dcmotor = DC()
    relay = Relay()
    sensor = UltrasonicSensor()
    status = 0
    robotIsRight = True

    def get_distance(self):
        self.relay.switchRelay(1)
        self.dcmotor.forward()
        self.status = 1
        while self.robotIsRight:
            time.sleep(0.5)
            dist = self.sensor.distance()
            print(str(dist))
            if dist < 10:
                self.motorStop()
                self.status = 0
            else:
                self.relay.switchRelay(1)
                self.dcmotor.forward()
                
    def startProgram(self):
        x = threading.Thread(target=self.get_distance).start()

    def motorStop(self):
        self.dcmotor.stop()
        self.relay.switchRelay(0)