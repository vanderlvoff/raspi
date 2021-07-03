# Import the PCA9685 module. Available in the bundle and here:
# https://github.com/adafruit/Adafruit_CircuitPython_PCA9685
import RPi.GPIO as GPIO

class DC:
    # set pins right engine
    
    in1 = 37
    in2 = 35
    enA = 25
    
    # set pins left engine
    in3 = 33
    in4 = 31
    enB = 29

    engine1 = None
    engine2 = None


    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.in1, GPIO.OUT)
        GPIO.setup(self.in2, GPIO.OUT)
        GPIO.setup(self.in3, GPIO.OUT)
        GPIO.setup(self.in4, GPIO.OUT)
        GPIO.setup(self.enA, GPIO.OUT)
        GPIO.setup(self.enB, GPIO.OUT)
        GPIO.output(self.in1, GPIO.LOW)
        GPIO.output(self.in2, GPIO.LOW)
        GPIO.output(self.in3, GPIO.LOW)
        GPIO.output(self.in4, GPIO.LOW)
        self.engine1 = GPIO.PWM(self.enA, 1000)
        self.engine2 = GPIO.PWM(self.enB, 1000)
        self.engine1.start(25)
        self.engine2.start(25)

    controlCourse = 0 
    controlCourseFlag = True
    getDirectionFlag = False 
    
    LE_MAX = 1
    LE_MAX_NEGATIVE = -1
    LE_CORRECTION = 0.7
    right_index = 100
    left_index = 100
    
    def setPower(self, rightEngine, leftEngine):
        self.right_index = rightEngine/100
        self.left_index = leftEngine/100
        print(self.right_index)

    def forward(self, speed):
        print("DC forward")
        GPIO.output(self.in1, GPIO.HIGH)
        GPIO.output(self.in2, GPIO.LOW)
        GPIO.output(self.in3, GPIO.HIGH)
        GPIO.output(self.in4, GPIO.LOW)
    
    def stop(self):
        self.getDirectionFlag = False
        #self.motor_hr.throttle = 0
        #self.motor_hl.throttle = 0
    
    def right(self, speed):
        self.getDirectionFlag = False
        #self.motor_hr.throttle = self.right_index * self.RE_MAX_NEGATIVE*speed/10
        #self.motor_hl.throttle = self.left_index * self.LE_MAX*speed/10

    def left(self, speed):
        self.getDirectionFlag = False
        #self.motor_hr.throttle = self.right_index * self.RE_MAX*speed/10
        #self.motor_hl.throttle = self.left_index * self.LE_MAX_NEGATIVE*speed/10
    
    def back(self, speed):
        self.getDirectionFlag = False
        #self.motor_hr.throttle = self.right_index * self.RE_MAX_NEGATIVE*speed/10
        #self.motor_hl.throttle = self.left_index * self.LE_MAX_NEGATIVE*speed/10
        
