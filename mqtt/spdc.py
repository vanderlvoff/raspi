# Import the PCA9685 module. Available in the bundle and here:
# https://github.com/adafruit/Adafruit_CircuitPython_PCA9685
import RPi.GPIO as GPIO

class DC:
    
    leftEnginePowerPin = 5
    leftEnginePowerLn1 = 6
    leftEnginePowerLn2 = 13
    
    rightEnginePowerPin = 25
    rightEnginePowerLn1 = 26
    rightEnginePowerLn2 = 19
    
  
    engine1 = None
    engine2 = None


    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        # set pins right engine
        GPIO.setup(self.rightEnginePowerPin, GPIO.OUT)
        #GPIO.output(self.rightEnginePowerPin, GPIO.LOW)
        GPIO.setup(self.rightEnginePowerLn1, GPIO.OUT)
        GPIO.output(self.rightEnginePowerLn1, GPIO.LOW)
        GPIO.setup(self.rightEnginePowerLn2, GPIO.OUT)
        GPIO.output(self.rightEnginePowerLn2, GPIO.LOW)
        
        # set pins left engine
        GPIO.setup(self.leftEnginePowerPin, GPIO.OUT)
        #GPIO.output(self.leftEnginePowerPin, GPIO.LOW)
        GPIO.setup(self.leftEnginePowerLn1, GPIO.OUT)
        GPIO.output(self.leftEnginePowerLn1, GPIO.LOW)
        GPIO.setup(self.leftEnginePowerLn2, GPIO.OUT)
        GPIO.output(self.leftEnginePowerLn2, GPIO.LOW)
        
        self.rightEngine = GPIO.PWM(self.rightEnginePowerPin, 1000)
        self.leftEngine = GPIO.PWM(self.leftEnginePowerPin, 1000)
        
        self.rightEngine.start(0)
        self.leftEngine.start(0)

   # controlCourse = 0 
   # controlCourseFlag = True
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
        rightEnSpeed = int(speed * 10 * 0.92)
        leftEnSpeed = speed * 10
        GPIO.output(self.leftEnginePowerLn1, GPIO.HIGH)
        GPIO.output(self.leftEnginePowerLn2, GPIO.LOW)

        GPIO.output(self.leftEnginePowerPin, GPIO.HIGH)
        
        GPIO.output(self.rightEnginePowerLn1, GPIO.HIGH)
        GPIO.output(self.rightEnginePowerLn2, GPIO.LOW)
        
        GPIO.output(self.rightEnginePowerPin, GPIO.HIGH)

        self.rightEngine.ChangeDutyCycle(rightEnSpeed)
        self.leftEngine.ChangeDutyCycle(leftEnSpeed)

        
    def stop(self):
        self.getDirectionFlag = False
        print("DC stop")
        GPIO.output(self.leftEnginePowerLn1, GPIO.HIGH)
        GPIO.output(self.leftEnginePowerLn2, GPIO.LOW)
        GPIO.output(self.leftEnginePowerPin, GPIO.LOW)
        GPIO.output(self.rightEnginePowerLn1, GPIO.HIGH)
        GPIO.output(self.rightEnginePowerLn2, GPIO.LOW)
        GPIO.output(self.leftEnginePowerPin, GPIO.LOW)
        
        self.rightEngine.ChangeDutyCycle(0)
        self.leftEngine.ChangeDutyCycle(0)
    
    def right(self, speed):
        self.getDirectionFlag = False
        rightEnSpeed = speed * 10
        leftEnSpeed = speed * 10
        GPIO.output(self.leftEnginePowerLn1, GPIO.HIGH)
        GPIO.output(self.leftEnginePowerLn2, GPIO.LOW)

        GPIO.output(self.leftEnginePowerPin, GPIO.HIGH)
        
        GPIO.output(self.rightEnginePowerLn1, GPIO.LOW)
        GPIO.output(self.rightEnginePowerLn2, GPIO.HIGH)
        
        GPIO.output(self.rightEnginePowerPin, GPIO.HIGH)

        self.rightEngine.ChangeDutyCycle(rightEnSpeed)
        self.leftEngine.ChangeDutyCycle(leftEnSpeed)

    def left(self, speed):
        self.getDirectionFlag = False
        rightEnSpeed = speed * 10
        leftEnSpeed = speed * 10
        GPIO.output(self.leftEnginePowerLn1, GPIO.LOW)
        GPIO.output(self.leftEnginePowerLn2, GPIO.HIGH)

        GPIO.output(self.leftEnginePowerPin, GPIO.HIGH)
        
        GPIO.output(self.rightEnginePowerLn1, GPIO.HIGH)
        GPIO.output(self.rightEnginePowerLn2, GPIO.LOW)
        
        GPIO.output(self.rightEnginePowerPin, GPIO.HIGH)

        self.rightEngine.ChangeDutyCycle(rightEnSpeed)
        self.leftEngine.ChangeDutyCycle(leftEnSpeed)
    
    def back(self, speed):
        self.getDirectionFlag = False
        rightEnSpeed = int(speed * 10 * 0.92)
        leftEnSpeed = speed * 10
        GPIO.output(self.leftEnginePowerLn1, GPIO.LOW)
        GPIO.output(self.leftEnginePowerLn2, GPIO.HIGH)

        GPIO.output(self.leftEnginePowerPin, GPIO.HIGH)
        
        GPIO.output(self.rightEnginePowerLn1, GPIO.LOW)
        GPIO.output(self.rightEnginePowerLn2, GPIO.HIGH)
        
        GPIO.output(self.rightEnginePowerPin, GPIO.HIGH)

        self.rightEngine.ChangeDutyCycle(rightEnSpeed)
        self.leftEngine.ChangeDutyCycle(leftEnSpeed)
        
