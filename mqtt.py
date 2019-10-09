import paho.mqtt.client as mqtt
#servo/dc motor part
import time
import json

from board import SCL, SDA
from threading import Thread
import busio 

# Import the PCA9685 module. Available in the bundle and here:
#   https://github.com/adafruit/Adafruit_CircuitPython_PCA9685
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
from adafruit_motor import motor
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)

MQTT_SERVER = "localhost"
MQTT_PATH = "rpi/gpio"
RELAIS_1_GPIO = 24
GPIO.setup(RELAIS_1_GPIO, GPIO.OUT)

#set GPIO Pins
GPIO_TRIGGER = 27
GPIO_ECHO = 17
 
#set GPIO direction (IN / OUT)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)

i2c = busio.I2C(SCL, SDA)

# Create a simple PCA9685 class instance.
pca = PCA9685(i2c)
pca.frequency = 100

current_angle = 72
current_angle_servo_1 = 90
current_angle_servo_2 = 90
current_distance = 0.1
#Initialize camera servo
servo13 = servo.Servo(pca.channels[13])
servo13.angle = current_angle

servo14 = servo.Servo(pca.channels[14])
servo14.angle = current_angle

servo15 = servo.Servo(pca.channels[15])
servo15.angle = current_angle

stop_thread = False

motor_rl = motor.DCMotor(pca.channels[10], pca.channels[9])
motor_rr = motor.DCMotor(pca.channels[7], pca.channels[8])
motor_hl = motor.DCMotor(pca.channels[2], pca.channels[1])
motor_hr = motor.DCMotor(pca.channels[3], pca.channels[4])

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
 
    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe(MQTT_PATH)
     
def moveCamera(msg):
    global current_angle
    global client
    global current_angle_servo_1
    global current_angle_servo_2
    global current_distance
    
    global t
    global stop_thread
    message_json = format(msg.payload.decode("UTF-8"))
    msg = json.loads(message_json)
    message = msg['msg']
    
    if message == "camera_stop" or message == "stop" or "sonic_stop":
        stop_thread = False
        
    if message == "from python":
        stop_thread = False        
        return
            
    dist = distance()
    current_distance = dist
    print ("Measured Distance = %.1f cm" % dist)
            
    while message == "sonic_left":
        time.sleep(0.1)
        local_angle = current_angle_servo_1 + 1
        servo14.angle = local_angle
        current_angle_servo_1 = local_angle
        if stop_thread:
            break

    while message == "sonic_right":
        time.sleep(0.1)
        local_angle = current_angle_servo_1 - 1
        servo14.angle = local_angle
        current_angle_servo_1 = local_angle
        if stop_thread:
            break

    while message == "sonic_down":
        time.sleep(0.1)
        local_angle = current_angle_servo_2 + 1
        servo13.angle = local_angle
        current_angle_servo_2 = local_angle
        if stop_thread:
            break

    while message == "sonic_up":
        time.sleep(0.1)
        local_angle = current_angle_servo_2 - 1
        servo13.angle = local_angle
        current_angle_servo_2 = local_angle
        if stop_thread:
            break

    while message == "camera_left":
        time.sleep(0.1)
        local_angle = current_angle + 1
        servo15.angle = local_angle
        current_angle = local_angle
        if stop_thread:
            client2 = mqtt.Client()
            client2.on_connect = on_connect
            client2.on_message = on_message2
            client2.on_publish = on_publish
            client2.connect(MQTT_SERVER, 1883, 60)
            client2.publish(MQTT_PATH, '{"msg":"from python","x":0,"y":0,"status":'+str(current_distance)+'}')
            client2.disconnect()
            break
        
    while message == "camera_right":
        time.sleep(0.1)
        local_angle = current_angle - 1
        servo15.angle = local_angle
        current_angle = local_angle
        if stop_thread:
            break
        
    if message == "camera_forward":
        servo15.angle = 72
        current_angle = 72
    # more callbacks, etc

        
    if message == "ultrasonic_align":
        servo13.angle = 150 #altitude
        current_angle_servo_2 = 150
        servo14.angle = 120#horizon
        current_angle_servo_1 = 120

    while message == "forward":
        motor_rr.throttle = 1
        motor_hr.throttle = 1
        motor_rl.throttle = 1
        motor_hl.throttle = 1
        if stop_thread:
            motor_rr.throttle = 0
            motor_hr.throttle = 0
            motor_rl.throttle = 0
            motor_hl.throttle = 0
            stop_thread = False
            break
        
    while message == "right":
        motor_rr.throttle = -1
        motor_hr.throttle = -1
        motor_rl.throttle = 1
        motor_hl.throttle = 1
        if stop_thread:
            motor_rr.throttle = 0
            motor_hr.throttle = 0
            motor_rl.throttle = 0
            motor_hl.throttle = 0
            stop_thread = False
            break
        
    while message == "left":
        motor_rr.throttle = 1
        motor_hr.throttle = 1
        motor_rl.throttle = -1
        motor_hl.throttle = -1
        if stop_thread:
            motor_rr.throttle = 0
            motor_hr.throttle = 0
            motor_rl.throttle = 0
            motor_hl.throttle = 0
            stop_thread = False
            break
        
    while message == "back":
        motor_rr.throttle = -1
        motor_hr.throttle = -1
        motor_rl.throttle = -1
        motor_hl.throttle = -1
        if stop_thread:
            motor_rr.throttle = 0
            motor_hr.throttle = 0
            motor_rl.throttle = 0
            motor_hl.throttle = 0
            stop_thread = False
            break

    if message == "switch-motor-engine":
        if msg['status'] == 0:
            GPIO.output(RELAIS_1_GPIO, GPIO.LOW)
        else:
            GPIO.output(RELAIS_1_GPIO, GPIO.HIGH)


t = Thread(target=moveCamera, args=())

def on_message2(client, userdata, msg):
    print("On message 2")

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    global t
    global stop_thread
    message2 = format(msg.payload.decode("UTF-8"))
    print(message2)
    if t.isAlive():
        stop_thread = True
        t.join()     
    t = Thread(target=moveCamera, args=(msg,))
    t.start()
    
def on_publish(client, userdata, mid):
    print("This is on publish")

def distance():
    # set Trigger to HIGH
    GPIO.output(GPIO_TRIGGER, True)
 
    # set Trigger after 0.01ms to LOW
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)
 
    StartTime = time.time()
    StopTime = time.time()
 
    # save StartTime
    while GPIO.input(GPIO_ECHO) == 0:
        StartTime = time.time()
 
    # save time of arrival
    while GPIO.input(GPIO_ECHO) == 1:
        StopTime = time.time()
 
    # time difference between start and arrival
    TimeElapsed = StopTime - StartTime
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = (TimeElapsed * 34300) / 2
 
    return distance

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
 
client.connect(MQTT_SERVER, 1883, 60)

# Blocking call that processes network traffic, dispatches callbacks and
# handles reconnecting.
# Other loop*() functions are available that give a threaded interface and a
# manual interface.
client.loop_forever()
pca.deinit()