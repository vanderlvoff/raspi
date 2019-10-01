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

i2c = busio.I2C(SCL, SDA)

# Create a simple PCA9685 class instance.
pca = PCA9685(i2c)
pca.frequency = 100

current_angle = 72
#Initialize camera servo
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
    global t
    global stop_thread
    message_json = format(msg.payload.decode("UTF-8"))
    msg = json.loads(message_json)
    message = msg['msg']
    
    if message == "camera_stop" or message == "stop":
        stop_thread = False
       
    while message == "camera_left":
        time.sleep(0.1)
        local_angle = current_angle + 1
        servo15.angle = local_angle
        current_angle = local_angle
        if stop_thread:
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