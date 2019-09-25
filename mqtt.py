import paho.mqtt.client as mqtt
#servo/dc motor part
import time

from board import SCL, SDA
from threading import Thread
import busio 

# Import the PCA9685 module. Available in the bundle and here:
#   https://github.com/adafruit/Adafruit_CircuitPython_PCA9685
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

MQTT_SERVER = "localhost"
MQTT_PATH = "rpi/gpio"

i2c = busio.I2C(SCL, SDA)

# Create a simple PCA9685 class instance.
pca = PCA9685(i2c)
pca.frequency = 50

current_angle = 72
#Initialize camera servo
servo15 = servo.Servo(pca.channels[15])
servo15.angle = current_angle
stop_thread = False

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
    message = format(msg.payload.decode("UTF-8"))
    
    if message == "camera_stop":
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
        current_angle = current_angle
        # more callbacks, etc
    
t = Thread(target=moveCamera, args=())

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    global t
    global stop_thread
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