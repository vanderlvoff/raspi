import paho.mqtt.client as mqtt
import time
import json

#custom classes
from dc4wd import DC4WD

MQTT_SERVER = "localhost"
MQTT_PATH = "rpi/gpio"

current_message = ''
current_distance = 0.1

dcmotor = DC4WD()

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    client.subscribe(MQTT_PATH)
     
def controlDc(msg):
    
    message_json = format(msg.payload.decode("UTF-8"))
    msg = json.loads(message_json)

    message = msg['msg']
    y_speed = msg['y']
    x_speed = msg['x']
    rightEngineHead = msg['rightEngineHead']
    leftEngineHead = msg['leftEngineHead']
    rightEngineRear = msg['rightEngineRear']
    leftEngineRear = msg['leftEngineRear']
    dcmotor.setPower(
        rightEngineHead = rightEngineHead,
        leftEngineHead = leftEngineHead,
        rightEngineRear = rightEngineRear,
        leftEngineRear = leftEngineRear
    )
    
    if message == "forward":    
        dcmotor.forward(speed = y_speed)
        
    if message == "right":
        dcmotor.right(speed = x_speed)

    if message == "move_right":
        dcmotor.right(speed = x_speed)

    if message == "left":
        dcmotor.left(speed = x_speed)
        
    if message == "move_left":
        dcmotor.left(speed = x_speed)
        
    if message == "back":
        dcmotor.back(speed = y_speed)
        
    if message == "stop":
        dcmotor.stop()

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    message = format(msg.payload.decode("UTF-8"))
    print("This is on publish: " + message)
    controlDc(msg)
    
def on_publish(client, userdata, mid):
    print("This is on publish")

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
 
client.connect(MQTT_SERVER, 1883, 60)

# Blocking call that processes network traffic, dispatches callbacks and
# handles reconnecting.
# Other loop*() functions are available that give a threaded interface and a
# manual interface.
client.loop_forever()