import paho.mqtt.client as mqtt
import time
import json
import RPi.GPIO as GPIO

#custom classes
from relay import Relay
#from ultrasonic_sensor import UltrasonicSensor
from spdc import DC

MQTT_SERVER = "192.168.1.15"
MQTT_PATH = "rpi/gpio"

current_message = ''

current_distance = 0.1

#Dc motor object
#dcmotor = DC()

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
 
    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe(MQTT_PATH)
     
def moveCamera(msg):
    message_json = format(msg.payload.decode("UTF-8"))
    print(message_json)
    msg = json.loads(message_json)
    message = msg['msg']
    y_speed = msg['y']
    x_speed = msg['x']
    rightEngine = msg['rightEngine']
    leftEngine = msg['leftEngine']
    rightEngineBack = msg['rightEngineBack']
    leftEngineBack = msg['leftEngineBack']

    if message == "forward":
        print("FORWARD: ",y_speed)
        dcmotor.setPower(rightEngine = 100, leftEngine = 100)
        dcmotor.forward(speed = 10)
        
    if message == "right":
        dcmotor.setPower(rightEngine = rightEngine, leftEngine = leftEngine)
        dcmotor.right(speed = x_speed)

    if message == "left":
        dcmotor.setPower(rightEngine = rightEngine, leftEngine = leftEngine)
        dcmotor.left(speed = x_speed)
        
    if message == "back":
        dcmotor.setPower(rightEngine = rightEngineBack, leftEngine = leftEngineBack)
        dcmotor.back(speed = y_speed)
        
    if message == "stop":
        dcmotor.stop()

    if message == "switch-motor-engine":
        relayObj = Relay()
        relayObj.switchRelay(status = msg['status'])

#t = Thread(target=moveCamera, args=())

# def on_message2(client, userdata, msg):
#     print("On message 2")

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    message2 = format(msg.payload.decode("UTF-8"))
    print("This is on publish1: "+message2)
    moveCamera(msg)
    
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
try :
    client.loop_forever()
except KeyboardInterrupt:
    print("Programm over")
finally:
    GPIO.cleanup()
    

