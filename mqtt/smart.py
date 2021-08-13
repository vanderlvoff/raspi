import paho.mqtt.client as mqtt
import time
import json
#from servo_motors import ServoMotors
import RPi.GPIO as GPIO

#custom classes
from shared.relay import Relay
#from ultrasonic_sensor import UltrasonicSensor
from gyro.smartdc import DC

MQTT_SERVER = "localhost"
MQTT_PATH = "rpi/gpio"

current_message = ''
right = 100
left = 100

#Dc motor object
dcmotor = DC()

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
    current_message = message
    y_speed = msg['y']
    speed = y_speed
    x_speed = msg['x']
    enginePowers = msg['enginePowers']
    formFactor = msg['formFactor']

    if message == "forward":
        print("FORWARD: ",y_speed)
        #dcmotor.setPower(rightEngine = 100, leftEngine = 100)
        dcmotor.setPower(right, left)
        dcmotor.forward(speed)
        
    if message == "right":
        #dcmotor.setPower(rightEngine = rightEngine, leftEngine = leftEngine)
        dcmotor.right(speed = x_speed)

    if message == "left":
        #dcmotor.setPower(rightEngine = rightEngine, leftEngine = leftEngine)
        dcmotor.left(speed = x_speed)
        
    if message == "back":
        #dcmotor.setPower(rightEngine = rightEngineBack, leftEngine = leftEngineBack)
        dcmotor.back(speed = y_speed)
        
    if message == "stop":
        dcmotor.stop()

    if message == "switch-motor-engine":
        relayObj = Relay()
        relayObj.switchRelay(status = msg['status'])
        
# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    message2 = format(msg.payload.decode("UTF-8"))
    #print("This is on publish1: "+message2)
    moveCamera(msg)
    
def on_publish(client, userdata, mid):
    print("This is on publish")

def setDcPower():
    print("##########################################")
    print(current_message)
    print("##########################################")
    
    if current_message == "forward":
        dcmotor.setPower(right, left)
        dcmotor.forward(speed=10)

def run():
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
    


