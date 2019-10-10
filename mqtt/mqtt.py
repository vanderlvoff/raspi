import paho.mqtt.client as mqtt
import time
import json
from threading import Thread

#custom classes
from relay import Relay
from ultrasonic_sensor import UltrasonicSensor
from dc import DC
from servo_motors import ServoMotors

MQTT_SERVER = "localhost"
MQTT_PATH = "rpi/gpio"

current_distance = 0.1
#turret basement servo motor 
servo13 = ServoMotors(channel = 13)
#turret angle servo motor
servo14 = ServoMotors(channel = 14)
#camera motor
servo15 = ServoMotors(channel = 15)
#thread termination flag
stop_thread = False

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
 
    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe(MQTT_PATH)
     
def moveCamera(msg):
    global client
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

    #UltraSonic sensor
    distanceObj = UltrasonicSensor() 
    dist = distanceObj.distance()
    current_distance = dist

    #Dc motor object
    dcmotor = DC()

    print ("Measured Distance = %.1f cm" % dist)
    
    while message == "sonic_left":
        time.sleep(0.1)
        servo14.moveForward()
        if stop_thread:
            break

    while message == "sonic_right":
        time.sleep(0.1)
        servo14.moveBack()
        if stop_thread:
            break

    while message == "sonic_down":
        time.sleep(0.1)
        servo13.moveForward()
        if stop_thread:
            break

    while message == "sonic_up":
        time.sleep(0.1)
        servo13.moveBack()
        if stop_thread:
            break

    while message == "camera_left":
        time.sleep(0.1)
        servo15.moveForward()
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
        servo15.moveBack()
        if stop_thread:
            break
        
    if message == "camera_forward":
        servo15.init()
        
    if message == "ultrasonic_align":
        servo13.init(150) #altitude
        servo14.init(120) #horizon

    while message == "forward":
        dcmotor.forward()
        if stop_thread:
            dcmotor.stop()
            stop_thread = False
            break
        
    while message == "right":
        dcmotor.right()
        if stop_thread:
            dcmotor.stop()
            stop_thread = False
            break
        
    while message == "left":
        dcmotor.left()
        if stop_thread:
            dcmotor.stop()
            stop_thread = False
            break
        
    while message == "back":
        dcmotor.back()
        if stop_thread:
            dcmotor.stop()
            stop_thread = False
            break

    if message == "switch-motor-engine":
        relayObj = Relay()
        relayObj.switchRelay(status = msg['status'])

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

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
 
client.connect(MQTT_SERVER, 1883, 60)

# Blocking call that processes network traffic, dispatches callbacks and
# handles reconnecting.
# Other loop*() functions are available that give a threaded interface and a
# manual interface.
client.loop_forever()