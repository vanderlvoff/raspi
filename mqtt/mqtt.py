import paho.mqtt.client as mqtt
import time
import json

#custom classes
from relay import Relay
from ultrasonic_sensor import UltrasonicSensor
from dc import DC
from servo_motors import ServoMotors
from led import Led
from robot import Robot

MQTT_SERVER = "localhost"
MQTT_PATH = "rpi/gpio"

current_message = ''

current_distance = 0.1
#camera motor
# servo13 = ServoMotors(channel = 13)
#turret angle servo motor
servo14 = ServoMotors(channel = 14)
#cturret basement servo motor
servo15 = ServoMotors(channel = 15)
#thread termination flag
stop_thread = False

#Initialize DC class
#Dc motor object
dcmotor = DC()
#Robot object
robot = Robot()

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
 
    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe(MQTT_PATH)
     
def moveCamera(msg):
    print("sign")
    message_json = format(msg.payload.decode("UTF-8"))
    print(message_json)
    msg = json.loads(message_json)
    message = msg['msg']
    
    
    #if message == current_message:
    #    return
    #else:
    #    self.servo15.stop()
    #    self.servo14.stop()
    #    self.dcmotor.stop()

    #UltraSonic sensor
    #distanceObj = UltrasonicSensor() 
    #dist = distanceObj.distance()
    #current_distance = dist

    #print ("Measured Distance = %.1f cm" % dist)
    
    if message == "sonic_left":
        servo15.moveForward()

    if message == "sonic_right":
        servo15.moveBack()

    if message == "sonic_down":
        servo14.moveForward()

    if message == "sonic_up":
        servo14.moveBack()

    # if message == "camera_left":
    #     servo13.moveForward()
    #     client2 = mqtt.Client()
    #     client2.on_connect = on_connect
    #     client2.on_message = on_message2
    #     client2.on_publish = on_publish
    #     client2.connect(MQTT_SERVER, 1883, 60)
    #     client2.publish(MQTT_PATH, '{"msg":"from python","x":0,"y":0,"status":'+str(current_distance)+'}')
    #     client2.disconnect()
        
    # if message == "camera_right":
    #     servo13.moveBack()
        
    if message == "camera_forward":
        servo15.init()
        
    if message == "ultrasonic_align":
        servo14.init(150) #altitude
        servo15.init(120) #horizon

    if message == "forward":
        dcmotor.forward()
        
    if message == "right":
        dcmotor.right()

    if message == "left":
        dcmotor.left()
        
    if message == "back":
        dcmotor.back()
        
    if message == "stop":
        dcmotor.stop()

    if message == "robot_one":
        robot.stop()

    if message == "toggle_light":
        ledObj = Led()
        if msg['status'] == 1:
            ledObj.lightsOn()
        else:
            ledObj.lightsOff()

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
client.loop_forever()