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

current_distance = 0.1
#servo13 = ServoMotors(channel = 12)

#Dc motor object


class Stepback():
    
    dcmotor = DC()
    current_message = ''
    current_speed = 0

    # The callback for when the client receives a CONNACK response from the server.
    def on_connect(self, client, userdata, flags, rc):
        print("Connected with result code "+str(rc))
     
        # Subscribing in on_connect() means that if we lose the connection and
        # reconnect then subscriptions will be renewed.
        client.subscribe(MQTT_PATH)
         
    def moveCamera(self, msg):
        message_json = format(msg.payload.decode("UTF-8"))
        #print(message_json)
        msg = json.loads(message_json)
        message = msg['msg']
        self.current_message = message
        y_speed = msg['y']
        speed = y_speed
        self.current_speed = speed
        x_speed = msg['x']
        enginePowers = msg['enginePowers']
        formFactor = msg['formFactor']

        if message == "forward":
            print("FORWARD: ",y_speed)
            #dcmotor.setPower(rightEngine = 100, leftEngine = 100)
            self.dcmotor.forward(speed)
            
        if message == "right":
            #dcmotor.setPower(rightEngine = rightEngine, leftEngine = leftEngine)
            self.dcmotor.right(speed = x_speed)

        if message == "left":
            #dcmotor.setPower(rightEngine = rightEngine, leftEngine = leftEngine)
            self.dcmotor.left(speed = x_speed)
            
        if message == "back":
            #dcmotor.setPower(rightEngine = rightEngineBack, leftEngine = leftEngineBack)
            self.dcmotor.back(speed = y_speed)
            
        if message == "stop":
            self.dcmotor.stop()

        if message == "switch-motor-engine":
            relayObj = Relay()
            relayObj.switchRelay(status = msg['status'])
            
        #if message == "camera_left":
        #    print("Camera left")
        #    servo13.moveForward()
        #     client2 = mqtt.Client()
        #     client2.on_connect = on_connect
        #     client2.on_message = on_message2
        #     client2.on_publish = on_publish
        #     client2.connect(MQTT_SERVER, 1883, 60)
        #     client2.publish(MQTT_PATH, '{"msg":"from python","x":0,"y":0,"status":'+str(current_distance)+'}')
        #     client2.disconnect()
            
        #if message == "camera_right":
        #    servo13.moveBack()

    #t = Thread(target=moveCamera, args=())

    # def on_message2(client, userdata, msg):
    #     print("On message 2")

    # The callback for when a PUBLISH message is received from the server.
    def on_message(self, client, userdata, msg):
        message2 = format(msg.payload.decode("UTF-8"))
        #print("This is on publish1: "+message2)
        self.moveCamera(msg)
        
    def on_publish(self, client, userdata, mid):
        print("This is on publish")

    def setDcPower(self, right, left):
        self.dcmotor.setPower(right, left)
        if self.current_message == "forward":
            self.dcmotor.forward(self.current_speed)

    def run(self):
        client = mqtt.Client()
        client.on_connect = self.on_connect
        client.on_message = self.on_message
         
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


sb = Stepback()
sb.run()
