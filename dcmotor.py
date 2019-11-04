# This example uses an Adafruit Stepper and DC Motor FeatherWing to run a DC Motor.
#   https://www.adafruit.com/product/2927

import time

from board import SCL, SDA
import busio

# Import the PCA9685 module. Available in the bundle and here:
#   https://github.com/adafruit/Adafruit_CircuitPython_PCA9685
from adafruit_pca9685 import PCA9685

from adafruit_motor import motor

i2c = busio.I2C(SCL, SDA)

# Create a simple PCA9685 class instance for the Motor FeatherWing's default address.
pca = PCA9685(i2c, address=0x40)
pca.frequency = 100

# Motor 1 is channels 9 and 10 with 8 held high.
# Motor 2 is channels 11 and 12 with 13 held high.
# Motor 3 is channels 3 and 4 with 2 held high.
# Motor 4 is channels 5 and 6 with 7 held high.

# DC Motors generate electrical noise when running that can reset the microcontroller in extreme
# cases. A capacitor can be used to help prevent this. The demo uses motor 4 because it worked ok
# in testing without a capacitor.
# See here for more info: https://learn.adafruit.com/adafruit-motor-shield-v2-for-arduino/faq#faq-13
#pca.channels[7].duty_cycle = 0xffff
motor_rl = motor.DCMotor(pca.channels[3], pca.channels[4])
#motor_rr = motor.DCMotor(pca.channels[7], pca.channels[8])
motor_hl = motor.DCMotor(pca.channels[2], pca.channels[1])
#motor_hr = motor.DCMotor(pca.channels[3], pca.channels[4])

print("Forwards slow")
#motor_rr.throttle = 0.5
#motor_hr.throttle = 0.5

motor_rl.throttle = 0.5
motor_hl.throttle = 0.5

print("throttle:", motor_rl.throttle)
time.sleep(1)

print("Forwards fast")
#motor_rr.throttle = -1
#motor_hr.throttle = -1

motor_rl.throttle = 1
motor_hl.throttle = 1

print("throttle:", motor_rl.throttle)
time.sleep(6)

print("Forwards")
#motor_rr.throttle = 0
#motor_rl.throttle = 0

motor_rl.throttle = 0
motor_hl.throttle = 0
print("throttle:", motor_rl.throttle)
time.sleep(1)

pca.deinit()
