#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import math


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

#motors and sensors
m1 = Motor(Port.B)
m2 = Motor(Port.C)
g = GyroSensor(Port.S3)
ev3 = EV3Brick()
t = TouchSensor(Port.S1)
f = UltrasonicSensor(Port.S4)

#Tune PID parameters
kp = 8
ki = 5
kd = 7

#Robot must start standing still upright
g.reset_angle(0)

# do not change these values as they serve only as a starting point
watch = StopWatch()
derivative = 0 
integral = 0
lasterror = 0
k = 0

#balancing
while True:
   #Turn off if the button is pressed or if it falls
    if t.pressed():
        m1.stop()
        m2.stop()
        ev3.speaker.beep(500,1000)
        break
    if g.angle() > 45 or g.angle() < -45:
        m1.stop()
        m2.stop()
        break

    # error from the desired angle where the robot balances correctly
    error = g.angle() 

    ev3.screen.print(g.angle())
  
    integral += error * 0.25
    integral = max(min(integral, 100), -100)
    

    derivative = (error - lasterror)
    
    # PID output to the wheels
    output = (kp * error + ki * integral + kd * derivative)
    
    # k = 5 to make the robot turn
    k = 0
    m1.dc(-output + k)
    m2.dc(-output - k)

    lasterror = error
    
    
    wait(5)