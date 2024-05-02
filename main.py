#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.nxtdevices import LightSensor
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

# Initialize the EV3 Brick
ev3 = EV3Brick()

# Killarm Motor is C

# Initialize the motors
left_motor = Motor(Port.D)
right_motor = Motor(Port.A)

# Initialize the drive base
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)

rotation_angle = 180

# Initialize the sensors
left_color_sensor = ColorSensor(Port.S4)
right_color_sensor = ColorSensor(Port.S3)
front_ultrasonic_sensor = UltrasonicSensor(Port.S2)
color_sensor = LightSensor(Port.S1)

# Define the balloon color
BALLOON_COLOR = Color.RED

reflection_left = 10
reflection_right = 10

# Define the distance to the balloon (in mm)
BALLOON_DISTANCE = 500

# Define the distance to the balloon (in mm) for the color sensor
COLOR_SENSOR_DISTANCE = 50

#Const for readColor
tollerance = 10

# 
def readColor():
    # Read the color sensor
    while True:
        refColor = color_sensor.reflection()
        print("Color: ", refColor)
    return refColor

# Const for searchForBalloon
isDriving = False

# Stay on table and look for balloon.
def search_for_balloon():
    isDriving = False
    while True:
        # Read the color sensors
        current_reflection_left = left_color_sensor.reflection();
        current_reflection_right = right_color_sensor.reflection();

        print("sensor left: ", current_reflection_left)
        print("sensor right: ", current_reflection_right)

        if (current_reflection_left < reflection_left):
            robot.stop
            robot.turn(rotation_angle)
            isDriving = False
        elif (current_reflection_right < reflection_right):
            robot.stop
            robot.turn(-rotation_angle)
            isDriving = False

        # Read the ultrasonic sensor
        #distance = front_ultrasonic_sensor.distance()
        
        
        
        # Otherwise, keep moving forward
        if isDriving == False:
            robot.drive(100, 0)
            isDriving = True

#search_for_balloon()
readColor()
