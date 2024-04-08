#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

# Initialize the EV3 Brick
ev3 = EV3Brick()

# Initialize the motors
left_motor = Motor(Port.D)
right_motor = Motor(Port.A)

# Initialize the drive base
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)

rotation_angle = 180

# Initialize the sensors
left_color_sensor = ColorSensor(Port.S3)
right_color_sensor = ColorSensor(Port.S4)
front_ultrasonic_sensor = UltrasonicSensor(Port.S2)

# Define the balloon color
BALLOON_COLOR = Color.RED

reflection_left = 10
reflection_right = 10

# Define the distance to the balloon (in mm)
BALLOON_DISTANCE = 500

# Define the distance to the balloon (in mm) for the color sensor
COLOR_SENSOR_DISTANCE = 50

isDriving = False

def search_for_balloon():
    isDriving = False
    while True:
        # Read the color sensors
        left_color = left_color_sensor.color()
        right_color = right_color_sensor.color()


        if (left_color_sensor.reflection() < reflection_left):
            robot.stop
            robot.turn(-rotation_angle)
            continue
        elif (right_color_sensor.reflection() < reflection_right):
            robot.stop
            robot.turn(rotation_angle)
            isDriving = False
            continue

        # Read the ultrasonic sensor
        distance = front_ultrasonic_sensor.distance()
        
        
        # Otherwise, keep moving forward
        if isDriving == False:
            robot.drive(100, 0)
            isDriving = True

search_for_balloon()