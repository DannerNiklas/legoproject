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
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)

# Initialize the drive base
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)

# Initialize the sensors
left_color_sensor = ColorSensor(Port.S1)
right_color_sensor = ColorSensor(Port.S2)
front_ultrasonic_sensor = UltrasonicSensor(Port.S3)
front_infrared_sensor = InfraredSensor(Port.S4)

# Define the balloon color
BALLOON_COLOR = Color.RED

# Define the table edge color
TABLE_EDGE_COLOR = Color.WHITE

# Define the distance to the balloon (in mm)
BALLOON_DISTANCE = 500

# Define the distance to the balloon (in mm) for the color sensor
COLOR_SENSOR_DISTANCE = 50

def search_for_balloon():
    while True:
        # Read the color sensors
        left_color = left_color_sensor.color()
        right_color = right_color_sensor.color()

        # If either sensor detects the table edge, stop and turn
        if left_color == TABLE_EDGE_COLOR or right_color == TABLE_EDGE_COLOR:
            robot.stop()
            wait(1000)
            robot.straight(-100)  # Move backwards
            robot.turn(90)  # Turn right
            continue

        # Read the ultrasonic sensor
        distance = front_ultrasonic_sensor.distance()

        # If the balloon is detected, move towards it until it's within range of the color sensor
        if distance < BALLOON_DISTANCE:
            while distance > COLOR_SENSOR_DISTANCE:
                robot.straight(100)  # Move forward
                distance = front_ultrasonic_sensor.distance()

            # Stop and check the balloon's color
            robot.stop()
            wait(1000)
            color = front_color_sensor.color()
            if color == BALLOON_COLOR:
                ev3.speaker.beep()  # Found the balloon!
                break

        # Otherwise, keep moving forward
        robot.straight(100)

search_for_balloon()