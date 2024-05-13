#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.nxtdevices import (LightSensor, TouchSensor)
#from pybricks.nxtdevices import (ColorSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

"""
-------------------------------------------------------
Initialize motors, sensors aswell as the drive base
-------------------------------------------------------
"""
# Initialize the EV3 Brick
ev3 = EV3Brick()

# Killarm Motor is C
killer_motor = Motor(Port.C)

# Initialize the motors
left_motor = Motor(Port.D)
right_motor = Motor(Port.A)


# Initialize the sensors
right_color_sensor = ColorSensor(Port.S3)
front_ultrasonic_sensor = UltrasonicSensor(Port.S2)
color_sensor = LightSensor(Port.S4)
touch_sensor = TouchSensor(Port.S1)

# Initialize the drive base
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)
"""
-------------------------------------------------------
CONSTATNS
-------------------------------------------------------
""" 
# Define the distance to the balloon (in mm)
BALLOON_DISTANCE = 500
# Define the balloon color
BALLOON_COLOR = Color.RED
ROTATION_ANGLE = 180
REFLECTION_RIGHT = 10

# Define the distance to the balloon (in mm) for the color sensor
COLOR_SENSOR_DISTANCE = 50
COLOR_TOLERANCE = 5

MAX_TEAM_BALLOONS = 3

# speed of killer arm
SPEED_KILL_ARM = 150
ANGLE_KILL_ARM = 180

"""
-------------------------------------------------------
Runtime variables
-------------------------------------------------------
""" 


is_driving = False

balloon_color = 0

# amount of destroyed ballons
destroyed_balloon_counter = 0


def destroy():    
    maxDistance = 80
    isDestroyed = False
    destroyed_balloon_counter = 0
    while front_ultrasonic_sensor.distance() > 0 and front_ultrasonic_sensor.distance() < maxDistance:
        killer_motor.run_target(SPEED_KILL_ARM, -ANGLE_KILL_ARM)
        killer_motor.run_target(SPEED_KILL_ARM, 0)
    destroyed_balloon_counter += 1


def scanInitialColor():
    i = 0
    sum_color = 0
    amount = 20
    while i < amount:
        sum_color += color_sensor.reflection()
        i += 1
    balloon_color = sum_color / amount

def scanForColor():
    curr_color = color_sensor.reflection()
    #check if color is within balloon color range
    if balloon_color - COLOR_TOLERANCE < curr_color < balloon_color + COLOR_TOLERANCE:
        #if color is within range, perform indepth colour check to avoid false positives
        i = 0
        sum_color = 0
        amount = 20
        while i < amount:
            sum_color += color_sensor.reflection()
            i += 1
        curr_color = sum_color / amount
        if balloon_color - COLOR_TOLERANCE < curr_color < balloon_color + COLOR_TOLERANCE:
            return True
        else:
            return False
    else:
        return False


def driveIntoPosition(): 
    # robot is in starting position right now
    # turn slighty away from edge to have maneuverability
    robot.turn(20)
    robot.straight(50) # 5cm TODO: to be tested
    robot.turn(-20)

    # drive straight ahead until touch sensor discovered end        
    robot.drive(100, 0)
    while not touch_sensor.pressed():
        pass
    # front edge has now been reached 
    
    robot.stop()
    
    # turn 90° and avoid obsticle
    robot.straight(-20) # TODO: testing
    robot.turn(90)

    # robot is now in place to start searching for balloons


def searchBallon():

    # drive forward and look for ballon until edge
    foundBalloon = False    
    robot.drive(100, 0)
    
    edge_reached = False
    while not edge_reached:

        if destroyedBallonCounter < MAX_TEAM_BALLOONS and scanForColor():
            destroy()

        current_reflection_right = right_color_sensor.reflection();
        if (current_reflection_right < REFLECTION_RIGHT):
            robot.stop()
            edge_reached = True


    # TODO: edge found
    #while destroyedBallonCounter <= maxBallonAmount and right_color_sensor.reflection() > reflection_right: # TODO add and
        # TODO:
        #if color_sensor.reflection > 0
            #robot.stop()
            

            #destroy()
        #if count = amount
            #set color to black 
    
    #if not finished, drive backwards and look for black ballon
    #stop and drive backwards
    #while destroyedBallonCounter <= maxBallonAmount:
        #if ballon found 
            #destroy()
        #if count = amount
            #set color to black 
    #robot.stop()


def main ():
    #scanInitialColor()
    #searchBallon()
    destroy()
    
main()