#!/usr/bin/env pybricks-micropython
# Killarm Motor is C
"""
-------------------------------------------------------
Copyright: Gruppe 5
-------------------------------------------------------
""" 
#TODO: add more comments

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor, TouchSensor)
from pybricks.nxtdevices import (LightSensor)
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
balloon_ultrasonic_sensor = UltrasonicSensor(Port.S4)
color_sensor = ColorSensor(Port.S1)
touch_sensor = TouchSensor(Port.S2)

# Initialize the drive base
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)
"""
-------------------------------------------------------
CONSTANTS
-------------------------------------------------------
""" 
# Define the distance to the balloon (in mm)
BALLOON_DISTANCE = 500
# Define the balloon color
BALLOON_COLOR = Color.RED
COLOR_WHITE = Color.WHITE
ROTATION_ANGLE = 180
REFLECTION_RIGHT = 10

SCAN_COUNT = 20

# Define the distance to the balloon (in mm) for the color sensor
COLOR_SENSOR_DISTANCE = 50
COLOR_TOLERANCE = 5

MAX_TEAM_BALLOONS = 1

# speed of killer arm
SPEED_KILL_ARM = 150
ANGLE_KILL_ARM = 180

#Sensor constants
ULTRASONIC_MAX_DISTANCE = 200
"""
-------------------------------------------------------
Runtime variables
-------------------------------------------------------
""" 

balloon_color_r = 0
balloon_color_g = 0
balloon_color_b = 0

# amount of destroyed ballons
destroyed_balloon_counter = 0

curr_color_to_kill = BALLOON_COLOR

finished = False

"""
-------------------------------------------------------
Functions
-------------------------------------------------------
""" 

def destroy(color_to_kill):    
    while balloon_ultrasonic_sensor.distance() > 0 and balloon_ultrasonic_sensor.distance() < ULTRASONIC_MAX_DISTANCE:
        killer_motor.run_target(SPEED_KILL_ARM, -ANGLE_KILL_ARM)
        killer_motor.run_target(SPEED_KILL_ARM, 0)
    destroyed_balloon_counter += 1
    if color_to_kill is COLOR_WHITE:
        finished = True
        


def scanInitialColor():
    i = 0
    sum_color_r = 0
    sum_color_g = 0
    sum_color_b = 0
    while i < SCAN_COUNT:
        color_tupple = color_sensor.rgb()
        sum_color_r += color_tupple[0]
        sum_color_g += color_tupple[1]
        sum_color_b += color_tupple[2]
        i += 1
    balloon_color_r = sum_color_r / SCAN_COUNT
    balloon_color_g = sum_color_g / SCAN_COUNT
    balloon_color_b = sum_color_b / SCAN_COUNT


def scanForColor(color_to_kill):
    curr_color = color_sensor.rgb()
    
    #check if color is within balloon color range
    r_correct = ((color_to_kill[0] - COLOR_TOLERANCE) < curr_color[0] < (color_to_kill[0] + COLOR_TOLERANCE))
    g_correct = ((color_to_kill[1] - COLOR_TOLERANCE) < curr_color[1] < (color_to_kill[1] + COLOR_TOLERANCE))
    b_correct = ((color_to_kill[2] - COLOR_TOLERANCE) < curr_color[2] < (color_to_kill[2] + COLOR_TOLERANCE))
    if r_correct and g_correct and b_correct:
        return True
    else:
        return False

def scanForColorDetailed(color_to_kill):
    #if color is within range, perform indepth colour check to avoid false positives
    i = 0
    sum_color_r = 0
    sum_color_g = 0
    sum_color_b = 0
    while i < SCAN_COUNT:
        color_tupple = color_sensor.rgb()
        sum_color_r += color_tupple[0]
        sum_color_g += color_tupple[1]
        sum_color_b += color_tupple[2]
        i += 1
    curr_color_r = sum_color_r / SCAN_COUNT
    curr_color_g = sum_color_g / SCAN_COUNT
    curr_color_b = sum_color_b / SCAN_COUNT
    curr_color = (curr_color_r, curr_color_g, curr_color_b)

    #TODO: own function to check if color is in range and return calc value instead
    r_correct = ((color_to_kill[0] - COLOR_TOLERANCE) < curr_color[0] < (color_to_kill[0] + COLOR_TOLERANCE))
    g_correct = ((color_to_kill[1] - COLOR_TOLERANCE) < curr_color[1] < (color_to_kill[1] + COLOR_TOLERANCE))
    b_correct = ((color_to_kill[2] - COLOR_TOLERANCE) < curr_color[2] < (color_to_kill[2] + COLOR_TOLERANCE))
    if r_correct and g_correct and b_correct:
        return True
    else:
       return False

def driveIntoPosition(): 
    # robot is in starting position right now
    # turn slighty away from edge to have maneuverability
    #TODO: constants
    #robot.turn(20)
    #robot.straight(200) # 5cm TODO: adjust distance for competetive setting
    #robot.turn(-35)

    # drive straight ahead until touch sensor discovered end 
    robot.turn(0)       
    robot.drive(100, 0)
    while not touch_sensor.pressed():
        pass
    # front edge has now been reached 
    
    robot.stop()
    
    # turn 90° and avoid obsticle
    robot.straight(-50) # TODO: adjust distance for competetive setting
    robot.turn(45)
    robot.turn(45)
    robot.turn(45)
    robot.turn(45)
    robot.turn(45)
    robot.turn(35) #TODO: Test winkl

    # robot is now in place to start searching for balloons

def returnToStartPos(distance_to_start_pos): 
    robot.reset() #resets distance driven
    while robot.distance() < distance_to_start_pos:
        robot.drive(-100, 0)
    robot.stop()
    searchBalloon()
    
def setCurrColorToKill():
    if destroyed_balloon_counter < MAX_TEAM_BALLOONS: 
        curr_color_to_kill = BALLOON_COLOR
    else:
        curr_color_to_kill = COLOR_WHITE

def searchBalloon():
    setCurrColorToKill()

    robot.reset() #resets distance driven

    # drive forward and look for ballon until edge
    robot.drive(100, 0)
    
    edge_reached = False
    while not edge_reached and not finished:
        #check if the sensor discovers the right balloon color 
        #TODO: rename to correct ballon color found or color in range/scope
        if scanForColor(curr_color_to_kill):
            robot.stop()
            if scanForColorDetailed(curr_color_to_kill):
                destroy(curr_color_to_kill)
                setCurrColorToKill()
            robot.drive(100, 0)

        #edge detection
        current_reflection_right = right_color_sensor.reflection();
        if (current_reflection_right < REFLECTION_RIGHT):
            robot.stop()
            edge_reached = True

    if not finished:
        returnToStartPos(robot.distance())

"""
-------------------------------------------------------
main
-------------------------------------------------------
""" 


def main_function():
    scanInitialColor()
    driveIntoPosition()
    searchBalloon()
    
main_function()