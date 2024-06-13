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
                                 UltrasonicSensor, TouchSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase


"""
-------------------------------------------------------
Initialize motors, sensors aswell as the drive base
-------------------------------------------------------
"""
# Initialize the EV3 Brick
ev3 = EV3Brick()

# Killarm Motor is B
killer_motor = Motor(Port.B)

# Initialize the motors
left_motor = Motor(Port.D)
right_motor = Motor(Port.A)


# Initialize the sensors
front_color_sensor = ColorSensor(Port.S3)
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

# Define the balloon color
BALLOON_COLOR = (255, 0, 0)
COLOR_WHITE = (255, 255, 255) #TODO: add correct values
#ROTATION_ANGLE = 180
DEFAULT_REFLECTION_FRONT = 10

SCAN_COUNT = 20

# Define the distance to the balloon (in mm) for the color sensor
#COLOR_SENSOR_DISTANCE = 50
COLOR_TOLERANCE = 5 #TODO: adjust tolerance

MAX_TEAM_BALLOONS = 1 #TODO: adjust amount of balloons

# Define the speed of the robot
ROBOT_SPEED_MAX = 100

# Define the default turn angle
ROBOT_ANGLE_DEFAULT = 0

# speed of killer arm
KILLARM_SPEED = 150
KILLARM_ANGLE = -180
KILLARM_ANGLE_DEFAULT = 0

#Sensor constants
ULTRASONIC_MAX_DISTANCE = 100
"""
-------------------------------------------------------
Runtime variables
-------------------------------------------------------
""" 
# amount of destroyed ballons
destroyed_balloon_counter = 0

curr_color_to_kill = BALLOON_COLOR

finished = False

"""
-------------------------------------------------------
Functions
-------------------------------------------------------
""" 

def destroy():    
    #balloon_ultrasonic_sensor.distance() > 0 and
    global finished
    global destroyed_balloon_counter
    while balloon_ultrasonic_sensor.distance() < ULTRASONIC_MAX_DISTANCE:
        killer_motor.run_target(KILLARM_SPEED, KILLARM_ANGLE)
        killer_motor.run_target(KILLARM_SPEED, KILLARM_ANGLE_DEFAULT)
    destroyed_balloon_counter += 1
    if curr_color_to_kill is COLOR_WHITE:
        finished = True
        
def getMedianColor():
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
    return (sum_color_r / SCAN_COUNT, sum_color_g / SCAN_COUNT, sum_color_b / SCAN_COUNT)

def scanInitialColor():
    global BALLOON_COLOR
    BALLOON_COLOR = getMedianColor()
    print(BALLOON_COLOR[0])
    print(BALLOON_COLOR[1])
    print(BALLOON_COLOR[2])

def isColorInRange(mes_color):
    #check if color is within balloon color range
    r_correct = ((curr_color_to_kill[0] - COLOR_TOLERANCE) < mes_color[0] < (curr_color_to_kill[0] + COLOR_TOLERANCE))
    g_correct = ((curr_color_to_kill[1] - COLOR_TOLERANCE) < mes_color[1] < (curr_color_to_kill[1] + COLOR_TOLERANCE))
    b_correct = ((curr_color_to_kill[2] - COLOR_TOLERANCE) < mes_color[2] < (curr_color_to_kill[2] + COLOR_TOLERANCE))

    return r_correct and g_correct and b_correct

def foundColor():
    curr_color = color_sensor.rgb()
    #print(' curr_color: ')
    #print(curr_color)
    #print(' curr_color_to_kill: ')
    #print(curr_color_to_kill)
    #print(isColorInRange(curr_color))
    return isColorInRange(curr_color)

def foundColorDetailed():
    #if color is within range, perform indepth colour check to avoid false positives
    curr_color = getMedianColor()
    return isColorInRange(curr_color)

def driveIntoPosition(): 
    # robot is in starting position right now
    # turn slighty away from edge to have maneuverability
    #robot.turn(10)
    #robot.straight(500) # 5cm TODO: adjust distance for competetive setting
    #robot.turn(-17)

    # drive straight ahead until touch sensor discovered end 
    robot.drive(ROBOT_SPEED_MAX, ROBOT_ANGLE_DEFAULT)
    while not touch_sensor.pressed():
        pass
    # front edge has now been reached 
    
    robot.stop()
    
    # turn 90° and avoid obsticle
    #TODO: Check if the turns can be summed up and recalculate
    robot.straight(-25) # TODO: adjust distance for competetive setting, so we have a distance to bricks of 1-1.5cm
    robot.turn(45)
    robot.turn(45)
    robot.turn(45)
    robot.turn(45)
    robot.turn(45)
    robot.turn(35) 
    # robot is now in place to start searching for balloons

def returnToStartPos(distance_to_start_pos): 
    robot.reset() #resets distance driven
    while robot.distance() < distance_to_start_pos:
        robot.drive(-ROBOT_SPEED_MAX, ROBOT_ANGLE_DEFAULT)
    robot.stop()
    searchBalloon()
    
def setCurrColorToKill():
    global curr_color_to_kill
    if destroyed_balloon_counter < MAX_TEAM_BALLOONS: 
        curr_color_to_kill = BALLOON_COLOR
    else:
        curr_color_to_kill = COLOR_WHITE

def searchBalloon():
    setCurrColorToKill()

    robot.reset() #resets distance driven

    # drive forward and look for ballon until edge
    robot.drive(ROBOT_SPEED_MAX, ROBOT_ANGLE_DEFAULT)
    
    edge_reached = False
    while not edge_reached and not finished:
        #check if the sensor discovers the right balloon color 
        #TODO: rename to correct ballon color found or color in range/scope
        if foundColor():
            robot.stop()
            if foundColorDetailed():
                destroy()
                setCurrColorToKill()
            robot.drive(ROBOT_SPEED_MAX, ROBOT_ANGLE_DEFAULT)

        #edge detection
        if (front_color_sensor.reflection() < DEFAULT_REFLECTION_FRONT):
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