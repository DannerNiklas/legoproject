"""
-------------------------------------------------------
Copyright: Gruppe 5
-------------------------------------------------------
""" 
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
BLACK_COLOR = Color.BLACK
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

balloon_color = 0

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
    maxDistance = 80
    isDestroyed = False
    destroyed_balloon_counter = 0
    while front_ultrasonic_sensor.distance() > 0 and front_ultrasonic_sensor.distance() < maxDistance:
        killer_motor.run_target(SPEED_KILL_ARM, -ANGLE_KILL_ARM)
        killer_motor.run_target(SPEED_KILL_ARM, 0)
    destroyed_balloon_counter += 1
    if color_to_kill = BLACK_COLOR:
        finished = True
        


def scanInitialColor():
    i = 0
    sum_color = 0
    amount = 20
    while i < amount:
        sum_color += color_sensor.reflection()
        i += 1
    balloon_color = sum_color / amount

def scanForColor(color_to_kill):
    curr_color = color_sensor.reflection()
    #check if color is within balloon color range
    if (color_to_kill - COLOR_TOLERANCE) < curr_color < (color_to_kill + COLOR_TOLERANCE):
        return True
    else:
        return False

def scanForColorDetailed(color_to_kill):
    #if color is within range, perform indepth colour check to avoid false positives
    i = 0
    sum_color = 0
    amount = 20
    while i < amount:
        sum_color += color_sensor.reflection()
        i += 1
    curr_color = sum_color / amount
    if (color_to_kill - COLOR_TOLERANCE) < curr_color < (color_to_kill + COLOR_TOLERANCE):
        return True
    else:
        return False

def driveIntoPosition(): 
    # robot is in starting position right now
    # turn slighty away from edge to have maneuverability
    robot.turn(20)
    robot.straight(50) # 5cm TODO: adjust distance for competetive setting
    robot.turn(-20)

    # drive straight ahead until touch sensor discovered end        
    robot.drive(100, 0)
    while not touch_sensor.pressed():
        pass
    # front edge has now been reached 
    
    robot.stop()
    
    # turn 90° and avoid obsticle
    robot.straight(-20) # TODO: adjust distance for competetive setting
    robot.turn(90)

    # robot is now in place to start searching for balloons

def returnToStartPos(distance_to_start_pos): 
    robot.reset() #resets distance driven
    while robot.distance() < distance_to_start_pos:
        robot.drive(-100, 0)
    robot.stop()
    searchBalloon()
    
def setCurrColorToKill():
    if destroyedBallonCounter < MAX_TEAM_BALLOONS: 
        curr_color_to_kill = BALLOON_COLOR
    else 
        curr_color_to_kill = BLACK_COLOR

def searchBalloon():
    setCurrColorToKill()

    robot.reset() #resets distance driven

    # drive forward and look for ballon until edge
    robot.drive(100, 0)
    
    edge_reached = False
    while not edge_reached and not finished:
        #check if the sensor discovers the right balloon color 
        if scanForColor():
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


def main ():
    scanInitialColor()
    driveIntoPosition()
    searchBalloon()
    
main()