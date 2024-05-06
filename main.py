#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.nxtdevices import (LightSensor)
#from pybricks.nxtdevices import (ColorSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

# Initialize the EV3 Brick
ev3 = EV3Brick()

# Killarm Motor is C
killer_motor = Motor(Port.C)

# Initialize the motors
left_motor = Motor(Port.D)
right_motor = Motor(Port.A)

# Initialize the drive base
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)

rotation_angle = 180

# Initialize the sensors
#left_color_sensor = ColorSensor(Port.S4)
right_color_sensor = ColorSensor(Port.S3)
front_ultrasonic_sensor = UltrasonicSensor(Port.S2)
color_sensor = LightSensor(Port.S4)

# Define the balloon color
BALLOON_COLOR = Color.RED

reflection_left = 10
reflection_right = 10

# Define the distance to the balloon (in mm)
BALLOON_DISTANCE = 500

# Define the distance to the balloon (in mm) for the color sensor
COLOR_SENSOR_DISTANCE = 50

isDriving = False

ballonColor = 0

# amount of destroyed ballons
destroyedBallonCounter = 0
maxBallonAmount = 3

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

def log_battle_data(): 
    DataLog.log(teamColour)
    DataLog.log(popedBallons)

# speed of killer arm
speedKiller = 150
angleKiller = 180

def destroy():    
    maxDistance = 80
    isDestroyed = False
    destroyedBallonCounter = 0
    while front_ultrasonic_sensor.distance() > 0 and front_ultrasonic_sensor.distance() < maxDistance:
        killer_motor.run_target(speedKiller, -angleKiller)
        killer_motor.run_target(speedKiller, 0)
    destroyedBallonCounter += 1
    


def scanInitialColor():
    i = 0
    sumColor = 0
    amount = 20
    while i < amount:
        sumColor += color_sensor.reflection()
        i += 1
    ballonColor = sumColor / amount

def searchBallon ():
    
    # turn slighty away from edge
    robot.turn(20)
    robot.straight(50) # 5cm TODO: to be tested
    robot.turn(-20)

    # drive straight ahead until touch sensor discovered end        
    robot.drive(100, 0)
    foundEdge = False
    while not foundEdge:
        # touch sensor should check
        # TODO
        foundEdge = True

    robot.stop()
    
    # turn 90° and avoid obsticle
    robot.straight(-20) # TODO: testing
    robot.turn(90)

    # drive forward and look for ballon until edge
    foundBallon = False    
    robot.drive(100, 0)
    
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