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
right_color_sensor = ColorSensor(Port.S3)
front_ultrasonic_sensor = UltrasonicSensor(Port.S2)
color_sensor = LightSensor(Port.S4)
touch_sensor = TouchSensor(Port.S1)

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

class DecisionNode:
    def __init__(self, name, test, true_branch, false_branch):
        self.name = name
        self.test = test
        self.true_branch = true_branch
        self.false_branch = false_branch

    def run(self):
        if self.test():
            self.true_branch.run()
        else:
            self.false_branch.run()

class ActionNode:
    def __init__(self, name, action):
        self.name = name
        self.action = action

    def run(self):
        self.action()

# Define a test
def is_balloon_visible():
    # Replace with actual implementation
    pass

# Define actions
def drive_forward():
    # Replace with actual implementation
    pass

def search_for_balloon():
    # Replace with actual implementation
    pass

# Create the decision tree
tree = DecisionNode(
    "IsBalloonVisible",
    is_balloon_visible,
    ActionNode("DriveForward", drive_forward),
    ActionNode("SearchForBalloon", search_for_balloon)
)










def main ():
    tree.run()

    
main()