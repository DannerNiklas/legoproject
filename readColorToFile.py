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

color_sensor = LightSensor(Port.S4)

def readColor():
    file = open ("ReadColors.txt", "a")
    i = 0
    # Read the color sensor
    file.write("\n")
    file.write("\n")
    file.write("New Color:")
    file.write("\n")

    color = 0
    amount = 50

    while i < amount:
        color += color_sensor.reflection()
        print("Color: ", color)
        i+=1
    
    file.write(str(color/amount) + "\n")
    file.close()

readColor()