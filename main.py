#!/usr/bin/env pybricks-micropython
# Create your objects here.5
from robotCore import * 
from pybricks.iodevices import LUMPDevice
from time import sleep, time
from testPointsFile import *
from tests import Test
from runsFile import *

robot.brick.light.on(Color.YELLOW)
robot.gyro.callibrate()
robot.gyro.reset_angle(-90)


with robot.lock:
    robot.x, robot.y = 0, 0

robot.spline(points_splinesecondFirst, lastTimesecondFirst, 4, 0.85, 0, 1.12)
robot.turn(10, 0.0007, 4, 0)

with robot.lock:
    print(robot.x, robot.y, robot.gyro.angle())