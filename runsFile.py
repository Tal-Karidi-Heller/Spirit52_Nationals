from pointsFile import *
from robotCore import *
import micropython

@micropython.native
def secondRun():
    robot.gyro.callibrate()
    robot.gyro.reset_angle(-90)

    with robot.lock:
        robot.x, robot.y = 0, 0
    
    robot.spline(points_splinesecondFirst, lastTimesecondFirst, 4, 3, 0.12)

    robot.turn(9, 0.0007, 4, -45)
    with robot.lock:
        print(robot.x, robot.y, robot.gyro.angle())

    robot.moveMotor(robot.Lrm, 1, -500)