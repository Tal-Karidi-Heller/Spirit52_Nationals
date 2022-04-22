print("robotCore: ")
from robotCore import *

print("pointsFile: ")
from pointsFile import *

print("micropython")
import micropython
from gc import collect

from time import time

@micropython.native
def firstRun2():
    # A
    robot.gyro.callibrate()
    robot.brick.speaker.beep()
    robot.gyro.reset_angle(-90)
    robot.Lrm.reset_angle(0); robot.Llm.reset_angle(0)

    with robot.lock:
        robot.x, robot.y = 0, 0

    robot.spline2(points_splinefirstFirst, lastTimefirstFirst, 4, 1.3, 0.24)

    robot.spline2(points_splinefirstHalf, lastTimefirstHalf, -4, 1.3, 0.2)

    robot.spline2(points_splinefirstSecond, lastTimefirstSecond, 4, 1.3, 0.2)

    robot.Lrm.run_angle(400, 340)

    robot.spline2(points_splinefirstThird, lastTimefirstThird, -4, 1.3, 0.2)

    robot.Lrm.run_angle(-800, 250)

    robot.turn2(5, 0.003, 0.02, -65)

    robot.spline2(points_splinefirstFourth, lastTimefirstFourth, 4, 1.3, 0.2)

    robot.turn2(5, 0.003, 0.01, -180)

    robot.spline2(points_splinefirstFifth, lastTimefirstFifth, 4, 1.3, 0.2)

    robot.spline2(points_splinefirstSixth, lastTimefirstSixth, 4, 1.2, 0.2)

    robot.turn2(5, 0.003, 0.1, -90)

    robot.Lrm.stop(); robot.Llm.stop()

    bc = robot.brick.buttons.pressed()
    while bc != [Button.CENTER]:
        bc = robot.brick.buttons.pressed()

    robot.Lrm.run_target(800, 82, then = Stop.COAST, wait=True)
    robot.Llm.run_target(800, 0, then = Stop.COAST, wait=True)


@micropython.native
def firstRun():
    robot.gyro.callibrate()
    robot.brick.speaker.beep()
    robot.gyro.reset_angle(-90)
    robot.Lrm.reset_angle(0); robot.Llm.reset_angle(0)

    with robot.lock:
        robot.x, robot.y = 0, 0

    robot.spline2(points_splinefirstFirst, lastTimefirstFirst, 4, 1.2, 0.3)

    robot.spline2(points_splinefirstHalf, lastTimefirstHalf, -4, 1.2, 0.3)

    robot.spline2(points_splinefirstSecond, lastTimefirstSecond, 4, 1.2, 0.3)

    robot.Lrm.run_angle(400, 340)
    
    robot.spline2(points_splinefirstThird, lastTimefirstThird, -4, 1.2, 0.3)

    robot.Lrm.run_angle(-800, 250)

    robot.turn2(4.7, 0.003, 0.1, -65, 10)

    robot.spline2(points_splinefirstFourth, lastTimefirstFourth, 4, 1.2, 0.2)

    robot.moveMotorAvi(robot.Llm, 2.2, 900)

    robot.turn2(7, 0.003, 0.12, 90, 0)

    robot.spline2(points_splinefirstFifth, lastTimefirstFifth, 8, 0.45, 0.11)

    # robot.spline2(points_splinefirstFifthHalf, lastTimefirstFifthHalf, -3, 1, 0)
    robot.moveTank(-300, -300, robot.Rotations(2))

    robot.moveMotorAvi(robot.Llm, 2.6, -700)

    robot.turn2(6, 0.003, 0.1, 180, 7)

    robot.moveTank(-300, -300, robot.Rotations(14))

    robot.spline2(points_splinefirstSixth, lastTimefirstSixth, 4, 1.25, 0.3)

    robot.turn2(5, 0.003, 0.1, 180)

    robot.spline2(points_splinefirstSeventh, lastTimefirstSeventh, 4, 1.3, 0.3)

    robot.turn2(7, 0.003, 0.1, 270, 10)

    robot.Lrm.stop(); robot.Llm.stop()

    bc = robot.brick.buttons.pressed()
    while bc != [Button.CENTER]:
        bc = robot.brick.buttons.pressed()

    robot.Lrm.run_target(800, 82, then = Stop.COAST, wait=True)
    robot.Llm.run_target(800, -10, then = Stop.COAST, wait=True)

@micropython.native
def secondRun():
    collect()
    robot.gyro.callibrate()
    robot.gyro.reset_angle(-90)
    robot.Lrm.reset_angle(0); robot.Llm.reset_angle(0)

    with robot.lock:
        robot.x, robot.y = 0, 0

    # spline(self, points, lastTime, l, Kp, Kd, Kacc)
    robot.spline2(points_splinesecondFirst, lastTimesecondFirst, 5, 0.97, 0.75)

    with robot.lock:
        print(robot.x, robot.y, robot.gyro.angle())

    robot.turn2(9.4, 0.003, 0.15, -31, 8)
    robot.moveMotor(robot.Llm, 0.79, -400)
    
    robot.spline2(points_splinesecondSecond, lastTimesecondSecond, -3, 1.2, 0.12)
    robot.brick.speaker.beep()

    with robot.lock:
        print(robot.x, robot.y, robot.gyro.angle())

    
    robot.moveMotorAvi(robot.Lrm, 0.5, -900)


    robot.spline2(points_splinesecondThird, lastTimesecondThird, 3, 1.7, 0.25)
    
    robot.turn2(8, 0.003, 0.15, -121, 0)
    print(robot.gyro.angle())

    robot.moveMotor(robot.Lrm, 0.5, 900)

    with robot.lock:
        print(robot.x, robot.y, robot.gyro.angle())


    robot.moveMotor(robot.Llm, 1, 900)

    robot.turn2(6.5, 0.003, 0.0, -200, 10)
    
    robot.moveMotor(robot.Llm, 1, -900)

    robot.spline2(points_splinesecondFourth, lastTimesecondFourth, -3, 1.12, 0.5)

    bc = robot.brick.buttons.pressed()
    while bc != [Button.CENTER]:
        bc = robot.brick.buttons.pressed()

    robot.Lrm.run_target(800, 0, then = Stop.COAST, wait=True)
    robot.Llm.run_target(800, -145, then = Stop.COAST, wait=True)


@micropython.native
def thirdRun():
    collect()
    robot.gyro.callibrate()
    robot.gyro.reset_angle(-90)
    robot.Lrm.reset_angle(0); robot.Llm.reset_angle(0)

    with robot.lock:
        robot.x, robot.y = 0, 0

    robot.spline2(points_splinethirdFirst, lastTimethirdFirst, 4, 1.15, 0.3)

    robot.spline2(points_splinethirdSecond, lastTimethirdSecond, 4, 1.15, 0.3)

    robot.turn(5, 0.003, 0.15, 65, 5)

    robot.spline2(points_splinethirdThird, lastTimethirdThird, 4, 1.15, 0.3)

    robot.spline2(points_splinethirdThirdHalf, lastTimethirdThirdHalf, -4, 1.2, 0.3)

    robot.turn2(5, 0.003, 0.15, 0)

    robot.spline2(points_splinethirdFourth, lastTimethirdFourth, 4, 1.2, 0.3)

    robot.spline2(points_splinethirdFifth, lastTimethirdFifth, -4, 1.2, 0.3)

    robot.moveMotor(robot.Llm, 1, 700)

    robot.turn2(5, 0.003, 0.15, -90, 0)

    robot.spline2(points_splinethirdSixth, lastTimethirdSixth, -4, 1.2, 0.3)

    robot.moveMotor(robot.Lrm, 0.4, -900)

    robot.spline2(points_splinethirdSeventh, lastTimethirdSeventh, -4, 1.2, 0.3)

    bc = robot.brick.buttons.pressed()
    while bc != [Button.CENTER]:
        bc = robot.brick.buttons.pressed()

    robot.Lrm.run_target(800, -158, then = Stop.COAST, wait=True)
    robot.Llm.run_target(800, 195, then = Stop.COAST, wait=True)

def fourthRun():
    collect()
    Kp, Ki, Kd = 6.2, 0.002, 0.18

    robot.gyro.callibrate()
    robot.brick.speaker.beep()
    robot.gyro.reset_angle(-90)
    robot.Lrm.reset_angle(0); robot.Llm.reset_angle(0)

    with robot.lock:
        robot.x, robot.y = 0, 0

    robot.spline2(points_splinefourthFirst, lastTimefourthFirst, 4, 1.12, 0.5)
    robot.turn2(5, 0.003, 0.2, -90, 0)

    robot.moveMotor(robot.Lrm, 1, 800)

    robot.turn2(9.4, 0.003, 0.1, -115, 20)
    robot.moveMotor(robot.Lrm, 1, -800)

    robot.turn2(Kp, Ki, Kd, 0, 10)
    robot.displayState()

    robot.spline2(points_splinefourthSecond, lastTimefourthSecond, 4, 1.17, 0.5)

    # robot.spline2(points_splinefourthThird, lastTimefourthThird, 4, 0, 0.25)
    
    robot.turn2(5, 0.003, 0.15, 3, 0)
    
    startTime = time()
    while time() - startTime <= 0.63:
        robot.move_on(robot.robot_velocity(20), robot.robot_velocity(30))

    with robot.lock:
        x, y = robot.x, robot.y

    print("\nstate:", x, y, robot.gyro.angle())

    robot.spline2(points_splinefourthFourth, lastTimefourthFourth, -4, 1.17, 0.3)

    robot.turn2(5, 0.003, 0.15, 0, 3)
    
    robot.spline2(points_splinefourthFifth, lastTimefourthFifth, -4, 1.17, 0.3)

    robot.turn2(5, 0.003, 0.15, 0, 0)

    robot.spline2(points_splinefourthSixth, lastTimefourthSixth, 3, 1.12, 0.5)

    # robot.moveMotor(robot.Llm, 4.2, -70)

    robot.spline2(points_splinefourthSeventh, lastTimefourthSeventh, -3, 1.12, 0.25)

    robot.turn(5, 0.003, 0.12, 0, 0)

@micropython.native
def judgesCode():
    robot.gyro.callibrate()
    robot.gyro.reset_angle(-33)
    
    with robot.lock:
        robot.x, robot.y = 19, 15.9
    
    robot.spline2(points_splinejudgesSpline, lastTimejudgesSpline, 4, 1.2, 0.175) 