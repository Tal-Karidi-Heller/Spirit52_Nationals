#!/usr/bin/env pybricks-micropython
# Create your objects here
from time import sleep, time
startTime = time()

from pybricks.iodevices import LUMPDevice
from pybricks.parameters import Button, Color
print("a")

from time import sleep, time
print("b")

from robotCore import robot

import runsCode 
print((time() - startTime) / 60)

robot.brick.light.on(Color.YELLOW)
robot.brick.speaker.beep(duration = 400)


def mainRuns():
    robot.Lrm.reset_angle(0); robot.Llm.reset_angle(0)
    while True :
        bc = robot.brick.buttons.pressed()
        
        if bc == [Button.RIGHT]: 
            runsCode.secondRun()

        if bc == [Button.LEFT]:
            runsCode.fourthRun()

        if bc == [Button.DOWN]:
            runsCode.thirdRun()

        if bc == [Button.UP]:
            runsCode.firstRun2()
        

        if bc == [Button.CENTER]:
            robot.brick.speaker.beep(duration = 400)
            runsCode.judgesCode()

mainRuns()                                                                                       