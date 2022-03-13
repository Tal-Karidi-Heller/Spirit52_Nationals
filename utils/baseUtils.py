from ._import import *
from math import cos, sin, radians, pi
from _thread import start_new_thread, allocate_lock
from pybricks.iodevices import LUMPDevice

class Gyro(LUMPDevice):
    def __init__(self, port):
        super().__init__(port)
        self.lastReset = 0
        self.startAngle = 0
        
    @micropython.native
    def reset_angle(self, angle):
        self.lastReset = self.read(3)[0]
        self.startAngle = angle

    @micropython.native
    def angle(self):
        return self.startAngle + self.read(3)[0] - self.lastReset

    @micropython.native
    def callibrate(self):
        sleep(2)
        self.read(4)
        sleep(0.7)

    @micropython.native
    def speed(self):
        return self.read(3)[1]
    
class twoGyro:
    def __init__(self, port1, port2):
        self.gyro1 = Gyro(port1)
        self.gyro2 = Gyro(port2)

    @micropython.native
    def reset_angle(self, angle):
        self.gyro1.reset_angle(angle)
        self.gyro2.reset_angle(angle)

    @micropython.native
    def angle(self):
        return (self.gyro1.angle() + self.gyro2.angle()) / 2

    @micropython.native
    def callibrate(self):
        sleep(2)

        self.gyro1.read(4); self.gyro2.read(4)

        sleep(0.4)
    
    @micropython.native
    def speed(self):
        return (self.gyro1.speed() + self.gyro2.speed()) / 2

class motorsGyro:
    def __init__(self, rm, lm):
        self.rm = rm
        self.lm = lm
        self.k = -0.37323345
        
        self.startAngle = 0
    
    @micropython.native
    def reset_angle(self, angle):
        self.startAngle = angle
    
    @micropython.native
    def angle(self):
        return self.startAngle + (self.rm.angle() - self.lm.angle()) * self.k

    def callibrate(self):
        self.rm.reset_angle(0)
        self.lm.reset_angle(0)

"""
class baseRobot:
    def __init__(self, rmPort, lmPort, gyroPort1, gyroPort2, LrmPort, LlmPort, wheelRad, halfDBM):
        self.rm, self.lm = Motor(rmPort), Motor(lmPort)
        self.Lrm, self.Llm = Motor(LrmPort), Motor(LlmPort)

        self.gyro = twoGyro(gyroPort1, gyroPort2)

        self.wheelRad, self.halfDBM = wheelRad, halfDBM
        self.brick = EV3Brick()

        self.lastTime = time()

        self.x, self.y = 0, 0
        self.rmStart, self.lmStart = self.rm.angle(), self.lm.angle()

        start_new_thread(self.updateLocation, ())
        self.lock = allocate_lock()

    def displayText(self, text):
        self.brick.screen.clear()
        self.brick.screen.draw_text(0, 0, text, text_color=Color.BLACK, background_color=None) 

    def clear(self):
        self.brick.clear()

    def move_on(self, rmSpeed, lmSpeed):
        if time() - self.lastTime >= 0.007:
            self.rm.run(rmSpeed); self.lm.run(lmSpeed)
            self.lastTime = time()

    def moveMotor(self, motor, _time, speed):
        moveMotorTime = time()
        while time() - moveMotorTime <= _time:
            motor.run(speed)

        motor.stop()

    def moveMotorAvi(self, motor, _time, speed):
        moveMotorTime = time()
        while time() - moveMotorTime <= _time:
            motor.run(speed)
        motor.hold()
    
    @micropython.native
    def moveTank(self, rmSpeed, lmSpeed, rotations):
        rmStart, lmStart = self.rm.angle(), self.lm.angle()
        rmLast, lmLast = self.rm.angle(), self.lm.angle()
        while abs(self.rm.angle() - rmStart) + abs(self.lm.angle() - lmStart) <= rotations * 360:
            distance = self.rm.angle() - rmLast + self.lm.angle() - lmLast
            self.move_on(rmSpeed, lmSpeed)
            rmLast, lmLast = self.rm.angle(), self.lm.angle()

        self.rm.brake(); self.lm.brake()

    @micropython.native
    def updateLocation(self):
        while True:
            distance = self.CM(((self.rm.angle() - self.rmStart) + (self.lm.angle() - self.lmStart)) / 720)
            
            angle = -radians(self.gyro.angle())
            with self.lock:
                self.x += cos(angle) * distance
                self.y += sin(angle) * distance

            self.rmStart, self.lmStart = self.rm.angle(), self.lm.angle()
            sleep(0.08)

    @micropython.native
    def robot_velocity(self, velocity):
        return self.Rotations(velocity) * 360

    @micropython.native
    def CMperSec(self, velocity):
        return self.CM(velocity / 360)

    @micropython.native
    def x_tilda(self, x, l):
        return x + cos(radians(-self.gyro.angle())) * l

    @micropython.native
    def y_tilda(self, y, l):
        return y + sin(radians(-self.gyro.angle())) * l

    @micropython.native
    def pointTilda(self, x, y, l):
        return [x + cos(radians(-self.gyro.angle())) * l, y + sin(radians(-self.gyro.angle())) * l]

    @micropython.native
    def CM(self, rotations):
        return rotations * 2 * self.wheelRad * pi

    @micropython.native
    def Rotations(self, cm):
        return cm / (2 * self.wheelRad * pi)
    
    @micropython.native
    def displayState(self):
        with self.lock:
            x, y = self.x, self.y
        
        self.brick.screen.clear()

        self.brick.screen.draw_text(0, 0, "x:", x, , text_color=Color.BLACK, background_color=None)
        self.brick.screen.draw_text(0, 50, "y:", y, , text_color=Color.BLACK, background_color=None)
        self.brick.screen.draw_text(0, 100, "gyro:", self.gyro.angle(), , text_color=Color.BLACK, background_color=None)

"""