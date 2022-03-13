from .baseUtils import *
from time import time
from gc import collect
from math import radians, cos, sin

@micropython.native
def motorsSpeed(Vx, Vy, angle, l, b):
    r_a = radians(angle)
    v, omega = Vx * cos(r_a) - Vy * sin(r_a), (Vx * sin(r_a) + Vy * cos(r_a)) / l

    return (v + omega * b, v - omega * b)

class Robot:
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

    # base utilities: 

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
        self.brick.screen.draw_text(0, 0, "x: " + str(x))
        self.brick.screen.draw_text(0, 50, "y: " + str(y))
        self.brick.screen.draw_text(0, 75, "gyro: " + str(self.gyro.angle()))
        
    
    # comp utilities: 

    @micropython.native
    def pid_calc(self, sensor, target, lastError, integral, Dt, De, Kp, Ki, Kd):
        error = sensor - target 
        P = error * Kp
        I = (((lastError + error) * Dt) / 2 + integral) * Ki
        D = De * Kd
        return [P + I + D, I, error]

    @micropython.native
    def spline(self, points, lastTime, l, Kp, Kacc, _print = True):
        collect(); counter = 0

        rmStart, lmStart = self.rm.angle(), self.lm.angle()
        
        xLastError, yLastError = 0, 0
        startTime = time()

        while (time() - startTime) <= lastTime:
            currentTime = time()
            rounded_t = int((currentTime - startTime) / 0.01)
            n_point = points[rounded_t]
            x, y = n_point["x"], n_point["y"]
            
            with self.lock:
                currentX, currentY = self.x, self.y
            
            currentX_tilda, currentY_tilda = self.pointTilda(currentX, currentY, l)

            Vx, Vy = (x - currentX_tilda) * Kp, (y - currentY_tilda) * Kp
            
            pidVr, pidVl = motorsSpeed(Vx, Vy, self.gyro.angle(), l, self.halfDBM)

            self.move_on(self.robot_velocity(n_point['Vr'] + pidVr + n_point['rightAcc'] * Kacc), self.robot_velocity(n_point['Vl'] + pidVl + n_point['leftAcc'] * Kacc))

            counter += 1

        self.rm.brake(); self.lm.brake()
        
        if _print:
            print("counter: " + str(counter / lastTime))
            self.displayState()


    
    @micropython.native
    def spline2(self, points, lastTime, l, Kp, Kacc, _print = True):
        counter = 0

        rmStart, lmStart = self.rm.angle(), self.lm.angle()
        
        xLastError, yLastError = 0, 0
        startTime = time()

        xList, yList, VrList, VlList, rightAccList, leftAccList = points 

        while (time() - startTime) <= lastTime:
            currentTime = time()
            rounded_t = int((currentTime - startTime) / 0.01)
            x, y, Vr, Vl, Vracc, Vlacc = xList[rounded_t], yList[rounded_t], VrList[rounded_t], VlList[rounded_t], rightAccList[rounded_t], leftAccList[rounded_t]
            
            with self.lock:
                currentX, currentY = self.x, self.y
            
            currentX_tilda, currentY_tilda = self.pointTilda(currentX, currentY, l)

            Vx, Vy = (x - currentX_tilda) * Kp, (y - currentY_tilda) * Kp
            
            pidVr, pidVl = motorsSpeed(Vx, Vy, self.gyro.angle(), l, self.halfDBM)

            self.move_on(self.robot_velocity(Vr + pidVr + Vracc * Kacc), self.robot_velocity(Vl + pidVl + Vlacc * Kacc))

            counter += 1

        self.rm.brake(); self.lm.brake()

        if _print:
            print("counter: " + str(counter / lastTime))
            self.displayState()

    @micropython.native
    def turn(self, Kp, Ki, Kd, target, speedAlowwed = 2):
        integral = 0; lastTime = time(); lastError = 0
        while self.gyro.angle() != target or abs(self.gyro.speed()) > speedAlowwed:
            pid_value = self.pid_calc(self.gyro.angle(), target, lastError, integral, (time() - lastTime), self.gyro.speed(), Kp, Ki, Kd)
            lastTime = time(); integral += pid_value[1]; lastError = pid_value[2]
            self.move_on(pid_value[0], -pid_value[0])

        self.rm.brake(); self.lm.brake()       

    @micropython.native
    def turn2(self, Kp, Ki, Kd, target, speedAlowwed = 2):
        integral = 0; lastTime = time(); lastError = 0
        while abs(self.gyro.angle() - target) > 0.5 or abs(self.gyro.speed()) > speedAlowwed:
            pid_value = self.pid_calc(self.gyro.angle(), target, lastError, integral, (time() - lastTime), self.gyro.speed(), Kp, Ki, Kd)
            lastTime = time(); integral += pid_value[1]; lastError = pid_value[2]
            self.move_on(pid_value[0], -pid_value[0])

        self.rm.brake(); self.lm.brake()  