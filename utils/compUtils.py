from .baseUtils import baseRobot
from time import time
from gc import collect
from math import radians, cos, sin


@micropython.native
def calc_omega_v(Vx, Vy, angle, l):
    r_a = radians(angle)
    v, omega = Vx * cos(r_a) - Vy * sin(r_a), (Vx * sin(r_a) + Vy * cos(r_a)) / l
    return (v, omega)

@micropython.native
def calc_velocity(omega, b, v):
    return (v - omega * b, v + omega * b)

class Robot(baseRobot):
    def __init__(self, rmPort, lmPort, gyroPort1, gyroPort2, LrmPort, LlmPort, wheelRad, halfDBM):
        super().__init__(rmPort, lmPort, gyroPort1, gyroPort2, LrmPort, LlmPort, wheelRad, halfDBM)
    
    @micropython.native
    def pid_calc(self, sensor, target, lastError, integral, Dt, De, Kp, Ki, Kd):
        error = sensor - target 
        P = error * Kp
        I = (((lastError + error) * Dt) / 2 + integral) * Ki
        D = De * Kd
        return [P + I + D, I, error]

    
    @micropython.native
    def spline(self, points, lastTime, l, Kp, Kd, Kacc):
        collect(); counter = 0

        rmStart, lmStart = self.rm.angle(), self.lm.angle()
        
        
        startTime = time()
        
        xLastError, yLastError = 0, 0
        startTime = time()
        # beforeTime = time()

        lastRightSpeed, lastLeftSpeed = 0, 0

        while (time() - startTime) <= lastTime:
            currentTime = time()
            rounded_t = int((currentTime - startTime) / 0.01)
            n_point = points[rounded_t]
            x, y = n_point["x"], n_point["y"]
            
            with self.lock:
                currentX, currentY = self.x, self.y
            
            currentX_tilda, currentY_tilda = self.pointTilda(currentX, currentY, l)
    

            xError, yError = x - currentX_tilda, y - currentY_tilda

            v, omega = calc_omega_v(n_point['x velocity'], n_point['y velocity'], self.gyro.angle(), l)
            Tl, Tr = calc_velocity(omega, self.halfDBM, v)
            
            Vx = n_point['x velocity'] + xError * Kp
            Vy = n_point['y velocity'] + yError * Kp

            v, omega = calc_omega_v(Vx, Vy, self.gyro.angle(), l)
            Vl, Vr = calc_velocity(omega, self.halfDBM, v)
            

            self.move_on(self.robot_velocity(Vr + (Tr - lastRightSpeed) * Kacc), self.robot_velocity(Vl + (Tl - lastLeftSpeed) * Kacc))

            beforeTime = currentTime
            xLastError, yLastError = xError, yError
            lastRightSpeed, lastLeftSpeed = Tr, Tl

            counter += 1

        self.rm.brake(); self.lm.brake()
        print("counter: " + str(counter / lastTime))

    @micropython.native
    def turn(self, Kp, Ki, Kd, target, base_speed = 0):
        integral = 0; lastTime = time(); lastError = 0
        while self.gyro.angle() != target or self.gyro.speed() != 0:
            pid_value = self.pid_calc(self.gyro.angle(), target, lastError, integral, (time() - lastTime), self.gyro.speed(), Kp, Ki, Kd)
            lastTime = time(); integral += pid_value[1]; lastError = pid_value[2]
            self.move_on(base_speed + pid_value[0], -(base_speed + pid_value[0]))

        self.rm.brake(); self.lm.brake()        