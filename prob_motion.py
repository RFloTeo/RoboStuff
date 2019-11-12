import random
import math
import time
import brickpi3
import time


class TheoreticalMotion:
    def __init__(self, x_mean, y_mean, straight_angle_mean, rotation_angle_mean, x_sd, y_sd, straight_angle_sd, rotation_angle_sd, x, y):
        self.mu = (x_mean,  y_mean, straight_angle_mean, rotation_angle_mean)
        self.sigma = (x_sd,  y_sd, straight_angle_sd, rotation_angle_sd)
        self.particleNum = 100
        self.pos = {"x": x, "y": y, "theta": 0}
        self.xMean = x
        self.yMean = y
        self.points = [(self.pos["x"], self.pos["y"], 0)
                       for i in range(self.particleNum)]
        print("drawParticles:" + str(self.points))

    def drawMove(self, x, y, a):
        line = (self.pos["x"], self.pos["y"], self.pos["x"] + x * math.cos(
            self.pos["theta"]), self.pos["y"] - y * math.sin(self.pos["theta"]))

        print("drawLine:" + str(line))
        print("Line:" + str(line))
        self.pos["x"] += x * math.cos(self.pos["theta"])
        self.pos["y"] -= y * math.sin(self.pos["theta"])
        self.pos["theta"] = (self.pos["theta"] + a) % (math.pi * 2)

    def drawParticles(self, x, y, a):
        newpoints = self.points[len(self.points) - self.particleNum:]
        if a != 0:
            newpoints = list(map(
                lambda p: (
                    p[0],
                    p[1],
                    p[2] + a + random.gauss(self.mu[3], self.sigma[3])
                ), newpoints))
        else:
            newpoints = list(map(
                lambda p: (
                    p[0] + (x + random.gauss(self.mu[0], self.sigma[0])
                            ) * math.cos(p[2]),
                    p[1] - (y + random.gauss(self.mu[1], self.sigma[1])
                            ) * math.sin(p[2]),
                    p[2] + random.gauss(self.mu[2], self.sigma[2])
                ), newpoints))
        self.points += newpoints
        self.xMean = sum(list(map(lambda p: p[0], newpoints)))/self.particleNum
        self.yMean = sum(list(map(lambda p: p[1], newpoints)))/self.particleNum
        print("xMean ", self.xMean, " YMean ", self.yMean)
        print("drawParticles:" + str(self.points))

    def moveAndUpdate(self, x, y, a):
        self.drawMove(x, y, a)
        self.drawParticles(x, y, a)

    def getDistance(self, x, y):
        return math.sqrt((x - self.xMean)**2 + (y - self.yMean)**2)

    def getRelativeAngle(self, x, y):
        ydiff = abs(y - self.yMean)
        angle = math.asin(ydiff/self.getDistance(x, y))

        print("angle: ", angle)
        if x >= self.xMean and y <= self.yMean:
            return angle

        if x >= self.xMean and y >= self.yMean:
            return (math.pi * 2) - angle

        if x <= self.xMean and y <= self.yMean:
            return math.pi - angle

        if x <= self.xMean and y >= self.yMean:
            return math.pi + angle


class RealMotion:
    def __init__(self, x, y):
        self.turn_dps = 100
        self.move_dps = -100
        self.R = 360
        self.theoreticalMotion = TheoreticalMotion(
            0, 0, 0, 0, 1.5, 1.5, 0.01, 0.04, x, y)

        self.BP = brickpi3.BrickPi3()
        self.BP.offset_motor_encoder(
            self.BP.PORT_A, self.BP.get_motor_encoder(self.BP.PORT_A))
        self.BP.offset_motor_encoder(
            self.BP.PORT_B, self.BP.get_motor_encoder(self.BP.PORT_B))

        self.BP.set_motor_limits(self.BP.PORT_A, 30, 0.5 * self.R)
        self.BP.set_motor_limits(self.BP.PORT_B, 30, 0.5 * self.R)

    def reset(self):
        self.BP = brickpi3.BrickPi3()
        self.BP.offset_motor_encoder(
            self.BP.PORT_A, self.BP.get_motor_encoder(self.BP.PORT_A))
        self.BP.offset_motor_encoder(
            self.BP.PORT_B, self.BP.get_motor_encoder(self.BP.PORT_B))

        self.BP.set_motor_limits(self.BP.PORT_A, 30, 0.5 * self.R)
        self.BP.set_motor_limits(self.BP.PORT_B, 30, 0.5 * self.R)

    def turnLeft(self):
        self.reset()
        self.BP.set_motor_dps(self.BP.PORT_A, self.turn_dps)
        self.BP.set_motor_dps(self.BP.PORT_B, -self.turn_dps)
        time.sleep(1.40)
        self.BP.reset_all()
        self.theoreticalMotion.moveAndUpdate(0, 0, math.pi/2)

    def turnRight(self):
        self.reset()
        self.BP.set_motor_dps(self.BP.PORT_A, -self.turn_dps)
        self.BP.set_motor_dps(self.BP.PORT_B, self.turn_dps)

        time.sleep(1.40)

        self.BP.reset_all()
        self.theoreticalMotion.moveAndUpdate(0, 0, -math.pi/2)

    def turnDegrees(self, angle):
        if angle > math.pi:
            angle -= (math.pi * 2)

        if angle < -math.pi:
            angle += (math.pi * 2)

        duration = (1.38 * abs(angle) * 2) / math.pi

        if angle > 0:
            print("turning right ", angle)
            self.reset()
            self.BP.set_motor_dps(self.BP.PORT_A, -self.turn_dps)
            self.BP.set_motor_dps(self.BP.PORT_B, self.turn_dps)

            time.sleep(duration)

            self.BP.reset_all()
            self.theoreticalMotion.moveAndUpdate(0, 0, -angle)
            print("new angle: ", self.theoreticalMotion.pos["theta"])
        else:
            print("turning left ", angle)
            self.reset()
            self.BP.set_motor_dps(self.BP.PORT_A, self.turn_dps)
            self.BP.set_motor_dps(self.BP.PORT_B, -self.turn_dps)

            time.sleep(duration)

            self.BP.reset_all()
            self.theoreticalMotion.moveAndUpdate(0, 0, -angle)
            print("new angle: ", self.theoreticalMotion.pos["theta"])

    def turnTowards(self, x, y):
        currentAngle = self.theoreticalMotion.pos["theta"]
        targetAngle = self.theoreticalMotion.getRelativeAngle(x, y)
        distance = self.theoreticalMotion.getDistance(x, y)

        print("curr angle: ", currentAngle)
        print("target: ", targetAngle)
        print("pos: ", self.theoreticalMotion.xMean,
              " ", self.theoreticalMotion.yMean)

        self.turnDegrees(currentAngle - targetAngle)
        self.move(distance)

    def unitMove(self, frac):
        duration = 1.653 * frac
        self.reset()
        self.BP.set_motor_dps(self.BP.PORT_A, self.move_dps)
        self.BP.set_motor_dps(self.BP.PORT_B, self.move_dps)

        time.sleep(duration)

        self.BP.reset_all()
        self.theoreticalMotion.moveAndUpdate(100 * frac, 100 * frac, 0)

    def move(self, dist):
        frac = (dist % 100) / 100
        count = int(dist / 100)

        for i in range(count):
            self.unitMove(1)

        if frac > 0:
            self.unitMove(frac)

    def drawSquare(self):
        for i in range(0, 4):
            self.move(400)
            self.turnLeft()

    def goToSquare(self):
        realMotion.turnTowards(500, 600)
        realMotion.turnTowards(500, 200)
        realMotion.turnTowards(100, 200)
        realMotion.turnTowards(100, 600)


realMotion = RealMotion(200, 500)
try:

    # realMotion.drawSquare()

    # realMotion.turnTowards(300, 500)
    # realMotion.turnTowards(300, 300)
    # realMotion.turnTowards(200, 500)
    # realMotion.turnTowards(200, 600)
    # realMotion.turnTowards(100, 600)

    realMotion.goToSquare()


except KeyboardInterrupt:
    realMotion.BP.reset_all()
