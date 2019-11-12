import random
import math
import time
import brickpi3
import time
import scipy
import scipy.stats


class Particle:
    def __init__(self, x, y, a, num):
        self.pos = (x, y, a)
        self.weight = 1/num

    def getPos(self):
        return self.pos

    def getWeight(self):
        return self.weight

    def setWeight(self, weight):
        self.weight = weight


class Canvas:
    def __init__(self, map_size=210):
        self.map_size = map_size    # in cm
        self.canvas_size = 768         # in pixels
        self.margin = 0.05*map_size
        self.scale = self.canvas_size/(map_size+2*self.margin)

    def drawLine(self, line):
        x1 = self.__screenX(line[0])
        y1 = self.__screenY(line[1])
        x2 = self.__screenX(line[2])
        y2 = self.__screenY(line[3])
        print("drawLine:" + str((x1, y1, x2, y2)))

    def drawParticles(self, data):
        display = [(self.__screenX(d.getPos()[0]), self.__screenY(
            d.getPos()[1])) + d.getPos()[2:] for d in data]
        print("drawParticles:" + str(display))

    def __screenX(self, x):
        return (x + self.margin)*self.scale

    def __screenY(self, y):
        return (self.map_size + self.margin - y)*self.scale


class Map:
    def __init__(self):
        self.walls = []
        self.canvas = Canvas()

    def add_wall(self, wall):
        self.walls.append(wall)

    def get_walls(self):
        return self.walls

    def clear(self):
        self.walls = []

    def draw(self):
        for wall in self.walls:
            self.canvas.drawLine(wall)


class TheoreticalMotion:
    def __init__(self, x_mean, y_mean, straight_angle_mean, rotation_angle_mean, x_sd, y_sd, straight_angle_sd, rotation_angle_sd, x, y):
        self.mu = (x_mean,  y_mean, straight_angle_mean, rotation_angle_mean)
        self.sigma = (x_sd,  y_sd, straight_angle_sd, rotation_angle_sd)
        self.particleNum = 100
        self.pos = {"x": x, "y": y, "theta": 0}
        self.xMean = x
        self.yMean = y
        self.points = [Particle(self.pos["x"], self.pos["y"], 0, self.particleNum)
                       for i in range(self.particleNum)]
        self.map = Map()
        self.canvas = Canvas()
        self.map.add_wall((0, 0, 0, 168))        # a
        self.map.add_wall((0, 168, 84, 168))     # b
        self.map.add_wall((84, 126, 84, 210))    # c
        self.map.add_wall((84, 210, 168, 210))   # d
        self.map.add_wall((168, 210, 168, 84))   # e
        self.map.add_wall((168, 84, 210, 84))    # f
        self.map.add_wall((210, 84, 210, 0))     # g
        self.map.add_wall((210, 0, 0, 0))        # h
        self.map.draw()

    def getClosestDistance(self, x, y, a):
        walls = self.map.get_walls()
        Ax = wall[0]
        Ay = wall[1]
        Bx = wall[2]
        By = wall[3]
        distances = []
        for wall in walls:
            distance.append(((By - Ay)*(Ax - x) - (Bx - Ax)*(Ay - y)) /
                            ((Bt-Ay)*math.cos(a) - (Bx - Ax)*math.sin(a)))

        return min(distances)

    def drawMove(self, x, y, a):
        line = (self.pos["x"], self.pos["y"], self.pos["x"] + x * math.cos(
            self.pos["theta"]), self.pos["y"] - y * math.sin(self.pos["theta"]))

        self.canvas.drawLine(line)

        self.pos["x"] += x * math.cos(self.pos["theta"])
        self.pos["y"] -= y * math.sin(self.pos["theta"])
        self.pos["theta"] = (self.pos["theta"] + a) % (math.pi * 2)

    def drawParticles(self, x, y, a, reading):
        self.updateParticles(x, y, a, reading)
        self.canvas.drawParticles(self.points)

    def updateParticles(self, x, y, a, reading):
        newpoints = self.points
        if a != 0:
            newpoints = list(map(
                lambda p: (
                    p.getPos()[0],
                    p.getPos()[1],
                    p.getPos()[2] + a + random.gauss(self.mu[3], self.sigma[3])
                ), newpoints))
        else:
            newpoints = list(map(
                lambda p: (
                    p.getPos()[0] + (x + random.gauss(self.mu[0], self.sigma[0])
                                     ) * math.cos(p.getPos()[2]),
                    p.getPos()[1] - (y + random.gauss(self.mu[1], self.sigma[1])
                                     ) * math.sin(p.getPos()[2]),
                    p.getPos()[2] + random.gauss(self.mu[2], self.sigma[2])
                ), newpoints))

        self.points = self.updateWeights(newpoints, reading)

        self.xMean = sum(
            list(map(lambda p: p.getPos()[0] * p.getWeight(), newpoints)))
        self.yMean = sum(
            list(map(lambda p: p.getPos()[1] * p.getWeight(), newpoints)))
        self.aMean = sum(
            list(map(lambda p: p.getPos()[2] * p.getWeight(), newpoints)))

    def updateWeights(self, points, reading):
        genePool = []
        for point in points:
            pos = point.getPos()
            closestDistance = self.getClosestDistance(pos[0], pos[1], pos[2])
            point.setWeight(scipy.stats.norm(reading, 2).pdf(
                closestDistance) * point.getWeight())

        total = sum(list(map(lambda p: p.getWeight(), points)))

        map(lambda p: p.setWeight(p.getWeight() / total), points)

        for point in points:
            numOfParticles = int(point.getWeight() * 100)
            genePool.extend([Particle(pos[0], pos[1], pos[2], prob)
                             for i in range(numOfParticles)])

        return [random.choice(genePool) for i in range(self.particleNum)]

    def moveAndUpdate(self, x, y, a, reading):
        self.drawMove(x, y, a)
        self.drawParticles(x, y, a, reading)

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

    # def turnLeft(self):
    #     self.reset()
    #     self.BP.set_motor_dps(self.BP.PORT_A, self.turn_dps)
    #     self.BP.set_motor_dps(self.BP.PORT_B, -self.turn_dps)
    #     time.sleep(1.40)
    #     self.BP.reset_all()
    #     self.theoreticalMotion.moveAndUpdate(0, 0, math.pi/2)

    # def turnRight(self):
    #     self.reset()
    #     self.BP.set_motor_dps(self.BP.PORT_A, -self.turn_dps)
    #     self.BP.set_motor_dps(self.BP.PORT_B, self.turn_dps)

    #     time.sleep(1.40)

    #     self.BP.reset_all()
    #     self.theoreticalMotion.moveAndUpdate(0, 0, -math.pi/2)

    def turnDegrees(self, angle):
        if angle > math.pi:
            angle -= (math.pi * 2)

        if angle < -math.pi:
            angle += (math.pi * 2)

        duration = (1.38 * abs(angle) * 2) / math.pi

        if angle > 0:
            self.reset()
            self.BP.set_motor_dps(self.BP.PORT_A, -self.turn_dps)
            self.BP.set_motor_dps(self.BP.PORT_B, self.turn_dps)

            time.sleep(duration)

            self.BP.reset_all()
            self.BP.set_sensor_type(
                self.BP.PORT_2, self.BP.SENSOR_TYPE.NXT_ULTRASONIC)
            time.sleep(1)

            reading = self.BP.get_sensor(self.BP.PORT_2)

            self.theoreticalMotion.moveAndUpdate(0, 0, -angle, reading)
        else:
            self.reset()
            self.BP.set_motor_dps(self.BP.PORT_A, self.turn_dps)
            self.BP.set_motor_dps(self.BP.PORT_B, -self.turn_dps)

            time.sleep(duration)

            self.BP.reset_all()
            self.BP.set_sensor_type(
                self.BP.PORT_2, self.BP.SENSOR_TYPE.NXT_ULTRASONIC)

            time.sleep(1)

            reading = self.BP.get_sensor(self.BP.PORT_2)
            self.theoreticalMotion.moveAndUpdate(0, 0, -angle, reading)

    def turnTowards(self, x, y):
        currentAngle = self.theoreticalMotion.pos["theta"]
        targetAngle = self.theoreticalMotion.getRelativeAngle(x, y)
        distance = self.theoreticalMotion.getDistance(x, y)

        self.turnDegrees(currentAngle - targetAngle)
        self.move(distance)

    def unitMove(self, frac):
        duration = 1.653 * frac
        self.reset()
        self.BP.set_motor_dps(self.BP.PORT_A, self.move_dps)
        self.BP.set_motor_dps(self.BP.PORT_B, self.move_dps)

        time.sleep(duration)

        self.BP.reset_all()
        self.BP.set_sensor_type(
            self.BP.PORT_2, self.BP.SENSOR_TYPE.NXT_ULTRASONIC)
        time.sleep(1)
        reading = self.BP.get_sensor(self.BP.PORT_2)

        self.theoreticalMotion.moveAndUpdate(10 * frac, 10 * frac, 0, reading)

    def move(self, dist):
        frac = (dist % 10) / 10
        count = int(dist / 10)

        for i in range(count):
            self.unitMove(1)

        if frac > 0:
            self.unitMove(frac)

    def drawSquare(self):
        for i in range(0, 4):
            self.move(40)
            self.turnLeft()

    def goToSquare(self):
        realMotion.turnTowards(40, 0)
        realMotion.turnTowards(40, 40)
        realMotion.turnTowards(0, 40)
        realMotion.turnTowards(0, 0)


realMotion = RealMotion(0, 0)
mymap = Map()
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
