import random
import math
import time
import brickpi333 as brickpi3
import time
import scipy
import random
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

    def setPos(self, pos):
        x = pos[0]
        y = pos[1]
        a = pos[2]

        if a > math.pi * 2:
            a -= math.pi * 2
        if a < 0:
            a += math.pi * 2

        self.pos = (x, y, a)


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
        self.particleNum = 50
        self.pos = {"x": x, "y": y, "theta": 0}
        self.xMean = x
        self.yMean = y
        self.aMean = 0
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

    def addObstacle(self, angle, distance):
        print("current angle before drawing wall: ", (angle / math.pi) * 180)
        s = math.sin(angle)
        c = math.cos(angle)
        centerX = (self.xMean + distance * c) + 5
        centerY = (self.yMean + distance * s) - 5
        Ax = centerX - s * 8
        Ay = centerY + c * 8
        Bx = centerX + s * 8
        By = centerY - c * 8
        self.map.add_wall((Ax, Ay, Bx, By))
        self.map.draw()

    def getClosestDistance(self, x, y, a):
        walls = self.map.get_walls()
        distances = []
        for wall in walls:
            Ax = wall[0]
            Ay = wall[1]
            Bx = wall[2]
            By = wall[3]
            if ((By-Ay)*math.cos(a) - (Bx - Ax)*math.sin(a)) != 0:

                distance = (((By - Ay)*(Ax - x) - (Bx - Ax)*(Ay - y)) /
                            ((By-Ay)*math.cos(a) - (Bx - Ax)*math.sin(a)))

                xIntersection = round(x + distance * math.cos(a))
                yIntersection = round(y + distance * math.sin(a))

                if distance > 0 and xIntersection >= min(Ax, Bx) and xIntersection <= max(Ax, Bx) and yIntersection >= min(Ay, By) and yIntersection <= max(Ay, By):
                    try:
                        if self.getNormalAngle(Ax, Ay, Bx, By) > math.pi / 4:
                            distances.append(
                                (distance,  1.2, (Ax, Ay, Bx, By)))
                        else:
                            distances.append((distance,  1, (Ax, Ay, Bx, By)))
                    except:
                        distances.append((distance,  1.2, (Ax, Ay, Bx, By)))
        try:
            minDistance = distances[0]
            for distance in distances:
                if distance[0] < minDistance[0]:
                    minDistance = distance

            return minDistance
        except ValueError:
            return (1000, 1, -1)

    def drawMove(self, x, y, a):
        line = (self.pos["x"], self.pos["y"], self.xMean, self.yMean)

        self.canvas.drawLine(line)

        self.pos["x"] = self.xMean
        self.pos["y"] = self.yMean
        self.pos["theta"] = self.aMean

    def drawParticles(self, x, y, a, reading):
        self.updateParticles(x, y, a, reading)
        # self.canvas.drawParticles(self.points)

    def updateParticles(self, x, y, a, reading):
        newpoints = self.points
        if a != 0:
            for point in newpoints:
                point.setPos((point.getPos()[0],
                              point.getPos()[1],
                              point.getPos()[2] + a + random.gauss(self.mu[3], self.sigma[3])))
        else:
            for point in newpoints:
                point.setPos((point.getPos()[0] + (x + random.gauss(self.mu[0], self.sigma[0])
                                                   ) * math.cos(point.getPos()[2]),
                              point.getPos()[1] + (y + random.gauss(self.mu[1], self.sigma[1])
                                                   ) * math.sin(point.getPos()[2]),
                              point.getPos()[2] + random.gauss(self.mu[2], self.sigma[2])))

        self.points = self.updateWeights(newpoints, reading)

        self.xMean = sum(
            list(map(lambda p: p.getPos()[0], self.points))) / self.particleNum
        self.yMean = sum(
            list(map(lambda p: p.getPos()[1], self.points))) / self.particleNum

        avgAngle = 0
        for p in self.points:
            angle = p.getPos()[2]
            if angle > math.pi:
                angle = angle - (math.pi * 2)
            avgAngle += angle

        avgAngle = avgAngle / self.particleNum

        if avgAngle < 0:
            avgAngle = (math.pi * 2) + avgAngle

        self.aMean = avgAngle

    def updateWeights(self, points, reading):
        total = 0
        sigma = 2
        for point in points:
            pos = point.getPos()
            closestDistance = self.getClosestDistance(
                pos[0], pos[1], pos[2])[0]
            if abs(closestDistance - reading) < 15:
                newWeight = scipy.stats.norm.pdf(
                    reading, sigma, closestDistance)
                point.setWeight(newWeight)
                total += newWeight
            else:
                total += point.getWeight()
        for p in points:
            p.setWeight(p.getWeight() / total)

        newParticles = []
        for i in range(self.particleNum):
            newParticleCDF = random.uniform(0, 1)
            cdf = 0
            for p in points:
                pos = p.getPos()
                cdf += p.getWeight()
                if cdf >= newParticleCDF:
                    newParticle = Particle(
                        pos[0], pos[1], pos[2], self.particleNum)
                    newParticles.append(newParticle)
                    break

        return newParticles

    def moveAndUpdate(self, x, y, a, reading):
        self.drawParticles(x, y, a, reading)
        self.drawMove(x, y, a)

    def getDistance(self, x, y):
        return math.sqrt((x - self.xMean)**2 + (y - self.yMean)**2)

    def getRelativeAngle(self, x, y):
        ydiff = abs(y - self.yMean)
        angle = math.asin(ydiff/self.getDistance(x, y))

        if x >= self.xMean and y >= self.yMean:
            print("Quadrant 1 angle ", angle, "returning ", angle)
            return angle

        if x >= self.xMean and y <= self.yMean:
            print("Quadrant 4 angle ", angle,
                  "returning ", (math.pi * 2) - angle)
            return (math.pi * 2) - angle

        if x <= self.xMean and y >= self.yMean:
            print("Quadrant 2 angle ", angle, "returning ",  math.pi - angle)
            return math.pi - angle

        if x <= self.xMean and y <= self.yMean:
            print("Quadrant 3 angle ", angle, "returning ",  math.pi + angle)
            return math.pi + angle

    def getNormalAngle(self, Ax, Ay, Bx, By):
        aux = math.sqrt((Ay - By) ** 2 + (Ax - Bx) ** 2)
        aux2 = math.cos(self.aMean) * (Ay - By) + \
            math.sin(self.aMean) * (Bx - Ax)
        return math.acos(aux2/aux)


class RealMotion:
    def __init__(self, x, y):
        self.turn_dps = 100
        self.move_dps = -200
        self.R = 360
        self.theoreticalMotion = TheoreticalMotion(
            0, 0, 0, 0, 0.1, 0.1, 0.01, 0.002, x, y)

        self.BP = brickpi3.BrickPi333()
        self.reset()

    def reset(self):
        self.BP.offset_motor_encoder(
            self.BP.PORT_A, self.BP.get_motor_encoder(self.BP.PORT_A))
        self.BP.offset_motor_encoder(
            self.BP.PORT_B, self.BP.get_motor_encoder(self.BP.PORT_B))
        self.BP.set_sensor_type(self.BP.PORT_3, self.BP.SENSOR_TYPE.TOUCH)
        self.BP.set_sensor_type(self.BP.PORT_4, self.BP.SENSOR_TYPE.TOUCH)
        self.BP.set_sensor_type(
            self.BP.PORT_1, self.BP.SENSOR_TYPE.NXT_ULTRASONIC)

        self.BP.set_motor_limits(self.BP.PORT_A, 30, 0.5 * self.R)
        self.BP.set_motor_limits(self.BP.PORT_B, 30, 0.5 * self.R)

    def turnDegreesPosition(self, angle):
        oneDegree = 70 / 45 # Rotation needed per wheel to move one degree, in degrees
        if abs(angle) <= (15 * math.pi / 180):
            # We use a particular conversion for small angles
            oneDegree = 91 / 45
            
        if angle > math.pi:
            angle -= (math.pi * 2)

        if angle < -math.pi:
            angle += (math.pi * 2)

        angleInDeg = angle * (180 / math.pi)
        toTurn = oneDegree * angleInDeg
        self.reset()
        self.BP.set_motor_position(self.BP.PORT_A, -toTurn)
        self.BP.set_motor_position(self.BP.PORT_B, toTurn)
        time.sleep(abs(angle * 2))   # Time our rotation late
        
        self.BP.reset_all()
        reading = self.getReading()
        self.theoreticalMotion.moveAndUpdate(0, 0, -angle, reading)
    
    def turnDegrees(self, angle):
        if angle > math.pi:
            angle -= (math.pi * 2)

        if angle < -math.pi:
            angle += (math.pi * 2)

        duration = (1.38 * abs(angle) * 2) / math.pi
        self.reset()
        if angle > 0:
            self.BP.set_motor_dps(self.BP.PORT_A, -self.turn_dps)
            self.BP.set_motor_dps(self.BP.PORT_B, self.turn_dps)
        else:
            self.BP.set_motor_dps(self.BP.PORT_A, self.turn_dps)
            self.BP.set_motor_dps(self.BP.PORT_B, -self.turn_dps)

        time.sleep(duration)
        self.BP.reset_all()
        reading = self.getReading()
        self.theoreticalMotion.moveAndUpdate(0, 0, -angle, reading)

    def getReading(self):
        self.reset()
        reading = []
        bad = False
        while len(reading) < 5:
            try:
                tryread = self.BP.get_sensor(self.BP.PORT_1)
                if tryread != 255:
                    reading.append(tryread)
                    bad = False
                else:
                    time.sleep(0.1)
                    if not bad:
                        print("I am getting 255s")
                        bad = True
            except:
                time.sleep(0.1)
                if not bad:
                    print("I am getting bad sensor readings")
                    bad = True
        mean = sum(reading) / len(reading)
        print(reading)
        print("I sensed: ", mean)
        return mean

    def nearby(self, x, y):
        return abs(self.theoreticalMotion.xMean - x) < 2 and abs(self.theoreticalMotion.yMean - y) < 2

    def lookTowards(self, x, y):
        currentAngle = self.theoreticalMotion.aMean
        targetAngle = self.theoreticalMotion.getRelativeAngle(x, y)
        self.turnDegrees(currentAngle - targetAngle)

    def findBottle(self, Ax, Ay, Bx, By):
        found = self.scanArea(Ax, Ay, Bx, By)
        if found:
            dist = self.unitMoveWithSenseStop()
            self.backwardsUnitMove(1)

            reading = self.getReading()
            self.theoreticalMotion.moveAndUpdate(
                dist - 20, dist - 20, 0, reading)
            print("Position after bottle" + str(self.theoreticalMotion.pos))
        else:
            self.unitMove(1)
            self.findBottle(Ax, Ay, Bx, By)

    # Assuming angle to (Ax, Ay) is > angle to (Bx, By)
    def scanArea(self, Ax, Ay, Bx, By):
        deg = 10 * math.pi/180
        original_angle = self.theoreticalMotion.aMean
        self.lookTowards(Ax, Ay)
        targetAngle = self.theoreticalMotion.getRelativeAngle(Bx, By)
        print("aMean: ", self.theoreticalMotion.aMean,
              "target angle: ", targetAngle)
        toTurn = 0
        if self.theoreticalMotion.aMean < math.pi:
            toTurn = self.theoreticalMotion.aMean - targetAngle
        else:
            toTurn = (2 * math.pi) - self.theoreticalMotion.aMean + targetAngle
            deg = -deg
        
        acc = 0
        while toTurn > 0:
            print("toTurn: ", toTurn)
            #self.turnDegrees(deg)
            self.turnDegreesPosition(deg)
            toTurn -= abs(deg)
            reading = self.getReading()
            expectedReading = self.theoreticalMotion.getClosestDistance(
                self.theoreticalMotion.xMean, self.theoreticalMotion.yMean, self.theoreticalMotion.aMean)
            acc += 1

            print("reading: ", reading, "check less than: ",
                  expectedReading[0] - 15 * expectedReading[1], "error", expectedReading[1])
            if reading < expectedReading[0] - 15 * expectedReading[1]:
                # We do one final rotation, since the sensor detects the obstacle in it's peripherals
                # If the obstacle is further away, we turn less. If it's closer, we turn more.
                multiplier = 70 / expectedReading[0]
                print(str(multiplier))
                self.turnDegreesPosition(15 * (math.pi / 180) * multiplier)
                print("Final ROt:" + str(15 * (math.pi / 180) * multiplier))
                # add wall
                angle = self.theoreticalMotion.aMean
                self.theoreticalMotion.addObstacle(angle, reading)
                return True

        print("ERROR: no bottle" )
        current_angle = self.theoreticalMotion.aMean
        print("original angle: ", original_angle,
                  "current angle: ", current_angle)
        while (acc  > 0):
            self.turnDegreesPosition(-1 * deg)
            acc -= 2
        #self.turnDegreesPosition(current_angle - original_angle)
        return False

    def moveDistance(self, x, y):
        distance = self.theoreticalMotion.getDistance(x, y)
        self.lookTowards(x, y)
        print("distance:", distance)

        print("I think I am at: ",
              (self.theoreticalMotion.xMean, self.theoreticalMotion.yMean))
        print("I plan to go to: ", (x, y))
        duration = 1.65 * (distance / 20)

        self.reset()
        self.BP.set_motor_dps(self.BP.PORT_A, self.move_dps)
        self.BP.set_motor_dps(self.BP.PORT_B, self.move_dps)

        time.sleep(duration)

        self.BP.reset_all()
        self.reset()

        reading = self.getReading()
        self.theoreticalMotion.moveAndUpdate(distance, distance, 0, reading)

    def localizedMove(self, x, y):
        while not self.nearby(x, y):
            distance = self.theoreticalMotion.getDistance(x, y)
            self.lookTowards(x, y)
            print("distance:", distance)

            print("I think I am at: ",
                  (self.theoreticalMotion.xMean, self.theoreticalMotion.yMean))
            print("I plan to go to: ", (x, y))

            if distance < 20:
                frac = distance / 20
                self.unitMove(frac)
            else:
                self.unitMove(1)

    def unitMove(self, frac):
        duration = 1.65 * frac
        self.reset()
        self.BP.set_motor_dps(self.BP.PORT_A, self.move_dps)
        self.BP.set_motor_dps(self.BP.PORT_B, self.move_dps)

        time.sleep(duration)

        self.BP.reset_all()
        self.reset()

        reading = self.getReading()
        self.theoreticalMotion.moveAndUpdate(20 * frac, 20 * frac, 0, reading)

    def readTouch(self):
        read = False
        value_touch_1 = 0
        value_touch_2 = 0
        while not read:
            try:
                value_touch_1 = self.BP.get_sensor(self.BP.PORT_3)
                value_touch_2 = self.BP.get_sensor(self.BP.PORT_4)
                read = True
            except:
                pass

        return (value_touch_1 == 1 or value_touch_2 == 1)

    def unitMoveWithSenseStop(self):
        duration = 1.65  # 1.65seconds for 20cm
        self.reset()
        self.BP.set_motor_dps(self.BP.PORT_A, self.move_dps)
        self.BP.set_motor_dps(self.BP.PORT_B, self.move_dps)

        start = time.time()
        while not self.readTouch():
            time.sleep(0.1)
            pass
        actual_time = time.time() - start

        self.BP.reset_all()
        actual_duration = actual_time / duration
        return actual_duration * 20

    def backwardsUnitMove(self, frac):
        duration = 1.65 * frac
        self.BP.reset_all()
        self.reset()
        self.BP.set_motor_dps(self.BP.PORT_A, -self.move_dps)
        self.BP.set_motor_dps(self.BP.PORT_B, -self.move_dps)

        time.sleep(duration)

        self.BP.reset_all()


realMotion = RealMotion(84, 30)
# Bottle 1
realMotion.moveDistance(104, 30)
realMotion.findBottle(168, 0, 168, 84)

# Bottle 2
realMotion.moveDistance(115, 90)
realMotion.moveDistance(115, 110)
realMotion.findBottle(84, 126, 168, 126)

# Bottle 3
realMotion.moveDistance(100, 50)
realMotion.moveDistance(50, 50)
realMotion.findBottle(0, 30, 80, 130)
