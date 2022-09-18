import random
import numpy as np
import math
from config import Config
import matplotlib.pyplot as plt
import sympy

color_list = ['blue', 'lightgreen', 'purple', 'yellow', 'brown', 'pink', 'orange', 'red', 'black']


class Drone:
    def __init__(self, theta, distance, ID=-1):
        self.theta = theta
        self.distance = distance
        self.id = ID


def polarToXY(angle, R):
    x = R * math.cos(angle)
    y = R * math.sin(angle)
    return x, y


def XYToPolar(x, y):
    theta = math.atan2(y, x)
    if theta < 0:
        theta += math.pi * 2
    return theta, dis((x, y))


def mul(A, B):
    return A[0] * B[0] + A[1] * B[1]


def plus(A, B):
    return A[0] + B[0], A[1] + B[1]


def mulScalar(a, A):
    return a * A[0], a * A[1]


def dis(A):
    return math.sqrt(mul(A, A))


def pos(x: int):
    if x == 0:
        return [0, 0]
    du = 2 * math.pi * (x - 1) / 9
    return [math.cos(du), math.sin(du)]


# 传理想状态
def CircleEquation(alpha, Sender, drone):
    SenderX, SenderY = polarToXY(Sender.theta, Sender.distance)
    Ideal = polarToXY(drone.theta, drone.distance)
    midpoint = (0.5 * SenderX, 0.5 * SenderY)
    vertical1 = (-SenderY / (2 * math.tan(alpha)), SenderX / (2 * math.tan(alpha)))
    vertical2 = (SenderY / (2 * math.tan(alpha)), -SenderX / (2 * math.tan(alpha)))
    center1 = plus(midpoint, vertical1)
    center2 = plus(midpoint, vertical2)
    temp1 = dis(plus(center1, mulScalar(-1, Ideal)))
    temp2 = dis(plus(center2, mulScalar(-1, Ideal)))
    if temp1 > temp2:
        center = center2
        radius = dis(plus(center2, mulScalar(-1, (SenderX, SenderY))))
    else:
        center = center1
        radius = dis(plus(center1, mulScalar(-1, (SenderX, SenderY))))
    # print("center:", center, "radius", radius)
    return center, radius


def angleCalculate(droneA, droneB):
    xA, yA = polarToXY(droneA.theta, droneA.distance)
    xB, yB = polarToXY(droneB.theta, droneB.distance)
    alpha_cos = mul((xA, yA), (xA - xB, yA - yB)) / (dis((xA, yA)) * dis((xA - xB, yA - yB)))
    return math.acos(alpha_cos)


def PositionCalculate(alpha, beta, Sender1, Sender2, drone):
    center1, radius1 = CircleEquation(alpha, Sender1, drone)
    center2, radius2 = CircleEquation(beta, Sender2, drone)
    if center2[0] - center1[0] == 0:
        y = 0
        x = 2 * center1[0]
        return x, y
    k = (center1[1] - center2[1]) / (center2[0] - center1[0])
    y = 2 * (center1[0] * k + center1[1]) / (k ** 2 + 1)
    x = k * y
    return x, y


class DroneGroupCircle:
    def __init__(self, number, initialAngle, initialRadius):
        self.SenderID = None
        self.R = Config.FlyRadius
        self.realPositionAngle = initialAngle
        self.realPositionRadius = initialRadius
        self.IdealDroneGroup = []
        self.followDrone = number
        for i, theta in enumerate(np.linspace(0, 2 * math.pi, number + 1)[:-1]):
            self.IdealDroneGroup.append(Drone(theta, self.R, i))
        self.Loss = [self.loss()]
        self.droneTraceAngle = [[theta] for theta in initialAngle]
        self.droneTraceRadius = [[d] for d in initialRadius]

    def SenderChosenRandom(self, number):
        self.SenderID = sorted(random.sample(range(9), number))
        # print("SenderID:", self.SenderID)

    def SimulateDroneControl(self, number):
        for _ in range(Config.epoch):
            self.SenderChosenRandom(number)
            # self.SenderID = [3, 7]
            for droneID in range(9):
                # print("droneID:", droneID)
                if droneID not in self.SenderID:
                    alpha = angleCalculate(Drone(self.realPositionAngle[droneID], self.realPositionRadius[droneID]),
                                           self.IdealDroneGroup[self.SenderID[0]])
                    beta = angleCalculate(Drone(self.realPositionAngle[droneID], self.realPositionRadius[droneID]),
                                          self.IdealDroneGroup[self.SenderID[1]])
                    gamma = angleCalculate(Drone(self.realPositionAngle[droneID], self.realPositionRadius[droneID]),
                                           self.IdealDroneGroup[self.SenderID[2]])
                    realPosition1 = PositionCalculate(alpha, beta, self.IdealDroneGroup[self.SenderID[0]],
                                                      self.IdealDroneGroup[self.SenderID[1]],
                                                      self.IdealDroneGroup[droneID])
                    realTheta1, realRadius1 = XYToPolar(realPosition1[0], realPosition1[1])
                    realPosition2 = PositionCalculate(alpha, gamma, self.IdealDroneGroup[self.SenderID[0]],
                                                      self.IdealDroneGroup[self.SenderID[2]],
                                                      self.IdealDroneGroup[droneID])
                    realTheta2, realRadius2 = XYToPolar(realPosition2[0], realPosition2[1])
                    realPosition3 = PositionCalculate(beta, gamma, self.IdealDroneGroup[self.SenderID[1]],
                                                      self.IdealDroneGroup[self.SenderID[2]],
                                                      self.IdealDroneGroup[droneID])
                    realTheta3, realRadius3 = XYToPolar(realPosition3[0], realPosition3[1])
                    if droneID == 0:
                        realTheta = 0
                    else:
                        realTheta = (realTheta1 + realTheta2 + realTheta3) / 3
                    realRadius = (realRadius1 + realRadius2 + realRadius3) / 3
                    # print("realTheta:", realTheta, "realR:", realRadius)
                    if droneID == 0:
                        if realTheta > math.pi:
                            self.realPositionAngle[droneID] = self.realPositionAngle[droneID] \
                                                              + Config.ChangeRateAngle * (math.pi * 2 - realTheta)
                        else:
                            self.realPositionAngle[droneID] = self.realPositionAngle[droneID] \
                                                              + Config.ChangeRateAngle * (0 - realTheta)
                    else:
                        self.realPositionAngle[droneID] = self.realPositionAngle[droneID] + Config.ChangeRateAngle \
                                                          * (self.IdealDroneGroup[droneID].theta - realTheta)

                    self.realPositionRadius[droneID] = self.realPositionRadius[droneID] + Config.ChangeRateRadius \
                                                       * (self.IdealDroneGroup[droneID].distance - realRadius)
                    self.droneTraceAngle[droneID].append(self.realPositionAngle[droneID])
                    self.droneTraceRadius[droneID].append(self.realPositionRadius[droneID])
            self.Loss.append(self.loss())

    def PrintPosition(self):
        print(self.realPositionAngle)
        print(self.realPositionRadius)

    def ShowPosition(self, color='red', size=10):
        plt.subplot(polar=True)
        plt.scatter(self.realPositionAngle, self.realPositionRadius, s=size, c=color)
        # plt.show()

    def ShowIdealPosition(self, color='red', size=20):
        plt.subplot(polar=True)
        plt.scatter([i.theta for i in self.IdealDroneGroup], [i.distance for i in self.IdealDroneGroup], s=size,
                    c=color)
        # plt.show()

    def ShowDroneTrace(self):
        plt.subplot(polar=True)
        for i in range(self.followDrone):
            plt.plot(self.droneTraceAngle[i], self.droneTraceRadius[i], c=color_list[i])

    def loss(self):
        loss = 0
        for i in range(self.followDrone):
            loss += self.realPositionRadius[i] ** 2 + self.IdealDroneGroup[i].distance ** 2 \
                    - 2 * self.realPositionRadius[i] * self.IdealDroneGroup[i].distance \
                    * math.cos(self.realPositionAngle[i] - self.IdealDroneGroup[i].theta)
        return loss

    def ShowLoss(self):
        plt.plot([i for i in range(Config.epoch + 1)], self.Loss, "--")
        plt.show()


Theta = [0, 40.10, 80.21, 119.75, 159.75, 199.96, 240.07, 280.17, 320.28]
Theta_ = [angle / 180 * math.pi for angle in Theta]
length = [100, 98, 112, 105, 98, 112, 105, 98, 112]
idealTheta = np.linspace(0, 2 * math.pi, 10)[:-1]
X = []
Y = []
for i in range(9):
    x, y = polarToXY(idealTheta[i], 100)
    X.append(x)
    Y.append(y)
print(X)
print(Y)
# testDrone = DroneGroupCircle(9, Theta_, length)
# testDrone.ShowPosition('green', 20)
# testDrone.ShowIdealPosition('red', 15)
# testDrone.SimulateDroneControl(3)
# testDrone.ShowPosition('pink', 10)
# testDrone.ShowDroneTrace()
# plt.clim(90, 120)
# plt.show()
# testDrone.ShowLoss()
# print(testDrone.Loss[0], testDrone.Loss[-1])
