import math

from config import *
import numpy as np
import random
from math import *
import matplotlib.pyplot as plt


# 测量两向量之间的夹角
def angleCalculate(vector1, vector2):
    alphaCos = np.dot(vector1, vector2) / (np.linalg.norm(vector1) * np.linalg.norm(vector2))
    return acos(alphaCos)


# 传理想状态
def CircleEquation(alpha, senderPos, idealReceiverPos):
    midpoint = senderPos * [0.5]
    SenderX = senderPos[0]
    SenderY = senderPos[1]
    if alpha == 0:
        return -1, -1
    vertical1 = np.array((-SenderY / (2 * tan(alpha)), SenderX / (2 * tan(alpha))))
    vertical2 = np.array((SenderY / (2 * tan(alpha)), -SenderX / (2 * tan(alpha))))
    center1 = midpoint + vertical1
    center2 = midpoint + vertical2
    radius = np.linalg.norm(center2 - senderPos)
    return center1, center2, radius


def helper(center1, center2):
    if center2[0] - center1[0] == 0:
        y = 0
        x = 2 * center1[0]
        return x, y
    k = (center1[1] - center2[1]) / (center2[0] - center1[0])
    y = 2 * (center1[0] * k + center1[1]) / (k ** 2 + 1)
    x = k * y
    return np.array((x, y))


def PositionCalculate(alpha, beta, Sender1Pos, Sender2Pos, idealReceiverPos):
    center1, center1_, radius1 = CircleEquation(alpha, Sender1Pos, idealReceiverPos)
    center2, center2_, radius2 = CircleEquation(beta, Sender2Pos, idealReceiverPos)
    pos = [0, 0, 0, 0]
    pos[0] = helper(center1, center2)
    pos[1] = helper(center1, center2_)
    pos[2] = helper(center1_, center2)
    pos[3] = helper(center1_, center2_)
    min_dis = np.linalg.norm(pos[0] - idealReceiverPos)
    min_pos = pos[0]
    for i in range(1, 4):
        if np.linalg.norm(pos[i] - idealReceiverPos) < min_dis:
            min_pos = pos[i]
            min_dis = np.linalg.norm(pos[i] - idealReceiverPos)
    return min_pos


class DroneGroup:
    def __init__(self, droneNumber, initialX, initialY, idealX, idealY):
        self.SenderID = None
        self.SenderNumber = 4
        self.realPositionX = initialX.copy()
        self.realPositionY = initialY.copy()
        self.IdealX = idealX
        self.IdealY = idealY
        self.InitX = initialX.copy()
        self.InitY = initialY.copy()
        self.droneNumber = droneNumber
        print(self.loss())
        self.Loss = [self.loss()]
        self.Time = 0
        self.droneTraceX = [[x] for x in initialX]
        self.droneTraceRadius = [[y] for y in initialY]
        self.average = 0
        self.helperX = []
        self.helperY = []

    # 抽取发送飞机的法则飞机的法则
    def SenderChosen(self):
        self.SenderID = sorted(random.sample([0, 2, 4, 5, 8, 9, 10, 13], self.SenderNumber))
        # self.SenderID = sorted(random.sample(range(9), self.SenderNumber))

    # 清空辅助容器
    def ClearHelper(self):
        self.helperX = []
        self.helperY = []

    # 改变发送信号无人机的数量
    def ChangeSenderNumber(self, n):
        self.SenderNumber = n

    # 恢复初始状态
    def Reset(self):
        self.realPositionX = self.InitX.copy()
        self.realPositionY = self.InitY.copy()
        self.Loss = [self.loss()]

    # 定义损失函数
    def loss(self, chosen='triangle'):
        loss = 0
        if chosen == 'triangle':
            dis = [self.dis2(0, 1), self.dis2(1, 2), self.dis2(2, 3), self.dis2(3, 4), self.dis2(0, 5),
                   self.dis2(1, 5), self.dis2(1, 6), self.dis2(2, 6), self.dis2(2, 7), self.dis2(3, 7),
                   self.dis2(3, 8), self.dis2(4, 8), self.dis2(5, 6), self.dis2(6, 7), self.dis2(7, 8),
                   self.dis2(5, 9), self.dis2(6, 9), self.dis2(-1, 6), self.dis2(-1, 7), self.dis2(7, 10),
                   self.dis2(8, 10), self.dis2(-1, 9), self.dis2(-1, 10), self.dis2(9, 11), self.dis2(-1, 11),
                   self.dis2(-1, 12), self.dis2(10, 12), self.dis2(11, 12), self.dis2(11, 13), self.dis2(12, 13)]
            self.average = np.average(dis)
            loss = np.var(dis) / 30
        if chosen == 'circle':
            dis = []
            for i in range(self.droneNumber - 1):
                dis.append(np.linalg.norm(np.array([self.realPositionX[i], self.realPositionY[i]])))
            loss = np.var(dis) / 9
            self.average = np.average(dis)
        return loss

    # 计算两架飞机之间的距离，id1<id2
    def dis2(self, id1, id2):
        if id1 == -1:
            return np.linalg.norm(np.array([self.realPositionX[id2], self.realPositionY[id2]]))
        else:
            return np.linalg.norm(np.array([self.realPositionX[id1] - self.realPositionX[id2],
                                            self.realPositionY[id1] - self.realPositionY[id2]]))

    # 计算出接收机的所有接收角
    def AllAngle(self, droneID):
        angleList = []
        for senderId in self.SenderID:
            angleList.append(angleCalculate(np.array([self.realPositionX[droneID], self.realPositionY[droneID]]),
                                            np.array([self.realPositionX[droneID] - self.realPositionX[senderId],
                                                      self.realPositionY[droneID] - self.realPositionY[senderId]])))
        return angleList

    # 根据角度两两组合计算出距离之和
    def SumDistance(self, angleList, dronePos, n=0):
        if n == self.SenderNumber - 1:
            return np.array([0, 0])
        else:
            sumX = 0
            sumY = 0
            alpha = angleList[n]
            Sender1ID = self.SenderID[n]
            Sender1Pos = np.array((self.IdealX[Sender1ID], self.IdealY[Sender1ID]))
            temp = 0
            for beta in angleList[n + 1:]:
                temp += 1
                Sender2ID = self.SenderID[n + temp]
                Sender2Pos = np.array((self.IdealX[Sender2ID], self.IdealY[Sender2ID]))
                X, Y = PositionCalculate(alpha, beta, Sender1Pos, Sender2Pos, dronePos)
                sumX += X
                sumY += Y
                self.helperX.append(X)
                self.helperY.append(Y)
            return np.array((sumX, sumY)) + self.SumDistance(angleList, dronePos, n + 1)

    # 一轮调整模拟
    def AdjustOneCycle(self, epoch):
        self.SenderChosen()
        for droneID in range(self.droneNumber - 1):
            if droneID not in self.SenderID:
                AngleList = self.AllAngle(droneID)
                dronePos = np.array((self.IdealX[droneID], self.IdealY[droneID]))
                self.ClearHelper()
                sumX, sumY = self.SumDistance(AngleList, dronePos)
                varX, varY = np.var(self.helperX), np.var(self.helperY)
                if varX < 500 or varY < 500:
                    # print(varX, varY)
                    matchNumber = self.SenderNumber * (self.SenderNumber - 1) / 2
                    averageX, averageY = sumX / matchNumber, sumY / matchNumber
                    rateX = Config.ChangeRateX
                    rateY = Config.ChangeRateY
                    rateX = rateX / log2(1 + exp(epoch))
                    rateY = rateY / log2(1 + exp(epoch))
                    self.realPositionX[droneID] = self.realPositionX[droneID] + rateX * \
                                                  (self.IdealX[droneID] - averageX)
                    self.realPositionY[droneID] = self.realPositionY[droneID] + rateY * \
                                                  (self.IdealY[droneID] - averageY)

    def AdjustSimulate(self):
        for i in range(Config.epoch):
            self.AdjustOneCycle(i)
            self.Loss.append(self.loss())

    def ShowPosition(self, size=20, color='blue'):
        plt.scatter(self.realPositionX, self.realPositionY, s=size, c=color)

    def ShowIdeal(self, size=10, color='red'):
        plt.scatter(self.IdealX, self.IdealY, s=size, c=color)

    def showLoss(self, n=0):
        plt.plot(range(len(self.Loss)), self.Loss, label=str(n))

    def ShowDifferentSenderNumber(self):
        for i in range(2, 8):
            self.Reset()
            self.ChangeSenderNumber(i)
            self.AdjustSimulate()
            self.showLoss(i)
        plt.legend()
        plt.show()


s = sqrt(3)
k = 10
IdealX = np.array([-50 * s] * 5 + [-25 * s] * 4 + [0] * 2 + [25 * s, 25 * s, 50 * s])
IdealY = np.array([100, 50, 0, -50, -100, 75, 25, -25, -75, 50, -50, 25, -25, 0])
randomSeedX = np.array(list(np.random.uniform(-2, -1.5, size=7) * k) + list(np.random.uniform(1.5, 2, size=7) * k))
randomSeedY = np.array(list(np.random.uniform(-2, -1.5, size=7) * k) + list(np.random.uniform(1.5, 2, size=7) * k))
# randomSeedX = np.array([7.79628639, -4.35579198, 3.08839604, 3.67063682, -3.86624106, -1.19097122,
#                         6.37099174, -5.28976998, -7.62432039, -3.42668103, 4.25249008, -1.00914967,
#                         7.5927625, -7.97201653])
# randomSeedY = np.array([-6.38121479, -1.02289905, -5.82353205, -6.78910702, 3.01882658, - 0.86318369,
#                         1.08443023, 4.80215291, -3.90645169, 5.89463361, -7.29855187, 2.11429899,
#                         3.11523375, -6.71558771])
# print(randomSeedX)
# print(randomSeedY)
initX = randomSeedX + IdealX
initY = randomSeedY + IdealY
testDrone = DroneGroup(15, initX, initY, IdealX, IdealY)
# testDrone.ShowPosition(size=30, color='green')
# testDrone.AdjustSimulate()
# testDrone.ShowPosition()
# testDrone.ShowIdeal()
# plt.show()
# testDrone.showLoss()
# plt.show()
testDrone.ShowDifferentSenderNumber()
# print("loss:", testDrone.Loss[-1])
# print("average:", testDrone.average)
# print(testDrone.realPositionX, testDrone.realPositionY)

# 圆形编队 initX2 = np.array( [100.0, 74.96229729000632, 19.044201170560154, -52.10273288589687, -91.94275092040344,
# -105.27229068983148, -52.38886563516713, 17.30380046043861, 86.14777204829524]) initY2 = np.array( [0.0,
# 63.124115716615876, 110.36901015128855, 91.16087552135994, 33.919471593594345, -38.23277145219296,
# -90.99674036722637, -96.46024305186735, -71.57207116686621]) IdealX2 = np.array( [100.0, 76.60444431189781,
# 17.36481776669304, -49.99999999999998, -93.96926207859083, -93.96926207859084, -50.00000000000004,
# 17.364817766692997, 76.60444431189778]) IdealY2 = np.array([0.0, 64.27876096865393, 98.4807753012208,
# 86.60254037844388, 34.20201433256689, -34.20201433256687, -86.60254037844385, -98.48077530122082,
# -64.27876096865396]) testDrone = DroneGroup(10, initX2, initY2, IdealX2, IdealY2) testDrone.ShowPosition(size=30,
# color='green') testDrone.AdjustSimulate() testDrone.ShowPosition() testDrone.ShowIdeal() plt.show()
# testDrone.showLoss()
