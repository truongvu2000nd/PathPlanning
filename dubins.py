import numpy as np
import math
from settings import *
from utils import move_backward, normalize, move, draw_angled_rec, draw_arrow
import time
import cv2 as cv

class Dubins:
    def __init__(self, start):
        self.start = start
        self.dict = {}
        angular_velocity = velocity / car_height
        self.period = math.ceil(math.pi * 2 / angular_velocity)
        self.max_d = math.ceil(math.sqrt(MAP_WIDTH * MAP_WIDTH + MAP_HEIGHT * MAP_HEIGHT)) // velocity
        self.compute_dubins_shortest_path()
        """Key:
            goal: (x, y, theta)

        Value:
            type: 'LRL', 'RLR', 'LSL', 'LSR', 'RSL', 'RSR'
            params: (alpha, d(beta), gamma) 
            dist: int
        """

    def dubins_LRL(self):
        LRL = np.zeros((self.period, self.period, self.period, 3))
        x, y, theta = self.start
        # alpha, beta, gamma
        LRL[0, 0, 0] = [*self.start]
        for alpha in range(1, self.period):
            LRL[alpha, 0, 0] = move_backward(*LRL[alpha-1, 0, 0], 'left')

        for alpha in range(self.period):
            for beta in range(1, self.period):
                LRL[alpha, beta, 0] = move_backward(*LRL[alpha, beta-1, 0], 'right')

        for alpha in range(self.period):
            for beta in range(self.period):
                for gamma in range(1, self.period):
                    LRL[alpha, beta, gamma] = move_backward(*LRL[alpha, beta, gamma-1], 'left')
        
        return {"path": LRL[:, self.period//2:, :, :], "type": 'RSR'}


    def dubins_RLR(self):
        RLR = np.zeros((self.period, self.period, self.period, 3))
        x, y, theta = self.start
        # alpha, beta, gamma
        RLR[0, 0, 0, :] = [*self.start]
        for alpha in range(1, self.period):
            RLR[alpha, 0, 0] = move_backward(*RLR[alpha-1, 0, 0], 'right')

        for alpha in range(self.period):
            for beta in range(1, self.period):
                RLR[alpha, beta, 0] = move_backward(*RLR[alpha, beta-1, 0], 'left')

        for alpha in range(self.period):
            for beta in range(self.period):
                for gamma in range(1, self.period):
                    RLR[alpha, beta, gamma] = move_backward(*RLR[alpha, beta, gamma-1], 'right')

        return {"path": RLR[:, self.period//2:, :, :], "type": 'RSR'}


    def dubins_LSL(self):
        LSL = np.zeros((self.period, self.max_d, self.period, 3))
        x, y, theta = self.start
        # alpha, beta, gamma
        LSL[0, 0, 0, :] = [*self.start]
        for alpha in range(1, self.period):
            LSL[alpha, 0, 0] = move_backward(*LSL[alpha-1, 0, 0], 'left')

        for alpha in range(self.period):
            for beta in range(1, self.max_d):
                LSL[alpha, beta, 0] = move_backward(*LSL[alpha, beta-1, 0], 'straight')

        for alpha in range(self.period):
            for beta in range(self.max_d):
                for gamma in range(1, self.period):
                    LSL[alpha, beta, gamma] = move_backward(*LSL[alpha, beta, gamma-1], 'left')

        return {"path": LSL, "type": 'LSL'}


    def dubins_RSL(self):
        RSL = np.zeros((self.period, self.max_d, self.period, 3))
        x, y, theta = self.start
        # alpha, beta, gamma
        RSL[0, 0, 0, :] = [*self.start]
        for alpha in range(1, self.period):
            RSL[alpha, 0, 0] = move_backward(*RSL[alpha-1, 0, 0], 'right')

        for alpha in range(self.period):
            for beta in range(1, self.max_d):
                RSL[alpha, beta, 0] = move_backward(*RSL[alpha, beta-1, 0], 'straight')

        for alpha in range(self.period):
            for beta in range(self.max_d):
                for gamma in range(1, self.period):
                    RSL[alpha, beta, gamma] = move_backward(*RSL[alpha, beta, gamma-1], 'left')

        return {"path": RSL, "type": 'RSL'}


    def dubins_RSR(self):
        RSR = np.zeros((self.period, self.max_d, self.period, 3))
        x, y, theta = self.start
        # alpha, beta, gamma
        RSR[0, 0, 0, :] = [*self.start]
        for alpha in range(1, self.period):
            RSR[alpha, 0, 0] = move_backward(*RSR[alpha-1, 0, 0], 'right')

        for alpha in range(self.period):
            for beta in range(1, self.max_d):
                RSR[alpha, beta, 0] = move_backward(*RSR[alpha, beta-1, 0], 'straight')

        for alpha in range(self.period):
            for beta in range(self.max_d):
                for gamma in range(1, self.period):
                    RSR[alpha, beta, gamma] = move_backward(*RSR[alpha, beta, gamma-1], 'right')

        return {"path": RSR, "type": 'RSR'}


    def dubins_LSR(self):
        LSR = np.zeros((self.period, self.max_d, self.period, 3))
        x, y, theta = self.start
        # alpha, beta, gamma
        LSR[0, 0, 0, :] = [*self.start]
        for alpha in range(1, self.period):
            LSR[alpha, 0, 0] = move_backward(*LSR[alpha-1, 0, 0], 'left')

        for alpha in range(self.period):
            for beta in range(1, self.max_d):
                LSR[alpha, beta, 0] = move_backward(*LSR[alpha, beta-1, 0], 'straight')

        for alpha in range(self.period):
            for beta in range(self.max_d):
                for gamma in range(1, self.period):
                    LSR[alpha, beta, gamma] = move_backward(*LSR[alpha, beta, gamma-1], 'right')

        return {"path": LSR, "type": 'LSR'}


    def update_dict(self, path):
        curve = path["path"]
        t = path["type"]
        for alpha in range(curve.shape[0]):
            for d in range(curve.shape[1]):
                for gamma in range(curve.shape[2]):
                    # states = normalize(curve[alpha, d, gamma])
                    # for state in states:
                    state = normalize(curve[alpha, d, gamma])
                    if state in self.dict and self.dict[state] < alpha + d + gamma:
                        continue
                    self.dict[state] = alpha + d + gamma

    def getShortestPath(self, start):
        return self.dict.get(normalize(start), 10000)

    def compute_dubins_shortest_path(self):
        lrl = self.dubins_LRL()
        rlr = self.dubins_RLR()
        lsl = self.dubins_LSL()
        rsr = self.dubins_RSR()
        rsl = self.dubins_RSL()
        lsr = self.dubins_LSR()

        self.update_dict(lrl)
        self.update_dict(rlr)
        self.update_dict(lsl)
        self.update_dict(rsr)
        self.update_dict(rsl)
        self.update_dict(lsr)


if __name__ == '__main__':
    # goal = (360, 640, math.pi/2)
    # normalized_goal = normalize(*goal)
    # print(normalized_goal)
    # dubins_shortest_path = self.dict.get(normalized_goal, 0)

    dubins_dict = Dubins((500, 500, math.pi))
    # print(dubins_shortest_path)
    # count = 0
    # for i in range(1, 64):
    #     for j in range(1, 48):
    #         for angle in range(8):
    #             if (i, j, angle) not in dubins_dict:
    #                 count += 1
    # print(count)
    # 
    start = (392, 600, math.pi/2)
    print(dubins_dict.getShortestPath(start))
    x, y, theta = move(*start, 'left')
    print(dubins_dict.getShortestPath((x, y, theta)))
    x, y, theta = move(x, y, theta, 'left')
    print(dubins_dict.getShortestPath((x, y, theta)))
    x, y, theta = move(x, y, theta, 'left')
    print(dubins_dict.getShortestPath((x, y, theta)))
    # start = (392, 600, math.pi/2)
    # goal = (500, 500, math.pi)
    # x, y, theta = start
    # img = np.ones((768, 1024, 3)) * 255
    # for i in range(12):
    #     x, y, theta = move(x, y, theta, 'left')
    #     #draw_angled_rec(img, x, y, theta)
    #     cv.circle(img, (int(x), int(y)),1, BLACK, thickness=-1)
    #     draw_arrow(img, *goal)
    #     cv.imshow(window, img)
    #     cv.waitKey(50)
    # for i in range(3):
    #     x, y, theta = move(x, y, theta, 'straight', velocity=8)
    #     #draw_angled_rec(img, x, y, theta)
    #     cv.circle(img, (int(x), int(y)),1, RED, thickness=-1)
    #     draw_arrow(img, *goal)
    #     cv.imshow(window, img)
    #     cv.waitKey(50)
    # for j in range(11):
    #     x, y, theta = move(x, y, theta, 'left')
    #     #draw_angled_rec(img, x, y, theta)
    #     cv.circle(img, (int(x), int(y)),1, RED, thickness=-1)
    #     draw_arrow(img, *goal)
    #     cv.imshow(window, img)
    #     cv.waitKey(50)
    

    # cv.waitKey(0)
    # cv.destroyAllWindows