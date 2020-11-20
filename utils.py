import numpy as np
import math
import cv2 as cv
from settings import *


def readMapFromFile(file):
    real_map = np.ones((MAP_HEIGHT, MAP_WIDTH, 3), dtype=np.uint8) * 255
    grid = np.zeros((GRIDHEIGHT, GRIDWIDTH), dtype=np.uint8)
    walls = []
    start_symbols = ['+', '-', '*', '/']
    with open(file) as f:
        content = f.read().splitlines()
        for row, line in enumerate(content):
            for col, cell in enumerate(line):
                if cell == '1':
                    real_map[row*16 : (row+1)*16, col*16 : (col+1)*16, :] = 0
                    grid[row, col] = 1

                if cell in start_symbols:
                    start_x, start_y = col*16, row*16
                    if cell == '+':
                        theta = 0
                    if cell == '-':
                        theta = math.pi/2
                    if cell == '*':
                        theta = -math.pi/2
                    if cell == '/':
                        theta = math.pi
                if cell == '#':
                    goal = (col*16, row*16, math.pi/2)

    start = (start_x, start_y, theta)
    return real_map, grid, start, goal


def check_collision(points, grid):
    res = 0
    for point in points:
        x, y = point
        if x < 0 or y < 0 or x >= MAP_WIDTH or y >= MAP_HEIGHT:
            res += 1
            continue
        x, y = int(x/16), int(y/16)
        if grid[y, x] == 1:
            res += 1
    
    return res


def euclid_distance(pos1, pos2):
    x1, y1 = pos1
    x2, y2 = pos2
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
 

def draw_car(img, pts, color):
    pt0, pt1, pt2, pt3 = pts
    cv.line(img, pt0, pt1, color)
    cv.line(img, pt1, pt2, color)
    cv.line(img, pt2, pt3, BLUE)
    cv.line(img, pt3, pt0, color)


def move_backward(x, y, theta, dir, velocity=velocity):
    assert dir == 'left' or dir == 'right' or dir == 'straight'
    if dir == 'left':
        r = -1
    if dir == 'right':
        r = 1
    if dir == 'straight':
        r = 0
    new_theta = theta - velocity * r / car_height
    new_x = x - velocity * math.cos(new_theta)
    new_y = y - velocity * math.sin(new_theta)
    return new_x, new_y, new_theta  


def move(x, y, theta, dir, velocity=velocity):
    assert dir == 'left' or dir == 'right' or dir == 'straight'
    if dir == 'left':
        r = -1
    if dir == 'right':
        r = 1
    if dir == 'straight':
        r = 0
    new_theta = theta + velocity * r / car_height
    new_x = x + velocity * math.cos(theta)
    new_y = y + velocity * math.sin(theta)
    return new_x, new_y, new_theta  


# def normalize2(args):
#     x, y, theta = args
#     k = math.floor(theta / (2 * math.pi))
#     theta = theta - k * 2 * math.pi
#     k = round(theta / math.pi * 4) 
#     k = k % 8
#     return round(x/16), round(y/16), k


def normalize(args):
    x, y, theta = args
    k = math.floor(theta / (2 * math.pi))
    theta = theta - k * 2 * math.pi
    k = round(theta / math.pi * 4) 
    k = k % 8
    return round(x/16), round(y/16), k


def normalizeTheta(theta):
    k = math.floor(theta / (2 * math.pi))
    theta = theta - k * 2 * math.pi
    k = round(theta / math.pi * 4) 
    k = k % 8
    return k


# def normalize(args):
#     x, y, theta = args
#     k = math.floor(theta / (2 * math.pi))
#     theta = theta - k * 2 * math.pi
#     k1 = math.floor(theta / math.pi * 4) % 8
#     k2 = math.ceil(theta / math.pi * 4) % 8
#     x1 = math.floor(x / 16)
#     x2 = math.ceil(x / 16)
#     y1 = math.floor(y / 16)
#     y2 = math.ceil(y / 16)
#     return [(x1, y1, k1), 
#             (x1, y2, k1),
#             (x1, y1, k2),
#             (x1, y2, k2),
#             (x2, y2, k1),
#             (x2, y2, k2),
#             (x2, y1, k1),
#             (x2, y1, k2)]


def centerToXY(centerX, centerY, theta):
    b = math.cos(theta)
    a = math.sin(theta)
    x = int(centerX - car_height*b*0.5)
    y = int(centerY - car_height*a*0.5)
    return x, y, theta


def draw_angled_rec(img, x, y, angle):
    color = BLUE
    b = math.cos(angle)
    a = math.sin(angle)

    pt0 = (int(x - a*car_width*0.5), int(y + b*car_width*0.5))
    pt1 = (int(x + a*car_width*0.5), int(y - b*car_width*0.5))
    pt3 = (int(pt0[0] + car_height*b), int(pt0[1] + car_height*a))
    pt2 = (int(pt1[0] + car_height*b), int(pt1[1] + car_height*a))

    cv.line(img, pt0, pt1, color)
    cv.line(img, pt1, pt2, color)
    cv.line(img, pt2, pt3, RED, thickness=2)
    cv.line(img, pt3, pt0, color)


def draw_arrow(img, x, y, theta):
    color = BLUE
    point = (x, y)
    b = math.cos(theta)
    a = math.sin(theta)
    point1 = (int(x + 50*b), int(y + 50*a))
    x1, y1 = point1
    pt2 = (int(x1 + 10*a), int(y1 - 10*b))
    pt3 = (int(x1 - 10*a), int(y1 + 10*b))
    pt4 = (int(x1 + 10*b), int(y1 + 10*a))
    cv.line(img, point, point1, color)
    cv.line(img, pt2, pt3, color)
    cv.line(img, pt2, pt4, color)
    cv.line(img, pt3, pt4, color)


# test
if __name__ == '__main__':
    print(centerToXY(500, 500, math.pi))