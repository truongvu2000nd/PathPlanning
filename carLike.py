import matplotlib.pyplot as plt 
import cv2 as cv
from matplotlib import colors, animation
import numpy as np
import math
from settings import *

img_size = 800, 800, 3
color = BLUE
window = 'test'
velocity = 40
height = 80
width = 40

def compute_corner_and_center(x, y, theta):
        """Compute coordinates of 4 corners and center point

        Returns:
            A tuple of 4 corners and center point
        """
        b = math.cos(theta)
        a = math.sin(theta)

        pt0 = (int(x - width*a*0.5), int(y + width*b*0.5))
        pt1 = (int(x + width*a*0.5), int(y - width*b*0.5))
        pt3 = (int(pt0[0] + height*b), int(pt0[1] + height*a))
        pt2 = (int(pt1[0] + height*b), int(pt1[1] + height*a))
        center = (int(x + height*b*0.5), int(y + height*a*0.5))

        return pt0, pt1, pt2, pt3, center


def draw_car(img, pts, color):
    pt0, pt1, pt2, pt3 = pts
    cv.line(img, pt0, pt1, color)
    cv.line(img, pt1, pt2, color)
    cv.line(img, pt2, pt3, RED, thickness=2)
    cv.line(img, pt3, pt0, color)


def draw_angled_rec(img, x, y, width, height, angle):
    b = math.cos(angle)
    a = math.sin(angle)

    pt0 = (int(x - a*width*0.5), int(y + b*width*0.5))
    pt1 = (int(x + a*width*0.5), int(y - b*width*0.5))
    pt3 = (int(pt0[0] + height*b), int(pt0[1] + height*a))
    pt2 = (int(pt1[0] + height*b), int(pt1[1] + height*a))

    cv.line(img, pt0, pt1, color)
    cv.line(img, pt1, pt2, color)
    cv.line(img, pt2, pt3, RED, thickness=2)
    cv.line(img, pt3, pt0, color)


def move(x, y, theta, phi):
    new_theta = theta + velocity * math.tan(phi) / height
    new_x = x + velocity * math.cos(theta)
    new_y = y + velocity * math.sin(theta)
    return new_x, new_y, new_theta  


def draw_arrow(img, x, y, theta):
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


if __name__ == '__main__':
    x, y, theta = 500, 500, 0


    for phi in [math.pi/4] * 12:
        x, y, theta = move(x, y, theta, phi)
        img = np.ones(img_size) * 255
        draw_angled_rec(img, x, y, width, height, theta)
        cv.imshow(window, img)
        cv.waitKey(0)

    cv.waitKey(0)
    cv.destroyAllWindows()