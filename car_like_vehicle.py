import math
import numpy as np
import cv2 as cv
from utils import euclid_distance, check_collision
from utils import draw_car, draw_angled_rec, draw_arrow
from utils import readMapFromFile
from heapq import heappush, heappop
from settings import *
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('-m', '--map', type=str,
                    default="map1.txt", help="Map filename")
parser.add_argument('-i', '--interval', type=int, default=0,
                    help='Interval between 2 frames')
# parser.add_argument('-v', '--velocity', type=int, default=40,
#                     help="Velocity of the car")
# parser.add_argument('-s', '--size', type=str,
#                     default="medium", help="Size of the car")
args = parser.parse_args()
# print(args)

# size = args.size
# assert size == 'small' or size == 'medium' or size == 'big'
# if size == 'small':
#     car_height = 60
#     car_width = 30
# if size == 'medium':
#     car_height = 75
#     car_width = 40
# if size == 'big':
#     car_height = 100
#     car_width = 50
# velocity = args.velocity
# window = 'Car-like Robot'


class Car:
    """This class represents a car (Car-like Robot)
    """

    def __init__(self, x, y, theta):
        """Constructor

        Args:
            x, y (int): coordinates of the car (midpoint of bottom line)
            theta (radius): the angle between the car and the horizontal axis.
        """
        super().__init__()
        self.x = x
        self.y = y
        self.theta = theta
        self.car_width = car_width
        self.car_height = car_height

        self.pt0, self.pt1, self.pt2, self.pt3, self.center = \
            self._compute_corner_and_center(x, y, theta)

    def _compute_corner_and_center(self, x, y, theta):
        """Compute coordinates of 4 corners and center point

        Returns:
            A tuple of 4 corners and center point
        """
        b = math.cos(theta)
        a = math.sin(theta)

        pt0 = (int(x - car_width*a*0.5), int(y + car_width*b*0.5))
        pt1 = (int(x + car_width*a*0.5), int(y - car_width*b*0.5))
        pt3 = (int(pt0[0] + car_height*b), int(pt0[1] + car_height*a))
        pt2 = (int(pt1[0] + car_height*b), int(pt1[1] + car_height*a))
        center = (int(x + car_height*b*0.5), int(y + car_height*a*0.5))

        return pt0, pt1, pt2, pt3, center

    def move(self, u, v=velocity):
        """Compute the new location when the car moves.

        Returns:
            A dictionary that contains information about new position.
        """
        new_theta = self.theta + v * u / self.car_height
        new_x = self.x + v * math.cos(self.theta)
        new_y = self.y + v * math.sin(self.theta)

        x, y, theta = new_x, new_y, new_theta
        pt0, pt1, pt2, pt3, center = \
            self._compute_corner_and_center(x, y, theta)

        new_pos = {"x": x, "y": y, "theta": theta,
                   "pt0": pt0, "pt1": pt1, "pt2": pt2, "pt3": pt3, "center": center}
        return new_pos

    def getPoints(self):
        return self.pt0, self.pt1, self.pt2, self.pt3

    def getInfo(self):
        return self.x, self.y, self.theta

    def __str__(self):
        return "({}, {}, {})".format(self.x, self.y, self.theta)


class State:
    """This class represents a state of the grid map.
    """

    def __init__(self, car, parent=None):
        """Constructor of State class.

        Args:
            position (tuple(int, int)): Decartes position. (0, 0) is top-left.
            parent (State, optional): The parent node of this. Defaults to None.
        """
        super().__init__()
        self.car = car
        self.parent = parent
        self.g = 0
        self.h = 0
        self.f = 0
        self.center = car.center

    def compute_dist(self, goal):
        """Compute distance(cost) from the State.
        g: distance from this to the start,
        h: estimated distance from this to the goal (heuristic function),
        f: combined distance.

        Args:
            goal (State): The goal State.
        """

        if self.parent:
            self.g = self.parent.g + velocity

        self.h = euclid_distance(self.center, goal)

        self.f = self.h + self.g

    def __lt__(self, other):
        """Override less than (<) operation.
        Heuristic evaluation.
        """
        return self.f < other.f

    def __eq__(self, other):
        return hash(self) == hash(other)

    def __hash__(self):
        """Hash value of the object.

        Returns:
            Hash value of the center of the car.
        """
        
        return hash((round(self.center[0]/16), round(self.center[1]/16)))

    def isGoal(self, goal):
        """Check whether a state is the goal.
        """
        return euclid_distance(self.center, goal) < 50

    def __str__(self):
        return "({}, {}, {})".format(self.car.x, self.car.y, self.car.theta)


class Map:
    """A class represents the grid map.
    """

    def __init__(self, image, grid):
        """Class constructor

        Args:
            grid (numpy.ndarray): A matrix represents the map
                                  0 - wall
                                  1 - path
        """
        super().__init__()
        self._map = image  # numpy array
        self.grid = grid

    def get_neighbors(self, state):
        """Get neighbors(paths) of a State.

        Returns:
            valid_neighbors: list type, contains all valid States
        """
        car = state.car
        u = [0, -1, 1]  # all possible angles
        valid_neighbors = []  

        for r in u:
            new_pos = car.move(r)
            pt0, pt1, pt2, pt3 = new_pos["pt0"], new_pos["pt1"], new_pos["pt2"], new_pos["pt3"]

            # collision detection
            # head of the car
            head = list(zip(
                np.linspace(pt2[0], pt3[0], num=car_width//TILESIZE+1),
                np.linspace(pt2[1], pt3[1], num=car_width//TILESIZE+1)
            ))

            # 2 sides of the car
            side1 = list(zip(
                np.linspace(pt0[0], pt3[0], num=car_height//TILESIZE+1),
                np.linspace(pt0[1], pt3[1], num=car_height//TILESIZE+1)
            ))
            side2 = list(zip(
                np.linspace(pt1[0], pt2[0], num=car_height//TILESIZE+1),
                np.linspace(pt1[1], pt2[1], num=car_height//TILESIZE+1)
            ))

            # check collision
            if not (check_collision(side1, self.grid) + check_collision(side2, self.grid) +
                    check_collision(head, grid)):
                new_car = Car(new_pos["x"], new_pos["y"], new_pos["theta"])

                # append valid "neighbors"
                valid_neighbors.append(State(new_car, parent=state))

        return valid_neighbors

    def draw(self, state, start, goal):
        """Visualization.
        """
        imgs = []
        img = np.array(self._map)
        color = GREEN
        while state is not None and state != start: 
            cv.circle(img, state.car.center, 2, BLACK, thickness=-1)
            img_tmp = np.array(img)
            cv.circle(img_tmp, goal, 5, RED, thickness=-1)
            draw_car(img_tmp, state.car.getPoints(), color)
            imgs.append(img_tmp)
            state = state.parent

        cv.imshow(window, self._map)
        cv.waitKey(0)
        imgs = imgs[::-1]
        for img in imgs:
            cv.imshow(window, img)
            cv.waitKey(args.interval)
        cv.waitKey(0)
        cv.destroyAllWindows()

        return imgs


class PathFinding:
    """This class demonstrates the A* algorithm path-finding.
    Find the shortest way from start State to the goal State in the Map.
    """

    def __init__(self, map, start, goal):
        """Class constructor.
        self.closed: set type, contains visited States,
        self.open: dictionary, (key, value): (state, state.f),
        self.min_heap: list type, a priority queue contains States.

        Args:
            map (Map): The Map object
            start (State): The start State
            goal (State): The goal State
        """
        super().__init__()
        self.start = start
        self.goal = goal
        self.map = map

        self.closed = set()  # contains visited States.
        self.open = {}
        self.min_heap = []

        self.min_heap.append(start)  # append the start State for the begining

    def solve(self):
        """A* algorithm

        Returns:
            path (list of tuples): if exists else False
        """

        n_nodes = 0
        while self.min_heap:
            # get the min element of the pri-queue
            currState = heappop(self.min_heap)

            n_nodes += 1
            # mark visited
            self.closed.add(currState)
            if n_nodes % 500 == 0:
                print(currState.center)

            # if reach the goal
            if currState.isGoal(self.goal):
                print(f'Number of processed nodes: {n_nodes}')
                return self.map.draw(currState, self.start, self.goal)

            # else get the neighbors and add to the priority queue
            neighbors = self.map.get_neighbors(currState)
            for neighbor in neighbors:

                if neighbor in self.closed:
                    continue

                # compute distances and add to the priority queue
                neighbor.compute_dist(self.goal)

                if neighbor in self.open:
                    if neighbor.g > self.open.get(neighbor):
                        continue

                heappush(self.min_heap, neighbor)  
                self.open[neighbor] = neighbor.g

        # if prioritity queue is empty
        # there is no solutions
        print("No solutions")
        print(currState.car.center)
        return False


if __name__ == '__main__':
    img, grid, start, goal3d = readMapFromFile(args.map)
    goal = goal3d[0], goal3d[1]
    car = Car(*start)
    start = State(car)
    map = Map(img, grid)

    astar = PathFinding(map, start, goal)
    imgs = astar.solve()    
    

