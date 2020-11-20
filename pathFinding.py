import matplotlib.pyplot as plt
from matplotlib import colors, animation
import numpy as np
import argparse

from heapq import heappush, heappop
from utils import readMapFromText


parser = argparse.ArgumentParser()
parser.add_argument('-id', '--image_dir', type=str,
                    default="./map/", help="Location of map folder")
parser.add_argument('-i', '--image', type=str,
                    default="map2.txt", help="Map filename")
parser.add_argument('-v', '--speed', type=int, default=5,
                    help="Speed of the animation")
parser.add_argument('-d', '--save_dir', type=str, default='./solution/',
                    help='Location for saving image solutions')
parser.add_argument('-s', '--save_name', type=str,
                    default=None, help="Filename of saved image solution")
args = parser.parse_args()
print(args)


fig, axes = plt.subplots(figsize=(10, 6))
cmap = colors.ListedColormap(['black', 'white', 'red', 'gray', 'yellow'])


class State:
    """This class represents a state of the grid map.
    """

    def __init__(self, position, parent=None):
        """Constructor of State class.

        Args:
            position (tuple(int, int)): Decartes position. (0, 0) is top-left.
            parent (State, optional): The parent node of this. Defaults to None.
        """
        super().__init__()
        self.position = position
        self.parent = parent
        self.g = 0
        self.h = 0
        self.f = 0

    def compute_dist(self, goal):
        """Compute distance(cost) from the State.
        g: distance from this to the start,
        h: estimated distance from this to the goal (heuristic function),
        f: combined distance.

        Args:
            goal (State): The goal State.
        """
        if self.parent:
            self.g = self.parent.g + 1

        # h: Manhattan distance
        x, y = self.position
        self.h = goal.position[0] - x + goal.position[1] - y

        self.f = self.h + self.g

    def __lt__(self, other):
        """Override less than (<) operation.
        Heuristic evaluation.
        """
        return self.f < other.f

    def __eq__(self, other):
        """Overrid equal(==) operation."""
        return self.position == other.position


class Map:
    """A class represents the grid map.
    """

    def __init__(self, grid):
        """Class constructor

        Args:
            grid (numpy.ndarray): A matrix represents the map
                                  0 - wall
                                  1 - path
        """
        super().__init__()
        self.map = grid  # numpy array
        self.height, self.width = grid.shape  # grid size
        self.imgs = []  # list type, contains animated images

    def get_neighbors(self, curr_state):
        """Get neighbors(paths) of a State.

        Returns:
            valid_neighbors: list type, contains all neighbors of curr_state 
        """
        x, y = curr_state.position
        neighbors = [(x+1, y), (x-1, y), (x, y+1),
                     (x, y-1)]  # all possible neighbors
        valid_neighbors = []  # neighbors that are not "walls"

        for neighbor in neighbors:
            x, y = neighbor
            # out of map
            if x < 0 or x >= self.height or y < 0 or y >= self.width:
                continue

            if self.map[x][y] != 0:
                valid_neighbors.append(State(neighbor, curr_state))
        return valid_neighbors

    def getSize(self):
        return self.height, self.width

    def getMap(self):
        return self.map

    def setCell(self, position, type):
        """Set value of a cell given position. Used for visualization purposes.
        path: 2
        in-queue: 3
        start: 4
        goal: 5

        Args:
            position ((int, int))
            type (string)
        """
        if type == "in-queue":
            grid[position] = 3

        elif type == "path":
            grid[position] = 2

        elif type == "start":
            grid[position] = 4

        elif type == "goal":
            grid[position] = 5

    def visualize(self):
        """Visualize the map.
        """
        img = plt.imshow(self.map, cmap=cmap, animated=True)
        self.imgs.append([img])

    def animation(self):
        """Create the animation.
        """
        ani = animation.ArtistAnimation(
            fig, self.imgs, interval=5/args.speed, blit=True, repeat=False)
        plt.show()


class PathFinding:
    """This class demonstrates the A* algorithm path-finding.
    Find the shortest way from start State to the goal State in the Map.
    """

    def __init__(self, map, start, goal):
        """Class constructor.
        self.closed: list type, contains visited States,
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

        self.closed = []  # contains visited States.
        self.min_heap = []
        self.min_heap.append(start)  # append the start State for the begining

    def solve(self):
        """A* algorithm

        Returns:
            path (list of tuples): if exists else False
        """
        while self.min_heap:

            # get the min element of the pri-queue
            currState = heappop(self.min_heap)

            # mark visited
            self.closed.append(currState)

            # if reach the goal
            if currState == self.goal:
                path = []
                while currState != self.start:
                    path.append(currState.position)
                    self.map.setCell(currState.position, "path")
                    currState = currState.parent
                path = path[::-1]

                # show the animation
                self.map.visualize()
                if args.save_name:
                    plt.imsave(args.save_dir + args.save_name,
                               self.map.getMap(), cmap=cmap, dpi=100)
                self.map.animation()

                return path

            # else get the neighbors and add to the priority queue
            neighbors = self.map.get_neighbors(currState)
            for neighbor in neighbors:

                # if visited continue
                if neighbor in self.closed:
                    continue

                # compute distances and add to the priority queue
                neighbor.compute_dist(self.goal)
                heappush(self.min_heap, neighbor)
                self.map.setCell(neighbor.position, "in-queue")
            self.map.visualize()

        # if prioritity queue is empty
        # there is no solutions
        self.map.animation()
        return False


# ----------Main----------
if __name__ == "__main__":

    # read Map, start, goal from file
    grid, startPos, goalPos = readMapFromText(args.image_dir + args.image)
    map = Map(grid)
    start = State(startPos)
    goal = State(goalPos)

    Astar = PathFinding(map, start, goal)
    path = Astar.solve()
    if not path:
        print("No solutions")
