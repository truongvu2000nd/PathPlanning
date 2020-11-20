# define some colors (R, G, B)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
DARKGREY = (40, 40, 40)
LIGHTGREY = (100, 100, 100)
GREEN = (0, 255, 0)
BLUE = (255, 0, 0)
RED = (0, 0, 255)
YELLOW = (255, 255, 0)

# map settings
MAP_WIDTH = 1024   # 16 * 64 or 32 * 32 or 64 * 16
MAP_HEIGHT = 768  # 16 * 48 or 32 * 24 or 64 * 12
WINDOW = "A* path finding with dubins shortest path"
BGCOLOR = DARKGREY

TILESIZE = 16
GRIDWIDTH = MAP_WIDTH // TILESIZE
GRIDHEIGHT = MAP_HEIGHT // TILESIZE

window = 'Car-like Robot'
velocity = 30
car_height = 80
car_width = 40