# Setting up some constants for the game window
WIDTH = 600
HEIGHT = 600
OFFSET = 50
DIRECTION = {'top': 0, 'bottom': 1, 'left': 2, 'right': 3}
THICC = 4

# Setting up color constants
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
BLUE = (0, 188, 212)
RED = (221, 44, 0)
LIGHT_BLUE = (178, 235, 242)
LIGHT_RED = (240, 128, 128)
COLOURS = {-1: BLUE, 1: RED, -2: LIGHT_BLUE, 2: LIGHT_RED}

# Setting up initial scores
SCORES = {-1: 0, 1: 0}
TEMP_SCORES = {-1: 0, 1: 0}   #This had to be defined for the AI to use scoring

# Board dimensions
m = 4 #Number of Horizontal dots
n = 4 #Number of Vertical dots