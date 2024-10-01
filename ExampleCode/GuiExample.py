### This code probably does not work 100% but should if you debugged it
import pygame
import time
import numpy as np

# Define some colors
BLACK = (0, 0, 0)  # BLACK
UNOCCUPIED = (255, 255, 255)  # WHITE
GOAL = (0, 255, 0)  # GREEN
START = (255, 0, 0)  # RED
OBSTACLE = (0, 0, 0)  # GRAY2
UNKNOWN = (220, 220, 200)  # GRAY

colors = {
    0: UNOCCUPIED,
    1: GOAL,
    100: OBSTACLE,
    -1: UNKNOWN
}

class Animation:
    def __init__(self,
                 world,
                 title="Occupancy Grid Plotting",
                 width=10,
                 height=10,
                 margin=0,
                 x_dim=100,
                 y_dim=50,
                 start=(0, 0),
                 goal=(50, 50),
                 ):
        """
        Initializes the animation environment.
        :param world: OccupancyGridMap - the world map
        :param title: str - title of the window
        :param width: int - width of each grid cell
        :param height: int - height of each grid cell
        :param margin: int - margin between grid cells
        :param x_dim: int - number of cells in x-direction
        :param y_dim: int - number of cells in y-direction
        :param start: (int, int) - starting position
        :param goal: (int, int) - goal position
        """
        self.width = width
        self.height = height
        self.margin = margin
        self.x_dim = x_dim
        self.y_dim = y_dim
        self.start = start
        self.current = start
        self.goal = goal

        pygame.init()

        # Set the 'width' and 'height' of the screen
        window_size = [(width + margin) * y_dim + margin,
                       (height + margin) * x_dim + margin]

        self.screen = pygame.display.set_mode(window_size)

        # Create occupancy grid map
        """
        Set initial values for the map occupancy grid
        |----------> y, column
        |           (x=0,y=2)
        |
        V (x=2, y=0)
        x, row
        """
        self.world = world  # OccupancyGridMap(x_dim=x_dim, y_dim=y_dim, exploration_setting='4N')

        # Set title of screen
        pygame.display.set_caption(title)

        # Set font
        pygame.font.SysFont('Comic Sans MS', 36)

        self.done = False

        self.clock = pygame.time.Clock()

    def set_position(self, pos):
        """
        Sets the current position of the robot.
        :param pos: (int, int) - the new position
        """
        self.current = pos

    def set_goal(self, goal):
        """
        Sets the goal position.
        :param goal: (int, int) - the new goal position
        """
        self.goal = goal

    def set_start(self, start):
        """
        Sets the starting position.
        :param start: (int, int) - the new starting position
        """
        self.start = start

    def set_world(self, world):
        self.world.set_map(world)


    def run_game(self):

        for event in pygame.event.get():
            if event.type == pygame.QUIT:  # if user clicked close
                print("quit")
                self.done = True  # flag that we are done so we can exit loop

        # Set the screen background
        self.screen.fill(BLACK)


        for row in range(self.x_dim):
            for column in range(self.y_dim):
                # Color the cells
                pygame.draw.rect(self.screen, colors[self.world.occupancy_grid_map[row][column]],
                                 [(self.margin + self.width) * column + self.margin,
                                  (self.margin + self.height) * row + self.margin,
                                  self.width,
                                  self.height])

        # Draw a moving robot, based on current coordinates
        robot_center = [int(round(self.current[1] * (self.width + self.margin) + self.width / 2) + self.margin),
                        int(round(self.current[0] * (self.height + self.margin) + self.height / 2) + self.margin)]

        # Draw robot position as red circle
        pygame.draw.circle(self.screen, START, robot_center, abs(int(round(self.width / 2) - 2)))

        # Set game tick
        self.clock.tick(20)

        # Go ahead and update screen with what we've drawn
        pygame.display.flip()

    # Be 'idle' friendly. If you forget this, the program will hang on exit
    pygame.quit()

def main():
    grid = np.zeros((2048, 2048))

    gui = Animation(grid,
                    width=7,
                    height=7,
                    margin=0,
                    x_dim=2048,
                    y_dim=2048,
                    start=(0,0),
                    goal=(100,100))

    while True:
        gui.run_game()

if __name__ == "__main__":
    main()