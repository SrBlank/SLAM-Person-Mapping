import pygame
import sys
import numpy as np

# Initialize pygame
pygame.init()

# Set up display
width, height = 600, 600  # Adjust if necessary based on grid size and resolution
window = pygame.display.set_mode((width, height))
pygame.display.set_caption("Occupancy Grid with Drone Position")

# Colors
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
GRAY = (100, 100, 100)  # Unexplored
GREEN = (0, 255, 0)     # Free space
RED = (255, 0, 0)       # Obstacle
BLUE = (0, 0, 255)      # Person

# Grid settings
grid_size = 10  # Size of each grid cell in pixels, adjust based on display
grid_dim = 50   # This is for the 50x50 subgrid; change to 2048 if working with full grid

# Load the occupancy grid from the data (example using random data here)
# Replace this with actual loading logic for whole_grid_complete or sub_grid_1_complete
occupancy_grid = np.random.choice([-1, 0, 100, 50], (grid_dim, grid_dim), p=[0.2, 0.6, 0.1, 0.1])


# Drone's initial position (grid coordinates)
drone_pos = (25, 25)  # Start at the center of the grid

# Function to draw the occupancy grid
def draw_occupancy_grid():
    for row in range(grid_dim):
        for col in range(grid_dim):
            cell_value = occupancy_grid[row][col]
            color = GRAY if cell_value == -1 else GREEN if cell_value == 0 else RED if cell_value == 100 else BLUE
            pygame.draw.rect(window, color, (col * grid_size, row * grid_size, grid_size, grid_size))

# Function to draw the drone
def draw_drone(pos):
    x = pos[0] * grid_size + grid_size // 2
    y = pos[1] * grid_size + grid_size // 2
    pygame.draw.circle(window, WHITE, (x, y), grid_size // 3)

# Main loop
running = True
while running:
    # Handle events
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()

    # Move drone based on key presses (for testing movement)
    keys = pygame.key.get_pressed()
    if keys[pygame.K_LEFT] and drone_pos[0] > 0:
        drone_pos = (drone_pos[0] - 1, drone_pos[1])
    if keys[pygame.K_RIGHT] and drone_pos[0] < grid_dim - 1:
        drone_pos = (drone_pos[0] + 1, drone_pos[1])
    if keys[pygame.K_UP] and drone_pos[1] > 0:
        drone_pos = (drone_pos[1], drone_pos[0] - 1)
    if keys[pygame.K_DOWN] and drone_pos[1] < grid_dim - 1:
        drone_pos = ( drone_pos[1], drone_pos[0] + 1)

    # Clear the screen
    window.fill(BLACK)

    # Draw the occupancy grid and drone
    draw_occupancy_grid()
    draw_drone(drone_pos)

    # Update the display
    pygame.display.flip()

    # Control the frame rate
    pygame.time.Clock().tick(10)
    