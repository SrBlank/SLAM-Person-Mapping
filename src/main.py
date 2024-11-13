import pygame
import sys
import numpy as np
import requests

from slam_handler import Grid, PoseEstimate

# Colors
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
GRAY = (100, 100, 100)  # Unexplored
GREEN = (0, 255, 0)     # Free space
RED = (255, 0, 0)       # Obstacle
BLUE = (0, 0, 255)      # Person

# Grid Settings
GRID_SIZE = 10 # Size of each grid cell in pixels, adjust based on display

# User is able to set or use SLAM position
# Drone's initial position (grid coordinates)
# drone_pos = (25, 25)  # Start at the center of the grid

"""
GRID 
"""
def get_grid_data(hector, wheresHector):
	hector.getOccupancyGrid()
	wheresHector.getRawPosition(hector.resolution, hector.width, hector.height)

def get_api_data(api_url="http://localhost:5000/get_people"):
    try:
        response = requests.get(api_url)
        if response.status_code == 200:
            data = response.json()
            print("Data retrieved successfully.")
            return data
        else:
            print(f"Failed to fetch data. Status code: {response.status_code}")
            return None
    except requests.RequestException as e:
        print(f"Error fetching data: {e}")
        return None

def plot_people_on_grid(hector, wheresHector, people):
    occupancy_grid = hector.grid.copy()

    # Plot robot position at grid value 25
    #if 0 <= robot_x < grid_width and 0 <= robot_y < grid_height:
    #    occupancy_grid[robot_y, robot_x] = 25

    # Plot each detected person in the grid with value 50
    for person in people:
        if person['confidence'] >= 0.5:  # Filter out low-confidence detections
            distance = person['depth']
            distance_conv = int(distance / hector.resolution)
            
            # Calculate person position based on distance and robot orientation
            offset_x = int(distance_conv * np.cos(np.radians(wheresHector.yaw)))
            offset_y = int(distance_conv * np.sin(np.radians(wheresHector.yaw)))

            person_x = wheresHector.conv_x + offset_x
            person_y = wheresHector.conv_y + offset_y

            # Ensure coordinates are within grid bounds
            if 0 <= person_x < hector.width and 0 <= person_y < hector.height:
                occupancy_grid[person_y, person_x] = 50
    
    return occupancy_grid

"""
GUI
"""
# Function to draw the occupancy grid
def draw_occupancy_grid(occupancy_grid):
    for row in range(occupancy_grid.shape[0]):
        for col in range(occupancy_grid.shape[1]):
            cell_value = occupancy_grid[row][col]
            color = GRAY if cell_value == -1 else GREEN if cell_value == 0 else RED if cell_value == 100 else BLUE
            pygame.draw.rect(window, color, (col * GRID_SIZE, row * GRID_SIZE, GRID_SIZE, GRID_SIZE))

# Function to draw the drone
def draw_drone(pos):
    x = pos[0] * GRID_SIZE + GRID_SIZE // 2
    y = pos[1] * GRID_SIZE + GRID_SIZE // 2
    pygame.draw.circle(window, WHITE, (x, y), GRID_SIZE // 3)

"""
DRIVER
"""
def main():
    hector = Grid()
    wheresHector = PoseEstimate()
    get_grid_data(hector, wheresHector) # intial gather data (blocking function) to know we are ready

    # Main loop
    running = True
    while running:
        # Handle events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

        # Clear the screen
        window.fill(BLACK)

        # Data gathering and processing
        get_grid_data(hector, wheresHector) # update occupancy grid and position
        person_data = get_api_data() # gather newest person detection

        # Load the occupancy grid from the data (example using random data here)
        updated_grid = plot_people_on_grid(hector, wheresHector, person_data) # plot people on grid   
        cropped_grid = hector.shrink_grid(updated_grid) # shrink for plotting
        # np.savetxt("./Data/temp_grid.txt", cropped_grid) # incase we want more data

        # optional: set drone pose to true pose
        drone_pos = wheresHector.convertGridSystems((wheresHector.conv_x, wheresHector.conv_y), hector.width, cropped_grid.shape[0])

        # Had to move this down so we can get the size of the grid before error checking
        # Move drone based on key presses (for testing movement) 
        keys = pygame.key.get_pressed()
        if keys[pygame.K_LEFT] and drone_pos[0] > 0:
            drone_pos = (drone_pos[0] - 1, drone_pos[1])
        if keys[pygame.K_RIGHT] and drone_pos[0] < cropped_grid.shape[0] - 1:
            drone_pos = (drone_pos[0] + 1, drone_pos[1])
        if keys[pygame.K_UP] and drone_pos[1] > 0:
            drone_pos = (drone_pos[1], drone_pos[0] - 1)
        if keys[pygame.K_DOWN] and drone_pos[1] < cropped_grid.shape[0] - 1:
            drone_pos = ( drone_pos[1], drone_pos[0] + 1)

        # Draw the occupancy grid and drone
        draw_occupancy_grid(cropped_grid)
        draw_drone(drone_pos)

        # Update the display
        pygame.display.flip()

        # Control the frame rate
        pygame.time.Clock().tick(10)

if __name__ == "__main__":
    # Initialize pygame
    pygame.init()

    # Set up display
    width, height = 600, 600  # Adjust if necessary based on grid size and resolution
    window = pygame.display.set_mode((width, height))
    pygame.display.set_caption("Occupancy Grid with Drone Position")

    # Start game
    main()