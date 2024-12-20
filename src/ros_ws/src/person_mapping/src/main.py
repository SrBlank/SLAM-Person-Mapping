# NOTE: TYPEHINTING IS NOT SUPPORTED IN PYTHON 2

import pygame
import sys
import numpy as np
import requests
import rospy
import math

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
USE_DYNAMIC_POS = False # If True, uses SLAM position. Otherwise, uses manual movement.
# User is able to set or use SLAM position
# Drone's initial position (grid coordinates)
# drone_pos = (25, 25)  # Start at the center of the grid

"""
GRID 
"""
def get_api_data(api_url="http://192.168.50.79:5000/get_people"):
    """
    Fetches person detection data from the API.

    Args:
        api_url: The URL of the API endpoint.

    Returns:
        A JSON object containing detected person data or None if there's an error.
    """
    try:
        response = requests.get(api_url)
        if response.status_code == 200:
            data = response.json()
            print("Data retrieved successfully.")
            return data
        else:
            print("Failed to fetch data. Status code: ",response.status_code)
            return None
    except requests.RequestException as e:
        print("Error fetching data: ",e)
        return None

def plot_people_on_grid(hector, wheresHector, people, circle_radius=2):
    """
    Updates the occupancy grid by plotting detected people.

    Args:
        hector: Grid object containing the occupancy grid data.
        wheresHector: PoseEstimate object containing robot position and orientation.
        people: List of detected people with depth and confidence information.
        circle_radius: Radius of the area to mark around each detected person.

    Returns:
        Updated occupancy grid with detected people plotted.
    """
    occupancy_grid = hector.grid.copy()

    for person in people:
        if person['confidence'] >= 0.5:  # Filter out low-confidence detections
            distance = person['depth']
            distance_conv = int(distance / hector.resolution)
            
            # Calculate person position based on distance and robot orientation
            offset_x = int(distance_conv * np.cos(np.radians(wheresHector.yaw)))
            offset_y = int(distance_conv * np.sin(np.radians(wheresHector.yaw)))

            person_x = wheresHector.conv_x + offset_x
            person_y = wheresHector.conv_y + offset_y

            # Draw a circle of 50 around the detected person
            for dx in range(-circle_radius, circle_radius + 1):
                for dy in range(-circle_radius, circle_radius + 1):
                    if dx**2 + dy**2 <= circle_radius**2:  # Check if the point is within the circle
                        circle_x = person_x + dx
                        circle_y = person_y + dy

                        if 0 <= circle_x < hector.width and 0 <= circle_y < hector.height:
                            occupancy_grid[circle_y, circle_x] = 50 # Mark as detected person
    
    return occupancy_grid

"""
GUI
"""
def draw_occupancy_grid(occupancy_grid):
    """
    Draws the occupancy grid on the Pygame window.

    Args:
        occupancy_grid: 2D NumPy array representing the occupancy grid.
    """
    for row in range(occupancy_grid.shape[0]):
        for col in range(occupancy_grid.shape[1]):
            cell_value = occupancy_grid[row][col]
            color = GRAY if cell_value == -1 else GREEN if cell_value == 0 else RED if cell_value == 100 else BLUE
            pygame.draw.rect(window, color, (col * GRID_SIZE, row * GRID_SIZE, GRID_SIZE, GRID_SIZE))

def draw_drone(pos, yaw):
    """
    Draws the drone on the grid, including its directional yaw arrow.

    Args:
        pos: Tuple containing the drone's (x, y) position in grid coordinates.
        yaw: Yaw angle (in degrees) indicating the drone's direction.
    """
    
    drone_radius = 20  # Fixed size for the drone's circle 

    # Drone position in pixels
    x = pos[0] * GRID_SIZE + GRID_SIZE // 2
    y = pos[1] * GRID_SIZE + GRID_SIZE // 2

    # Draw the drone as a circle
    pygame.draw.circle(window, WHITE, (x, y), drone_radius)

    # Calculate the direction the drone is facing (yaw is in degrees)
    # Convert yaw to radians
    yaw_rad = math.radians(yaw)

    # Define the length of the direction arrow
    arrow_length = 30  

    # Calculate the arrow endpoint (direction the drone is facing)
    end_x = x + int(arrow_length * math.cos(yaw_rad))
    end_y = y + int(arrow_length * math.sin(yaw_rad))

    # Draw the direction arrow
    pygame.draw.line(window, BLUE, (x, y), (end_x, end_y), 3)

"""
DRIVER
"""
def main():
    hector = Grid()
    wheresHector = PoseEstimate()

    # intial gather data (blocking function) to know we are ready
    hector.getOccupancyGrid()  
    wheresHector.getRawPosition(hector.resolution, hector.width, hector.height)
    original_pos = (wheresHector.conv_x, wheresHector.conv_y)

    is_drone_pos_init = False #

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
        hector.getOccupancyGrid()
        wheresHector.getRawPosition(hector.resolution, hector.width, hector.height)
        person_data = get_api_data() # gather newest person detection

        # Load the occupancy grid from the data (example using random data here)
        updated_grid = plot_people_on_grid(hector, wheresHector, person_data) # plot people on grid   
        cropped_grid = hector.shrink_grid(updated_grid) # shrink for plotting
        # np.savetxt("./Data/temp_grid.txt", cropped_grid) # incase we want more data

        if USE_DYNAMIC_POS:
            wheresHector.getRawPosition(hector.resolution, hector.width, hector.height)
            drone_pos = wheresHector.convertGridSystems(original_pos, hector.shrink_x, hector.shrink_y)
            is_drone_pos_init = True

        if not is_drone_pos_init:
            drone_pos = wheresHector.convertGridSystems(original_pos, hector.shrink_x, hector.shrink_y)
            is_drone_pos_init = True

        # Move drone based on key presses (for testing movement) 
        keys = pygame.key.get_pressed()
        if keys[pygame.K_LEFT] and drone_pos[0] > 0:
            drone_pos = (drone_pos[0] - 1, drone_pos[1])
        if keys[pygame.K_RIGHT] and drone_pos[0] < cropped_grid.shape[1] - 1:
            drone_pos = (drone_pos[0] + 1, drone_pos[1])
        if keys[pygame.K_UP] and drone_pos[1] > 0:
            drone_pos = (drone_pos[0], drone_pos[1] - 1)
        if keys[pygame.K_DOWN] and drone_pos[1] < cropped_grid.shape[0] - 1:
            drone_pos = (drone_pos[0], drone_pos[1] + 1)

        # Draw the occupancy grid and drone
        draw_occupancy_grid(cropped_grid)
        draw_drone(drone_pos, wheresHector.yaw)

        # Update the display
        pygame.display.flip()

        # Control the frame rate
        pygame.time.Clock().tick(10)

if __name__ == "__main__":
    # Initalize ros
    rospy.init_node("map_listener", anonymous=True)

    # Initialize pygame
    pygame.init()

    # Set up display
    width, height = 1200, 1200  # Adjust if necessary based on grid size and resolution
    window = pygame.display.set_mode((width, height))
    pygame.display.set_caption("Occupancy Grid with Drone Position")

    # Start game
    main()