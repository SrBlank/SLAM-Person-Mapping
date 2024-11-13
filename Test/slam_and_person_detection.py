import math
import numpy as np
import cv2

class Grid():
    def __init__(self):
        self.grid = None
        self.width = None
        self.height = None
        self.resolution = None
        
    def getOccupancyGrid(self):
        print("Getting Occupancy Grid...")
        self.grid = np.loadtxt("./Data/fully_mapped_grid.txt")

        self.width, self.height = self.grid.shape
        self.resolution = 0.05 # meters
        print("Grid dimensions: {}x{}, resolution: {}".format(self.width, self.height, self.resolution))

class PoseEstimate():
    def __init__(self):
        self.raw_scan = None
        
        self.raw_x = None
        self.raw_y = None
        self.conv_x = None
        self.conv_y = None
        
        self.orient_x = None
        self.orient_y = None
        self.orient_z = None
        self.orient_w = None
        self.roll = None
        self.pitch = None
        self.yaw = None
                
    def getRawPosition(self, grid_res, grid_w, grid_h):
        print("Getting Raw Position...")
        pos_data = np.loadtxt("./Data/fully_mapped_pose_estimation.txt", dtype=str)
        pose_dict = {row[0]: float(row[1]) for row in pos_data}
        
        self.raw_x = pose_dict["raw_x"]
        self.raw_y = pose_dict["raw_y"]
        self.conv_x = pose_dict["conv_x"]
        self.conv_y = pose_dict["conv_y"]
        
        self.orient_x = pose_dict["orient_x"]
        self.orient_y = pose_dict["orient_y"]
        self.orient_z = pose_dict["orient_z"]
        self.orient_w = pose_dict["orient_w"]
        self.roll = pose_dict["roll"]
        self.pitch = pose_dict["pitch"]
        self.yaw = pose_dict["yaw"]

def plot_people_on_grid(grid, people, robot_x, robot_y, robot_yaw, grid_res, grid_width, grid_height):
    occupancy_grid = grid.copy()

    # Plot robot position at grid value 25
    #if 0 <= robot_x < grid_width and 0 <= robot_y < grid_height:
    #    occupancy_grid[robot_y, robot_x] = 25

    # Plot each detected person in the grid with value 50
    for person in people:
        if person['confidence'] >= 0.5:  # Filter out low-confidence detections
            distance = person['depth']
            distance_conv = int(distance / grid_res)
            
            # Calculate person position based on distance and robot orientation
            offset_x = int(distance_conv * np.cos(np.radians(robot_yaw)))
            offset_y = int(distance_conv * np.sin(np.radians(robot_yaw)))

            person_x = robot_x + offset_x
            person_y = robot_y + offset_y

            # Ensure coordinates are within grid bounds
            if 0 <= person_x < grid_width and 0 <= person_y < grid_height:
                occupancy_grid[person_y, person_x] = 50

    return occupancy_grid

def shrink_grid(grid):
    mask = np.isin(grid, [0, 100, 50]).astype(np.uint8)
    coords = cv2.findNonZero(mask)

    if coords is not None:
        x, y, w, h = cv2.boundingRect(coords)
        
        # Calculate the side length needed to make the bounding box square
        side_length = max(w, h)

        # Center the square bounding box around the original rectangle
        cx, cy = x + w // 2, y + h // 2  # Center of the bounding box

        # Calculate new top-left corner for the square bounding box
        half_side = side_length // 2
        new_x = max(0, cx - half_side)
        new_y = max(0, cy - half_side)

        # Ensure the square bounding box stays within grid bounds
        if new_x + side_length > grid.shape[1]:
            new_x = grid.shape[1] - side_length
        if new_y + side_length > grid.shape[0]:
            new_y = grid.shape[0] - side_length

        # Crop the grid using the square bounding box coordinates
        cropped_grid = grid[new_y:new_y + side_length, new_x:new_x + side_length]
    else:
        return grid
    
    return cropped_grid



def getData(hector, wheresHector):
	hector.getOccupancyGrid()
	wheresHector.getRawPosition(hector.resolution, hector.width, hector.height)

def getFakeData():
    return [{'depth': 1.7810001373291016, 'confidence': 0.9949648976325989}]
     
def main():
    hector = Grid()
    wheresHector = PoseEstimate()

    getData(hector, wheresHector)
    person_data = getFakeData()

    if person_data is not None:
        updated_grid = plot_people_on_grid(
            hector.grid,
            person_data,
            int(wheresHector.conv_x),
            int(wheresHector.conv_y),
            wheresHector.yaw,
            hector.resolution,
            hector.width,
            hector.height
            )
        cropped_grid = shrink_grid(updated_grid)
        
        np.savetxt("./Data/temp_grid.txt", cropped_grid)

if __name__ == "__main__":
	main()