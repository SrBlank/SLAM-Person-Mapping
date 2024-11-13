import requests
import rospy
import tf.transformations
import math
import numpy as np

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped


class Grid():
    def __init__(self):
        self.grid = None
        self.width = None
        self.height = None
        self.resolution = None
        
    def getOccupancyGrid(self):
        print("Getting Occupancy Grid...")
        scan = rospy.wait_for_message("/map", OccupancyGrid)
        self.grid = np.array(scan.data).reshape(scan.info.height, scan.info.width)
        self.width = scan.info.width
        self.height = scan.info.height
        self.resolution = scan.info.resolution
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
        scan = rospy.wait_for_message("/slam_out_pose", PoseStamped)

        self.raw_x = scan.pose.position.x
        self.raw_y = scan.pose.position.y
        self.conv_x = int(self.raw_x / grid_res + (grid_w // 2))
        self.conv_y = int(self.raw_y / grid_res + (grid_h // 2))
        
        self.orient_x = scan.pose.orientation.x
        self.orient_y = scan.pose.orientation.y
        self.orient_z = scan.pose.orientation.z
        self.orient_w = scan.pose.orientation.w
        self.convertQuaternion()
        
    def convertQuaternion(self):
        quat = (self.orient_x, self.orient_y, self.orient_z, self.orient_w)
        euler = tf.transformations.euler_from_quaternion(quat)
        self.roll, self.pitch, self.yaw = euler
        self.yaw = (((self.yaw*180)/math.pi)+360)%360 # conver to degrees then flip 

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

def getAPIData(api_url="http://localhost:5000/get_people"):
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

     
def getData(hector, wheresHector):
	hector.getOccupancyGrid()
	wheresHector.getRawPosition(hector.resolution, hector.width, hector.height)

def main():
    hector = Grid()
    wheresHector = PoseEstimate()

    getData(hector, wheresHector)
    person_data = getAPIData()

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