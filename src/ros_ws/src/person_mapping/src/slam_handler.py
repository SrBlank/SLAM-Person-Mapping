import rospy
import tf.transformations
import math
import numpy as np
import cv2

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped


class Grid():
    def __init__(self):
        self.grid = None
        self.width = None
        self.height = None
        self.resolution = None

        self.shrink_x = None
        self.shrink_y = None
        
    def getOccupancyGrid(self):
        print("Getting Occupancy Grid...")
        scan = rospy.wait_for_message("/map", OccupancyGrid)
        self.grid = np.array(scan.data).reshape(scan.info.height, scan.info.width)
        self.width = scan.info.width
        self.height = scan.info.height
        self.resolution = scan.info.resolution
        print("Grid dimensions: {}x{}, resolution: {}".format(self.width, self.height, self.resolution))

    def shrink_grid(self, grid):
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
            
            self.shrink_x=new_x
            self.shrink_y=new_y

            # Crop the grid using the square bounding box coordinates
            return grid[new_y:new_y + side_length, new_x:new_x + side_length]
        else:
            return grid
        

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

    def convertGridSystems(self, original_point, shrink_x, shrink_y):
        # Calculate coordinates in the subgrid
        subgrid_x = original_point[0] - shrink_x
        subgrid_y = original_point[1] - shrink_y

        return (subgrid_x, subgrid_y)
