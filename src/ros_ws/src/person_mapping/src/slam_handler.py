# THE FOLLOWING CODE IS PROVIDED BY THE HIGH-PERFORMANCE ENGINEERING (HiPE) LAB
# THE FOLLOWING FUNCTIONS WERE ADDED AS PART OF THIS PROJECT: shrink_grid(), convertGridSystems()
# PLEASE REACH OUT TO THE FOLLOWING AUTHORS OF THE ORIGINAL CODE FOR ANY QUESTIONS
# ORIGINAL AUTHORS: Saad Rafiq (sar287@txstate.edu), Daniyar Boztayev (hmi11@txstate.edu)

# NOTE: TYPEHINTING IS NOT SUPPORTED IN PYTHON 2

import rospy
import tf.transformations
import math
import numpy as np
import cv2
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped


class Grid():
    def __init__(self):
        """
        Initializes the Grid class to handle occupancy grid data.
        """
        self.grid = None # 2D NumPy array to store occupancy grid
        self.width = None 
        self.height = None 
        self.resolution = None # Grid resolution, meters per cell

        self.shrink_x = None # X-coordinate of the shrunken grid's top-left corner
        self.shrink_y = None # Y-coordinate of the shrunken grid's top-left corner
        
    def getOccupancyGrid(self):
        """
        Retrieves the Occupancy Grid from the ROS topic `/map`.
        Updates the grid, width, height, and resolution attributes.
        """
        print("Getting Occupancy Grid...")
        scan = rospy.wait_for_message("/map", OccupancyGrid)
        self.grid = np.array(scan.data).reshape(scan.info.height, scan.info.width)
        self.width = scan.info.width
        self.height = scan.info.height
        self.resolution = scan.info.resolution
        print("Grid dimensions: {}x{}, resolution: {}".format(self.width, self.height, self.resolution))

    def shrink_grid(self, grid):
        """
        Shrinks the grid to a square bounding box containing non-zero occupancy data.

        Args:
            grid: Original occupancy grid as a 2D NumPy array.

        Returns:
            The cropped grid as a 2D NumPy array.
        """
        # Create a mask for valid occupancy values
        mask = np.isin(grid, [0, 100, 50]).astype(np.uint8)
        coords = cv2.findNonZero(mask)

        if coords is not None:
            # Find bounding rectangle of the non-zero values
            x, y, w, h = cv2.boundingRect(coords)
            
            # Calculate the side length needed to make the bounding box square
            side_length = max(w, h)

            # Find the center of the bounding box
            cx, cy = x + w // 2, y + h // 2  

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

            # Return the cropped grid
            return grid[new_y:new_y + side_length, new_x:new_x + side_length]
        else:
            return grid
        

class PoseEstimate():
    def __init__(self):
        """
        Initializes the PoseEstimate class to handle pose data and transformations.
        """
        self.raw_scan = None # Raw PoseStamped data from the ROS topics
        
        # Raw xy-coordinate of the position
        self.raw_x = None 
        self.raw_y = None
        # Grid based xy-coordinate 
        self.conv_x = None
        self.conv_y = None
        
        # Orientation quaternion 
        self.orient_x = None
        self.orient_y = None
        self.orient_z = None
        self.orient_w = None
        self.roll = None # Roll angle (radians)
        self.pitch = None # Pitch angle (radians)
        self.yaw = None # Yaw angle (degrees)
                
    def getRawPosition(self, grid_res, grid_w, grid_h):
        """
        Retrieves the raw position and orientation from the ROS topic `/slam_out_pose`.
        Converts raw coordinates to grid coordinates.

        Args:
            grid_res: Grid resolution in meters per cell.
            grid_w: Grid width in cells.
            grid_h: Grid height in cells.
        """
        print("Getting Raw Position...")
        scan = rospy.wait_for_message("/slam_out_pose", PoseStamped)

        # Extract raw position
        self.raw_x = scan.pose.position.x
        self.raw_y = scan.pose.position.y
        # Convert to grid coordinates
        self.conv_x = int(self.raw_x / grid_res + (grid_w // 2))
        self.conv_y = int(self.raw_y / grid_res + (grid_h // 2))
        
        # Extract orientation as a quaternion
        self.orient_x = scan.pose.orientation.x
        self.orient_y = scan.pose.orientation.y
        self.orient_z = scan.pose.orientation.z
        self.orient_w = scan.pose.orientation.w

        # Convert quaternion to Euler angles
        self.convertQuaternion()
        
    def convertQuaternion(self):
        """
        Converts quaternion orientation to Euler angles (roll, pitch, yaw).
        Converts yaw to degrees and normalizes to the range [0, 360].
        """
        quat = (self.orient_x, self.orient_y, self.orient_z, self.orient_w)
        euler = tf.transformations.euler_from_quaternion(quat)
        self.roll, self.pitch, self.yaw = euler
        self.yaw = (((self.yaw*180)/math.pi)+360)%360 # conver to degrees then flip 

    def convertGridSystems(self, original_point, shrink_x, shrink_y):
        """
        Converts a point from the original grid to the subgrid system.

        Args:
            original_point: Tuple containing the x and y coordinates in the original grid.
            shrink_x: X-coordinate of the subgrid's top-left corner in the original grid.
            shrink_y: Y-coordinate of the subgrid's top-left corner in the original grid.

        Returns:
            Tuple containing the x and y coordinates in the subgrid.
        """
        # Calculate coordinates in the subgrid
        subgrid_x = original_point[0] - shrink_x
        subgrid_y = original_point[1] - shrink_y

        return (subgrid_x, subgrid_y)
