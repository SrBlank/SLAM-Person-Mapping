import rospy
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

    # there is a function I have removed because of dependecny issues that we might need later for calculating roll, pitch, and yaw        

def main():
    rospy.init_node("map_listener", anonymous=True)

    map = Grid()
    whereInMap = PoseEstimate()

    map.getOccupancyGrid()
    print(map.size) # should print 2048x2048

    whereInMap.getRawPosition(map.resolution, map.width, map.height)
    print(f"World Coordinate: ({whereInMap.raw_x}, {whereInMap.raw_y})     Grid Coordinate: ({whereInMap.conv_x}, {whereInMap.conv_y})")


if __name__ == "__main__":
     main()

