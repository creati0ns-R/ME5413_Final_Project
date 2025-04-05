#!/usr/bin/env python3 
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose

def loader(csv_filename):
    # Load and return the CSV file as a NumPy array
    map2D = np.loadtxt(csv_filename, delimiter=',')
    return map2D

def converter(map2D, resol, w, h, o):
    '''
    converts the coordinate points in x-y plane to OccupancyGrid ROS message
    '''
    # Init the grid 
    gridmap = OccupancyGrid()
    
    # Copy contents to message type
    gridmap.header.frame_id = "map"
    gridmap.info.resolution = resol
    gridmap.info.width = w
    gridmap.info.height = h
    gridmap.info.origin = Pose()
    gridmap.info.origin.position.x = o[1]
    gridmap.info.origin.position.y = o[0]

    # Fill in the grid
    grid_data = np.full(w*h, 0, dtype=int)  # init with -1 : for unknown values
    for x, y in map2D:
        # Convert x-y coordinates to grid coordinates
        grid_x = int((x - o[1]) / resol)
        grid_y = int((y - o[0]) / resol)
        if 0 <= grid_x < w and 0 <= grid_y < h:
            index = grid_y * w + grid_x
            grid_data[index] = 100  # 100: occupied

    gridmap.data = list(grid_data)
    return gridmap

def publisher_method(csv_filename, resol, w, h, o):
    rospy.init_node('mapper')
    map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=10, latch=True)

    # Load and create the occupancy grid
    map2D = loader(csv_filename)
    grid = converter(map2D, resol, w, h, o)

    # Publish the map
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        grid.header.stamp = rospy.Time.now()
        map_pub.publish(grid)
        rate.sleep()

if __name__ == '__main__':
    file_name = '/home/risheek/Documents/ME5413_Final_Project/src/pointcloud2map/maps/2d_map.csv'
    resol = 0.05  # 5cm/cell
    w = 1504  # No of cells in x axis
    h = 2080  # No of cells in y axis
    o = (-55.000000, -30.0000)  # map origin

    publisher_method(file_name, resol, w, h, o)
