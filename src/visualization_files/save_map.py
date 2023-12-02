#!/usr/bin/python3

import rospy
import numpy as np

from nav_msgs.msg import OccupancyGrid
import roslib
import copy

class SaveMap:       
    def __init__(self):
        self.i = 1
        self.map_r1 = None
        self.map_c1 = None
        self.res = None
        self.occupancy_map = None

        self.map_subscriber = rospy.Subscriber("/map", OccupancyGrid, self.save_map)
    
    def save_map(self,data):
        # Fetch meta deta of map: 
        self.map_r1 = data.info.height  #y
        self.map_c1 = data.info.width   #x
        self.res = data.info.resolution
        print(self.map_r1,self.map_c1)
        self.occupancy_map = np.array(data.data).reshape(self.map_r1,self.map_c1)
        self.occupancy_map = np.where(self.occupancy_map != -1, self.occupancy_map, 0)
        self.occupancy_map = np.where(self.occupancy_map != 100, self.occupancy_map, 1)
        
        dil_size =  0.25/2 # Size of dilation needed

        np.savetxt("simple_maze.txt",self.occupancy_map)

    def dilate_map(self,dil_size):

        octomap = copy.deepcopy(self.occupancy_map)

        grid_size = int(dil_size / self.res)

        for r in range(grid_size, self.map_r1-grid_size):
            for c in range(grid_size, self.map_c1-grid_size):

                for i in range(-grid_size, grid_size+1):
                    for j in range(-grid_size, grid_size+1):

                        if self.occupancy_map[r+i, c+j] == 1:
                            octomap[r,c] = 1
                            break
        return np.array(octomap) 



if __name__ == '__main__':
    rospy.init_node('save_map', anonymous=True)
    n = SaveMap()
    rospy.spin()