import numpy as np
import math
from ompl import base as ob
from ompl import geometric as og

def wrap_angle(angle):
    """Wrap an angle to the range of -pi to pi."""
    return math.atan2(math.sin(angle), math.cos(angle))

class PotentialField:

    # Constructor
    def __init__(self, obstacle_radius):
        # map: 2D array of integers which categorizes world occupancy
        self.map = None 
        self.map_dim = None

        # map sampling resolution (size of a cell))                            
        self.resolution = None
        
        # world position of cell (0, 0) in self.map                      
        self.origin = None
        
        # set method has been called                          
        self.there_is_map = False
        
        # radius arround the robot used to check occupancy of a given position                 
        self.obstacle_radius = obstacle_radius                    

        # Force field for potential function
        self.force_field = None
    
    # Set occupancy map, its resolution and origin. 
    def set(self, data, resolution, origin):
        self.map = data
        self.map_dim = self.map.shape
        self.resolution = resolution
        self.origin = np.array(origin)
        self.there_is_map = True
    
    def compute(self,map): 
        '''
        computes the force field of a given map.
        Format: Size of nxn, each element containing the force vector [x,y] at given position
        '''
 
        brushfire_map = self._brushfire(map,max_radius=self.obstacle_radius/self.resolution)
        self.force_field = self._create_force_field(brushfire_map)

    def _get_force_vector(self,pose):
        
        grid_pose = self._position_to_map(pose)
        grid_pose = (int(round(grid_pose[0],0)),int(round(grid_pose[1],0)))

        return self.force_field[grid_pose] 
    # Transform position with respect the map origin to cell coordinates

    def _brushfire(self,map, max_radius):
        '''
        '''
        map_shape = np.shape(map)

        motions = [[-1,0], [1,0], [0,-1], [0,1],\
                    [-1,-1], [-1,1], [1,-1], [1,1]]
        
        value = 1 # value of the obstacle position
        map = np.array(map)
        result = np.where(map == 1)
        [x_s,y_s] = result[0:2]
        ones_loc = []
        for i in range(len(x_s)):
            ones_loc.append([x_s[i], y_s[i]])
        queue = ones_loc
        
        while queue != []:
            value += 1
            if value > max_radius:
                break
            new_queue = []
            for p in queue:
                for m in motions:
                    new_idx = [p[0]+m[0],p[1]+m[1]]    # p+m
                    if new_idx[0]>-1 and new_idx[1]>-1 and new_idx[0]<map_shape[0] and new_idx[1]<map_shape[1]:
                        if map[new_idx[0],new_idx[1]] == 0:
                            map[new_idx[0],new_idx[1]] = value
                            new_queue.append(new_idx) 
            queue = new_queue
        return map

    def _position_to_map(self, p):
        # TODO: convert world position to map coordinates. If position outside map return `[]` or `None`

        # print('point', p)
        # print('self.origin', self.origin)

        mx = (p[0]-self.origin[0])/self.resolution 
        my = (p[1]-self.origin[1])/self.resolution
        if self.__in_map__([mx,my]):
            return [mx,my]
        return [] 
    
    def _in_map(self, loc):
        '''
        loc: list of index [x,y]
        returns True if location is in map and false otherwise
        '''
        [mx,my] = loc
        if mx >= self.map.shape[0]-1 or my >= self.map.shape[1]-1 or mx < 0 or my < 0:
            return False 
        return True
    
