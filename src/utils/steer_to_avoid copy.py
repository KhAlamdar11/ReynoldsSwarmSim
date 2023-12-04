import rospy
import numpy as np
import math
import copy
from scipy.ndimage import morphology

class SteerToAvoid:

    # Constructor
    def __init__(self,obstacle_radius,step_angle,max_steering_angle):
        # map: 2D array of integers which categorizes world occupancy
        self.map = None 
        self.map_dim = None
        self.step_angle = step_angle
        self.max_steering_angle = max_steering_angle
        self.obstacle_radius=obstacle_radius
        self.resolution = None        # map sampling resolution (size of a cell))                            
        self.origin = None            # world position of cell (0, 0) in self.map                      
        self.there_is_map = False          # set method has been called                          
        self.steering_force = None
    
    def set(self, data, resolution, origin):     
        # Set occupancy map, its resolution and origin. 
        self.map = data
        self.map_dim = self.map.shape
        self.resolution = resolution
        self.origin = np.array(origin)
        rospy.loginfo('map dimensions: %s', self.map_dim)

    def _position_to_map(self, p):
        mx = (p[0]-self.origin[0])/self.resolution 
        my = (p[1]-self.origin[1])/self.resolution
        # if self._in_map([mx,my]):
        return [int(round(mx)),int(round(my))]
        # return [] 
    
    def _in_map(self, loc):
        '''
        loc: list of index [x,y]
        returns True if location is in map and false otherwise
        '''
        [mx,my] = loc
        if mx >= self.map.shape[0]-1 or my >= self.map.shape[1]-1 or mx < 0 or my < 0:
            return False 
        return True
    
    #----------------------------------------------------------------behavior core functions----------------------------#

    def calc_lookahead (self,boid_x,boid_y,boid_theta):
        new_x = boid_x + self.obstacle_radius * math.cos(boid_theta)
        new_y = boid_y + self.obstacle_radius * math.sin(boid_theta)
        return self._position_to_map([new_x, new_y])
        # return new_grid_x, new_grid_y
         
    def turn_back(self,boid_theta):
        return boid_theta + 3.14  # Return the opposite angle if outside the map
    
    def _get_steering_direction(self, boid_pos, boid_vel):
        boid_x, boid_y = boid_pos
        boid_theta = math.atan2(boid_vel[1], boid_vel[0]) #we calc the theta using the direction of velocity
        steering_adjustment = 0  #no adjustment done yet

        map_coords = self.calc_lookahead(boid_x,boid_y,boid_theta)

        if map_coords == []:
            return self.turn_back(boid_theta)  # Return the opposite angle if outside the map
        
        grid_x, grid_y = int(map_coords[0]), int(map_coords[1])

        while self.map[grid_y][grid_x] == 100: #and abs(steering_adjustment) <= self.max_steering_angle:
            steering_adjustment += self.step_angle

            new_theta = boid_theta + steering_adjustment
            map_coords = self.calc_lookahead(boid_x,boid_y,new_theta)
            if map_coords == []:
                return self.turn_back(boid_theta)  # Return the opposite angle if outside the map
            new_grid_x, new_grid_y = map_coords
            if self.map[new_grid_x,new_grid_y] != 100:
                boid_theta = new_theta
                break

            new_theta = boid_theta - steering_adjustment
            map_coords = self.calc_lookahead(boid_x,boid_y,new_theta)
            if map_coords == []:
                return self.turn_back(boid_theta)  # Return the opposite angle if outside the map
            new_grid_x, new_grid_y = map_coords
            if self.map[new_grid_x,new_grid_y] != 100:
                boid_theta = new_theta
                break

            if steering_adjustment == 0.0:
                return None 
        return boid_theta
    
    def _create_steering_force(self,steering_angle):
        fx = math.cos(steering_angle)
        fy = math.sin(steering_angle)
        return np.array([fx,fy]) #TODO: check if this is correct
    
    def _steer_to_avoid(self, boid_pose, boid_vel, boid_goal=None): 
        if boid_vel == [0.0,0.0]:
            return np.array([0.0,0.0])
        steering_angle = self._get_steering_direction(boid_pose[:2], boid_vel) #TODO: check if this is correct
        if steering_angle is None:
            return np.array([0.0, 0.0])
        self.steering_force = self._create_steering_force(steering_angle)
        return self.steering_force





    
