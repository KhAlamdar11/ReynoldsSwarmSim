import rospy
import numpy as np
import math
from ompl import base as ob
from ompl import geometric as og


class SteerToAvoid:

    # Constructor
    def __init__(self,obstacle_radius,step_angle,max_steering_angle):
        # map: 2D array of integers which categorizes world occupancy
        self.map = None 
        self.map_dim = None
        self.step_angle = step_angle
        self.max_steering_angle = max_steering_angle
        self.obstacle_radius=obstacle_radius
        # map sampling resolution (size of a cell))                            
        self.resolution = None
        
        # world position of cell (0, 0) in self.map                      
        self.origin = None
        
        # set method has been called                          
        self.there_is_map = False
                
        self.steering_force = None
    
    # Set occupancy map, its resolution and origin. 
    def set(self, data, resolution, origin):
        self.map = data
        self.map_dim = self.map.shape
        self.resolution = resolution
        self.origin = np.array(origin)
        rospy.loginfo('map dimensions: %s', self.map_dim)
    
    def _get_steering_direction(self, boid_pos, boid_vel):
        boid_x, boid_y = boid_pos

        boid_theta = math.atan2(boid_vel[1],boid_vel[0])

        print('Steering angle (Original): ', boid_theta)
        steering_adjustment = 0

        lookahead_x = boid_x + self.obstacle_radius * math.cos(boid_theta)
        lookahead_y = boid_y + self.obstacle_radius * math.sin(boid_theta)

        map_coords = self._position_to_map([lookahead_x, lookahead_y])
        if not map_coords:
            return boid_theta  # Return the original angle if outside the map

        grid_x, grid_y = int(map_coords[0]), int(map_coords[1])
        
        while self.map[grid_y][grid_x] == 100 and abs(steering_adjustment) <= self.max_steering_angle:

            print('CELL BUSY', grid_y,grid_x)
            steering_adjustment += self.step_angle
            new_theta = boid_theta + steering_adjustment

            print('Added new theta (add): ', new_theta)

            new_x = boid_x + self.obstacle_radius * math.cos(new_theta)
            new_y = boid_y + self.obstacle_radius * math.sin(new_theta)
            print('catesian: ', new_x,new_y)
            new_grid_x, new_grid_y = self._position_to_map([new_x, new_y])
            print('grid: ', new_grid_x,new_grid_y)

            if self.map[new_grid_y,new_grid_x] != 100:
                boid_theta = new_theta
                print("CELL FREE 1: ", new_grid_y,new_grid_x)
                break

            new_theta = boid_theta - steering_adjustment

            print('Added new theta (sub): ', new_theta)

            new_x = boid_x + self.obstacle_radius * math.cos(new_theta)
            new_y = boid_y + self.obstacle_radius * math.sin(new_theta)

            new_grid_x, new_grid_y = self._position_to_map([new_x, new_y])

            if self.map[new_grid_y,new_grid_x] != 100:
                boid_theta = new_theta
                print("CELL FREE 2: ", new_grid_y,new_grid_x)
                break

            if steering_adjustment == 0.0:
                return None 
        return boid_theta
    
    def compute(self,boid_pose, boid_vel): 
        steering_angle = self._get_steering_direction(boid_pose, boid_vel)
        
        if steering_angle is None:
            return [0.0,0.0]
        
        self.steering_force = self._create_steering_force(steering_angle)
        return self.steering_force


    def _create_steering_force(self,steering_angle):
        print('Steering angle (NEW): ', steering_angle)
        fx= math.cos(steering_angle)
        fy= math.sin(steering_angle)
        steering_force=[fx,fy]
        return steering_force

    def _position_to_map(self, p):
        # TODO: convert world position to map coordinates. If position outside map return `[]` or `None`
        # print('point', p)
        # print('self.origin', self.origin)

        mx = (p[0]-self.origin[0])/self.resolution 
        my = (p[1]-self.origin[1])/self.resolution
        if self._in_map([mx,my]):
            return [int(round(mx)),int(round(my))]
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
    
