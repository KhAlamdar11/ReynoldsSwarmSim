import numpy as np
import math
from ompl import base as ob
from ompl import geometric as og


class Steer_to_avoid:

    # Constructor
    def __init__(self):
        # map: 2D array of integers which categorizes world occupancy
        self.map = None 
        self.map_dim = None

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
        self.there_is_map = True
    
    def get_steering_direction(self, grid_map, boid, max_steering_angle, neighborhood_radius, step_angle):
        boid_x, boid_y, boid_theta = boid
        steering_adjustment = 0

        lookahead_x = boid_x + neighborhood_radius * math.cos(math.radians(boid_theta))
        lookahead_y = boid_y + neighborhood_radius * math.sin(math.radians(boid_theta))

        map_coords = self._position_to_map([lookahead_x, lookahead_y])
        if not map_coords:
            return boid_theta  # Return the original angle if outside the map

        grid_x, grid_y = int(map_coords[0]), int(map_coords[1])

        while grid_map[grid_y][grid_x] == 100 and abs(steering_adjustment) <= max_steering_angle:
            steering_adjustment += step_angle
            new_theta = boid_theta + steering_adjustment

            new_grid_x = int(boid_x + neighborhood_radius * math.cos(math.radians(new_theta)))
            new_grid_y = int(boid_y + neighborhood_radius * math.sin(math.radians(new_theta)))

            if grid_map[new_grid_y][new_grid_x] != 100:
                boid_theta = new_theta
                break

            new_theta = boid_theta - steering_adjustment

            new_grid_x = int(boid_x + neighborhood_radius * math.cos(math.radians(new_theta)))
            new_grid_y = int(boid_y + neighborhood_radius * math.sin(math.radians(new_theta)))

            if grid_map[new_grid_y][new_grid_x] != 100:
                boid_theta = new_theta
                break

        return boid_theta
    
    def compute(self,map,boid,local_r,step_angle,max_steering_angle): 

        steering_angle = self.get_steering_direction(map,boid,max_steering_angle,local_r,step_angle)
        self.steering_force = self._create_steering_force(steering_angle)


    def _create_steering_force(steering_angle):
        fx= math.cos(steering_angle)
        fy= math.sin(steering_angle)
        steering_force=(fx,fy)
        return(steering_force)



    def _position_to_map(self, p):
        # TODO: convert world position to map coordinates. If position outside map return `[]` or `None`
        # print('point', p)
        # print('self.origin', self.origin)

        mx = (p[0]-self.origin[0])/self.resolution 
        my = (p[1]-self.origin[1])/self.resolution
        if self._in_map([mx,my]):
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
    
