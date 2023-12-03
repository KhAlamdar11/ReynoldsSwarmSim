import numpy as np

class Boid:

    def __init__(self,id,weights,percep_field,state, **kwargs):
        
        self.id = id
        self.w_a, self.w_c, self.w_s = weights
        self.local_r, self.local_theta = percep_field

        # [x,y,theta,v_x,v_y,w]
        self.state = state

        # list of neigbors as boid objects
        self.neighbors = None

        # Desired goal - [x, y, radius, tolerance]
        self.goal = None

        # Max speed
        self.max_speed = kwargs['max_speed']
        # Max acceleration
        self.max_acc = kwargs['max_acc']

        # Use prioritized acceleration
        self.use_prioritized_acc = kwargs['use_prioritized_acc']
        self.priority_list = kwargs['priority_list']


    def get_pos(self):
        '''
        Returns position as [x,y]
        '''
        return self.state[:2]
    
    def get_theta(self):
        '''
        Returns heading theta
        '''
        return self.state[2]

    def get_pose(self):
        '''
        Returns pose as [x,y,theta]
        '''
        return self.state[:3]
    
    def get_linvel(self):
        '''
        Returns linear velocity as [v_x,v_y]
        '''
        return self.state[3:5]
    
    def get_angvel(self):
        '''
        Returns angular velocity w (theta_dot)
        '''
        return self.state[5]

    def get_vel(self):
        '''
        Returns lin and angular velocity as [v_x,v_y,w]
        '''
        return self.state[3:]
    
    def get_goal(self):
        '''
        Returns goal as [x,y,radius,tolerance]
        '''
        return self.goal
    
    def set_state(self,state):
        '''
        Updates state of boid
        '''
        self.state = state

    def set_neighbors(self, neighbor):
        '''
        Updates the neighbors of the boid
        '''
        self.neighbors = neighbor

    def set_goal(self, goal):
        '''
        Updates the goal of the boid
        '''
        self.goal = goal

    def behavior_function(self, behavior):
        '''
        Dynamically calls the behavior function
        '''
        name = list(behavior.keys())[0]
        weight = list(behavior.values())[0]
        return weight * getattr(self, name)()
            
    def update(self, boids=None):
        '''
        Calls and combines behaviors.

        Parameters:
        boids: list of all boids as Boid instances

        Returns:
        - cmd_vel: desired velocities of the boid as [v_x,v_y]
        '''
        # Compute acceleration
        acc = self._combine_behaviours()

        # Compute velocity
        if self.get_goal() != None:
            cmd_vel = np.array([0., 0.]) if acc[0] == 0.0 and acc[1] == 0.0 else np.tanh(acc) # **
        else:
            cmd_vel = np.array(self.get_linvel()) + acc

        # Clip the velocity before returning it. 
        cmd_vel = np.clip(cmd_vel, -self.max_speed, self.max_speed)

        return cmd_vel
    
    def _seek(self):
        ''' Computes required acceleration for the boid to seek the goal.'''
        acc = np.array([0., 0.])
        if self.goal != None:
            distance = np.array(np.array(self.get_pos()) - np.array(self.goal))
            square_distance = (np.linalg.norm(np.array(self.get_pos()) - np.array(self.goal)))**2
            # Mitigate zero-division
            square_distance = 1 if square_distance == 0 else square_distance
            acc = (distance/square_distance)
        return acc

    def _move_to_goal(self):
        ''' Computes required acceleration due to goal. '''
        desired_velocity = np.array([0., 0.])
        if self.goal != None:
            target_offset = np.array(self.goal[:2]) - np.array(self.get_pos())   # goal[x, y], not r or tolerance. 
            distance = np.linalg.norm(target_offset)
            if distance < self.goal[3]:
                print("Distance: ", distance)
                #TODO: send a flag that goal has been reached and generate a new goal
                
                return np.array([0., 0.]) # Set the distance to 0 when it falls below the tolerance. 

            ramped_speed = self.max_speed * (distance / self.goal[2])
            clipped_speed = np.minimum(ramped_speed, self.max_speed)
            desired_velocity = (clipped_speed / distance) * target_offset
        return desired_velocity

    def _alignment(self):
        ''' Computes required acceleration due to alignment. '''
        acc = np.array([0., 0.])
        if self.neighbors != None and len(self.neighbors) != 0:
            for boid in self.neighbors:
                acc +=  np.array(boid.get_linvel()) - np.array(self.get_linvel()) 
            acc = acc/len(self.neighbors) 
        return acc

    def _cohesion(self):
        ''' Computes required acceleration due to cohesion. '''
        acc = np.array([0., 0.]) 
        if self.neighbors != None and len(self.neighbors) != 0:
            for boid in self.neighbors:
                acc +=  np.array(boid.get_pos()) - np.array(self.get_pos())
            acc = acc/len(self.neighbors) 
        return acc

    def _separation(self):
        ''' Computes required acceleration due to seperation. '''
        acc = np.array([0., 0.])
        if self.neighbors != None and len(self.neighbors) != 0:
            for boid in self.neighbors:
                distance = np.array(boid.get_pos()) - np.array(self.get_pos())
                square_distance = (np.linalg.norm(np.array(boid.get_pos()) - np.array(self.get_pos())))**2
                acc += (distance/square_distance)
            acc = acc/len(self.neighbors) 
        return - acc # Negative because we want to move away from the neighbors.


    def _combine_behaviours(self):
        ''' Calls behaviours, and computes the net weighted acceleration. '''
        if self.get_goal() != None:
            print(f"This is boid {self.id}")
            acc = 0.7 * self._move_to_goal() + self.w_a * self._alignment() + self.w_c * self._cohesion() + self.w_s * self._separation()
        else:
            acc = np.array([0., 0.])
            if self.use_prioritized_acc:
                for behavior in self.priority_list:
                    acc_request = self.behavior_function(behavior)
                    acc_mag = np.linalg.norm(acc_request)
                    if np.linalg.norm(acc) + acc_mag > self.max_acc:
                        # Clip the acceleration
                        return np.clip(acc + acc_request, -self.max_acc, self.max_acc)
                    acc += acc_request
            else:
                acc = self.w_a * self._alignment() + self.w_c * self._cohesion() + self.w_s * self._separation()
        return acc
    

    # TODO: def _obstacle_avoidance(self):