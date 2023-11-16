class Boid:

    def __init__(self,id,weights,percep_field,state):
        
        self.id = id
        self.w_a, self.wc, self.ws = weights
        self.local_r, self.local_theta = percep_field

        # [x,y,theta,v_x,v_y,w]
        self.state = state

        # list of neigbors as boid objects
        self.neigbors = None

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
    
    def set_state(self,state):
        '''
        Updates state of boid
        '''
        self.state = state
            
    def update(self,boids):
        '''
        Updates the robot pose and neighborhood.

        Parameters:
        boids: list of all boids as Boid instances

        Returns:
        - cmd_vel: desired velocities of the boid as [v_x,v_y]
        '''
        
        # TODO: update neighborhood 

        # Compute acceleration
        acc = self._combine_behaviours()

        # TODO: Compute desired velocities from desired accel
        cmd_vel = 0 # REMOVE

        return cmd_vel


    def _alignment(self):
        ''' Computes required acceleration due to alignment. '''
        # TODO
        acc = 0 # REMOVE
        return acc
    

    def _cohesion(self):
        ''' Computes required acceleration due to cohesion. '''
        # TODO
        acc = 0 # REMOVE
        return acc
    

    def _seperation(self):
        ''' Computes required acceleration due to seperation. '''
        # TODO
        acc = 0 # REMOVE
        return acc


    def _combine_behaviours(self):
        ''' Calls behaviours, and computes the net weighted acceleration. '''
        acc = self.w_a*self._alignment() + self.w_c*self._cohesion() + self.w_s*self._seperation()
        return acc
    

    # TODO: def _obstacle_avoidance(self):
    # TODO: def _pursuits(self)