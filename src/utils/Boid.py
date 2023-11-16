class Boid:

    def __init__(self,id,weights,percep_field):
        
        self.id = id
        self.w_a, self.wc, self.ws = weights
        self.local_r, self.local_theta = percep_field

        self.pose = None

        # list of poses [x,v] for bois in neigborhood
        self.neigbors = None
            
    def update(self,self_pose,all_poses):
        '''
        Updates the robot pose and neighborhood.

        Parameters:
        - self_pose: pose of the boid as [x,y,theta]
        - all_poses: poses of all boids as a list of [[x,y,theta],[v_x,v_y,w]]

        Returns:
        - cmd_vel: desired velocities of the boid as [v_x,v_y]
        '''

        self.pose = self_pose
        
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