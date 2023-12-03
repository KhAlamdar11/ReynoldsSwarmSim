import numpy as np

class Boid:

    def __init__(self,id,weights,percep_field,state):
        
        self.id = id
        self.w_a, self.w_c, self.w_s = weights
        self.local_r, self.local_theta = percep_field

        # [x,y,theta,v_x,v_y,w]
        self.state = state

        # list of neigbors as boid objects
        self.neighbors = None

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

    def set_neighbors(self, neighbor):
        '''
        Updates the neighbors of the boid
        '''
        self.neighbors = neighbor
            
    def update(self,boids=None):
        '''
        Calls and combines behaviors.

        Parameters:
        boids: list of all boids as Boid instances

        Returns:
        - cmd_vel: desired velocities of the boid as [v_x,v_y]
        '''

        # Compute acceleration
        acc = self._combine_behaviours()

        # TODO: Compute desired velocities from desired accel
        cmd_vel = self.get_linvel() + acc

        return cmd_vel


    def _alignment(self):
        ''' Computes required acceleration due to alignment. '''
        # TODO
        acc = [0, 0]
        if self.neighbors != None and len(self.neighbors) != 0:
            for boid in self.neighbors:
                acc += np.array(self.get_linvel()) - np.array(boid.get_linvel()) 
            acc = acc/len(self.neighbors) 
        return acc
    

    def _cohesion(self):
        ''' Computes required acceleration due to cohesion. '''
        # TODO
        acc = [0, 0] 
        if self.neighbors != None and len(self.neighbors) != 0:
            for boid in self.neighbors:
                acc += np.array(self.get_pos()) - np.array(boid.get_pos()) 
            acc = acc/len(self.neighbors) 
        return acc
    

    def _seperation(self):
        ''' Computes required acceleration due to seperation. '''
        # TODO
        acc = [0, 0]
        if self.neighbors != None and len(self.neighbors) != 0:
            for boid in self.neighbors:
                distance = np.array(self.get_pos()) - np.array(boid.get_pos()) 
                square_distance = (np.linalg.norm(np.array(self.get_pos()) - np.array(boid.get_pos())))**2
                acc += (distance/square_distance)
            acc = acc/len(self.neighbors) 
        return acc


    def _combine_behaviours(self):
        ''' Calls behaviours, and computes the net weighted acceleration. '''
        acc = - self.w_a*self._alignment() - self.w_c*self._cohesion() + self.w_s*self._seperation()
        return acc
    
    def test_obstacle_avoidance(self,ob=None):
        print('in test obs avoidance')
        if self.id == 1:
            return [0.0,0.0]
        vel = np.array([0.2, 0.0])
        # vel += np.array(ob.get_force_vector(self.get_pos()))
        print("BOID ID:", self.id)
        obs_vel = 0.8*np.array(ob.compute(self.get_pos(),self.get_linvel())) 
        
        if obs_vel[0] != 0.0 and obs_vel[1] != 0.0:
            vel = obs_vel
        
        print('Velocity: ', vel)
        # print('Velocity Frome: ', np.array(ob.compute(self.get_pos(),self.get_linvel())) )
        return vel
