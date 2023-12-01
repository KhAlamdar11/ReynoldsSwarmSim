#!/usr/bin/python3

import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

import numpy as np

from utils.utils import pose_msg_to_state
from utils.Boid import Boid

class Reynolds:
    def __init__(self):

        # Get parameters
        self.n_boids = rospy.get_param('~n_boids')
        self.weights = [rospy.get_param('~w_a'),
                        rospy.get_param('~w_c'),
                        rospy.get_param('~w_s')]
        self.percep_field = [rospy.get_param('~local_r'),
                              rospy.get_param('~local_theta')]

        rospy.loginfo('n_boids: %s', self.n_boids)
        rospy.loginfo('weights [w_a, w_c, w_s]: %s', self.weights)
        rospy.loginfo('perceptive field [local_r, local_theta]: %s', self.percep_field)

        # list of poses [x,v] for bois in neigborhood. Robot id = list index
        self.boids = [None for _ in range(self.n_boids)]

        # Create subscribers and publishers to n robots dynamically
        self.subs = []
        self.pubs = []
        for i in range(self.n_boids):

            sub = rospy.Subscriber('/robot_{}/odom'.format(i), Odometry, self.odom_callback, callback_args=i)
            self.subs.append(sub)

            pub = rospy.Publisher('/robot_{}/cmd_vel'.format(i), Twist, queue_size=1)
            self.pubs.append(pub)

        # # Create publisher to n robots' cmd_vel dynamically Publishers /robot_{i}/cmd_vel
        
        for i in range(self.n_boids):
            topic_name = '/robot_{}/cmd_vel'.format(i)
            

        # execute algorithm 
        rospy.Timer(rospy.Duration(0.1), self.run)

        # tests
        # rospy.Timer(rospy.Duration(1.0), self._test_odom)


    def odom_callback(self, data, robot_id):
        '''
        Takes odom msg and robot id, and creates boids if not created.
        Otherwise, updates the state of the boid.
        '''
        
        x = pose_msg_to_state(data)

        if self.boids[robot_id] != None: # common case
            # Update boid state
            self.boids[robot_id].set_state(x)
        else:
            # Create boid instance
            self.boids[robot_id] = Boid(robot_id,self.weights,self.percep_field,x)


    def find_neighbors(self, boid):
        # Retrieve the position of each boid. 
        neighbors = []
        for i in range(self.n_boids):
            # Skip if boid is self or if boid has not been created yet
            if self.boids[i] == None:
                continue
            # Calculate the distance between the boid and the other boids
            # Do this only if the other boid has a position
            if self.boids[i].get_pos() != boid.get_pos() and self.boids[i].get_pos() != None:
                distance = np.linalg.norm(np.array(boid.get_pos()) - np.array(self.boids[i].get_pos()))
                if distance <= boid.local_r:
                    neighbors.append(self.boids[i])
        return neighbors
    
    # Write a function that takes in a boid and cmd_vel and publishes it to the appropriate topic
    def publish_cmd_vel(self, boid, vel):
        # Make cmd_vel a Twist message
        cmd_vel = Twist()
        cmd_vel.linear.x = vel[0]
        cmd_vel.linear.y = vel[1]
        # Publish to the appropriate topic
        self.pubs[boid.id].publish(cmd_vel)


    def run(self,event):
        print('-------------------------')
        # TODO: Change the control of the number of boids from the simulator.
        print(self.boids)
        # Ensure that all boids have been created and have a position and velocity before running the algorithm
        for b in self.boids:
            if b == None:
                continue
            if b.get_pos() == None:
                continue

            print(f"Boid {b.id} is at loc: {b.get_pos()} with velocty: {b.get_linvel()}")
            neighbors = self.find_neighbors(b)

            b.set_neighbors(neighbors)
            cmd_vel = b.update(self.boids)

            # TODO: publish the cmd velocity to the appropriate boids topic
            self.publish_cmd_vel(b, cmd_vel)
        

    def _test_odom(self,event):
        print('-------------------------')
        for b in self.boids:
            print(f"Boid {b.id} is at loc: {b.get_pos()} with velocty: {b.get_linvel()}")


if __name__ == '__main__':
    rospy.init_node('reynolds', anonymous=True)
    try:
        node = Reynolds()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
