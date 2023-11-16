#!/usr/bin/python3

import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

import numpy as np

from utils.utils import pose_msg_to_pose

class Reynolds:
    def __init__(self):

        # Get parameters
        self.n_boids = rospy.get_param('~n_boids')
        self.weights = [rospy.get_param('~w_a'),
                        rospy.get_param('~w_c'),
                        rospy.get_param('~w_s')]
        self.percept_field = [rospy.get_param('~local_r'),
                              rospy.get_param('~local_theta')]

        rospy.loginfo('n_boids: %s', self.n_boids)
        rospy.loginfo('weights [w_a, w_c, w_s]: %s', self.weights)
        rospy.loginfo('perceptive field [local_r, local_theta]: %s', self.percept_field)

        # list of poses [x,v] for bois in neigborhood. Robot id = list index
        self.boids = [[] for _ in range(self.n_boids)]

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
        # rospy.Timer(rospy.Duration(0.1), self.run)

        # tests
        rospy.Timer(rospy.Duration(1.0), self._test_odom)


    def odom_callback(self, data, robot_id):
        x,v = pose_msg_to_pose(data)
        self.boids[robot_id] = [x,v]


    def run(self,event):
        pass

    def _test_odom(self,event):
        # print(np.array(self.boids))
        # if [] not in self.boids:
        print('-------------------------')
        for b in range(len(self.boids)):
            print(f"Boid {b} is at loc: {self.boids[b][0]} with velocty: {self.boids[b][1]}")


if __name__ == '__main__':
    rospy.init_node('reynolds', anonymous=True)
    try:
        node = Reynolds()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
