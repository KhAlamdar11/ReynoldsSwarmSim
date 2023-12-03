#!/usr/bin/python3

import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from scipy.spatial import ConvexHull
import matplotlib.pyplot as plt

import numpy as np

from utils.state_tf  import pose_msg_to_state
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
        self.goal = [rospy.get_param('~goal_x'),
                     rospy.get_param('~goal_y'),
                     rospy.get_param('~goal_radius'),
                     rospy.get_param('~goal_tolerance')]
        
        self.goal_list = rospy.get_param('~goal_list')

        self.max_speed = rospy.get_param('~max_speed')
        self.max_acc = rospy.get_param('~max_acc')
        self.n_leader = rospy.get_param('~n_leader')
        self.leader_method = rospy.get_param('~leader_method')
        self.inter_goal_dist = rospy.get_param('~inter_goal_dist')
        self.use_prioritized_acc = rospy.get_param('~use_prioritized_acc')
        self.priority_list = rospy.get_param('~priority_list')
        self.kwargs = {'goal': self.goal, 'max_speed': self.max_speed, 
                       'max_acc': self.max_acc, 'use_prioritized_acc': self.use_prioritized_acc,
                       'priority_list': self.priority_list}

        self.compute_goals = True if self.n_leader > 0 else False
        # All boids have been created
        self.all_boids_created = False

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
            self.boids[robot_id] = Boid(robot_id, self.weights, self.percep_field, x, **self.kwargs)
            # Set a flag when all boids have been created
            if None not in self.boids:
                self.all_boids_created = True


    @classmethod
    def point_angle_length(cls, p1=[], angle=0, length=1):
        # A segment can be initialized with a Point object, an angle, and a segment length.
        x2 = p1[0] + np.cos(angle) * length
        y2 = p1[1] + np.sin(angle) * length
        return [x2, y2]


    def generate_goals(self):
        positions = np.array([boid.get_pos() for boid in self.boids])

        # Find the boids closest to the goal
        distances = np.linalg.norm(positions - self.goal[:2], axis=1)
        closest_boids = np.argsort(distances)[:self.n_leader]

        # # Create the goals using point angle length. Each angle is 60 degrees from the previous angle.
        # # The length is the boid's local radius.
        goals = [self.goal]
        # Convert 60 degrees to radians
        angle = np.pi / 3
        for i in range(self.n_leader - 1):
            goals.append([*self.point_angle_length(self.goal[:2], angle, self.percep_field[0] * self.inter_goal_dist), self.goal[2], self.goal[3]])
            angle += np.pi / 3

        # Find the closest boid to each goal
        for goal in goals:
            distances = np.linalg.norm(positions[closest_boids] - goal[:2], axis=1)
            closest_boid = closest_boids[np.argmin(distances)]
            # Delete the boid from the closest boids so that it is not assigned to another goal
            closest_boids = np.delete(closest_boids, np.argmin(distances))
            self.boids[closest_boid].set_goal(goal)

        # Plot the goals assigned to each boid
        for boid in self.boids:
            if boid == None:
                continue
            if boid.get_goal() == None:
                continue
            plt.plot(boid.get_goal()[0], boid.get_goal()[1], 'ro')
        plt.show()


    def convex_hull(self):
        positions = np.array([boid.get_pos() for boid in self.boids])

        # Find the boids that are the 4 corners of the convex hull
        hull = ConvexHull(positions)
        hull_vertices = hull.vertices
        hull_points = positions[hull_vertices]
        hull_points = np.append(hull_points, hull_points[0]).reshape(-1, 2)

        # Find the index of each of the hull points in the positions array
        hull_indices = []
        for hull_point in hull_points:
            hull_indices.append(np.where((positions == hull_point).all(axis=1))[0][0])

        # Draw a line between each of the hull points using matplotlib
        for i in range(len(hull_points) - 1):
            plt.plot([hull_points[i][0], hull_points[i+1][0]], [hull_points[i][1], hull_points[i+1][1]], 'k-')

        # Create the goals using point angle length. Each angle is 60 degrees from the previous angle.
        # The length is the boid's local radius.
        goals = [self.goal]
        # Convert 60 degrees to radians
        angle = np.pi / 3
        for i in range(self.n_leader - 1):
            goals.append([*self.point_angle_length(self.goal[:2], angle, self.percep_field[0] * self.inter_goal_dist), self.goal[2], self.goal[3]])
            angle += np.pi / 3

        # Find the closest boid to each goal
        for goal in goals:
            distances = np.linalg.norm(hull_points - goal[:2], axis=1)
            closest_boid = np.argmin(distances)
            index = hull_indices[closest_boid]
            # Delete the boid from the hull indices so that it is not assigned to another goal
            del hull_indices[closest_boid]
            # Delete the boid from the hull points so that it is not assigned to another goal
            hull_points = np.delete(hull_points, closest_boid, axis=0)
            self.boids[index].set_goal(goal)

        # Plot the goals assigned to each boid
        for boid in self.boids:
            if boid == None:
                continue
            if boid.get_goal() == None:
                continue
            plt.plot(boid.get_goal()[0], boid.get_goal()[1], 'ro')
        plt.show()


    def find_neighbors(self, boid):
        # Retrieve the position of each boid. 
        neighbors = []
        for i in range(self.n_boids):
            # Skip if boid is self or if boid has not been created yet
            if self.boids[i] == None:
                continue
            if self.boids[i] != boid:
                angle_diff = abs(self.boids[i].get_theta() - boid.get_theta())
                angle_diff = (angle_diff + np.pi) % (2 * np.pi) - np.pi
                if abs(angle_diff) < boid.local_theta / 2:
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


    def run(self, event):
        # Generate goals (if necessary) for leader boids.
        if self.compute_goals and self.all_boids_created:
            if self.leader_method == 0:
                self.generate_goals()
            elif self.leader_method == 1:
                self.convex_hull()
            self.compute_goals = False

        if self.all_boids_created == False:
            return 
        # Ensure that all boids have been created and have a position and velocity before running the algorithm
        for b in self.boids:
            if b == None:
                continue
            if b.get_pos() == None:
                continue

            neighbors = self.find_neighbors(b)
            b.set_neighbors(neighbors)
            cmd_vel = b.update(self.boids)

            #TODO: check if the goal has been reached, then set compute goal to get a new goal
            #based on the list of goals provided

            # publish the cmd velocity to the appropriate boids topic
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
