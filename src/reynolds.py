#!/usr/bin/python3

import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

import math
import numpy as np
import tf
from scipy.spatial import ConvexHull
import matplotlib.pyplot as plt

from utils.state_tf  import pose_msg_to_state
from utils.Boid import Boid
from utils.potential_field import PotentialField
from utils.steer_to_avoid import SteerToAvoid
from utils.navigation import Navigate


class Reynolds:
    def __init__(self):
        # Initialize different components of the class
        self.initialize_parameters()
        self.initialize_behaviors()
        self.initialize_boids()
        self.initialize_ros_components()
        self.start_algorithm()

    def initialize_parameters(self):
        # Extracting parameters from rospy
        self.n_boids = rospy.get_param('~n_boids')
        self.weights = [rospy.get_param('~w_a'),
                        rospy.get_param('~w_c'),
                        rospy.get_param('~w_s')]
        self.percep_field = [rospy.get_param('~local_r'),
                             rospy.get_param('~local_theta')]
        self.obs_r = rospy.get_param('~obs_r')
        self.step_angle = rospy.get_param('~step_angle')
        self.max_steering_angle = rospy.get_param('~max_steering_angle')
        self.goal = [rospy.get_param('~goal_x'),
                     rospy.get_param('~goal_y'),
                     rospy.get_param('~goal_radius'),
                     rospy.get_param('~goal_tolerance')]
        self.goal_list = rospy.get_param('~goal_list')
        self.max_speed = rospy.get_param('~max_speed')
        self.max_acc = rospy.get_param('~max_acc')
        self.n_leaders = min(rospy.get_param('~n_leaders'), self.n_boids)
        self.leader_method = rospy.get_param('~leader_method')
        self.leader_type = rospy.get_param('~leader_type')
        self.inter_goal_dist = rospy.get_param('~inter_goal_dist')
        self.use_prioritized_acc = rospy.get_param('~use_prioritized_acc')
        self.priority_list = rospy.get_param('~priority_list')
        self.hard_arrival = rospy.get_param('~hard_arrival')
        self.avoid_obstacles = SteerToAvoid(self.obs_r, self.step_angle, self.max_steering_angle)
        self.all_goals = []  # Initialize all_goals, adjust as per your logic
        self.leading_boid_ids = []

    def initialize_ros_components(self):
        # Initialize ROS subscribers and publishers
        self.subs = [rospy.Subscriber('/robot_{}/odom'.format(i), Odometry, self.odom_callback, callback_args=i)
                     for i in range(self.n_boids)]
        self.pubs = [rospy.Publisher('/robot_{}/cmd_vel'.format(i), Twist, queue_size=1)
                     for i in range(self.n_boids)]
        self.marker_array_publisher = rospy.Publisher('boids_marker_array', MarkerArray)
        self.goal_marker_publisher = rospy.Publisher('goal_marker', Marker)
        self.subgoal_marker_pub = rospy.Publisher('subgoal_marker_array', MarkerArray)
        self.map_subscriber = rospy.Subscriber("/map", OccupancyGrid, self.map_cb)

    def initialize_behaviors(self):
        # Initialize behaviors and behavior list
        self.all_behaviors = self.create_behavior_dictionary()
        self.behavior_list = self.create_behavior_list()

    def create_behavior_dictionary(self):
        # Create a dictionary of all behaviors
        return {
            '_separation': 'self', 
            '_cohesion': 'self', 
            '_alignment': 'self',
            '_steer_to_avoid': self.avoid_obstacles, 
            '_seek': Navigate(self.max_acc, self.max_speed),
            '_arrival': Navigate(self.max_acc, self.max_speed)
        }

    def create_behavior_list(self):
        # Create a list of behaviors for dynamic function calls
        behavior_list = []
        for behavior_dict in self.priority_list:
            # Assuming behavior_dict is a dictionary with one key-value pair
            behavior_name = list(behavior_dict.keys())[0]
            if behavior_name in self.all_behaviors:
                behavior_weight = behavior_dict[behavior_name]
                behavior_list.append({behavior_name: [behavior_weight, self.all_behaviors[behavior_name]]})
        return behavior_list

    def initialize_kwargs(self):
        # Initialize kwargs dictionary with relevant parameters
        self.kwargs = {
            'max_speed': self.max_speed, 
            'use_prioritized_acc': self.use_prioritized_acc,
            'behavior_list': self.behavior_list, 
            'max_acc': self.max_acc, 
            'hard_arrival': self.hard_arrival
        }

    def initialize_boids(self):
        # Initialize boid related attributes
        self.boids = [None for _ in range(self.n_boids)]
        self.compute_goals = self.n_leaders > 0
        self.boids_created = False
        self.initialize_kwargs()  # Initialize the kwargs here

    def start_algorithm(self):
        # Start the algorithm with a ROS timer
        rospy.Timer(rospy.Duration(0.1), self.run)

    def set_goals(self, all_goals):
        self.all_goals = all_goals
        

    def publish_goal_marker(self, goal):
        marker = Marker()
        marker.header.frame_id = "map"  # Replace with your frame ID
        marker.type = Marker.SPHERE
        marker.id = 10000
        marker.header.stamp = rospy.Time.now()
        marker.pose.position.x = goal[0]
        marker.pose.position.y = goal[1]
        marker.pose.position.z = 0
        marker.scale.x = 2*self.percep_field[0]*self.inter_goal_dist  # Adjust size as needed
        marker.scale.y = 2*self.percep_field[0]*self.inter_goal_dist
        marker.scale.z = 0
        marker.lifetime = rospy.Duration(8937393)
        marker.color = ColorRGBA(0.0, 1.0, 0.0, 0.5)
        self.goal_marker_publisher.publish(marker)
        pass

    def publish_subgoals(self, subgoals):
        if subgoals != None:
            marker_array = MarkerArray()
            for i in range(len(subgoals)):
                marker = Marker()
                marker.header.frame_id = "map"  # Replace with your frame ID
                marker.type = Marker.SPHERE
                marker.id = i
                marker.header.stamp = rospy.Time.now()
                marker.pose.position.x = subgoals[i][0]
                marker.pose.position.y = subgoals[i][1]
                marker.pose.position.z = 0
                marker.scale.x = 0.08  # Adjust size as needed
                marker.scale.y = 0.08
                marker.scale.z = 0.0
                marker.lifetime = rospy.Duration(57437204)
                marker.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
                marker_array.markers.append(marker)
            self.subgoal_marker_pub.publish(marker_array)
        pass

    def publish_formation(self):
        """this function publishes the pose at which the scan was taken as a marker

        :param xk_cloned: the pose at which the scan was taken
        :type xk_cloned: 2D numpy array

        :return: None
        :rtype: None
        """
        marker_array = MarkerArray()
        for boid in self.boids:
            position = boid.get_pose()
            marker = Marker()
            marker.header.frame_id = "map"  # Replace with your frame ID
            marker.type = Marker.SPHERE
            marker.id = boid.id
            marker.header.stamp = rospy.Time.now()
            marker.pose.position.x = position[0]
            marker.pose.position.y = position[1]
            marker.pose.position.z = 0
            marker.scale.x = 0.1  # Adjust size as needed
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.lifetime = rospy.Duration(0.1)
            if self.n_leaders > 0 and len(self.leading_boid_ids) != 0:
                marker.color = ColorRGBA(1.0, 1.0, 0.0, 1.0) if boid.id in self.leading_boid_ids else ColorRGBA(1.0, 0.0, 0.0, 1.0)
            else:
                marker.color = ColorRGBA(0.0, 1.0, 0.0, 1.0) 
            #convert the yaw angle to a euler to quaternion
            quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, position[2])
            marker.pose.orientation.x = quaternion[0]
            marker.pose.orientation.y = quaternion[1]
            marker.pose.orientation.z = quaternion[2]
            marker.pose.orientation.w = quaternion[3]
            marker_array.markers.append(marker)
        self.marker_array_publisher.publish(marker_array)
        self.leading_boid_ids = []
        pass

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
            # Set a flag to know when all boids have been created
            if None not in self.boids:
                self.boids_created = True

    def map_cb(self, gridmap):
        env = np.array(gridmap.data).reshape(gridmap.info.height, gridmap.info.width).T
        # Set avoid obstacles - The steer to avoid behavior (IN THE DICTIONARY) requires the map, resolution, and origin
        self.all_behaviors['_steer_to_avoid'].set(data=env, 
                                resolution=gridmap.info.resolution, 
                                origin=[gridmap.info.origin.position.x, gridmap.info.origin.position.y])
        # self.avoid_obstacles.set(data=env, 
        #                         resolution=gridmap.info.resolution, 
        #                         origin=[gridmap.info.origin.position.x, gridmap.info.origin.position.y])

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
        closest_boids = np.argsort(distances)[:self.n_leaders]

        # # Create the goals using point angle length. Each angle is 60 degrees from the previous angle.
        # # The length is the boid's local radius.
        goals = [self.goal]
        # Convert 60 degrees to radians
        angle = np.pi / 3
        for i in range(self.n_leaders - 1):
            goals.append([*self.point_angle_length(self.goal[:2], angle, self.percep_field[0] * self.inter_goal_dist), self.goal[2], self.goal[3]])
            angle += np.pi / 3
        
        #save all the generated goals
        self.set_goals(goals)

        # Find the closest boid to each goal
        for goal in goals:
            distances = np.linalg.norm(positions[closest_boids] - goal[:2], axis=1)
            closest_boid = closest_boids[np.argmin(distances)]
            # Delete the boid from the closest boids so that it is not assigned to another goal
            if self.leader_type == 0:
                closest_boids = np.delete(closest_boids, np.argmin(distances))
            self.boids[closest_boid].set_goal(goal)
            
            #NOTE: JOSEPH: Added the following line
            self.leading_boid_ids.append(closest_boid)

        # Plot the goals assigned to each boid
        for boid in self.boids:
            if boid == None:
                continue
            if boid.get_goal() == None:
                continue
            plt.plot(boid.get_goal()[0], boid.get_goal()[1], 'ro')
        # plt.show()

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
        for i in range(self.n_leaders - 1):
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
        # plt.show()

    def find_neighbors(self, boid):
        # Retrieve the position of each boid. 
        neighbors = []
        for i in range(self.n_boids):
            # Skip if boid is self or if boid has not been created yet
            if self.boids[i] == None:
                continue
            if self.boids[i] != boid :
                dx = self.boids[i].get_pos()[0] - boid.get_pos()[0]
                dy = self.boids[i].get_pos()[1] - boid.get_pos()[1]
                alpha= math.atan2(dy,dx)
                angle_diff = abs(alpha - boid.get_theta())
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
        # Ensure that all boids have been created and have a position and velocity before running the algorithm
        for b in self.boids:
            if b == None:
                continue
            if b.get_pos() == None:
                continue

            # print(f"Boid {b.id} is at loc: {b.get_pos()} with velocty: {b.get_linvel()}")
            neighbors = self.find_neighbors(b)
            b.set_neighbors(neighbors)

            # Generate goals (if necessary) for leader boids.
            if self.compute_goals and self.boids_created:
                if self.leader_method == 0:
                    self.generate_goals()
                elif self.leader_method == 1:
                    self.convex_hull()
                if self.leader_type == 1:
                    self.compute_goals = False # IF THIS IS FALSE, THE LEADERS WILL BE FIXED
            cmd_vel = b.update(self.boids)

            ## Test obstacle avoidance in isolation
            # cmd_vel = b.test_obstacle_avoidance(self.avoid_obstacles)

            #publish the cmd velocity to the appropriate boids topic
            self.publish_cmd_vel(b, cmd_vel)
            #publish the goal
            self.publish_goal_marker(self.goal)
            #publish the subgoals
            self.publish_subgoals(self.all_goals)
            #publish the formation
            if self.boids_created:
                self.publish_formation()
        

    def _test_odom(self,event):
        print('-------------------------')
        for b in self.boids:
            print(f"Boid {b.id} is at loc: {b.get_pos()} with velocty: {b.get_linvel()}")




#TODO: Subscribe to the odometry message of each boid and remap as a markerArray.


if __name__ == '__main__':
    rospy.init_node('reynolds', anonymous=True)
    try:
        node = Reynolds()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
