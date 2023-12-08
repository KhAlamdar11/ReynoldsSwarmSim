#!/usr/bin/python3

import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Header  
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3, PoseStamped, Pose

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
        self.start_visualization()

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

        #print the goal list to know what type of list it is
        print(f"Goal list: {self.goal_list}")   
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


        #NOTE: JOSEPH: Added the following line to make the goal a list of lists
        self.use_multigoal = rospy.get_param('~use_multigoal')
        self.current_goal_index = 0 # to store the index of the current goal in the goal list
        self.arrival_threshold = rospy.get_param('~arrival_threshold') # the percentage of boids that should be within the goal radius for the goal to be considered reached

    def initialize_ros_components(self):
        # Initialize ROS subscribers and publishers
        self.subs = [rospy.Subscriber('/robot_{}/odom'.format(i), Odometry, self.odom_callback, callback_args=i)
                     for i in range(self.n_boids)]
        self.pubs = [rospy.Publisher('/robot_{}/cmd_vel'.format(i), Twist, queue_size=1)
                     for i in range(self.n_boids)]
        
        #define path publisher to publish the trajectory of each boid
        self.path_pub = [rospy.Publisher('/robot_{}/path'.format(i), Path, queue_size=1)
                     for i in range(self.n_boids)]

        #define a path message for each boid
        self.boid_path = [Path() for i in range(self.n_boids)]
        self.boid_traj = [Marker() for i in range(self.n_boids)]
        #define a publisher for the linestrip trajectory of each boid
        self.traj_pub = [rospy.Publisher('/robot_{}/trajectory'.format(i), Marker, queue_size=1)
                        for i in range(self.n_boids)]
        #create an array of different random RGBA colors to be used for each boid path trajectory
        self.traj_colors = [ColorRGBA(np.random.rand(), np.random.rand(), np.random.rand(), 1.0) for i in range(self.n_boids)]
        #define some basic attributes of the path message
        for i in range(self.n_boids):
            #basic path attributes
            self.boid_path[i].header.frame_id = "map"
            self.boid_path[i].header.stamp = rospy.Time.now()

            #basic marker trajectory attributes
            self.boid_traj[i].header.frame_id = "map"
            self.boid_traj[i].header.stamp = rospy.Time.now()
            self.boid_traj[i].type = Marker.LINE_STRIP
            self.boid_traj[i].action = Marker.ADD
            self.boid_traj[i].lifetime = rospy.Duration(423864)
            self.boid_traj[i].scale.x = 0.05
            self.boid_traj[i].color = self.traj_colors[i]

        self.marker_array_publisher = rospy.Publisher('boids_marker_array', MarkerArray, queue_size=1)
        self.goal_marker_publisher = rospy.Publisher('goal_marker', Marker, queue_size=1)
        self.subgoal_marker_pub = rospy.Publisher('subgoal_marker_array', MarkerArray, queue_size=1)
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


    def start_visualization(self):
        #____________________________   visualize   ____________________________
        if rospy.get_param('~visualize'):

            self.truncate_trajectories = rospy.get_param('~truncate_trajectories')

            self.map_msg = None
            self.map_dilated = False

            self.trajectory_pubs = {}

            self.boids_markers = MarkerArray()
            self.boids_markers.markers = []
            self.robot_trajectories = {}
            # self.robot_trajectories = dict((i, Path(poses=[])) for i in range(self.n_boids))
            for i in range(self.n_boids):
                self.robot_trajectories[i] = Path(poses=[])

            self.boids_markers_pub = rospy.Publisher("/vis/boid_positions", MarkerArray,queue_size=1) 
            self.dilated_obs_pub = rospy.Publisher("/vis/obstacle_dilated", OccupancyGrid, queue_size=1)
            self.goal_pub = rospy.Publisher("/vis/goal", Marker, queue_size=1)
            
            for robot_id in range(self.n_boids):
                self.trajectory_pubs[robot_id] = rospy.Publisher(f"/vis/robot_{robot_id}/trajectory", Path, queue_size=1)

            rospy.Timer(rospy.Duration(0.05), self._visualize) # 20fps

    def set_goals(self, all_goals):
        self.all_goals = all_goals
        
    #define a function that checks if at least 80% of the boids are within the goal radius
    def is_goal_reached(self):
        #get the positions of all the boids
        positions = np.array([boid.get_pos() for boid in self.boids])
        #find the distance between each boid and the goal
        distances = np.linalg.norm(positions - self.goal[:2], axis=1)
        #count the number of boids within the goal radius
        num_boids_in_goal = np.sum(distances <= (self.percep_field[0]*self.inter_goal_dist + self.goal[3])) 
        #check if at least 80% of the boids are within the goal radius
        if num_boids_in_goal >= self.arrival_threshold*self.n_boids:
            return True
        else:
            return False
        
    #define a function that does what the previous 8 lines of code do but also generates new goals once the previous goals have been reached
    def assign_goal(self, goal):
        if self.leader_method == 0:
            self.generate_goals(goal)
        elif self.leader_method == 1:
            self.convex_hull(goal)
        if self.leader_type == 1:
            self.compute_goals = False # IF THIS IS FALSE, THE LEADERS WILL BE FIXED
        self.goal = goal
        pass


    def update_goal(self):
        self.current_goal_index += 1
        if self.current_goal_index < len(self.goal_list):
            self.assign_goal(self.goal_list[self.current_goal_index])
        pass


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
        marker.color = ColorRGBA(0.0, 1.0, 0.0, 0.2)
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
        self.map_msg = gridmap   

    @classmethod
    def point_angle_length(cls, p1=[], angle=0, length=1):
        # A segment can be initialized with a Point object, an angle, and a segment length.
        x2 = p1[0] + np.cos(angle) * length
        y2 = p1[1] + np.sin(angle) * length
        return [x2, y2]

    def generate_goals(self, goal=None):
        positions = np.array([boid.get_pos() for boid in self.boids])

        # Find the boids closest to the goal
        distances = np.linalg.norm(positions - goal[:2], axis=1)
        closest_boids = np.argsort(distances)[:self.n_leaders]

        # # Create the goals using point angle length. Each angle is 60 degrees from the previous angle.
        # # The length is the boid's local radius.
        goals = [goal]
        # Convert 60 degrees to radians
        angle = np.pi / 3
        for i in range(self.n_leaders - 1):
            goals.append([*self.point_angle_length(goal[:2], angle, self.percep_field[0] * self.inter_goal_dist/2), goal[2], goal[3]])
            angle += np.pi / 3
        
        #save all the generated goals
        self.set_goals(goals)

        # Find the closest boid to each goal
        for g in goals:
            distances = np.linalg.norm(positions[closest_boids] - g[:2], axis=1)
            closest_boid = closest_boids[np.argmin(distances)]
            # Delete the boid from the closest boids so that it is not assigned to another goal
            if self.leader_type == 0:
                closest_boids = np.delete(closest_boids, np.argmin(distances))
            self.boids[closest_boid].set_goal(g)
            
            #NOTE: JOSEPH: Added the following line
            self.leading_boid_ids.append(closest_boid)

        # # Plot the goals assigned to each boid
        # for boid in self.boids:
        #     if boid == None:
        #         continue
        #     if boid.get_goal() == None:
        #         continue
        #     plt.plot(boid.get_goal()[0], boid.get_goal()[1], 'ro')
        # plt.show()

    def convex_hull(self, goal=None):
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
        goals = [goal]
        # Convert 60 degrees to radians
        angle = np.pi / 3
        for i in range(self.n_leaders - 1):
            goals.append([*self.point_angle_length(goal[:2], angle, self.percep_field[0] * self.inter_goal_dist), goal[2], goal[3]])
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
    
    #write a function to publish the trajectory of each boid as a path
    def publish_path(self):
        #publish the position of each boid as a path
        for i, boid in enumerate(self.boids):
            #create a pose message
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = boid.get_pos()[0]
            pose.pose.position.y = boid.get_pos()[1]
            pose.pose.position.z = 0
            #convert the yaw angle to a euler to quaternion
            quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, boid.get_theta())
            pose.pose.orientation.x = quaternion[0]
            pose.pose.orientation.y = quaternion[1]
            pose.pose.orientation.z = quaternion[2]
            pose.pose.orientation.w = quaternion[3]
            #append the pose to the path
            self.boid_path[i].poses.append(pose)
            #publish the path
            self.path_pub[boid.id].publish(self.boid_path[i])
    
    def publish_boid_trajectory(self):
        for i, boid in enumerate(self.boids):
            boid_pose = boid.get_pose()
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = boid_pose[0]
            pose.pose.position.y = boid_pose[1]
            pose.pose.position.z = 0
            #convert the yaw angle to a euler to quaternion
            quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, boid_pose[2])
            pose.pose.orientation.x = quaternion[0]
            pose.pose.orientation.y = quaternion[1]
            pose.pose.orientation.z = quaternion[2]
            pose.pose.orientation.w = quaternion[3]
        
            #the trajectory is a linestrip marker
            self.boid_traj[i].points.append(pose.pose.position)
            self.traj_pub[i].publish(self.boid_traj[i])
        pass
    

    def _visualize(self, event):
        self.boids_markers.markers = []

        # Extra pos and leader data from boids
        pos_list, leader_list = [], []

        # Ensure that all boids have been created and have a position and velocity before running the algorithm
        if not(self.boids_created):
            return

        for b in self.boids:
            pos_list.append(b.get_pos())
            leader_list.append(0 if b.get_goal() is None else 1)
        
        for i in range(self.n_boids):

            #___________________  create boid markers  ___________________
            marker = Marker(
                header=Header(frame_id="map"),
                type=Marker.SPHERE,
                action=Marker.ADD,
                pose=Pose(Point(*pos_list[i], 0), Quaternion(0, 0, 0, 1)),
                # red for leader, blue for normal
                color=ColorRGBA(leader_list[i], 0, 1-leader_list[i], 1),
                scale=Vector3(0.1, 0.1, 0.1),
                lifetime=rospy.Duration(0),
            )
            marker.id = i
            self.boids_markers.markers.append(marker)

            #___________________  create local neighbordhoods  ___________________
            marker = Marker(
                header=Header(frame_id="map"),
                type=Marker.CYLINDER,
                action=Marker.ADD,
                pose=Pose(Point(*pos_list[i], 0), Quaternion(0, 0, 0, 1)),
                # red for leader, blue for normal
                color=ColorRGBA(0, 1, 0, 0.15),
                scale=Vector3(self.percep_field[0], self.percep_field[0], 0.05),
                lifetime=rospy.Duration(0),
            )
            marker.id = i + self.n_boids
            self.boids_markers.markers.append(marker)
        self.boids_markers_pub.publish(self.boids_markers)

        #___________________   publish goal  ___________________
        marker = Marker(
                header=Header(frame_id="map"),
                type=Marker.CYLINDER,
                action=Marker.ADD,
                pose=Pose(Point(*[self.goal[0], self.goal[1]], 0), Quaternion(0, 0, 0, 1)),
                # red for leader, blue for normal
                color=ColorRGBA(1, 1, 0, 0.4),
                # scale=Vector3(self.goal[-2], self.goal[-2], 0.1),
                scale=Vector3(2*self.percep_field[0]*self.inter_goal_dist, 2*self.percep_field[0]*self.inter_goal_dist, 0.1),
                lifetime=rospy.Duration(0),
            )
        self.goal_pub.publish(marker)

        #___________________  Publish dilated map  ___________________
        if not(self.map_dilated):
            grid_map = np.array(self.map_msg.data).reshape(self.map_msg.info.height, self.map_msg.info.width)
            dilated_obs = self.all_behaviors['_steer_to_avoid'].dilate_obstacles(grid_map, 
                                    dilation_length = round(self.obs_r/self.map_msg.info.resolution))
            dilation = (dilated_obs - grid_map)*0.5
            self.map_msg.data = dilation.flatten().astype(int).tolist()
            self.map_dilated = True 
        self.dilated_obs_pub.publish(self.map_msg)


        #___________________  Publish Trajectories  ___________________
        def update_trajectory(pos_list, robot_id,m=100):
            # Create a new pose stamped with the current position
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = rospy.Time.now()
            pose_stamped.header.frame_id = "map"
            pose_stamped.pose.position = Point(*pos_list[robot_id], 0.0)

            # Update the trajectory for the specified robot
            if robot_id not in self.robot_trajectories:
                self.robot_trajectories[robot_id] = Path()
            self.robot_trajectories[robot_id].poses.append(pose_stamped)

            traj_to_keep = self.truncate_trajectories
            if len(self.robot_trajectories[robot_id].poses) > traj_to_keep:
                self.robot_trajectories[robot_id].poses = self.robot_trajectories[robot_id].poses[-traj_to_keep:]

            self.robot_trajectories[robot_id].header.frame_id = "map"

            self.trajectory_pubs[robot_id].publish(self.robot_trajectories[robot_id])

        [update_trajectory(pos_list, i) for i in range(self.n_boids)]

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
            cmd_vel = b.update(self.boids)
            ## Test obstacle avoidance in isolation
            # cmd_vel = b.test_obstacle_avoidance(self.avoid_obstacles)
            #publish the cmd velocity to the appropriate boids topic
            self.publish_cmd_vel(b, cmd_vel)
        
        #NOTE: I see a possible bug because boids have started moving before the goals are generated, but its not a problem for now.
        # Generate goals (if necessary) for leader boids. Single goal case
        if self.compute_goals and self.boids_created and not self.use_multigoal:
            self.assign_goal(self.goal) #generate and assign the goals to the boids
        #if the multigoal option is selected the boids would be assigned new goals from the goal list once the previous goals have been reached
        if self.compute_goals and self.boids_created and self.use_multigoal:
            # Assign the first goal if no goal has been assigned yet
            if self.current_goal_index == 0:
                self.assign_goal(self.goal_list[0])
            # Check if the current goal is reached
            if self.is_goal_reached():
                #set the goal of all boids to None
                for boid in self.boids:
                    boid.set_goal(None)
                # Update the goal
                self.update_goal()
                
        #publish the goal
        # self.publish_goal_marker(self.goal)
        #publish the subgoals
        self.publish_subgoals(self.all_goals)
        #publish the formation
        if self.boids_created:
            self.publish_formation()
            #publish the path of each boid
            # self.publish_path()
            #publish the trajectory of each boid
            self.publish_boid_trajectory()
        

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
