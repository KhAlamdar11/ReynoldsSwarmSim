import sys, os
import random
import numpy as np
import matplotlib.pyplot as plt
import math

parent_dir = os.path.dirname('/home/alamdar11/ROS_WS/mrs_ws/src/mrs-r1/src/')
sys.path.append(parent_dir)

print("Current Working Directory:", os.getcwd())

from utils.Boid import Boid
from reynolds import Reynolds

def visualize_neighborhoods_2d(character, characters, neighborhood, neighborhood_distance, field_of_view):

    print(field_of_view)

    for c in characters:
        plt.scatter(c[0], c[1], color='blue')

    plt.scatter(character[0], character[1], color='red')

    for neighbor in neighborhood:
        plt.plot([character[0], neighbor[0]], [character[1], neighbor[1]], color='red', linestyle='dotted')

    # Plot field of view lines
    for angle in [character[2] - field_of_view / 2, character[2] + field_of_view / 2]:
        x_end = character[0] + neighborhood_distance * math.cos(angle)
        y_end = character[1] + neighborhood_distance * math.sin(angle)
        plt.plot([character[0], x_end], [character[1], y_end], color='gray')

    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    plt.title('Neighborhood Visualization in 2D with Field of View')
    plt.grid(True)
    plt.axis('equal')
    plt.show()

# Dummy parameters
weights = [1., 1., 1.]

percep_field = [3.5, 180.0]

# \left(y-3\right)^{2}=(3^{2}-\left(x-5\right)^{2})\ 
angle = 0
# all points in range 5
boids_xy = [[4,6,0],[6,6,0],[5,3,angle],[7.5,4.5,0],[7.6,1.5,0],[5,0,0],[2.8,1,0],[2,3,0],[2.6,4.8,0]]
boid_id = 2
n_boids = len(boids_xy)

boids = [Boid(robot_id,weights,percep_field, boids_xy[robot_id]+[0,0,0]) 
                                    for robot_id in range(n_boids)]
boid = boids[2]

# print('boids: ', [b.get_pose() for b in boids])

def find_neighbors(boid):
        # Retrieve the position of each boid. 
        neighbors = []
        for i in range(n_boids):
            # Skip if boid is self or if boid has not been created yet
            if boids[i] == None:
                continue
            if boids[i] != boid :
                dx = boids[i].get_pos()[0] - boid.get_pos()[0]
                dy = boids[i].get_pos()[1] - boid.get_pos()[1]
                # np.degrees only used for visualization not final code!!
                alpha= np.degrees(math.atan2(dy,dx))
                angle_diff = abs(alpha - boid.get_theta())
                print(angle_diff)
                # angle_diff = (angle_diff + np.pi) % (2 * np.pi) - np.pi
                if abs(angle_diff) < boid.local_theta / 2:
                    distance = np.linalg.norm(np.array(boid.get_pos()) - np.array(boids[i].get_pos()))
                    if distance <= boid.local_r:
                        neighbors.append(boids[i])
        print("here",neighbors)
        return neighbors

neighbors = find_neighbors(boid)

character = boid.get_pose()
characters = [b.get_pose() for b in boids]
neighs = [b.get_pose() for b in neighbors]

print('all boids:', len(characters))
print('neighbor boids:', len(neighs))

# visualize_neighborhoods_2d(character, characters, neighs, percep_field[0], percep_field[1])

# visualize_neighborhoods_2d(characters, boid, neighborhood, neighborhood_distance, field_of_view)

