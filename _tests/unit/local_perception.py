from utils.Boid import Boid
import random
import numpy as np

# Dummy parameters
n_boids = 5
weights = [1., 1., 1.]

percep_field = [5, 180]

# \left(y-3\right)^{2}=(3^{2}-\left(x-5\right)^{2})\ 
angle = 0
main_boid = (5,3,angle)
# all points in range 5
all_boids = [(4,6,0),(6,6,0),(5,3,angle),(7.5,4.5,0),(7.6,1.5,0),(5,0,0),(2.8,1,0),(2,3,0),(2.6,4.8,0)]
boid_id = 2


neighbors = [Boid(robot_id,weights,percep_field, all_boids[robot_id]+[0,0,0]) 
                                    for robot_id in range(n_boids)]

print(neighbors)