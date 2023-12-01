from p5 import size, background, run
from utils.Boid_testing_visualization import Boid
import random
import numpy as np

# Set the P5 window dimension. 
height, width = Boid.get_canvas()

# Get parameters
n_boids = 100
weights = [1, 1, 1]
percep_field = [50.5, 180] # Radius, angle. 

x = [[random.randint(0, 800), random.randint(0, 800), random.randint(-180, 180), 
        random.randint(0, 10), random.randint(0, 10), random.randint(0, 5)] for robot_id in range(n_boids)]
# list of poses [x,v] for bois in neigborhood. Robot id = list index
boids = [Boid(robot_id,weights,percep_field, x[robot_id]) for robot_id in range(n_boids)]



def setup():
    #this happens just once
    size(width, height) #instead of create_canvas



def draw():
    #this happens every time
    background(30, 30, 47)
    for boid in boids:
        boid.show()
        neighbors = find_neighbors(boid)
        boid.set_neighbors(neighbors)
        boid.update()
        boid.edges()

def find_neighbors(current_boid):
    # Retrieve the position of each boid. 
    neighbors = []
    for i in range(len(boids)):
        if boids[i].get_pos() != current_boid.get_pos():
            distance = np.linalg.norm(np.array(current_boid.get_pos()) - np.array(boids[i].get_pos()))
            if distance <= current_boid.local_r:
                neighbors.append(boids[i])
    return neighbors

run()
