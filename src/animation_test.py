import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from functools import partial
import matplotlib.patches as patches
import random
from utils.Boid2 import Boid

# Get parameters
n_boids = 100
weights = [1, 1, 1]
percep_field = [50.5, 180] # Radius, angle. 

x = [[random.randint(0, 800), random.randint(0, 800), random.randint(-180, 180), 
        random.randint(0, 10), random.randint(0, 10), random.randint(0, 5)] for robot_id in range(n_boids)]
# list of poses [x,v] for bois in neigborhood. Robot id = list index
boids = [Boid(robot_id,weights,percep_field, x[robot_id]) for robot_id in range(n_boids)]



def find_neighbors(current_boid):
    # Retrieve the position of each boid. 
    neighbors = []
    for i in range(len(boids)):
        if boids[i].get_pos() != current_boid.get_pos():
            distance = np.linalg.norm(np.array(current_boid.get_pos()) - np.array(boids[i].get_pos()))
            if distance <= current_boid.local_r:
                neighbors.append(boids[i])
    return neighbors

def boid_update(boids):
    boid_positions = []
    for boid in boids:
        neighbors = find_neighbors(boid)
        boid.set_neighbors(neighbors)
        boid.update()
        boid_positions.append(boid.get_pos())
        boid.edges()
    out = np.array(boid_positions)
    return out


fig = plt.figure()
ax = fig.add_subplot(111, autoscale_on=False)
line1, = ax.plot([], [], 'ro')

# Things to keep track of in the animation.
points, = ax.plot([], [], 'rx') # Robot positions. 
patch = patches.Circle((0, 0), radius=50.5, fill=False)
local_radius = ax.add_patch(patch) # Local radius as a circular patch. 
neighbors =  ax.plot([], [], marker='*')


def init():
    points.set_data([], [])
    ax.set_xlim(0, 1000)
    ax.set_ylim(0, 1000)
    return points,

def update(frame, boid, point):
    out = boid_update(boid)
    x, y = out[:,0], out[:,1]
    point.set_data(x.tolist(), y.tolist())
    return point,


ani = FuncAnimation(
    fig, partial(update, boid=boids, point=points),
    frames=np.linspace(0, 2*np.pi, 128),
    init_func=init, blit=True, repeat=True)

plt.show()