from utils.Boid import Boid
import random
import numpy as np

class Reynolds:
    def __init__(self):

        # Get parameters
        self.n_boids = 5
        self.weights = [1,
                        1,
                        1]
        self.percep_field = [5, 180]
        
        x = [[random.randint(0, 10), random.randint(0, 10), random.randint(-180, 180), 
             random.randint(0, 10), random.randint(0, 10), random.randint(0, 5)] for robot_id in range(self.n_boids)]


        # list of poses [x,v] for bois in neigborhood. Robot id = list index
        self.boids = [Boid(robot_id,self.weights,self.percep_field, x[robot_id]) for robot_id in range(self.n_boids)]

    def find_neighbors(self, current_boid):
        # Retrieve the position of each boid. 
        neighbors = []
        for i in range(self.n_boids):
            if self.boids[i].get_pos() != current_boid.get_pos():
                distance = np.linalg.norm(np.array(current_boid.get_pos()) - np.array(self.boids[i].get_pos()))
                print("distance: ", distance)
                if distance <= current_boid.local_r:
                    neighbors.append(self.boids[i])
        return neighbors
    

if __name__ == '__main__':
    random.seed(100)
    cl = Reynolds()

    neighbors = cl.find_neighbors(cl.boids[0])
    print("Current: ", cl.boids[0].get_pos())
    print([boids.state for boids in cl.boids])
    print(neighbors)

    cl.boids[0].set_neighbors(neighbors)

    print(cl.boids[0].update())
