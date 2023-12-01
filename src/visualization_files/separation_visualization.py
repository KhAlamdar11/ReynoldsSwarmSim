from utils.Boid import Boid
import random
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

class Reynolds:
    def __init__(self):
        # Get parameters
        self.n_boids = 5
        self.weights = [1, 1, 1]
        self.percep_field = [1.5, 180]
        
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
                if distance <= current_boid.local_r:
                    neighbors.append(self.boids[i])
        return neighbors
    
    def plot_neighborhood(self):
        figure, axes = plt.subplots()
        scatter_plots = []  # List to store scatter plot objects for boids
        neighbor_plots = []  # List to store scatter plot objects for neighbors
        ids = []  # List to store boid IDs for the legend

        for i, boid in enumerate(self.boids):
            col = (np.random.random(), np.random.random(), np.random.random())
            ids.append(boid.id)

            # Plot the boid and add to scatter plots list
            scatter = axes.scatter(boid.get_pos()[0], boid.get_pos()[1], marker='o', c=np.array([col]))
            scatter_plots.append(scatter)

            # Create and add circle patches (not included in the legend)
            patch = patches.Circle((boid.get_pos()[0], boid.get_pos()[1]), radius=boid.local_r, fill=False)
            axes.add_patch(patch)

            # Plot neighbors
            for n in boid.neighbors:
                neighbor_scatter = axes.scatter(n.get_pos()[0], n.get_pos()[1], c=np.array([col]), marker='*', s=100)
                neighbor_plots.append(neighbor_scatter)

        # Create boid legend
        boid_legend = plt.legend(scatter_plots, ids, loc='upper left', title='Boids')

        # Create neighbor legend (using one sample scatter plot)
        if neighbor_plots:
            neighbor_legend = plt.legend([*neighbor_plots], ['Neighbors'], loc='upper right', markerfirst=False)

        # Add the boid legend back after adding the neighbor legend
        axes.add_artist(boid_legend)

        plt.show()

    def find_all_neighbors(self):
        for boid in self.boids:
            neighbors = cl.find_neighbors(boid)
            boid.set_neighbors(neighbors)


if __name__ == '__main__':
    random.seed(1200)
    cl = Reynolds()

    cl.find_all_neighbors()
    cl.plot_neighborhood()
