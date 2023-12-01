# -*- coding: utf-8 -*-
"""MRS_project_local_neighborhood

Automatically generated by Colaboratory.

Original file is located at
    https://colab.research.google.com/drive/135EeJVhhraEKWkzaaekQIQER6JlsUGbT
"""

import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy
import random

def visualize_neighborhoods_2d(characters, neighborhoods, neighborhood_distance, field_of_view):
    plt.figure(figsize=(10, 10))

    for character in characters:
        plt.scatter(character[0], character[1], color='blue')

    for idx, character in enumerate(characters):
        for neighbor in neighborhoods[idx]:
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

def find_neighborhood(character, characters, neighborhood_distance, field_of_view):
    neighbors = []

    for other_char in characters:
        if other_char != character:
            dx = other_char[0] - character[0]
            dy = other_char[1] - character[1]
            angle_diff = abs(other_char[2] - character[2])
            #to normalize the angle difference to fall within the range
            angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi

            if abs(angle_diff) < field_of_view/2:
                distance = math.sqrt(dx ** 2 + dy ** 2)
                if distance < neighborhood_distance:
                    neighbors.append(other_char)

    return neighbors




def main():
    num_boids=10
    boids_position =  [[random.uniform(0, 10), random.uniform(0, 10), random.uniform(0, 2 * math.pi)] for _ in range(num_boids)]

    neighborhood_distance = 0.75
    field_of_view = math.pi * (270 / 180)  # Convert 270 degrees to radians

    neighborhoods = []
    for boid in boids_position:
        neighborhood = find_neighborhood(boid, boids_position, neighborhood_distance, field_of_view)
        neighborhoods.append(neighborhood)
        # print(boid,neighborhoods)

    visualize_neighborhoods_2d(boids_position, neighborhoods, neighborhood_distance, field_of_view)

    # Create a list of each boid and its neighbors
    boid_neighbors = {tuple(boids_position[i]): neighborhoods[i] for i in range(len(boids_position))}
    for boid, neighbors in boid_neighbors.items():
        print(f"Boid {boid} has neighbors: {neighbors}")

if __name__ == "__main__":
    main()