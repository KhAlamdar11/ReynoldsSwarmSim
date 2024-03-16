
### UNIVERSITY OF ZAGREB 

# Controlling a Swarm of Robots in a Simulator Stage using Reynolds’ Rules

### Joseph Adeola, Khawaja Ghulam Alamdar, Moses Ebere, Nada Elsayed Abbas

![ref1] ✦ ![ref1]

# MRS Part 1 - Reynold's Rule

1  **MOTIVATION**

This project focuses on implementing and evaluating Craig Reynolds’ behavioral rules—separation, alignment, and cohesion—within a Robot Operating System (ROS) framework and the Stage simulator. The project aims to en- hance the capabilities of robotic swarms by adding Naviga- tion and Obstacle Avoidance behaviors to allow the swarm to move towards specific points and stop, while Obstacle Avoidance enables them to detect and evade obstacles, thus preventing collisions. The robots are tested in various scenarios to assess their collective behavior, adaptability, and robustness. This project combines theoretical concepts with practical application, shedding light on the dynamics of robotic swarms and their potential in simulated environ- ments. Video results are provided to illustrate the findings. [here ](https://www.youtube.com/playlist?list=PLEhDw-EN_WqGiUwWmusO_4fW-CWt7HmLE)and [here.](https://www.youtube.com/playlist?list=PL_eKUkXsvo0Y0a8hcg0ggh9Zck--EYpBf)



# Implementation & Code

## recent changes made by nada :
- state_tf - the transform function file 
- boid2 - Boid_testing_visualization
- all visualization file are inside visualization_files folder inside src
- results folder to add the figures & plots 

## Dependencies:

All dependencies can be installed by:

**Note: Do not do this now!**

```
pip install -r requirements.txt
```

## How to Run

1. Launch the simulator

2. Launch the reynold's algorithm

```
roslaunch mrs-r1 part1.launch
```

## Code Structure:

- ROS script: src/reynolds.py

    Main ros script to:
    - subcribe to all the boids
    - creates all boids
    - updates their states with callbacks
    - calls the update of each boid with a set frequency (run function) **TODO**
    - publishes command velocities to each boid **TODO**

- Boid algorithm: src/utils/Boid.py

    Boid class to do all the processing:
    - state defined as [x,y,theta,v_x,v_y,w]
    - getter functions to extract the states and get what you want
    - **TODO**: algorithmic implementations

- Utility functions: src/utils/utils.py
    
    Helper functions

![](media/system_arc.png)

## Tunable parameters

- n_boids: Number of boids. Must be consistent with the number of boids in the simulator.
- w_a: Weight for alignment behaviour.
- w_c: Weight for cohesion behaviour.
- w_s: Weight for seperation behaviour.
- local_r: Radius of perceptive field of a boid. Defines the range of the local neighborhood.
- local_theta: Range of perceptive field of a boid. Defines the field of view of the local neighborhood.

Parameters can be modified in the params.yaml file.

<!-- <img src="media/set4_vis/d1.png" alt="testing" height="400" width="400"> -->

## dev notes

- You can use get_{functions} to directly extract parts of state from the state vector for simplicity
- Two simulation modes: 
    - **Matplotlib**: The boids are visualized in a matplotlib window. This is useful for debugging and testing. See the `animation_test.py` script for an example.
    - **P5**: The boids are visualized in the P5 simulator. This is the actual simulation mode. See the `p5_test.py` script for an example. This script is a good starting point for testing your code. You can use it to test your code before integrating it with the ROS script.
- You can use the 'separation_visualization.py' script to visualize the separation behaviour. 

