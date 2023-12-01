﻿# MRS Part 1 - Reynold's Rule

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

