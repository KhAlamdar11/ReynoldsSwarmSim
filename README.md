# MRS Part 1 - Reynold's Rule

## Dependencies:

All dependencies can be installed by:

**Note: Do not do this now! (@Nada, Moses, Joses)**

```
pip install -r requirements.txt
```

## How to Run

1. Launch the simulator

2. Launch the reynold's algorithm

## Tunable parameters

- n_boids: Number of boids. Must be consistent with the number of boids in the simulator.
- w_a: Weight for alignment behaviour.
- w_c: Weight for cohesion behaviour.
- w_s: Weight for seperation behaviour.
- local_r: Radius of perceptive field of a boid. Defines the range of the local neighborhood.
- local_theta: Range of perceptive field of a boid. Defines the field of view of the local neighborhood.

Parameters can be modified from the params.yaml file.

<img src="media/set4_vis/d1.png" alt="testing" height="400" width="400">

## dev notes

- In **update** of boid, you actually don't need the self_pose seperately. The pose of the current boid is contained in the list of all_poses and can be easily extracted using the robot_id variable (list of poses is ordered according to robot_id).

