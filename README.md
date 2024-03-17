
<!-- ### UNIVERSITY OF ZAGREB  -->

# Controlling a Swarm of Robots in a Simulator Stage using Reynolds’ Rules
[![ROS](https://img.shields.io/badge/ROS-Noetic%20or%20later-blue.svg)](http://wiki.ros.org/ROS/Installation)
[![Python](https://img.shields.io/badge/Python-3.7%20or%20later-blue.svg)](https://www.python.org/downloads/)
[![Stage Simulator](https://img.shields.io/badge/Simulator-Stage-orange.svg)](http://wiki.ros.org/stage_ros)
[![Project Website](https://img.shields.io/badge/Website-green.svg)](https://adeolajoseph.github.io/multirobotswarm/)
[![Paper](https://img.shields.io/badge/Paper-purple.svg)](https://adeolajoseph.github.io/multirobotswarm/assets/MRS_project1_report.pdf)

This project focuses on implementing and evaluating Craig Reynolds’ behavioral rules—separation, alignment, and cohesion within a Robot Operating System (ROS) framework and the Stage simulator. The project aims to enhance the capabilities of robotic swarms by adding Navigation and Obstacle Avoidance behaviors to allow the swarm to move towards specific points and stop, while Obstacle Avoidance enables them to detect and evade obstacles, thus preventing collisions. The robots are tested in various scenarios to assess their collective behavior, adaptability, and robustness. 


<table>
  <tr>
    <td>
      <img src="media/map_b1.gif" alt="Two Lines" width="350">
      <!-- <p>Hexagon Formation with Steer-to-avoid</p> -->
    </td>
    <td>
      <img src="media/map_a1.gif" alt="Three Lines" width="350">
      <!-- <p>Rectangle Formation with Steer-to-avoid</p> -->
    </td>
  </tr>
</table>


Note: This project combines theoretical concepts with practical applications, shedding light on the dynamics of robotic swarms and their potential in simulated environments. Video results are provided to illustrate the findings. [here ](https://www.youtube.com/playlist?list=PLEhDw-EN_WqGiUwWmusO_4fW-CWt7HmLE)and [here.](https://www.youtube.com/playlist?list=PL_eKUkXsvo0Y0a8hcg0ggh9Zck--EYpBf)


## Table of Contents

- [Installation](#installation)
- [How to Run](#how-to-run)
- [Configuration Guide](#configuration-guide)
- [Customization](#customization)
- [Visualizing Results](#visualizing-results)
- [Contributing](#contributing)
- [License](#license)
- [Authors](#authors)



## Installation

1. **Dependencies**: Ensure ROS (Robot Operating System) and Python 3 are installed on your system.
2. **Clone the Repository**: Clone this repository into your ROS workspace's `src` directory.
   ```bash
   cd ~/catkin_ws/src
   git clone git@github.com:KhAlamdar11/mrs-r1.git
   ```
3. **Build the Package**: From the root of your ROS workspace, build the package using `catkin_make`.
   ```bash
   cd ~/catkin_ws
   catkin_make
   ```
4. **Source the Workspace**: Source your ROS workspace to make the package available.
   ```bash
   source devel/setup.bash
   ```


## How To Run

To run a simulation, use the provided launch files. For instance, to start a simulation with the default configuration:

1. Go to the `stage_simulator` launch yaml file and change the number of robots to the desired value.

2. Go to the `params.yaml` file and change the number of boids (i.e., n) to the same value as the above.

3. Spawn the robots in the Stage Simulator:
    ```bash
    rosrun sphero_stage start.py
    ```
4. The robots will be spawned in Stage in the predefined formation (as defined in the launch yaml file of the simulator).

5. Launch the formation control node:
    ```bash
    roslaunch mrs-r1 part1.launch

<!-- ## Directory Structure:

 - TODO -->
<!-- - ROS script: src/reynolds.py

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
    
    Helper functions -->

<!-- ![](media/system_arc.png) -->




## Configuration Guide

The `params.yaml` file serves as the primary means to customize your simulation's settings. This section provides an in-depth look at each parameter within the configuration file, ensuring you can tailor the package to your specific needs.


### General Structure

The configuration file is organized into several sections, each pertaining to different aspects of the simulation:

- **Visualization**: Configure visual output and trajectory tracking for the simulation.
- **Number of Boids**: Defines the total number of boids (agents) in the simulation.
- **Behavior Weights**: Adjust the weights for various boid behaviors including alignment, cohesion, separation, arrival, and steer-to-avoid.
- **Neighborhood Parameters**: Sets the parameters defining each boid's perception of its neighbors.
- **Goal Parameters**: Configure the primary goal and optional multiple goal settings.
- **Kinematic Parameters**: Sets the maximum speed and acceleration for the boids.
- **Leadership Configuration**: Determines the selection and behavior of leaders within the swarm.
- **Prioritized Acceleration**: Enables and configures prioritized acceleration to manage multiple behaviors.
- **Obstacle Parameters**: Configure obstacle detection and avoidance settings.
- **Arrival-specific Parameters**: Customizes behavior when approaching the goal.


### Visualization

Configure visual output and trajectory tracking for the simulation.

- **Visualize via RViz**: Enables real-time visualization in RViz.
  - `visualize`: Set to `True` to enable visualization.
- **Truncate Trajectories**: Limits the length of trajectory history for display.
  - `truncate_trajectories`: Maximum number of points to display. Set to `10000000` for virtually no limit.

```yaml
visualize: True
truncate_trajectories: 10000000
```

### Number of Boids

Defines the total number of boids (agents) in the simulation.

- **n_boids**: Total number of boids participating.
  ```yaml
  n_boids: 10
  ```

### Behavior Weights

Adjust the weights for various boid behaviors including alignment, cohesion, separation, arrival, and steer-to-avoid.

- **Alignment** (`w_a`): Adjusts alignment with nearby boids.
- **Cohesion** (`w_c`): Pulls boid towards the center of mass of neighbors.
- **Separation** (`w_s`): Keeps boid away from close neighbors to avoid crowding.
- **Arrival** (`w_arrival`): Controls the effort to reach the goal position.
- **Steer-to-Avoid** (`w_steer_to_avoid`): Influences avoidance maneuvers around obstacles.

```yaml
w_a: &w_a 0.1
w_c: &w_c 0.35
w_s: &w_s 0.2
w_arrival: &w_arrival 0.1
w_steer_to_avoid: &w_steer_to_avoid 0.85
```

### Neighborhood Parameters

Sets the parameters defining each boid's perception of its neighbors.

- **Local Radius** (`local_r`): The distance within which other boids are considered neighbors.
- **Perception Angle** (`local_theta`): The field of view angle for detecting neighbors.

```yaml
local_r: 1.9
local_theta: 360.0
```

### Goal Parameters

Configure the primary goal and optional multiple goal settings.

- **Goal Coordinates** (`goal_x`, `goal_y`): Sets the target location for the swarm.
- **Goal Radius** and **Tolerance**: Defines the goal area and the tolerance for reaching the goal.
- **Inter Goal Distance** (`inter_goal_dist`): Specifies the ratio of distance between sequential goals for multi-goal paths.
- **Use Multiple Goals** (`use_multigoal`): Enables or disables the use of a series of goals.
- **Arrival Threshold** (`arrival_threshold`): The proportion of boids required to reach a goal before proceeding to the next.
- **Goal List** (`goal_list`): A list of goals defined by coordinates, radius, and tolerance.

```yaml
goal_x: 4.0
goal_y: 4.0
goal_radius: 0.5
goal_tolerance: 0.02
inter_goal_dist: 0.4
use_multigoal: False
arrival_threshold: 0.9
goal_list: 
  - [3.0, 2.0, 0.5, 0.02]
  - [-3.0, -4.0, 0.5, 0.02]
  - [4.0, -4.0, 0.5, 0.02]
  - [3.0, 4.0, 0.5, 0.02]
  - [-3.0, 4.0, 0.5, 0.02]
```

### Kinematic Parameters

Sets the maximum speed and acceleration for the boids.

- **Max Speed** (`max_speed`): The highest speed a boid can achieve.
- **Max Acceleration** (`max_acc`): The maximum rate of change of speed.

```yaml
max_speed: 3.3
max_acc: 2.2
```

### Leadership Configuration

Determines the selection and behavior of leaders within the swarm.

- **Number of Leaders** (`n_leaders`): Specifies how many leaders are in the swarm.
- **Leader Method** (`leader_method`): Chooses the method for leader selection.
- **Leader Type** (`leader_type`): Determines if leaders are dynamic or fixed.

```yaml
n_leaders: 5 # Should not be greater than 7
leader_method: 0 # 0: Closest to goal, 1: ConvexHull
leader_type: 0 # 0: Dynamic, 1: Fixed
```

### Prioritized Acceleration

Enables and configures prioritized acceleration to manage multiple behaviors.

- **Use Prioritized Acceleration** (`use_prioritized_acc`): Toggles prioritized acceleration on or off.
- **Priority List** (`

priority_list`): Orders behaviors by priority with their associated weights.

```yaml
use_prioritized_acc: True
priority_list: 
  - '_arrival': *w_arrival
  - '_separation': *w_s
  - '_alignment': *w_a
  - '_cohesion': *w_c
```

### Obstacle Parameters

Configure obstacle detection and avoidance settings.

- **Obstacle Radius** (`obs_r`): Sets the detection radius for obstacles.
- **Step Angle** (`step_angle`): The angle increment for obstacle scanning.
- **Max Steering Angle** (`max_steering_angle`): Limits the steering angle for avoidance maneuvers.

```yaml
obs_r:  0.8
step_angle : 0.174533
max_steering_angle : 6.28
```

### Arrival-specific Parameters

Customizes behavior when approaching the goal.

- **Hard Arrival** (`hard_arrival`): When enabled, boids prioritize reaching the goal over all other behaviors.

```yaml
hard_arrival: True
```

## Customizing Your Simulation

This package is designed to be highly configurable. To customize your simulation, edit the `params.yaml` file, adjusting the parameters as needed based on the descriptions provided above. Experimenting with different configurations can help you understand the impact of various behaviors and parameters on the multi-robot system's overall performance.


## Visualizing Results

Results can be visualized in real-time using RViz. Ensure the `visualize` parameter is set to `True` and use the provided RViz configuration to observe the boids' behaviors, trajectories, and interactions with obstacles.


## Contributing

We welcome contributions to the package. For more information on how to contribute, please reach out to any of the authors.
## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Authors:

- [Moses Ebere](https://github.com/MosesEbere)
- [Joseph Adeola](https://github.com/AdeolaJoseph)
- [Khawaja Alamdar](https://github.com/KhAlamdar11)
- [Nada Abbas](https://github.com/NadaAbbas444)

<!-- ## Tunable parameters

- n_boids: Number of boids. Must be consistent with the number of boids in the simulator.
- w_a: Weight for alignment behaviour.
- w_c: Weight for cohesion behaviour.
- w_s: Weight for seperation behaviour.
- local_r: Radius of perceptive field of a boid. Defines the range of the local neighborhood.
- local_theta: Range of perceptive field of a boid. Defines the field of view of the local neighborhood.

Parameters can be modified in the params.yaml file.

<!-- <img src="media/set4_vis/d1.png" alt="testing" height="400" width="400"> -->

<!-- ## dev notes

- You can use get_{functions} to directly extract parts of state from the state vector for simplicity
- Two simulation modes: 
    - **Matplotlib**: The boids are visualized in a matplotlib window. This is useful for debugging and testing. See the `animation_test.py` script for an example.
    - **P5**: The boids are visualized in the P5 simulator. This is the actual simulation mode. See the `p5_test.py` script for an example. This script is a good starting point for testing your code. You can use it to test your code before integrating it with the ROS script.
- You can use the 'separation_visualization.py' script to visualize the separation behaviour. 
 --> 
