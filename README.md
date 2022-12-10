# Obstacle avoidance with MPC

## Usage:
1) Install dependencies (required only for running this in ROS)-
Packages- First make sure you have [panda_simulator](https://github.com/justagist/panda_simulator/tree/noetic-devel); then clone this repository. 
2) Run this [file](/lib/convex_set/final_project.ipynb)

<!-- ![rrt_algo](imgs/rrt_algo.png) -->


<img src=imgs/convergence.png height="489" width="567" > <p></p>
Figure 1: Convergence curve Heirarchial MPC (obstacle avoiding waypoints generated using RRT).


<img src=imgs/convergence_convex_set.png height="489" width="567" > <p></p>
Figure 2: Convergence curve for obstacle avoidance using MPC and convex sets.