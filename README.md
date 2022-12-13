# MPC approaches for collision free manipulator arm control

## Usage:
1) Install dependencies (required only for running this in ROS)-
Packages- First make sure you have [panda_simulator](https://github.com/justagist/panda_simulator/tree/noetic-devel); then clone this repository. Rename the file rename_to_meam520_labs to meam520_labs
2) Run this [file](/rename_to__meam520_labs/lib/final_project.ipynb)

<!-- ![rrt_algo](imgs/rrt_algo.png) -->

## Performance:
<img src=imgs/pandampc.png height="489" width="567" > <p></p>
Figure 1: Trajectory optimization using RRT and MPC

<img src=imgs/convex_set.png height="489" width="567" > <p></p>
Figure 2:  Defining obstacles as convex constraint

<img src=imgs/convergence.png height="489" width="567" > <p></p>
Figure 3: Convergence curve Heirarchial MPC (obstacle avoiding waypoints generated using RRT).

<img src=imgs/convergence_convex_set.png height="489" width="567" > <p></p>
Figure 4: Convergence curve for obstacle avoidance using MPC and convex sets.

<img src=imgs/statistics.png height="453" width="563" > <p></p>
Figure 5: Performance of MPC algorithm.

<img src=imgs/mpctestcases.png height="453" width="563" > <p></p>
Figure 6: Time taken by different control algorithms to manipulate robot (in seconds)

<img src=imgs/Joint_pos.png height="453" width="563" > <p></p>

<img src=imgs/Joint_vel.png height="453" width="563" > <p></p>

<img src=imgs/Joint_accelerations.png height="453" width="563" > <p></p>
