#!/usr/bin/env python

"""
Simulate Franka Panda Arm
"""

import numpy as np
from math import sin, cos, pi
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
import importlib

from lib.arm_sim import *
from lib.robotic_arm import *

"""
Load in the animation function
"""
# import create_animation
# importlib.reload(create_animation)
# from create_animation import create_animation

def run_to_position(x_start, x_goal):
    # Weights of LQR cost
    weight_R = 1
    weight_Q = 10

    #Q and R are tunable parameters
    R = weight_R*np.eye(7)
    Q = weight_Q*np.eye(21)

    for i in [1,2,4,5,7,8,10,11,13,14,16,17,19,20]:
        Q[i,i] = 0
    Q = np.eye(21)
    Qf = Q

    # End time of the simulation
    tf = 30

    # Construct our quadrotor controller 
    robotic_arm = Robot(Q, R, Qf)

    x0          = np.zeros(21)
    x_final     = np.zeros(21)

    x0[0]       = x_start[0]
    x0[3]       = x_start[1]
    x0[6]       = x_start[2]
    x0[9]       = x_start[3]
    x0[12]      = x_start[4]
    x0[15]      = x_start[5]
    x0[18]      = x_start[6]

    x_final[0]  = x_goal[0]
    x_final[3]  = x_goal[1]
    x_final[6]  = x_goal[2]
    x_final[9]  = x_goal[3]
    x_final[12] = x_goal[4]
    x_final[15] = x_goal[5]
    x_final[18] = x_goal[6]

    x, u, t, loss, optimal_cost = simulate_arm(x0, x_final, tf, robotic_arm)

    return x, u, t, loss, optimal_cost

