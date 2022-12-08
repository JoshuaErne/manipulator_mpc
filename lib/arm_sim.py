#!/usr/bin/env python

import numpy as np
from math import sin, cos, pi
from scipy.integrate import solve_ivp
from lib.robotic_arm import Robot
import matplotlib.pyplot as plt

def simulate_arm(x0, x_final, tf, robotic_arm):
  t0 = 0.0
  dt = 1e-1

  x = [x0]
  u = [np.zeros((7,))]
  t = [t0]
  printvar = False
  xf        = np.array([x_final[0], x_final[3], x_final[6], x_final[9], x_final[12], x_final[15], x_final[18]])
  current_q = np.array([x0[0], x0[3], x0[6], x0[9], x0[12], x0[15], x0[18]])

  error_list = []
  count = 0
  while np.linalg.norm(xf-current_q) > 1e-3 and t[-1] < tf:
    count+=1
    if(printvar and count%10==0):
        print("Error: ", np.linalg.norm(xf-current_q))
        print("Time step: ", t[-1])
        print(" ")


    error_list.append(np.linalg.norm(xf-current_q))
    current_x = x[-1]
    current_q = np.array([current_x[0], current_x[3], current_x[6], current_x[9], current_x[12], current_x[15], current_x[18]])

    current_time        = t[-1]

    current_u_command   = np.zeros(7)
    current_u_command   = robotic_arm.compute_mpc_feedback(current_x, x_final, t[-1], tf)
    current_u_real      = np.clip(current_u_command, robotic_arm.umin, robotic_arm.umax)

    # Autonomous ODE for constant inputs to work with solve_ivp
    def f(t, x):
        return robotic_arm.continuous_time_full_dynamics(current_x, current_u_real)
    sol = solve_ivp(f, (0, dt), current_x, first_step=dt)


    # Record time, state, and inputs
    t.append(t[-1] + dt)
    x.append(sol.y[:, -1])
    u.append(current_u_command)
  

  x = np.array(x)
  u = np.array(u)
  t = np.array(t)
  return x, u, t, error_list

def visualize_error(error_list, t, x_final):
    error_desired = [0]*len(error_list)
    plt.title("MPC Convergence")
    # plt.title("X final : {}".format(x_final))
    plt.xlabel("time")
    plt.ylabel("Error")
    plt.plot(t[:-1], error_desired, "r-")
    plt.plot(t[:-1], error_list, color='blue')
    plt.show()
