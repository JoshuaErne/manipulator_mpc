import numpy as np
import matplotlib.pyplot as plt
from numpy.linalg import inv
from numpy.linalg import cholesky
from math import sin, cos
import math
from scipy.interpolate import interp1d
from scipy.integrate import ode
from scipy.integrate import solve_ivp
from scipy.linalg import expm
from scipy.linalg import solve_continuous_are
from scipy.linalg import block_diag

from pydrake.solvers import mathematicalprogram as mp
from pydrake.solvers.osqp import OsqpSolver
from pydrake.solvers.snopt import SnoptSolver
from pydrake.solvers.mathematicalprogram import MathematicalProgram, Solve
import pydrake.symbolic as sym

from pydrake.all import MonomialBasis, OddDegreeMonomialBasis, Variables

import pdb

class Robot(object):
  def __init__(self, Q, R, Qf):
    # Input limits
    self.umin   = np.array([-7500, -3750, -5000, -6250, -7500, -10000, -10000])
    self.umax   = np.array([ 7500,  3750,  5000,  6250,  7500,  10000,  10000])
    self.Q      = Q
    self.R      = R
    self.Qf     = Qf

    #Limits are on ----------joints,  velocities, accelerations-----------------
    self.xmax   = np.array([ 2.8973,  2.1750,  15,  
                             1.7628,  2.1750,  7.5,  
                             2.8973,  2.1750,  10, 
                             -0.0698, 2.1750,  12.5,  
                             2.8973,  2.6100,  15, 
                             3.7525,  2.6100,  20,  
                             2.8973,  2.6100,  20])
    #Limits are on ----------joints,  velocities, accelerations-----------------
    self.xmin   = np.array([ -2.8973,  -2.1750,  -15,  
                             -1.7628,  -2.1750,  -7.5,  
                             -2.8973,  -2.1750,  -10, 
                             -3.0718,  -2.1750,  -12.5,  
                             -2.8973,  -2.6100,  -15, 
                             -0.0175,  -2.6100,  -20,  
                             -2.8973,  -2.6100,  -20])

    self.xl = []
    self.xu = []

  def new_constraints(self, new_constraints_param):
    self.xu.append(np.array([np.abs(new_constraints_param[0]),  2.1750,  15,  
                             np.abs(new_constraints_param[1]), 2.1750,  7.5,  
                             np.abs(new_constraints_param[2]), 2.1750,  10, 
                             np.abs(new_constraints_param[3]), 2.1750,  12.5,  
                             np.abs(new_constraints_param[4]), 2.6100,  15, 
                             np.abs(new_constraints_param[5]), 2.6100,  20,  
                             np.abs(new_constraints_param[6]), 2.6100,  20]))


    self.xl.append(np.array([(-1) * np.abs(new_constraints_param[0]), - 2.1750, - 15,  
                             (-1) * np.abs(new_constraints_param[1]), -2.1750,  -7.5,  
                             (-1) * np.abs(new_constraints_param[2]), -2.1750,  -10, 
                             (-1) * np.abs(new_constraints_param[3]), -2.1750,  -12.5,  
                             (-1) * np.abs(new_constraints_param[4]), -2.6100,  -15, 
                             (-1) * np.abs(new_constraints_param[5]), -2.6100,  -20,  
                             (-1) * np.abs(new_constraints_param[6]), -2.6100,  -20]))


  def add_obs_constraint(self, prog, x, q_avoid_list, N, side = 4, center = 0.75):

        limit_val = np.arctan2(side, (center-(side/2))) - np.arctan2((side/2), (center/2)-(center-(side/2))) # Obstacle is cube of side 0.25 and 0.5m away from robot in XY plane. This helps maintain a buffer and cover all possible configurations in the 7 dimensional hyperplane.
        for k in range(len(q_avoid_list)):
            for horizon in range(N):
                q_in_x = np.array([x[horizon][0], x[horizon][3], x[horizon][6], x[horizon][9], x[horizon][12], x[horizon][15], x[horizon][18]])
                prog.AddLinearConstraint(np.mean(q_in_x-np.array(q_avoid_list))<=limit_val) # converge to a convex pocket near the obstacle


  def x_d(self):
    # Nominal state
    return np.zeros(21)


  def u_d(self):
    # Nominal input
    return np.zeros(7)

  def continuous_time_full_dynamics(self, x, u):
    Ac = np.array([[0, 1, 0],
                   [0, 0, 1],
                   [0, 0, 0]])
    
    Bc = np.array([[0],[0],[1]])
    
    #Expand for 7D
    Acontinuous = block_diag(Ac, Ac, Ac, Ac, Ac, Ac, Ac)
    Bcontinuous = block_diag(Bc, Bc, Bc, Bc, Bc, Bc, Bc)
    xdot = Acontinuous@(x) + Bcontinuous@(u) 
    return xdot


  def discrete_time_linearized_dynamics(self, N, time, tf):
    n               = 21
    I               = np.identity(n) 
    tsim            = tf - time
    h               = tsim/(N-1)
    
    phi_tilde       = np.array([[1, h, h**2/2], 
                                [0, 1,      h], 
                                [0, 0,      1]])
    gamma_tilde     = np.array([[h**3/6], 
                               [h**2/2], 
                               [h]])
    gamma1_tilde    = np.array([[h**4/24], 
                                [h**3/6], 
                                [h**2/2]])

    phi             = block_diag(phi_tilde,     phi_tilde,      phi_tilde,      phi_tilde,      phi_tilde,      phi_tilde,      phi_tilde)
    gamma1          = block_diag(gamma1_tilde,  gamma1_tilde,   gamma1_tilde,   gamma1_tilde,   gamma1_tilde,   gamma1_tilde,   gamma1_tilde)
    gamma           = block_diag(gamma_tilde,   gamma_tilde,    gamma_tilde,    gamma_tilde,    gamma_tilde,    gamma_tilde,    gamma_tilde)

    return phi, gamma, gamma1, h

  def add_initial_state_constraint(self, prog, x, x_current):
    prog.AddLinearEqualityConstraint(x[0] - x_current, np.zeros(21))

  def add_final_state_constraint(self, prog, x, x_final):
    prog.AddLinearEqualityConstraint(x[-1] - x_final, np.zeros(21))

  def add_input_saturation_constraint(self, prog, x, u, N):
    for j in range(N-1):
        prog.AddBoundingBoxConstraint((self.umin-self.u_d()), (self.umax-self.u_d()), u[j]) # syntax (lb, ub, vars)
    for j in range(N):    
        prog.AddBoundingBoxConstraint((self.xmin), (self.xmax), x[j]) # syntax (lb, ub, vars)


  def add_dynamics_constraint(self, prog, x, u, N, time, tf):
    phi, gamma, gamma1, h = self.discrete_time_linearized_dynamics(N, time, tf)
    for k in range(1, N-1):
        prog.AddLinearEqualityConstraint(x[k]-  phi@(x[k-1]) - (gamma1/h)@(u[k]) - (gamma - gamma1/h)@(u[k-1]), np.zeros(21))


  def add_cost(self, prog, x, x_final, u, N):
    for l in range(0, N-1):
        err = x[l]-x_final
        prog.AddQuadraticCost(err.T@(self.Q)@(err) + u[l].T@(self.R)@(u[l]))
    prog.AddQuadraticCost((x[-1]-x_final)@(self.Q)@((x[-1]-x_final)))

  # def compute_mpc_feedback(self, x_current, x_final, time, qconvex, tf):
  def compute_mpc_feedback(self, x_current, x_final, time, tf):
    '''
    This function computes the MPC controller input u
    '''
    # Parameters for the QP
    N = 15

    # Initialize mathematical program and decalre decision variables
    prog = MathematicalProgram()
    x = np.zeros((N, 21), dtype="object")
    for i in range(N):
      x[i] = prog.NewContinuousVariables(21, "x_" + str(i))
    
    u = np.zeros((N-1, 7), dtype="object")
    for i in range(N-1):
      u[i] = prog.NewContinuousVariables(7, "u_" + str(i))


    # Add constraints and cost
    self.add_initial_state_constraint(prog, x, x_current)
    self.add_final_state_constraint(prog, x, x_final)
    self.add_input_saturation_constraint(prog, x, u, N)
    self.add_dynamics_constraint(prog, x, u, N, time, tf)

    ##################### Uncomment this function to test for MPC based on Convex sub-regions #############################
    #Add obstacle Cost
    # self.add_obs_constraint(prog, x, qconvex, N)

    
    self.add_cost(prog, x, x_final, u, N)


    # Solve the QP
    solver = OsqpSolver()
    result = solver.Solve(prog)

    u_mpc = np.zeros(7)
    u_mpc = result.GetSolution(u)
    x_mpc = result.GetSolution(x)

    optimal_cost = result.get_optimal_cost()

    return (u_mpc[0]+(self.u_d())), optimal_cost