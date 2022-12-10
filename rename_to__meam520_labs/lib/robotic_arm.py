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
    self.xmax   = np.array([ 2.7,  2.0,  14.5,  
                             1.6,  2.0,  7,  
                             2.7,  2.0,  9.5, 
                            -0.0,  2.0,  12,  
                             2.7,  2.0,  14.5, 
                             3.6,  2.0,  19.5,  
                             2.7,  2.0,  19.5])

    self.xmin   = np.array([ -2.7,  -2.0,  -14.5,  
                             -1.6,  -2.0,  -7,  
                             -2.7,  -2.0,  -9.5, 
                             -3.0,  -2.0,  -12,  
                             -2.7,  -2.0,  -14.5, 
                             -0.0,  -2.0,  -19.5,  
                             -2.7,  -2.0,  -19.5])

   
  def x_d(self):
    # Nominal state
    xd_ = np.array([np.pi/2, 0, 0, 0, 0, 0, 0, 0, 0, -np.pi/2, 0, 0, 0, 0, 0, np.pi/2, 0, 0 , np.pi/4, 0, 0])
    return np.zeros(21)


  def u_d(self):
    # Nominal input
    # return np.array([self.m*self.g/2, self.m*self.g/2])
    return np.zeros(7)

  def continuous_time_full_dynamics(self, x, u):
    Ac = np.array([[0, 1, 0],
                   [0, 0, 1],
                   [0, 0, 0]])
    Bc = np.array([[0],[0],[1]])
    
    Acontinuous = block_diag(Ac, Ac, Ac, Ac, Ac, Ac, Ac)
    Bcontinuous = block_diag(Bc, Bc, Bc, Bc, Bc, Bc, Bc)
    xdot = Acontinuous@(x) + Bcontinuous@(u) 
    return xdot

  def discrete_time_linearized_dynamics(self, N, time, tf):
    """
    Model function
    Generates the model matrixes A, B and Dz\n
        h is the sampling period\n
        n is the number of degrees of freedom (3 per joint; 1, 2 or 3 joints)
    """
    # span      = 1
    # step_size = 0.02
    # m         = 7
    # n         = 21                                          #No. of degrees of freedom
    # I         = np.identity(n)
    # time      = np.arange(start=0,stop=span,step=step_size)
    
    n               = 21
    I               = np.identity(n) 
    tsim            = tf - time                    #Simulation Time
    h               = tsim/(N-1)
    # print("h ",h)
    # print("tsim ",tsim)
    # print("tf ", tf)
    
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
    # A_d             = phi
    # B_d             = gamma + (phi - I)@(gamma1)/h
    # Dz              = gamma1/h 

    return phi, gamma, gamma1, h

  def add_initial_state_constraint(self, prog, x, x_current):
    """THIS"""
    # prog.AddBoundingBoxConstraint(x_current, x_current, x[0]) # prog.AddBoundingBoxConstraint(lb, ub, var)
    """ OR """
    prog.AddLinearEqualityConstraint(x[0] - x_current, np.zeros(21))

  def add_final_state_constraint(self, prog, x, x_final):
    prog.AddLinearEqualityConstraint(x[-1] - x_final, np.zeros(21))

  def add_input_saturation_constraint(self, prog, x, u, N):
    for j in range(N-1):
        prog.AddBoundingBoxConstraint((self.umin-self.u_d()), (self.umax-self.u_d()), u[j]) # syntax (lb, ub, vars)
    for j in range(N):    
        prog.AddBoundingBoxConstraint((self.xmin), (self.xmax), x[j]) # syntax (lb, ub, vars)


  def add_dynamics_constraint(self, prog, x, u, N, time, tf):
    # pdb.set_trace()
    phi, gamma, gamma1, h = self.discrete_time_linearized_dynamics(N, time, tf)
    # print(x)
    # print(np.shape(x))
    for k in range(1, N-1):
        # print(k)
        prog.AddLinearEqualityConstraint(x[k]-  phi@(x[k-1]) - (gamma1/h)@(u[k]) - (gamma - gamma1/h)@(u[k-1]), np.zeros(21))


  def add_cost(self, prog, x, x_final, u, N):
    for l in range(0, N-1):
        err = x[l]-x_final
        prog.AddQuadraticCost(err.T@(self.Q)@(err) + u[l].T@(self.R)@(u[l]))
        # prog.AddQuadraticCost((x[l].T@(self.Q)@(x[l]) + u[l].T@(self.R)@(u[l])))
        # raise
    prog.AddQuadraticCost((x[-1]-x_final)@(self.Q)@((x[-1]-x_final)))


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

    
    self.add_cost(prog, x, x_final, u, N)


    # Solve the QP
    solver = OsqpSolver()
    result = solver.Solve(prog)

    u_mpc = np.zeros(7)
    u_mpc = result.GetSolution(u)
    x_mpc = result.GetSolution(x)

    optimal_cost = result.get_optimal_cost()
    # print("u_mpc: ", (u_mpc+(self.u_d())))
    # print("x_mpc: ", (x_mpc+(self.x_d())))
    # print("x_mpc: ", (x_mpc))
    # print("___________________________________")
    # print(" ")

    return (u_mpc[0]+(self.u_d())), optimal_cost