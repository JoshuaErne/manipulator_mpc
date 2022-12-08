#!/usr/bin/env python

import sys
import rospy
import numpy as np
from math import pi
from time import perf_counter
import geometry_msgs
from visualization_msgs.msg import Marker

import os
import pickle
import time

from core.interfaces import ArmController
from lib.rrt import rrt
from lib.mpc import run_to_position

from lib.loadmap import loadmap
from copy import deepcopy


starts = [np.array([-pi/4, 0, 0, -pi/2, 0, pi/2, pi/4]),
          np.array([0, 0.4, 0, -2.5, 0, 2.7, 0.707]),
          np.array([-pi/4, 0, 0, -pi/2, 0, pi/2, pi/4]),
          np.array([0, -1, 0, -2, 0, 1.57, 0])]
goals = [np.array([pi/4, pi/5, 0, -pi/3, 0, pi/2, pi/4]),
         np.array([1.9, 1.57, -1.57, -1.57, 1.57, 1.57, 0.707]),
         np.array([pi/4, pi/4, 0, -pi/3, 0, pi/2, pi/4]),
         np.array([1.9, 1.57, -1.57, -1.57, 1.57, 1.57, 0.707])]
mapNames = ["map1",
            "map2",
            "map3",
            "map4",
            "emptyMap"]


if __name__ == "__main__":

    if len(sys.argv) < 2:
        print("usage:\n\tpython rrt_demo.py 1\n\tpython rrt_demo.py 2\n\tpython rrt_demo.py 3 ...")
        exit()

    rospy.init_node('RRT')

    print(sys.argv[1])
    arm = ArmController()
    index = int(sys.argv[1])-1
    
    print("Running test "+sys.argv[1])
    print("Moving to Start Position")
    
    arm.move_to_position(starts[index])
    map_struct = loadmap("../../maps/"+mapNames[index] + ".txt")
    
    print("Map = "+ mapNames[index])
    print("Starting to plan")
    
    start = perf_counter()
    path = rrt(deepcopy(map_struct), deepcopy(starts[index]), deepcopy(goals[index]))
    stop = perf_counter()
    dt = stop - start
    print('--------Implementing RRT------------')

    print("RRT took {time:2.2f} sec. Path is.".format(time=dt))
    print(np.round(path,4))
    
    # input("Press Enter to Send Path to Arm")
    # print(len(path))

    print('-------Implementing MPC--------------')
    
    saved_state = {'q':[], 'qdot':[], 'qddot':[]}

    for idx1 in range(len(path)-1):
        p1 = path[idx1]
        p2 = path[idx1+1]
        q_joints, Control_input, time_stamp, loss_vals = run_to_position(p1, p2)

        for idx2 in range(len(q_joints)):
            # if idx2%25 == 0:
                q = np.array([q_joints[idx2][0],
                            q_joints[idx2][3],
                            q_joints[idx2][6],
                            q_joints[idx2][9],
                            q_joints[idx2][12],
                            q_joints[idx2][15],
                            q_joints[idx2][18]])

                q_dot = np.array([q_joints[idx2][1],
                                    q_joints[idx2][4],
                                    q_joints[idx2][7],
                                    q_joints[idx2][10],
                                    q_joints[idx2][13],
                                    q_joints[idx2][16],
                                    q_joints[idx2][19]])

                q_ddot =  np.array([q_joints[idx2][2],
                                    q_joints[idx2][5],
                                    q_joints[idx2][8],
                                    q_joints[idx2][11],
                                    q_joints[idx2][14],
                                    q_joints[idx2][17],
                                    q_joints[idx2][20]])

                # arm.safe_set_joint_positions_velocities(q, q_dot)
                # arm.set_joint_positions_velocities_torque(q,  q_dot, q_ddot)

                ############# DONT REMOVE PRINT STATEMENTS #####################################
                # print('q: ',q)
                # print('----------')
                # print('q_dot: ',q_dot)
                # print('----------')
                # time.sleep(0.02)

    
                # arm.move_to_position(qq)
                saved_state['q'].append(q)
                saved_state['qdot'].append(q_dot)
                saved_state['qddot'].append(q_ddot)
    

    for idx in range(len(saved_state['q'])):
        arm.set_joint_positions_velocities_torque(saved_state['q'][idx],  saved_state['qdot'][idx], saved_state['qddot'][idx])
        time.sleep(0.02)


    # with open("mpc_var", "wb") as fp:
    #     pickle.dump(saved_state, fp)
    print("Trajectory Complete!")


    
