#!/usr/bin/env python

import sys
import rospy
import numpy as np
from math import pi
from time import perf_counter
import geometry_msgs
from visualization_msgs.msg import Marker

import rospy
from std_srvs.srv import Empty

import os
import pickle
import time

from core.interfaces import ArmController
from lib.rrt import rrt
from lib.mpc import run_to_position

from lib.loadmap import loadmap
from copy import deepcopy

#Change the start and goal state based on your maps
starts = [np.array([-pi/4, 0, 0, -pi/2, 0, pi/2, pi/4]),
          np.array([0, 0.4, 0, -2.5, 0, 2.7, 0.707]),
          np.array([-pi/2, 0, 0, -pi/1.5, 0, pi/2, pi/4]),
          np.array([0, -1, 0, -2, 0, 1.57, 0])]
goals = [np.array([pi/4, pi/5, 0, -pi/3, 0, pi/2, pi/4]),
         np.array([1.9, 1.57, -1.57, -1.57, 1.57, 1.57, 0.707]),
         np.array([pi/4, 0.25, 0, -pi/1.5, 0, pi/2, pi/4]),
         np.array([1.4, 1.57, -1.57, -2, 1.57, 1.57, 0.707])]
mapNames = ["map1",
            "map2",
            "map3",
            "map4",
            "emptyMap"]


def mpc_func(path):
    saved_state = {'q':[], 'qdot':[], 'qddot':[]}
    total_cost = []
    for idx1 in range(len(path)-1):
        p1 = path[idx1]
        p2 = path[idx1+1]
        q_joints, Control_input, time_stamp, loss_vals, optimal_cost = run_to_position(p1, p2)
        total_cost.append(optimal_cost)

        for idx2 in range(len(q_joints)):
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

                saved_state['q']    .append(q)
                saved_state['qdot'] .append(q_dot)
                saved_state['qddot'].append(q_ddot)
    
    return saved_state, sum(total_cost)


if __name__ == "__main__":

    if len(sys.argv) < 2:
        print("usage:\n\tpython rrt_demo.py 1\n\tpython rrt_demo.py 2\n\tpython rrt_demo.py 3 ...")
        exit()

    rospy.init_node('RRT')
    rospy.wait_for_service('/gazebo/reset_world')

    print(sys.argv[1])
    arm = ArmController()
    index = int(sys.argv[1])-1
    arm.move_to_position(np.array([0, -0.5, 0, -pi/2, 0, pi/2, pi/4]))
    # input('here')
    
    print("Running test "+sys.argv[1])
    print("Moving to Start Position")
    
    arm.move_to_position(starts[index])
    map_struct = loadmap("../../maps/"+mapNames[index] + ".txt")
    
    print("Map = "+ mapNames[index])
    print("----------Starting to plan----------")
    
    states_list = []
    costs_list  = []
    time_test   = {'position':[],'position_velocity':[],'position_velocity_torque':[]}
    choose_mpc  = input("Choose RRT or RRT+MPC: \n Input either 1 or 2: ")

    if choose_mpc == '1':
        print('----------Implementing RRT----------')
        path = rrt(deepcopy(map_struct), deepcopy(starts[index]), deepcopy(goals[index]))
        for joint_set in path:
            arm.move_to_position(joint_set)
        print("Trajectory Complete!")

    elif choose_mpc == '2':
        optimize_rrt = input("Optimize RRT ??: \n Input either T/F: ")
        
        if optimize_rrt == 'T' or optimize_rrt == 'True' or optimize_rrt == 'true': 
            total_paths  = input("Number of optimization Trajectories to test: Enter number between 1-10: ")
            total_paths  = int(total_paths)
            for i in range(total_paths):
                print('----------Optimzation Step {}----------'.format(i))
                print('----------Implementing RRT   ----------')
                path = rrt(deepcopy(map_struct), deepcopy(starts[index]), deepcopy(goals[index]))
                print('-------Implementing MPC--------------')
                print()
                saved_state, total_cost = mpc_func(path)
                states_list.append(saved_state)
                costs_list .append(total_cost)

            print('costs_list',costs_list)
            print('--------------------')

            idx = np.argmin(np.array(costs_list))

            chosen_states = states_list[idx]
            for idx in range(len(chosen_states['q'])):
                arm.set_joint_positions_velocities_torque(chosen_states['q'][idx],  chosen_states['qdot'][idx], chosen_states['qddot'][idx])
                time.sleep(0.02)
            

        else:
            print('--------Implementing RRT------------')
            path = rrt(deepcopy(map_struct), deepcopy(starts[index]), deepcopy(goals[index]))
            print(path)
            saved_state, total_cost = mpc_func(path)
            print('-------Implementing MPC--------------')

            check_speeds = input("Test Speeds? Y/N: ")
            if check_speeds == 'Y':
                # for test in range(3):
                    test = input("Test: Position - Enter 1 \n Test: Position - Enter 2 \n Test: Position - Enter 3 \n: ")
                    if test == '1':
                        start = perf_counter()
                        for idx in range(len(saved_state['q'])):
                            if idx%25 == 0:
                                arm.move_to_position(saved_state['q'][idx])
                        stop = perf_counter()
                        print('Time for Position:', stop-start)
                        time_test['position'].append(stop-start)
                    
                        for idx in reversed(range(len(saved_state['q']))):
                            if idx%50 == 0:
                                arm.move_to_position(saved_state['q'][idx])
                        # rospy.wait_for_service('/gazebo/reset_world')
                        # reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
                        # reset_world()
                        # arm.move_to_position(saved_state['q'][0])
                        stop = perf_counter()
                        print('Time for Position (2-Way):', stop-start)
                        print('------------------------------------------------')

                    if test == '2':
                        start = perf_counter()
                        for idx in range(len(saved_state['q'])):
                                arm.safe_set_joint_positions_velocities(saved_state['q'][idx],  saved_state['qdot'][idx])
                                time.sleep(0.02)
                        stop = perf_counter()
                        print('Time for Position and Velocity:', stop-start)
                        time_test['position_velocity'].append(stop-start)
                    
                        for idx in reversed(range(len(saved_state['q']))):
                            # if idx%50 == 0:
                                # arm.move_to_position(saved_state['q'][idx])
                                arm.safe_set_joint_positions_velocities(saved_state['q'][idx],  saved_state['qdot'][idx])
                                time.sleep(0.02)
                        stop = perf_counter()
                        print('Time for Position & Velocity (2-Way):', stop-start)
                        print('------------------------------------------------')
                        # rospy.wait_for_service('/gazebo/reset_world')
                        # reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
                        # reset_world()
                        # arm.move_to_position(starts[index])
                    
                    if test == '3':
                        # arm.move_to_position(starts[index])
                        # print('Test: Position, Velocity, Acceleration')
                        start = perf_counter()
                        for idx in range(len(saved_state['q'])):
                            arm.set_joint_positions_velocities_torque(saved_state['q'][idx],  saved_state['qdot'][idx], saved_state['qddot'][idx])
                            time.sleep(0.02)
                        stop = perf_counter()
                        print('Time for Position, Velocity and Acceleration:', stop-start)
                        time_test['position_velocity_torque'].append(stop-start)

                        for idx in reversed(range(len(saved_state['q']))):
                            # if idx%50 == 0:
                                # arm.move_to_position(saved_state['q'][idx])
                                arm.set_joint_positions_velocities_torque(saved_state['q'][idx],  saved_state['qdot'][idx], saved_state['qddot'][idx])
                                time.sleep(0.02)
                        stop = perf_counter()
                        print('Time for Position, Velocity and Acceleration (2-way):', stop-start)
                        print('------------------------------------------------')
                        # rospy.wait_for_service('/gazebo/reset_world')
                        # reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
                        # reset_world()
                        # arm.move_to_position(starts[index])

            
            elif check_speeds == 'N':
                for idx in range(len(saved_state['q'])):
                    arm.set_joint_positions_velocities_torque(saved_state['q'][idx],  saved_state['qdot'][idx], saved_state['qddot'][idx])
                    time.sleep(0.02)
        
        to_save_states = input('Do you want to save the current state vectors: Y/N: ')

        if to_save_states == 'Y':
            print('States saved to your current directory under: saved_state_{}'.format(index))
            with open('saved_state_{}'.format(index), "wb") as fp:
                pickle.dump(saved_state, fp)

            print('Time durations saved to your current directory under: Time_test_file_{}'.format(index))
            with open('Time_test_file_{}'.format(index), "wb") as fp:
                pickle.dump(time_test, fp)
        

        print("Trajectory Complete!")


    
