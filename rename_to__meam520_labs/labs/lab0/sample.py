
"""
Date: 08/26/2021

Purpose: This script creates an ArmController and uses it to command the arm's
joint positions and gripper.

Try changing the target position to see what the arm does!

"""

import sys
import rospy
import numpy as np
from math import pi
from scipy.spatial.transform import Rotation as R
from core.interfaces import ArmController
from lib.mpc import run_to_position
from lib.solveIK import IK
import random
from lib.loadmap import loadmap
import pickle
import copy

def generate_random_matrices(trans,num_mat = 10):
        R_list = []
        
        for i in range(num_mat):
            alpa_x = random.uniform(-np.pi,np.pi)
            beta = random.uniform(-np.pi,np.pi)
            gamma = random.uniform(-np.pi,np.pi)
            R_x = np.array([[1,0,0],[0,np.cos(alpa_x),-np.sin(alpa_x)],[0,np.sin(alpa_x),np.cos(alpa_x)]])
            R_y = np.array([[np.cos(beta),0,np.sin(beta)],[0,1,0],[-np.sin(beta),0,np.cos(beta)]])
            R_z = np.array([[np.cos(gamma), -np.sin(gamma), 0],[np.sin(gamma), np.cos(gamma),0],[0,0,1]])
            R_final = np.eye(4)
            R_final[:3,:3] = R_x @ R_y @ R_z
            R_final[:3,-1] = trans
            R_list.append(R_final)
        return R_list

if __name__ == "__main__":
    # rospy.init_node('demo')


    ik = IK()
    # arm = ArmController()
    # arm.set_arm_speed(.2)

    # arm.close_gripper()

    # q = arm.neutral_position()
    # arm.move_to_position(q)
    # arm.open_gripper()

    
    ####################################################################################################################################

    seed =  np.array([ 0.0000000e+00, -7.8500000e-01,  0.0000000e+00, -2.3668750e+00, 8.8817842e-16,  1.5700000e+00,  7.8500000e-01])
    
    map_struct = loadmap("/home/josh/Desktop/meam520_drake/src/meam520_labs/maps/map1.txt")
    map = copy.deepcopy(map_struct)
    obs = map.obstacles

    right_cube = []
    left_cube  = []
    up_cube    = []
    down_cube  = []
    front_cube = []
    back_cube  = []
    
    #Assuming one obstacle
    d = 2
    cube    = []
    corners = []
    cube_centers = [] 
    for i in range(obs.shape[0]):
        obs_center           = np.array([(obs[i][0] + obs[i][3])/2, (obs[i][1] + obs[i][4])/2, (obs[i][2] + obs[i][5])/2])

        back_left_bottm      = np.array([obs[i][0], obs[i][1], obs[i][2]])   #Back  left Bottom
        back_left_top        = np.array([obs[i][0], obs[i][1], obs[i][5]])   #Back  left Top
        back_right_bottm     = np.array([obs[i][0], obs[i][4], obs[i][2]])   #Back  right  Bottom
        front_left_bottm     = np.array([obs[i][3], obs[i][1], obs[i][2]])   #Front left Bottom
        back_right_top       = np.array([obs[i][0], obs[i][4], obs[i][5]])   #Back  right  Top
        front_left_top       = np.array([obs[i][3], obs[i][1], obs[i][5]])   #Front left Top
        front_right_bottm    = np.array([obs[i][3], obs[i][4], obs[i][2]])   #Front right  Bottom
        front_right_top      = np.array([obs[i][3], obs[i][4], obs[i][5]])   #Front right  Top

        cube_coords = np.array([[obs[i][0], obs[i][1], obs[i][2]],   #Back  left  Bottom
                                [obs[i][0], obs[i][1], obs[i][5]],   #Back  left  Top
                                [obs[i][0], obs[i][4], obs[i][2]],   #Back  right Bottom
                                [obs[i][3], obs[i][1], obs[i][2]],   #Front left  Bottom
                                [obs[i][0], obs[i][4], obs[i][5]],   #Back  right Top
                                [obs[i][3], obs[i][1], obs[i][5]],   #Front left  Top
                                [obs[i][3], obs[i][4], obs[i][2]],   #Front right Bottom
                                [obs[i][3], obs[i][4], obs[i][5]],   #Front right Top
                                [(obs[i][0] + obs[i][3])/2, (obs[i][1] + obs[i][4])/2, (obs[i][2] + obs[i][5])/2]])

        rcenter  = (obs_center -  np.array([0, np.abs((back_right_bottm[0] - front_right_top[0])/2)*3, 0]))
        cube_centers.append(rcenter)
        corners.append(np.hstack((np.array([rcenter - np.array([d,d,d])]), np.array([rcenter + np.array([d,d,d])])))[0])
        
        lcenter  = (obs_center +  np.array([0, np.abs((back_right_bottm[0] - front_right_top[0])/2)*3, 0]))
        cube_centers.append(lcenter)
        corners.append(np.hstack((np.array([lcenter - np.array([d,d,d])]), np.array([lcenter + np.array([d,d,d])])))[0])

        fcenter  = (obs_center +  np.array([np.abs((front_right_bottm[1]   - front_left_top[1])/2)*3, 0, 0]))
        cube_centers.append(fcenter)
        corners.append(np.hstack((np.array([fcenter - np.array([d,d,d])]), np.array([fcenter + np.array([d,d,d])])))[0])

        bcenter  = (obs_center -  np.array([np.abs((back_right_bottm[1]    - back_left_top[1])/2)*3, 0, 0]))
        cube_centers.append(bcenter)
        corners.append(np.hstack((np.array([bcenter - np.array([d,d,d])]), np.array([bcenter + np.array([d,d,d])])))[0])

        ucenter  = (obs_center +  np.array([0, 0, np.abs((front_right_top[1] - back_left_top[1])/2)*3]))
        cube_centers.append(ucenter)
        corners.append(np.hstack((np.array([ucenter - np.array([d,d,d])]), np.array([ucenter + np.array([d,d,d])])))[0])

        dcenter  = (obs_center -  np.array([0, 0, np.abs((front_right_bottm[1]- back_left_bottm[1])/2)*3]))
        cube_centers.append(dcenter)
        corners.append(np.hstack((np.array([dcenter - np.array([d,d,d])]), np.array([dcenter + np.array([d,d,d])])))[0])

        for j in range(len(corners)):
            cube.append(np.array(  [[corners[i][0], corners[i][1], corners[i][2]],   #Back  left  Bottom
                                    [corners[i][0], corners[i][1], corners[i][5]],   #Back  left  Top
                                    [corners[i][0], corners[i][4], corners[i][2]],   #Back  right Bottom
                                    [corners[i][3], corners[i][1], corners[i][2]],   #Front left  Bottom
                                    [corners[i][0], corners[i][4], corners[i][5]],   #Back  right Top
                                    [corners[i][3], corners[i][1], corners[i][5]],   #Front left  Top
                                    [corners[i][3], corners[i][4], corners[i][2]],   #Front right Bottom
                                    [corners[i][3], corners[i][4], corners[i][5]],   #Front right Top
                                    [(corners[i][0] + corners[i][3])/2, (corners[i][1] + corners[i][4])/2, (corners[i][2] + corners[i][5])/2]]))#Corner


    ####################################################################################################################################

    # with open("constraint_q.pkl", 'rb') as f:
    #     qqq =pickle.load(f)
    # print(len(qqq))
    # with open("constraint_q1.pkl", 'wb') as f:
    #     pickle.dump(qqq, f)
    # raise

    valid_q = []
    number_matrices = 50
    for coord_idx in range(1):
        inner_valid_q = []
        for num in range(len(cube[coord_idx])):
            R_list = generate_random_matrices(cube[coord_idx][num], num_mat=number_matrices)
            for idx in range(len(R_list)):
                target = R_list[idx]
                q, success, rollout = ik.inverse(target, seed)
                if success:
                    valid_q.append(q)
                    print("success")
    print("Number of successes:        ",len(valid_q))
    print("Total number of iterations: ",number_matrices*len(cube))

    with open("constraints450_q.pkl", 'wb') as f:
        pickle.dump(valid_q, f)
        pickle.dum(cube_centers, f)

    print('DOne')
    # q = np.array([-0.01779104, 0,  0.01978416, -1.9,  0.02984079,  1.54119363,  0.75344862])
    # arm.move_to_position(q)
    # print(arm.get_positions())
    ####################################
    ####################################
    #########      MPC        ##########
    ####################################
    ####################################

    # x0      =  np.array([-0.7854,  0.   ,   0.    , -1.5708  ,0.,      1.5708 , 0.7854])
    # x_final =  np.array([ 0.7854 , 0.6283 , 0.  ,   -1.0472  ,0.    ,  1.5708 , 0.7854]) 

    # arm.move_to_position(x0)

    # q_joints, Control_input, time, loss_vals = run_to_position(x0, x_final)

    # for idx in range(len(q_joints)):
    #     if idx%50 == 0:
    #         qq = np.array([q_joints[idx][0],
    #                        q_joints[idx][3],
    #                        q_joints[idx][6],
    #                        q_joints[idx][9],
    #                        q_joints[idx][12],
    #                        q_joints[idx][15],
    #                        q_joints[idx][18]])

    #         arm.move_to_position(x_final)

    ###################################
    # arm.close_gripper()

    

    # initial Pos: [ 0.0000000e+00 -7.8500000e-01  0.0000000e+00 -2.3668750e+00 8.8817842e-16  1.5700000e+00  7.8500000e-01]
    #[-0.01779104 -0.76012406  0.01978416 -2.34205092  0.02984079  1.54119363 0.75344862]


