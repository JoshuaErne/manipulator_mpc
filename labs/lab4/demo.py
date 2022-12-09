
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
import pickle
from time import perf_counter
import time
from lib.mpc import run_to_position

from core.interfaces import ArmController

def mpc_func(path):

    saved_state = {'q':[], 'qdot':[], 'qddot':[]}
    total_cost = []

    for idx1 in range(len(path)-1):
        p1 = path[idx1]
        p2 = path[idx1+1]
        q_joints, Control_input, time_stamp, loss_vals, optimal_cost = run_to_position(p1, p2)
        total_cost.append(optimal_cost)

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

                saved_state['q'].append(q)
                saved_state['qdot'].append(q_dot)
                saved_state['qddot'].append(q_ddot)
    

    return saved_state, sum(total_cost)

rospy.init_node('demo')

arm = ArmController()
arm.set_arm_speed(.2)

arm.close_gripper()
# arm.move_to_position(np.array([0,0,0,0,0,0,0]))
q = arm.neutral_position()
# arm.move_to_position(q)
arm.open_gripper()

####################################
####################################
#########      MPC        ##########
####################################
####################################

# with open("mpc_var", "rb") as fp:
#     z = pickle.load(fp)
path =np.array([[ 0.        ,  0.4       ,  0.        , -2.5       ,  0.        ,  2.7       ,    0.707     ],
                [-0.74586163, -0.26656078,  0.62554239, -0.14219179, -2.56407488,  2.33605905,   -1.02371856],
                [ 1.91054951, -0.76894274,  1.4524496 , -1.76404297, -0.57854381,  1.95557212,   -0.85556369],
                [ 1.9       ,  1.57      , -1.57      , -1.57      ,  1.57      ,  1.57      ,    0.707     ]])

arm.move_to_position(path[0])

# saved_state, total_cost = mpc_func(path)


# with open('States_Data', "wb") as fp:
#     pickle.dump(saved_state, fp)
with open("States_Data", "rb") as fp:
    saved_state = pickle.load(fp)

start = perf_counter()
for idx in range(len(saved_state['q'])):
        # arm.safe_set_joint_positions_velocities(saved_state['q'][idx],  saved_state['qdot'][idx])
        arm.set_joint_positions_velocities_torque(saved_state['q'][idx], saved_state['qdot'][idx], saved_state['qddot'][idx])
        time.sleep(0.02)
stop = perf_counter()
print('Time for Position and Velocity:', stop-start)
# time_test['position_velocity'].append(stop-start)

for idx in reversed(range(len(saved_state['q']))):
        # arm.safe_set_joint_positions_velocities(saved_state['q'][idx],  saved_state['qdot'][idx])
        arm.set_joint_positions_velocities_torque(saved_state['q'][idx], saved_state['qdot'][idx], saved_state['qddot'][idx])
        time.sleep(0.02)
stop = perf_counter()
print('Time for Position & Velocity (2-Way):', stop-start)
print('------------------------------------------------')

###################################
arm.close_gripper()

