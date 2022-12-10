
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

from core.interfaces import ArmController
from lib.meam517_final_project import run_to_position

rospy.init_node('demo')

arm = ArmController()
arm.set_arm_speed(.2)

arm.close_gripper()

q = arm.neutral_position()
arm.move_to_position(q)
arm.open_gripper()

####################################
####################################
#########      MPC        ##########
####################################
####################################

x0      =  np.array([-0.7854,  0.   ,   0.    , -1.5708  ,0.,      1.5708 , 0.7854])
x_final =  np.array([ 0.7854 , 0.6283 , 0.  ,   -1.0472  ,0.    ,  1.5708 , 0.7854]) 

arm.move_to_position(x0)

q_joints, Control_input, time, loss_vals = run_to_position(x0, x_final)

for idx in range(len(q_joints)):
    if idx%50 == 0:
        qq = np.array([q_joints[idx][0],
                       q_joints[idx][3],
                       q_joints[idx][6],
                       q_joints[idx][9],
                       q_joints[idx][12],
                       q_joints[idx][15],
                       q_joints[idx][18]])

        arm.move_to_position(x_final)

###################################
arm.close_gripper()

