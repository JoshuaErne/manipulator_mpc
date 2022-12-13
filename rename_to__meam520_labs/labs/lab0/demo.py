
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
from lib.mpc import run_to_position

rospy.init_node('demo')

arm = ArmController()
arm.set_arm_speed(.2)

arm.close_gripper()
q = arm.neutral_position()
arm.move_to_position(q)

# arm.open_gripper()

# q = np.array([ 0.0000000e+00, -7.8500000e-01+0.6,  0.0000000e+00, -2.3668750e+00+0.6, 8.8817842e-16,  1.5700000e+00+np.pi/2,  7.8500000e-01])
q = np.array([ 1.39,  1.55,  -1.56,  -2.,  1.56,  1.57, 0.7   ])
arm.move_to_position(q)
print(arm.get_positions())
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
arm.close_gripper()





# initial Pos: [ 0.0000000e+00 -7.8500000e-01  0.0000000e+00 -2.3668750e+00 8.8817842e-16  1.5700000e+00  7.8500000e-01]
#[-0.01779104 -0.76012406  0.01978416 -2.34205092  0.02984079  1.54119363 0.75344862]

