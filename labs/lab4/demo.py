
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


from core.interfaces import ArmController

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

with open("mpc_var", "rb") as fp:
    z = pickle.load(fp)

arm.move_to_position(z['q'][0])
for idx in range(len(z['q'])):
    # if idx%50 == 0:
        arm.safe_set_joint_positions_velocities(z['q'][idx], z['qdot'][idx])
        arm.set_joint_positions_velocities_torque(z['q'][idx], z['qdot'][idx], z['qddot'][idx])

        # arm.move_to_position(z['q'][idx])


###################################
arm.close_gripper()

