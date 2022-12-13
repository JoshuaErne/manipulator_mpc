
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


rospy.init_node('demo')

arm = ArmController()
arm.set_arm_speed(.2)

arm.close_gripper()
q = arm.neutral_position()
arm.move_to_position(q)

###########################################################################################################
# You can test your start and goal positions here
start = np.array([])
goal  = np.array([])

arm.move_to_position(start)
arm.open_gripper()
arm.move_to_position(goal)
###########################################################################################################

arm.close_gripper()

