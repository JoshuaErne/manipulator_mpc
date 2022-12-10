import sys
import rospy
import numpy as np
from math import sin

from core.interfaces import ArmController
from core.utils import time_in_seconds

# Extremely simple joint-space controller
def SimpleJointSpacePDController(state):

    # wait to complete untuck before control starts
    if not ready:
        return

    kp = 10
    kd = 3
    t = time_in_seconds() - start
    ep = state['position'] - (center + 1/2 * sin(t))
    ed = state['velocity']
    command = -kp * ep - kd * ed
    arm.exec_torque_cmd(command)


rospy.init_node('demo')
ready = False

# the callback will fire every time the robot's state is updated
arm = ArmController(on_state_callback=SimpleJointSpacePDController)

# move to neutral position
arm.untuck()

center = arm.get_positions() # oscillate around this position
ready = True
start = time_in_seconds()

# This loop keeps program alive

rate = rospy.Rate(1000)
while(True):
    rate.sleep()
