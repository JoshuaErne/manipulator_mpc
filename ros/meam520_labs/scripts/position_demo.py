import sys
import rospy
import numpy as np
from math import sin

from core.interfaces import ArmController
from core.utils import time_in_seconds

rospy.init_node('demo')

arm = ArmController()

# move to neutral position
arm.untuck() # blocking
center = arm.get_positions() # oscillate around this position
arm.open_gripper()

# move to a new target
arm.move_to_position(center + .5) # blocking

# move back to neutral position
arm.close_gripper()
arm.untuck()

# directly command periodic setpoints for arm
rate = rospy.Rate(1000)
start = time_in_seconds()
while(True):

    t = time_in_seconds() - start
    q = center + 1/2 * sin(t)

    # directly update position controller setpoint
    arm.exec_position_cmd(q)

    rate.sleep()
