import sys
from math import pi
import numpy as np

import rospy
import roslib
import tf
import geometry_msgs.msg


from core.interfaces import ArmController

from lib.calculateFK import FK
from lib.PlanarIK import PlanarIK

rospy.init_node("visualizer")

# Using your solution code
fk = FK()
ik = PlanarIK()

#########################
##  RViz Communication ##
#########################

tf_broad  = tf.TransformBroadcaster()
point_pubs = [
    rospy.Publisher('/vis/joint'+str(i), geometry_msgs.msg.PointStamped, queue_size=10)
    for i in range(7)
]

# Publishes the position of a given joint on the corresponding topic
def show_joint_position(joints,i):
    msg = geometry_msgs.msg.PointStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = 'world'
    msg.point.x = joints[i,0]
    msg.point.y = joints[i,1]
    msg.point.z = joints[i,2]
    point_pubs[i].publish(msg)

# Broadcasts a T0e as the transform from given frame to world frame
def show_pose(T0e,frame):
    tf_broad.sendTransform(
        tf.transformations.translation_from_matrix(T0e),
        tf.transformations.quaternion_from_matrix(T0e),
        rospy.Time.now(),
        frame,
        "world"
    )

# Uses the above methods to visualize the full results of your FK
def show_all_FK(state):
    q = state['position']
    joints, T0e = fk.forward(q)
    show_pose(T0e,"endeffector")
    for i in range(7):
        show_joint_position(joints,i)

# visualize the chosen IK target
def show_target(target):
    x = target['o'][0]
    z = target['o'][1]
    theta = target['theta']
    T0_target = tf.transformations.translation_matrix(np.array([x,0,z])) @ tf.transformations.euler_matrix(0,-theta-pi/2,pi)
    show_pose(T0_target,"target")


########################
##  FK Configurations ##
########################

# TODO: Try testing other configurations!

# The first configuration below matches the dimensional drawing in the handout
configurations = [
    np.array([ 0,    0,     0, -pi/2,     0, pi/2, pi/4 ]),
    np.array([ pi/2, 0,  pi/4, -pi/2, -pi/2, pi/2,    0 ]),
    np.array([ 0,    0, -pi/2, -pi/4,  pi/2, pi,   pi/4 ]),
]

#################
##  IK Targets ##
#################

# TODO: Try testing your own targets!

targets = [
    {
        'o': np.array([0.1, 0.8]),
        'theta': 0
    },
    {
        'o': np.array([.4, .6]),
        'theta': pi/4
    },
    {
        'o': np.array([0.4, -0.4]),
        'theta': pi/2+0.2
     },
    {
        'o': np.array([.1, .3]),
        'theta': 5 * pi/4
    }
]

####################
## Test Execution ##
####################

if __name__ == "__main__":

    if len(sys.argv) < 2:
        print("usage:\n\tpython visualize.py FK\n\tpython visualize.py IK")
        exit()

    arm = ArmController(on_state_callback=show_all_FK)

    if sys.argv[1] == 'FK':

        # Iterates through the given configurations, visualizing your FK solution
        # Try editing the configurations list above to do more testing!
        for i, q in enumerate(configurations):
            print("Moving to configuration " + str(i) + "...")
            arm.move_to_position(q)
            if i < len(configurations) - 1:
                input("Press Enter to move to next configuration...")

        arm.move_to_position(q)

    elif sys.argv[1] == 'IK':

        # Iterates through the given targets, using your IK solution
        # Try editing the targets list above to do more testing!
        for i, target in enumerate(targets):
            print("Moving to target " + str(i) + "...")
            show_target(target)
            solutions = ik.panda_ik(target)
            q = solutions[0,:] # choose the first of multiple solutions
            arm.move_to_position(q)
            if i < len(targets) - 1:
                input("Press Enter to move to next target...")

    else:
        print("invalid option")
