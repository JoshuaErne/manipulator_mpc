from math import pi
import numpy as np
import rospy
import tf

from geometry_msgs.msg import TwistStamped

from core.interfaces import ArmController

from lib.calcJacobian import calcJacobian

#########################
##  RViz Communication ##
#########################

rospy.init_node("visualizer")

twist_pub = rospy.Publisher('/vis/twist', TwistStamped, queue_size=10)
joint_pub = rospy.Publisher('/vis/jointvel', TwistStamped, queue_size=10)
listener = tf.TransformListener()

# rotate vector v by quaternion q
def qv_mult(q, v):
    v = [v[0],v[1],v[2],0]
    return tf.transformations.quaternion_multiply(
        tf.transformations.quaternion_multiply(q, v),
        tf.transformations.quaternion_conjugate(q)
    )[:3]

# Publishes the linear and angular velocity of a frame on the corresponding topic
def show_twist(pub, velocity, frame):
    msg = TwistStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = frame
    msg.twist.linear.x  = velocity[0]
    msg.twist.linear.y  = velocity[1]
    msg.twist.linear.z  = velocity[2]
    msg.twist.angular.x = velocity[3]
    msg.twist.angular.y = velocity[4]
    msg.twist.angular.z = velocity[5]
    pub.publish(msg)

# Publishes the velocity of a given joint on the corresponding topic
def show_end_effector_velocity(velocity):
    show_twist(twist_pub,velocity,'endeffector')

# Publishes the velocity of a given joint on the corresponding topic
def show_joint_velocity(qdot,i):
    # joints are always along z axis
    show_twist(joint_pub, qdot[i] * np.array([0,0,0,0,0,1]), 'joint' + str(i))

# Uses the above methods to visualize the joint velocities and end effector velocity using your jacobian
def show_all_velocity(q,i):

    qdot = np.zeros(7)
    qdot[i] = 1

    J = calcJacobian(q)
    velocity = J @ qdot

    # frame conversion
    try:
        (trans,rot) = listener.lookupTransform('world', 'endeffector', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        return
    quat = tf.transformations.quaternion_conjugate(rot)
    velocity[0:3] = qv_mult(quat,velocity[0:3])
    velocity[3:6] = qv_mult(quat,velocity[3:6])

    show_end_effector_velocity(velocity)
    show_joint_velocity(qdot,i)

#####################
##  Configurations ##
#####################

# TODO: Try testing other configurations!

# The first configuration below matches the dimensional drawing in the handout
configurations = [
    np.array([ 0,    0,     0, -pi/2,     0, pi/2, pi/4 ]),
    np.array([ pi/2, 0,  pi/4, -pi/2, -pi/2, pi/2,    0 ]),
    np.array([ 0,    0, -pi/2, -pi/4,  pi/2, pi,   pi/4 ]),
]

####################
## Test Execution ##
####################

if __name__ == "__main__":

    arm = ArmController()

    # Iterates through the given configurations, visualizing the velocities
    # Try editing the configurations list above to do more testing!
    for i, q in enumerate(configurations):
        print("Moving to configuration " + str(i) + "...")
        arm.move_to_position(q)
        # iterate thru each joint, activating each, one at a time
        for j in range(7):
            show_all_velocity(q,j)
            if j < 6:
                input("Press Enter to move to next joint...")
        if i < len(configurations) - 1:
            input("Press Enter to move to next configuration...")

    print("Done!")
