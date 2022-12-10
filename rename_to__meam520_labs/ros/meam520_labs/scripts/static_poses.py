#!/usr/bin/env python3

import rospy
import yaml
import numpy as np

from gazebo_msgs.msg import LinkStates, ModelStates
from geometry_msgs.msg import Pose, Twist, Point, Quaternion, Vector3
from tf.transformations import quaternion_from_matrix,translation_from_matrix

from core.utils import trans, transform

def toVector3(v):
    return Vector3(v[0],v[1],v[2])

def toQuaternion(q):
    return Quaternion(q[0],q[1],q[2],q[3])

if __name__ == "__main__":

    rospy.init_node('static_poses')

    try:
        team = rospy.get_param("team") # 'red' or 'blue'
    except KeyError:
        print('Team must be red or blue - make sure you are running final.launch!')
        exit()

    sign = (1 if team == 'blue' else -1)
    robot_to_world = trans(np.array([0,sign*.978,0]))
    static_to_robot = trans(np.array([.562,sign*.169,.225]))

    text = rospy.get_param('static_block_configuration')

    # compute world frame pose of all models
    models = yaml.safe_load(text)
    names = []
    poses = []
    for name, pose in models['models'].items():
        names.append(name)
        if 'static' in name:
            pre = static_to_robot @ robot_to_world
        elif 'dynamic' in name:
            pre = np.eye(4)
        else: # 'other' i.e. camera and tag0
            pre = robot_to_world
        poses.append(pre @ np.array(pose).reshape((4,4)))

    # spoof gazebo model states message
    gazebo_pub = rospy.Publisher('/gazebo/model_states', ModelStates, queue_size=10)
    msg = ModelStates()
    msg.name = names
    msg.pose = [ Pose(toVector3(translation_from_matrix(T)),toQuaternion(quaternion_from_matrix(T))) for T in poses]
    msg.twist = [Twist(Vector3(x=0,y=0,z=0),Vector3(x=0,y=0,z=0)) for name in names]

    r = rospy.Rate(10) # Hz
    while not rospy.is_shutdown():
        gazebo_pub.publish(msg)
        r.sleep()
