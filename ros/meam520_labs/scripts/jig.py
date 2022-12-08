import rospy
import yaml
import numpy as np

from gazebo_msgs.msg import LinkStates, ModelStates
from geometry_msgs.msg import Pose, Twist, Point, Quaternion, Vector3
from tf.transformations import quaternion_from_matrix,translation_from_matrix, quaternion_matrix

from core.utils import trans, transform
import sys

rospy.init_node('static_poses')

try:
    team = rospy.get_param("team") # 'red' or 'blue'
except KeyError:
    print('Team must be red or blue - make sure you are running final.launch!')
    exit()

sign = (1 if team == 'blue' else -1)
robot_to_world = trans(np.array([0,sign*.978,0]))
platform_to_robot = trans(np.array([.562,sign*.169,.225]))

platform_to_world = robot_to_world @ platform_to_robot
world_to_platform = np.linalg.inv(platform_to_world)

def pose_to_transform(pose):
    T = quaternion_matrix([
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w ])
    T[:3,3] = np.array([
        pose.position.x,
        pose.position.y,
        pose.position.z ]);
    return T

done = False

def gazebo_cb(msg):
    global done
    if done:
        return

    counter = 0
    data = dict(
        models=dict()
    )
    for (name, pose, twist) in zip(msg.name, msg.pose, msg.twist):
        block_to_world = pose_to_transform(pose)
        if 'static' in name and sign*block_to_world[1,-1] > 0:
            # my side static cube
            counter = counter + 1
            block_to_platform = world_to_platform @ block_to_world
            block_to_platform[0:3,-1] = block_to_platform[0:3,-1].round(3)

            data['models']['cube'+str(counter)+'_static'] = block_to_platform.reshape((16)).tolist()

    print(data)

    with open('data.yaml', 'w') as outfile:
        yaml.dump(data, outfile, default_flow_style=None)

    print("done")
    done = True

gazebo_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, gazebo_cb)


rate = rospy.Rate(100) # Hz
while not rospy.is_shutdown() and not done:
    rate.sleep()
