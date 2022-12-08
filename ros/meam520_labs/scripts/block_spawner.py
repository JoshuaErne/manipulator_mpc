#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose
from std_srvs.srv import Empty, EmptyRequest
from math import sin, cos, pi
import numpy as np
from time import sleep
import os
import subprocess

from std_msgs.msg import Float64

import tf

rospy.init_node('block_spawner')

from gazebo_msgs.srv import SpawnModel
spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)


path = os.path.expanduser('~/meam520_ws/src/meam520_labs/ros/meam520_labs/urdf/cube.xacro')
command = 'rosrun xacro xacro '+path+' color:='
colors = ["'0.043 0.611 0.192 1'","'.25 .15 .5 1'"]
xmls = [subprocess.check_output(command+color,shell=True).decode('utf-8') for color in colors]


count = 0
def place(x,y,z,type):
    global count
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    r = np.random.randint(0,3) * 2 * np.pi / 4
    p = np.random.randint(0,3) * 2 * np.pi / 4
    y = np.random.rand(1) * 2 * pi
    q = tf.transformations.quaternion_from_euler(r,p,y)
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]
    count = count + 1
    success = False
    while not success and not rospy.is_shutdown():
        try:
            success = spawn_model_client(
                    model_name='cube'+str(count)+'_'+type,
                    model_xml= xmls[0] if type=='static' else xmls[1],
                    robot_namespace='/foo',
                    initial_pose=pose,
                    reference_frame='world')
        except:
            print('Waiting to spawn cubes...')
            sleep(1)


def noise(radius):
    return radius * (np.random.rand(1) - .5)


# for i in [1]:
#     for j in [1]:
for i in [-1,1]:
    for j in [-1,1]:
        x = .562 + 2.5*.0254 * i
        y = 1.147 + 2.5*.0254 * j
        place(x + noise(.025) ,y + noise(.025),.23,'static')
        place(x + noise(.025) ,-y + noise(.025),.23,'static')

# n = 1
n = 8
r = 9.5*.0254
for i in range(n):
    place((r + noise(.0254)) * cos(2*pi/n * (i + noise(pi/n))),(r + noise(r/5)) * sin(2*pi/n * (i + noise(pi/n))),.23,'dynamic')



spin_pub = rospy.Publisher('/turntable/turntable/turntable_controller/command',Float64,queue_size=1)
msg = Float64()
msg.data = .0523 # about .5 rpm

rospy.loginfo('spinning turntable...')
r = rospy.Rate(1) # Hz
while not rospy.is_shutdown():
    spin_pub.publish(msg)
    r.sleep()
