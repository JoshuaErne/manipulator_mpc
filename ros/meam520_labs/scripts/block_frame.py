import rospy
import tf
import numpy as np
from math import sin, cos, pi

from core.utils import transform

rospy.init_node("visualizer")
tf_broad  = tf.TransformBroadcaster()
# Broadcasts a T0e as the transform from given frame to world frame
def show_pose(T0e,frame):
    tf_broad.sendTransform(
        tf.transformations.translation_from_matrix(T0e),
        tf.transformations.quaternion_from_matrix(T0e),
        rospy.Time.now(),
        frame,
        "block"
    )

def generate_tag_transforms(r=.025):
    transforms = []
    for i in range(4):
        x = np.array([r,0,0])
        rpy = np.array([0,-pi/2,pi/2])
        T = transform(np.zeros(3),np.array([0,0,pi/2 * i])) @ transform(x,rpy)
        transforms.append(T)
    for j in [-1,1]:
        x = r * np.array([0,0,j])
        rpy = np.array([0,pi * (j - 1)/2,-pi/2+pi * (j - 1)/2])
        T = transform(x,rpy)
        transforms.append(T)
    return transforms

tags_to_block = generate_tag_transforms()
r = rospy.Rate(10) # Hz

while not rospy.is_shutdown():
    r.sleep()
    for i, tag_to_block in enumerate(tags_to_block):
        print(tag_to_block)
        show_pose(tag_to_block,'tag'+str(i+6+1))
