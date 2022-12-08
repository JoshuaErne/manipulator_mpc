#!/usr/bin/python

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from al5d_gazebo.msg import TransformStampedList

JOINT_NAMES = ["panda_link0", "panda_link1", "panda_link1"]

#This is a somewhat hacky buffer to publish joint positions from tf

class TfTranslator:
    def __init__(self):
        rospy.init_node('tf_translator', anonymous=True)
        self.timer = rospy.Timer(rospy.Duration(0.02), self.tf_cb, oneshot=False)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.trans_pub = rospy.Publisher("joint_poses", TransformStampedList, queue_size=1)
        self.matlab = []

        #new publisher for matlab
        for i in range(6):
            self.matlab.append(rospy.Publisher("joint_poses/"+str(JOINT_NAMES[i]) , TransformStamped, queue_size=1))


    def tf_cb(self, timer):
        joint_poses = TransformStampedList()
        rospy.sleep(.05)
        try:
            for i in range(6):
                trans = self.tf_buffer.lookup_transform("base", JOINT_NAMES[i], rospy.Time())
                joint_poses.transforms.append(trans)
                self.matlab[i].publish(trans)
        except:
            pass

        if len(joint_poses.transforms) == len(JOINT_NAMES):
            self.trans_pub.publish(joint_poses)

if __name__=='__main__':
    trans = TfTranslator()
    rospy.spin()
