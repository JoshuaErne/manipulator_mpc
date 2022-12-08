#!/usr/bin/python

import rospy, tf
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Point, Quaternion
import time
import os




def load_sdf(path, block, pose):
    f = open(path+ block + "/model.sdf", "r")
    sdff = f.read()
    f.close()
    
    rospy.wait_for_service("gazebo/spawn_sdf_model")
    spawn_model_prox = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
   


    #['model_name', 'model_xml', 'robot_namespace', 'initial_pose', 'reference_frame'] 
    spawn_model_prox(block+str(time.time()), sdff, "~/obs2_2", pose, "world")
    

def get_pose(position):

    
    orient =  Quaternion(x=0,y=0,z=0,w=1)
    pose = Pose(Point(x=position[0], y=position[1],z=position[2]),orient)
    return pose


if __name__=='__main__':
    rospy.init_node('insert_object',log_level=rospy.INFO)
    # input the map number you want to load
    rospy.loginfo("Enter the number of map you want to load:")
    num = input()
    path = '../models/map'+str(num) + "/"
    
    if not os.path.isdir(path):
        rospy.loginfo("Please input valid number !")


    #orient =  Quaternion(x=0,y=0,z=0,w=1)
    #pose1 = Pose(Point(x=0.275, y=0.06335,z=5),orient)
    #pose2 = Pose(Point(x=0.275, y=-0.06335,z=5),orient)
    rospy.loginfo("Enter the position and rotation of the object:  (six number, with x,y,z,row pitch yaw)")
    for i in range(6):
        position[i] = input()

    orient =  Quaternion(x=0,y=0,z=0,w=1)
    pose1 = Pose(Point(x=position[0], y=position[1],z=position[2]),orient)


    blocks = sorted(os.listdir(os.path.dirname(path)))
   # for i in range(len(blocks)):
    #    rospy.loginfo("Now process "+ blocks[i] + " ...")
    #    load_sdf(path, blocks[i], pose1)
    load_sdf(path, blocks[0], pose1)
    load_sdf(path, blocks[1], pose2)
    rospy.loginfo("Sucessfully set the map "+ str(num) + " environment!")
