#!/usr/bin/env python
#
# MEAM 520 Arm Controller, Fall 2021
#
# *** MEAM520 STUDENTS SHOULD NOT MODIFY THIS FILE ***
#
# This code is *HEAVILY* based on / directly modified from the PandaRobot
# package authored by Saif Sidhik. The license and attribution of the original
# open-source package is below.
#
# Copyright (c) 2019-2021, Saif Sidhik
# Copyright (c) 2013-2014, Rethink Robotics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# **************************************************************************/

"""
    @info
        Interface class for the Franka Robot in hardware and simulation

"""

import copy
import rospy
import logging
import argparse
import quaternion
import numpy as np
import franka_interface
import itertools
from core.safety import Safety
from core.utils import time_in_seconds
from franka_core_msgs.msg import JointCommand

from gazebo_msgs.msg import LinkStates, ModelStates
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_matrix
import std_msgs.msg
from math import acos, sin, cos, pi

from core.utils import time_in_seconds, transform

class ObjectDetector:

    def __init__(self):

        self.detection_angle = 50 * np.pi / 180.0
        self.fov = 85 * np.pi / 180.0
        self.tag_to_block_transforms = self.generate_tag_to_block_transforms()

        self.gazebo_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.gazebo_cb);
        self.gazebo_data = None

        self.detections = []

    ################
    ## SIMULATION ##
    ################

    # Simualated tag detection!

    def generate_tag_to_block_transforms(self,r=.025):
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

    def get_tag_to_block_transform(self,id):
        index = id - 1 if id <= 6 else id - 1 - 6
        return self.tag_to_block_transforms[index]

    def gazebo_cb(self,msg):

        # get world to camera first
        for (name,pose) in zip(msg.name,msg.pose):
            if 'camera' in name:
                world_to_camera = np.linalg.inv(self.pose_to_transform(pose))

        # find all tags in environment
        tags = []
        for (name,pose) in zip(msg.name,msg.pose):
            if 'cube' in name: # cubes - generate virtual tags
                block_to_world = self.pose_to_transform(pose)
                for idx in range(6):
                    id = idx + 1 if 'static' in name else idx + 6 + 1
                    tag_to_block = self.get_tag_to_block_transform(id)
                    tag_to_camera =  world_to_camera @ block_to_world @ tag_to_block
                    tag = ('tag'+str(id),tag_to_camera)
                    tags.append(tag)
            if 'tag' in name: # for real calibration tag(s) on table
                tag_to_world = self.pose_to_transform(pose)
                tag_to_camera =  world_to_camera @ tag_to_world
                tag = (name,tag_to_camera)
                tags.append(tag)

        tags = [tag for tag in tags if self.check_tag_visibility(tag)]
        self.detections = tags

    def angle(self,v1,v2):
        return acos(min(1,max(-1,np.dot(v1,v2))))

    def check_tag_visibility(self,tag):
        (name, T) = tag
        # return ('tag6' in name or 'tag0' in name or 'tag12' in name)
        z = T[:3,2] # tag frame
        x = T[:3,3] # tag position
        ray = x / np.linalg.norm(x)
        if self.angle(z,-ray) > self.detection_angle:
            # angle between camera vector and cube too steep for detection
            return False
        elif self.angle(np.array([0,0,1]),ray) > self.fov/2:
            # outside view cone
            return False
        else:
            return True

    def pose_to_transform(self,pose):
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


    ##############
    ## HARDWARE ##
    ##############


    def vision_cb(self, msg):
        # TODO: implement!
        pass

    ############
    ## CLIENT ##
    ############

    def get_detections(self):
        """

        """
        return self.detections.copy()


    def get_static_blocks(self):
        """
        returns a list of np arrays containing homogenous transformations of blocks to base
        """
        blocks = []
        H = transform( np.array([.5, .0, .08]), np.array([0,pi,pi])            )
        blocks.append(H)

        H = transform( np.array([.5, .0, .05]), np.array([0,pi,pi])            )
        blocks.append(H)
        return blocks

class ArmController(franka_interface.ArmInterface):
    """
        :bases: :py:class:`franka_interface.ArmInterface`

        :param on_state_callback: optional callback function to run on each state update

    """
    #############################
    ##                         ##
    ##    "UNDER THE HOOD"     ##
    ##      FUNCTIONALITY      ##
    ##                         ##
    ##   Students don't need   ##
    ##   to use these methods  ##
    ##                         ##
    #############################

    def __init__(self, on_state_callback=None):
        """
            Constructor class.  Functions from `franka_interface.ArmInterface <https://justagist.github.io/franka_ros_interface/DOC.html#arminterface>`_

            :param on_state_callback: optional callback function to run on each state update
        """

        self._logger = logging.getLogger(__name__)

        # ----- don't update robot state value in this class until robot is fully configured
        self._arm_configured = False

        # Parent constructor
        franka_interface.ArmInterface.__init__(self)

        self._jnt_limits = [{'lower': self.get_joint_limits().position_lower[i],
                             'upper': self.get_joint_limits().position_upper[i]}
                            for i in range(len(self.joint_names()))]

        # number of joints
        self._nq = len(self._jnt_limits)
        # number of control commands
        self._nu = len(self._jnt_limits)

        self._configure(on_state_callback)

        self._tuck = [self._neutral_pose_joints[j] for j in self._joint_names]

        self._untuck = self._tuck

        self._q_mean = np.array(
            [0.5 * (limit['lower'] + limit['upper']) for limit in self._jnt_limits])

        self._franka_robot_enable_interface = franka_interface.RobotEnable(
            self._params)

        if not self._franka_robot_enable_interface.is_enabled():
            self._franka_robot_enable_interface.enable()

        self._time_now_old = time_in_seconds()

        self._arm_configured = True

        self.safe = Safety()


    def _configure(self, on_state_callback):

        if on_state_callback:
            self._on_state_callback = on_state_callback
        else:
            self._on_state_callback = lambda m: None

        self._configure_gripper(
            self.get_robot_params().get_gripper_joint_names())

        if self.get_robot_params()._in_sim:
            # Frames interface is not implemented for simulation controller
            self._frames_interface = None

    def _configure_gripper(self, gripper_joint_names):
        self._gripper = franka_interface.GripperInterface(
            ns=self._ns, gripper_joint_names=gripper_joint_names)
        if not self._gripper.exists:
            self._gripper = None
            return

    def _on_joint_states(self, msg):
        # Parent callback function is overriden to update robot state of this class

        franka_interface.ArmInterface._on_joint_states(self, msg)

        if self._arm_configured:
            self._state = self._update_state()
            self._on_state_callback(self._state)


    def _update_state(self):

        now = rospy.Time.now()

        state = {}
        state['position'] = self.get_positions()
        state['velocity'] = self.get_velocities()
        state['effort'] = self.get_torques()
        state['timestamp'] = {'secs': now.secs, 'nsecs': now.nsecs}
        state['gripper_state'] = self.get_gripper_state()

        return state

    def _format_command_with_limits(self, cmd):

        for (angle,limit,number) in zip(cmd,self.joint_limits(),range(7)):
            if angle < limit['lower'] or angle > limit['upper']:
                cmd[number] = min(limit['upper'],max(limit['lower'],angle))
                rospy.logwarn("Position {angle:2.2f} for joint {number} violates joint limits [{lower:2.5f},{upper:2.5f}]. Constraining within range.".format(
                    number=number,
                    lower=limit['lower'],
                    upper=limit['upper'],
                    angle=angle
                ))

        return dict(zip(self.joint_names(), cmd[:7]))


    #####################
    ##                 ##
    ##  CONFIGURATION  ##
    ##       AND       ##
    ##   PARAMETERS    ##
    ##                 ##
    #####################

    def joint_limits(self):
        """
        :return: joint limits
        :rtype: [{'lower': float, 'upper': float}]
        """
        return self._jnt_limits

    def q_mean(self):
        """
        :return: mean of joint limits i.e. "center position"
        :rtype: [float]
        """
        return self._q_mean

    def n_joints(self):
        """
        :return: number of joints
        :rtype: int
        """
        return self._nq

    def n_cmd(self):
        """
        :return: number of control commands (normally same as number of joints)
        :rtype: int
        """
        return self._nu

    def enable_robot(self):
        """
            Re-enable robot if stopped due to collision or safety.
        """
        self._franka_robot_enable_interface.enable()

    def set_arm_speed(self, speed):
        """
        Set joint position speed (only effective for :py:meth:`move_to_joint_position`

        :type speed: float
        :param speed: ratio of maximum joint speed for execution; range = [0.0,1.0]
        """
        self.set_joint_position_speed(speed)

    def set_gripper_speed(self, speed):
        """
            Set velocity for gripper motion

            :param speed: speed ratio to set
            :type speed: float
        """
        if self._gripper:
            self._gripper.set_velocity(speed)

    def neutral_position(self):
        return np.array(list(self._params.get_neutral_pose().values()))


    ########################
    ##                    ##
    ##  FEEDBACK / STATE  ##
    ##                    ##
    ########################

    def get_positions(self, include_gripper=False):
        """
        :return: current joint angle positions
        :rtype: [float]

        :param include_gripper: if True, append gripper joint positions to list
        :type include_gripper: bool
        """
        joint_angles = self.joint_angles()

        joint_names = self.joint_names()

        all_angles = [joint_angles[n] for n in joint_names]

        if include_gripper and self._gripper:
            all_angles += self._gripper.joint_ordered_positions()

        return np.array(all_angles)

    def get_velocities(self, include_gripper=False):
        """
        :return: current joint velocities
        :rtype: [float]

        :param include_gripper: if True, append gripper joint velocities to list
        :type include_gripper: bool
        """
        joint_velocities = self.joint_velocities()

        joint_names = self.joint_names()

        all_velocities = [joint_velocities[n] for n in joint_names]

        if include_gripper and self._gripper:
            all_velocities += self._gripper.joint_ordered_velocities()

        return np.array(all_velocities)

    def get_torques(self, include_gripper=False):
        """
        :return: current joint efforts (measured torques)
        :rtype: [float]

        :param include_gripper: if True, append gripper joint efforts to list
        :type include_gripper: bool
        """
        joint_efforts = self.joint_efforts()

        joint_names = self.joint_names()

        all_efforts = [joint_efforts[n] for n in joint_names]

        if include_gripper and self._gripper:
            all_efforts += self._gripper.joint_ordered_efforts()

        return np.array(all_efforts)

    def get_gripper_state(self):
        """
        Return just the Gripper state {'position', 'force'}.
        Only available if Franka gripper is connected.

        Note that the gripper has two jaws, so there are two position / force values.

        :rtype: dict ({str : numpy.ndarray (shape:(2,)), str : numpy.ndarray (shape:(2,))})
        :return: dict of position and force

          - 'position': :py:obj:`numpy.ndarray`
          - 'force': :py:obj:`numpy.ndarray`
        """
        gripper_state = {}

        if self._gripper:
            gripper_state['position'] = self._gripper.joint_ordered_positions()
            gripper_state['force'] = self._gripper.joint_ordered_efforts()

        return gripper_state

    def get_state(self):
        """
        Gets the full robot state including the gripper state and timestamp.
        See _update_state() above for fields.

        :return: robot state as a dictionary
        :rtype: dict {str: obj}
        """
        return self._state

    #######################
    ##                   ##
    ##  MOTION COMMANDS  ##
    ##                   ##
    #######################


    def move_to_position(self, joint_angles, timeout=10.0, threshold=0.00085, test=None, is_safe=False):
        """
        Move to joint position specified (attempts to move with trajectory action client).
        This function will smoothly interpolate between the start and end positions
        in joint space, including ramping up and down the speed.

        This is a blocking call! Meaning your code will not proceed to the next instruction
        until the robot is within the threshold or the timeout is reached.

        .. note:: This method stops the currently active controller for trajectory tracking (and automatically restarts the controller(s) after execution of trajectory).

        :param joint_angles: desired joint positions, ordered from joint1 to joint7
        :type joint_angles: [float]
        :type timeout: float
        :param timeout: seconds to wait for move to finish [10]
        :type threshold: float
        :param threshold: position threshold in radians across each joint when
         move is considered successful [0.00085]
        :param test: optional function returning True if motion must be aborted
        """
        # if is_safe==False:
        #     raise Exception("!!!++++++++++++++++++++++++++You are not using safe command!!!++++++++++++++++++++++++++++++++!!!")
        self.move_to_joint_positions(
            self._format_command_with_limits(joint_angles), timeout=timeout, threshold=threshold, test=test, use_moveit=False)

    def untuck(self):
        """
        Move to neutral pose (using trajectory controller)
        """
        self.move_to_position(self.neutral_position())

    def exec_gripper_cmd(self, pos, force=None):
        """
        Move gripper joints to the desired width (space between finger joints), while applying
        the specified force (optional)

        :param pos: desired width [m]
        :param force: desired force to be applied on object [N]
        :type pos: float
        :type force: float

        :return: True if command was successful, False otherwise.
        :rtype: bool
        """
        if self._gripper is None:
            return False

        width = min(self._gripper.MAX_WIDTH, max(self._gripper.MIN_WIDTH, pos))

        if force:
            holding_force = min(
                max(self._gripper.MIN_FORCE, force), self._gripper.MAX_FORCE)

            return self._gripper.grasp(width=width, force=holding_force)

        else:
            return self._gripper.move_joints(width)

    def open_gripper(self):
        """
        Convenience function to open gripper all the way
        """
        # behavior at true limit is unreliable
        self.exec_gripper_cmd(self._gripper.MAX_WIDTH * (1 - 1e-2) )

    def close_gripper(self):
        """
        Convenience function to close gripper all the way
        Note: good grasping performance requires applying a force as well!
        """
        self.exec_gripper_cmd(self._gripper.MIN_WIDTH)


    def exec_position_cmd(self, cmd):
        """
        Execute position control on the robot (raw positions). Be careful while using. Send smooth
        commands (positions that are very small distance apart from current position).

        :param cmd: desired joint postions, ordered from joint1 to joint7
                        (optionally, give desired gripper width as 8th element of list)
        :type cmd: [float]
        """

        if len(cmd) > 7:
            gripper_cmd = cmd[7:]
            self.exec_gripper_cmd(*gripper_cmd)

        joint_command = self._format_command_with_limits(cmd)

        self.set_joint_positions(joint_command)

    def exec_velocity_cmd(self, cmd):
        """
        Execute velocity command at joint level (using internal velocity controller)

        :param cmd: desired joint velocities, ordered from joint1 to joint7
        :type cmd: [float]
        """
        joint_names = self.joint_names()

        velocity_command = dict(zip(joint_names, cmd))

        self.set_joint_velocities(velocity_command)

    def exec_torque_cmd(self, cmd):
        """
        Execute torque command at joint level directly

        :param cmd: desired joint torques, ordered from joint1 to joint7
        :type cmd: [float]
        """
        joint_names = self.joint_names()

        torque_command = dict(zip(joint_names, cmd))

        self.set_joint_torques(torque_command)


    def set_joint_positions_velocities(self, positions, velocities, is_safe=False):
        """
        Commands the joints of this limb using specified positions and velocities using impedance control.
        Command at time t is computed as:

        :math:`u_t= coriolis\_factor * coriolis\_t + K\_p * (positions - curr\_positions) +  K\_d * (velocities - curr\_velocities)`


        :type positions: [float]
        :param positions: desired joint positions as an ordered list corresponding to joints given by self.joint_names()
        :type velocities: [float]
        :param velocities: desired joint velocities as an ordered list corresponding to joints given by self.joint_names()
        """
        if is_safe==False:
            raise Exception("!!!++++++++++++++++++++++++++You are not using safe command!!!++++++++++++++++++++++++++++++++!!!")
        self._command_msg.names = self._joint_names
        self._command_msg.position = positions
        self._command_msg.velocity = velocities
        self._command_msg.mode = JointCommand.IMPEDANCE_MODE
        self._command_msg.header.stamp = rospy.Time.now()
        self._joint_command_publisher.publish(self._command_msg)

    def set_joint_positions_velocities_torque(self, positions, velocities, torques):
        """
        ###########################OUR FUNCTION HAHAHAHAHHA ##########################################
        Commands the joints of this limb using specified positions and velocities using impedance control.
        Command at time t is computed as:

        :math:`u_t= coriolis\_factor * coriolis\_t + K\_p * (positions - curr\_positions) +  K\_d * (velocities - curr\_velocities)`


        :type positions: [float]
        :param positions: desired joint positions as an ordered list corresponding to joints given by self.joint_names()
        :type velocities: [float]
        :param velocities: desired joint velocities as an ordered list corresponding to joints given by self.joint_names()
        """
        self._command_msg.names = self._joint_names
        self._command_msg.position = positions
        self._command_msg.velocity = velocities
        self._command_msg.effort = torques
        self._command_msg.mode = JointCommand.IMPEDANCE_MODE
        self._command_msg.header.stamp = rospy.Time.now()
        self._joint_command_publisher.publish(self._command_msg)


    def safe_move_to_position(self, joint_angles, timeout=10.0, threshold=0.00085, test=None):
        # The safety layer
        cur_safe = self.safe.test_new_configuration(joint_angles)

        if cur_safe == True:
            self.move_to_position(joint_angles, timeout, threshold, test, is_safe=True)
        else:
            print("Robot will hit the table!!! Aborting the current configuration.")

    def safe_set_joint_positions_velocities(self, positions, velocities):
        # The safety layer
        cur_safe = self.safe.test_new_configuration(positions)

        # Compute if the pose is safe
        pose_thresh = 0.25 # in rad
        cur_pose = self.get_positions(False)

        pose_dist = np.linalg.norm(positions - cur_pose)

        # Comput if the velocity is safe
        vel_thresh = 2.0
        cur_vel = self.get_velocities(False)

        vel_dist = np.linalg.norm(velocities - cur_vel)

        if pose_dist > pose_thresh:
            print("Next provided pose is too far. Aborting the current configuration")
        elif vel_dist > vel_thresh:
            print("Next provided velocity is too fast. Aborting the current configruation")
        elif cur_safe == False:
            print("Robot will hit the table!!! Aborting the current configuration.")
        else:
            self.set_joint_positions_velocities(positions, velocities, is_safe=True) # for impedance control
