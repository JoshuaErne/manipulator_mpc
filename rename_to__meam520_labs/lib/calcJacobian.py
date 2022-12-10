#!/usr/bin/env python

import numpy as np
from numpy import cos, sin
from math import pi


class FK():

    def __init__(self):
        self.dh_params = self.init_dh_params()
        self.joint_offsets = self.init_joint_offsets()

    def init_dh_params(self):
        """
        Initialize pre-computed dh parameters
        from all intermediate frames in the form
        [a, alpha, d] and theta will be given as input
        """
        dh_params = [
            [0, -np.pi/2, 0.333],
            [0, np.pi/2, 0],
            [0.082, np.pi/2, 0.316],
            [-0.082, -np.pi/2, 0],
            [0, np.pi/2, 0.384],
            [0.088, np.pi/2, 0],
            [0, 0, 0.051 + 0.159]
        ]
        return dh_params

    def init_joint_offsets(self):
        """
        Initialize pre-computed joint position offsets
        relative to intermediate frames defined using
        DH conventions
        """
        joint_offsets = [
            [0, 0, 0.141],
            [0, 0, 0],
            [0, 0, 0.195],
            [0, 0, 0],
            [0, 0, 0.125],
            [0, 0, -0.015],
            [0, 0, 0.051]
        ]
        return joint_offsets

    def build_dh_transform(self, a, alpha, d, theta):
        """
        Helper function to construct transformation matrix
        using DH parameters and conventions
        """
        A = np.array([[cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), a * cos(theta)],
                      [sin(theta), cos(theta) * cos(alpha), -
                       cos(theta) * sin(alpha), a * sin(theta)],
                      [0, sin(alpha), cos(alpha), d],
                      [0, 0, 0, 1]])
        return np.round(A, 5)

    def forward(self, q):
        """
        INPUT:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]

        OUTPUTS:
        jointPositions - 7 x 3 matrix, where each row corresponds to a rotational joint of the robot
                          Each row contains the [x,y,z] coordinates in the world frame of the respective joint's center in meters.
                          The base of the robot is located at [0,0,0].
        T0e       - a 4 x 4 homogeneous transformation matrix,
                          representing the end effector frame expressed in the
                          world frame
        """

        # Your code starts here
        o = np.array([0, 0, 0])
        jointPositions = np.zeros((7, 3))
        T0e = np.identity(4)

        for i in range(7):
            # calculate joint position relative to intermediate frames
            jointPositions[i] = np.matmul(T0e, np.append(
                (o + self.joint_offsets[i]), [1]))[:3]

            # get pre-computed dh params
            a, alpha, d = self.dh_params[i]

            # get homogenous transformation matrix
            T0e = np.matmul(T0e, self.build_dh_transform(a, alpha, d, q[i]))

        # apply 45 degree offset for end-effector frame
        T0e = np.matmul(T0e, self.build_dh_transform(0, 0, 0, -np.pi/4))

        # Your code ends here
        return jointPositions, T0e


def calcJacobian(q):
    """
    Calculate the full Jacobian of the end effector in a given configuration
    :param q: 0 x 7 configuration vector (of joint angles) [q0,q1,q2,q3,q4,q5,q6]
    :return: J - 6 x 7 matrix representing the Jacobian, where the first three
    rows correspond to the linear velocity and the last three rows correspond to
    the angular velocity, expressed in world frame coordinates
    """
    fk = FK()
    joint_positions, T0e = fk.forward(q)

    o_n = np.matmul(T0e, np.array([0, 0, 0, 1]))[:3]
    o_i = (o_n.T - joint_positions)

    z_i = []
    T0e = np.identity(4)
    for i in range(7):
        # apply rotation matrix to z hat
        z = np.matmul(T0e[:3, :3], np.array([0, 0, 1]))
        z_i.append(z / np.linalg.norm(z))

        # forward kinematics to get rotation matrix
        a, alpha, d = fk.dh_params[i]
        T0e = np.matmul(T0e, fk.build_dh_transform(a, alpha, d, q[i]))

    J_v = np.array([np.cross(z_i[i], o_i[i]) for i in range(7)])
    z_i = np.array(z_i)
    J = np.append(J_v, z_i, axis=1).T

    return J

if __name__ == '__main__':
    q= np.array([0, 0, 0, -np.pi/2, 0, np.pi/2, np.pi/4])
    print(np.round(calcJacobian(q),3))