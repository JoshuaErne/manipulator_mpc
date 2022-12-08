#!/usr/bin/env python

import numpy as np
from lib.calcJacobian import calcJacobian


def IK_velocity(q_in, v_in, omega_in):
    """
    :param q: 0 x 7 vector corresponding to the robot's current configuration.
    :param v: The desired linear velocity in the world frame. If any element is
    Nan, then that velocity can be anything
    :param omega: The desired angular velocity in the world frame. If any
    element is Nan, then that velocity is unconstrained i.e. it can be anything
    :return:
    dq - 0 x 7 vector corresponding to the joint velocities. If v and omega
         are infeasible, then dq should minimize the least squares error. If v
         and omega have multiple solutions, then you should select the solution
         that minimizes the l2 norm of dq
    """

    J = calcJacobian(q_in)
    v_w = np.append(v_in, omega_in, axis=0)

    v_w_ = []
    J_ = []
    for i in range(len(v_w)):
        if not np.isnan(v_w[i]):
            v_w_.append(v_w[i])
            J_.append(J[i])

    if (len(v_w_) == 0):
        return np.zeros((7,))

    v_w_ = np.array(v_w_).reshape((len(v_w_), 1))

    dq = np.linalg.lstsq(J_, v_w_, rcond=None)[0][:, 0]

    return dq