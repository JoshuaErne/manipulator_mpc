#!/usr/bin/env python

import numpy as np
from math import pi, acos, asin
from scipy.linalg import null_space

from lib.calcJacobian import calcJacobian
from lib.calculateFK import FK
from lib.IK_velocity import IK_velocity

class IK:

    # JOINT LIMITS
    lower = np.array([-2.8973,-1.7628,-2.8973,-3.0718,-2.8973,-0.0175,-2.8973])
    upper = np.array([2.8973,1.7628,2.8973,-0.0698,2.8973,3.7525,2.8973])

    center = lower + (upper - lower) / 2 # compute middle of range of motion of each joint
    fk = FK()

    def __init__(self,linear_tol=1e-4, angular_tol=1e-3, max_steps=500, min_step_size=1e-5):
        """
        Constructs an optimization-based IK solver with given solver parameters.
        Default parameters are tuned to reasonable values.

        PARAMETERS:
        linear_tol - the maximum distance in meters between the target end
        effector origin and actual end effector origin for a solution to be
        considered successful
        angular_tol - the maximum angle of rotation in radians between the target
        end effector frame and actual end effector frame for a solution to be
        considered successful
        max_steps - number of iterations before the algorithm must terminate
        min_step_size - the minimum step size before concluding that the
        optimizer has converged
        """

        # THIS FUNCTION HAS BEEN FULLY IMPLEMENTED FOR YOU

        # solver parameters
        self.linear_tol = linear_tol
        self.angular_tol = angular_tol
        self.max_steps = max_steps
        self.min_step_size = min_step_size


    ######################
    ## Helper Functions ##
    ######################

    @staticmethod
    def displacement_and_axis(target, current):
        """
        Helper function for the End Effector Task. Computes the displacement
        vector and axis of rotation from the current frame to the target frame

        This data can also be interpreted as an end effector velocity which will
        bring the end effector closer to the target position and orientation.

        INPUTS:
        target - 4x4 numpy array representing the desired transformation from
        end effector to world
        current - 4x4 numpy array representing the "current" end effector orientation

        OUTPUTS:
        displacement - a 3-element numpy array containing the displacement from
        the current frame to the target frame, expressed in the world frame
        axis - a 3-element numpy array containing the axis of the rotation from
        the current frame to the end effector frame. The magnitude of this vector
        must be sin(angle), where angle is the angle of rotation around this axis
        """

        ## STUDENT CODE STARTS HERE

        # compute displacement vector
        current_pos = np.matmul(current, np.array([0, 0, 0, 1]))
        target_pos  = np.matmul(target, np.array([0, 0, 0, 1]))
        displacement = (target_pos - current_pos)[:3]

        # get rotation matrices
        R_c_w = current[:3, :3]
        R_t_w = target[:3, :3]
        R_w_t = R_t_w.T

        # relative rotation from current to target
        R_c_t = np.matmul(R_w_t, R_c_w)

        # get skew symmetic matrix
        S = 0.5 * (R_c_t - R_c_t.T)

        # get vector a from S
        a = np.array([
            -S[2, 1],
            -S[0, 2],
            -S[1, 0]
        ])

        # transfrom axis to world coordinates
        axis = np.matmul(R_t_w, a)

        ## END STUDENT CODE

        return displacement, axis

    @staticmethod
    def distance_and_angle(G, H):
        """
        Helper function which computes the distance and angle between any two
        transforms.

        This data can be used to decide whether two transforms can be
        considered equal within a certain linear and angular tolerance.

        Be careful! Using the axis output of displacement_and_axis to compute
        the angle will result in incorrect results when |angle| > pi/2

        INPUTS:
        G - a 4x4 numpy array representing some homogenous transformation
        H - a 4x4 numpy array representing some homogenous transformation

        OUTPUTS:
        distance - the distance in meters between the origins of G & H
        angle - the angle in radians between the orientations of G & H


        """

        ## STUDENT CODE STARTS HERE

        # compute distance
        displacement, axis = IK.displacement_and_axis(G, H)
        distance = np.linalg.norm(displacement)

        # compute angle
        angle = asin(np.clip(np.linalg.norm(axis), -1, 1))

        ## END STUDENT CODE

        return distance, angle

    def is_valid_solution(self,q,target):
        """
        Given a candidate solution, determine if it achieves the primary task
        and also respects the joint limits.

        INPUTS
        q - the candidate solution, namely the joint angles
        target - 4x4 numpy array representing the desired transformation from
        end effector to world

        OUTPUTS:
        success - a Boolean which is True if and only if the candidate solution
        produces an end effector pose which is within the given linear and
        angular tolerances of the target pose, and also respects the joint
        limits.
        """

        ## STUDENT CODE STARTS HERE

        # check joint limits
        q = np.array(q)
        if ((q < IK.lower) | (q > IK.upper)).any():
            print("[IK SOLVER FAILURE] Joint limit exceeded!")
            return False

        # check distance and angle
        _, T0e = IK.fk.forward(q)
        distance, angle = IK.distance_and_angle(target, T0e)

        if (distance > self.linear_tol) and (angle > self.angular_tol):
            print("[IK SOLVER FAILURE] Both linear and angular tolerance exceeded!")
        elif distance > self.linear_tol:
            print("[IK SOLVER FAILURE] Linear tolerance exceeded!")
        elif angle > self.angular_tol:
            print("[IK SOLVER FAILURE] Angular tolerance exceeded!")

        success = not bool((distance > self.linear_tol) or (angle > self.angular_tol))
        
        ## END STUDENT CODE

        return success

    ####################
    ## Task Functions ##
    ####################

    @staticmethod
    def end_effector_task(q,target):
        """
        Primary task for IK solver. Computes a joint velocity which will reduce
        the error between the target end effector pose and the current end
        effector pose (corresponding to configuration q).

        INPUTS:
        q - the current joint configuration, a "best guess" so far for the final answer
        target - a 4x4 numpy array containing the desired end effector pose

        OUTPUTS:
        dq - a desired joint velocity to perform this task, which will smoothly
        decay to zero magnitude as the task is achieved
        """

        ## STUDENT CODE STARTS HERE

        _, T0e = IK.fk.forward(q)
        displacement, axis = IK.displacement_and_axis(target, T0e)
        dq = IK_velocity(q, displacement, axis)

        ## END STUDENT CODE

        return dq

    @staticmethod
    def joint_centering_task(q,rate=5e-1):
        """
        Secondary task for IK solver. Computes a joint velocity which will
        reduce the offset between each joint's angle and the center of its range
        of motion. This secondary task acts as a "soft constraint" which
        encourages the solver to choose solutions within the allowed range of
        motion for the joints.

        INPUTS:
        q - the joint angles
        rate - a tunable parameter dictating how quickly to try to center the
        joints. Turning this parameter improves convergence behavior for the
        primary task, but also requires more solver iterations.

        OUTPUTS:
        dq - a desired joint velocity to perform this task, which will smoothly
        decay to zero magnitude as the task is achieved
        """

        # THIS FUNCTION HAS BEEN FULLY IMPLEMENTED FOR YOU

        # normalize the offsets of all joints to range from -1 to 1 within the allowed range
        offset = 2 * (q - IK.center) / (IK.upper - IK.lower)
        dq = rate * -offset # proportional term (implied quadratic cost)

        return dq

    ###############################
    ## Inverse Kinematics Solver ##
    ###############################

    def inverse(self, target, seed):
        """
        Uses gradient descent to solve the full inverse kinematics of the Panda robot.

        INPUTS:
        target - 4x4 numpy array representing the desired transformation from
        end effector to world
        seed - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6], which
        is the "initial guess" from which to proceed with optimization

        OUTPUTS:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6], giving the
        solution if success is True or the closest guess if success is False.
        success - True if the IK algorithm successfully found a configuration
        which achieves the target within the given tolerance. Otherwise False
        rollout - a list containing the guess for q at each iteration of the algorithm
        """

        q = seed
        rollout = []

        while True:

            rollout.append(q)

            # Primary Task - Achieve End Effector Pose
            dq_ik = self.end_effector_task(q, target)

            # Secondary Task - Center Joints
            dq_center = self.joint_centering_task(q)

            # Project dq_center to null space of jacobian
            nullspace = null_space(calcJacobian(q)).ravel()
            dq_center_proj = nullspace * np.dot(dq_center, nullspace) / np.dot(nullspace, nullspace)

            # Task Prioritization
            dq = dq_ik + dq_center_proj

            # Termination Conditions
            exit_conditions = (len(rollout) == self.max_steps) or (np.linalg.norm(dq) < self.min_step_size)
            if exit_conditions:
                break
            
            # Clip joint angles to be within joint limits
            q = q + dq
            q[:-1] = np.clip(q, IK.lower, IK.upper)[:-1]

        # if angle is in joint_7's deadzone, clip it
        if (abs(q[-1]) - IK.upper[-1]) < 0.4887:
            q[-1] = np.clip(q[-1], IK.lower[-1], IK.upper[-1])

        # if angle is greater than 360, wrap it around
        if q[-1] < -np.radians(360):
            q[-1] = q[-1] % -np.radians(360)
        elif q[-1] > np.radians(360):
            q[-1] = q[-1] % np.radians(360)

        # if angle exceeds joint limit, wrap it around
        if q[-1] < IK.lower[-1]:
            print("before", q[-1])
            q[-1] = np.radians(360) + q[-1]
            print("after", q[-1])
        elif q[-1] > IK.upper[-1]:
            print("before", q[-1])
            q[-1] = -np.radians(360) + q[-1]
            print("after", q[-1])

        success = self.is_valid_solution(q,target)
        return q, success, rollout

################################
## Simple Testing Environment ##
################################

if __name__ == "__main__":

    np.set_printoptions(suppress=True,precision=5)

    ik = IK()

    # matches figure in the handout
    seed = np.array([0,0,0,-pi/2,0,pi/2,pi/4])

    target =np.array([[ 1.00000910e+00,  3.17312491e-17,  0.00000000e+00 , 5.54000000e-01],
                      [-3.17312491e-17, -1.00000910e+00,  0.00000000e+00 , 0.00000000e+00],
                      [ 0.00000000e+00,  0.00000000e+00, -1.00000000e+00 , 5.21000000e-01],
                      [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00 , 1.00000000e+00]])

    q, success, rollout = ik.inverse(target, seed)

    for i, q in enumerate(rollout):
        joints, pose = ik.fk.forward(q)
        d, ang = IK.distance_and_angle(target,pose)
        print('iteration:',i,' q =',q, ' d={d:3.4f}  ang={ang:3.3f}'.format(d=d,ang=ang))

    print("Success: ",success)
    print("Solution: ",q)
    print("Iterations:", len(rollout))
