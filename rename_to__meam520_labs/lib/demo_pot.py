import numpy as np
from math import pi, acos
# from scipy.linalg import null_space
from copy import deepcopy
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
import os 
import sys
import json
'''
from lib.calcJacobian import calcJacobian
from lib.calculateFK import FK
from lib.detectCollision import detectCollision
from lib.loadmap import loadmap
'''

from calcJacobian import calcJacobian
from calculateFK import FK
# from lib.detectCollision import detectCollision
from loadmap import loadmap

DEBUG = False
if DEBUG:
    import pdb 

# test_file = './test_pf_00.json'

class PotentialFieldPlanner:

    fk = FK()
    cnt = 0
    def __init__(self, tol=1e-4, max_steps=3000, min_step_size=5e-3):
        """
        Constructs a potential field planner with solver parameters.

        PARAMETERS:
        tol - the maximum distance between two joint sets
        max_steps - number of iterations before the algorithm must terminate
        min_step_size - the minimum step size before concluding that the
        optimizer has converged
        """

        # YOU MAY NEED TO CHANGE THESE PARAMETERS

        # solver parameters
        self.tol = tol
        self.max_steps = max_steps
        self.min_step_size = min_step_size


    ######################
    ## Helper Functions ##
    ######################
    # The following functions are provided to you to help you to better structure your code
    # You don't necessarily have to use them. You can also edit them to fit your own situation 

    @staticmethod
    def attractive_force(target, current):
        """
        Helper function for computing the attactive force between the current position and
        the target position for one joint. Computes the attractive force vector between the 
        target joint position and the current joint position 

        INPUTS:
        target - 3x1 numpy array representing the desired joint position in the world frame
        current - 3x1 numpy array representing the current joint position in the world frame

        OUTPUTS:
        att_f - 3x1 numpy array representing the force vector that pulls the joint 
        from the current position to the target position 
        """

        ## STUDENT CODE STARTS HERE

        att_f = np.zeros((3, 1)) 

        # parameters 
        d = 0.1 # distance threshold between conic well/parabolic well
        zeta = 10 # parabolic well field strength

        # distance vector 
        dist_vec = current - target
        dist_norm = np.linalg.norm(dist_vec, axis=0)
        if dist_norm < 1e-4: 
           # do nothing 
           att_f = att_f.flatten()
        else:
            if dist_norm >= d: 
                # print('conic well')
                att_f = -dist_vec/dist_norm
            else:
                # print('parabolic well')
                att_f = -zeta*dist_vec

        ## END STUDENT CODE
        return att_f

    @staticmethod
    def repulsive_force(obstacle, current, unitvec=np.zeros((3,1)), c_dist=0):
        """
        Helper function for computing the repulsive force between the current position
        of one joint and one obstacle. Computes the repulsive force vector between the 
        obstacle and the current joint position 

        INPUTS:
        obstacle - 1x6 numpy array representing the an obstacle box in the world frame
        current - 3x1 numpy array representing the current joint position in the world frame
        unitvec - 3x1 numpy array representing the unit vector from the current joint position 
        to the closest point on the obstacle box 

        OUTPUTS:
        rep_f - 3x1 numpy array representing the force vector that pushes the joint 
        from the obstacle
        """

        ## STUDENT CODE STARTS HERE

        rep_f = np.zeros((3, 1)) 
        
        # parameters 
        d = 0.05 # distance threshold between zero/repulsive field
        eta = 0.5 # repulsive field strength

        # distance vector 
        # dist_vec = (obstacle[0:3] + obstacle[3:])/2 - current
        # dist_norm = np.linalg.norm(dist_vec, axis=0)
        # diag_norm = np.linalg.norm(obstacle[0:3] - (obstacle[0:3] + obstacle[3:])/2, axis=0)

        if c_dist > d: 
            rep_f = np.zeros((3,1))
        else:
            rep_f = -eta*(1/c_dist - 1/d)/(c_dist**2)*unitvec.reshape((3,1))

        ## END STUDENT CODE

        return rep_f

    @staticmethod
    def dist_point2box(p, box):
        """
        Helper function for the computation of repulsive forces. Computes the closest point
        on the box to a given point 
    
        INPUTS:
        p - nx3 numpy array of points [x,y,z]
        box - 1x6 numpy array of minimum and maximum points of box

        OUTPUTS:
        dist - nx1 numpy array of distance between the points and the box
                dist > 0 point outside
                dist = 0 point is on or inside box
        unit - nx3 numpy array where each row is the corresponding unit vector 
        from the point to the closest spot on the box
            norm(unit) = 1 point is outside the box
            norm(unit)= 0 point is on/inside the box

         Method from MultiRRomero
         @ https://stackoverflow.com/questions/5254838/
         calculating-distance-between-a-point-and-a-rectangular-box-nearest-point
        """
        # THIS FUNCTION HAS BEEN FULLY IMPLEMENTED FOR YOU

        # Get box info
        print(type(box))
        print(box.shape)
        print(box)
        print(box[0])
        print(box[1])
        print(box[2])
        print(box[3])
        print(box[4])

        boxMin = np.array([box[0], box[1], box[2]])
        boxMax = np.array([box[3], box[4], box[5]])
        boxCenter = boxMin*0.5 + boxMax*0.5
        p = np.array(p)

        # Get distance info from point to box boundary
        dx = np.amax(np.vstack([boxMin[0] - p[:, 0], p[:, 0] - boxMax[0], np.zeros(p[:, 0].shape)]).T, 1)
        dy = np.amax(np.vstack([boxMin[1] - p[:, 1], p[:, 1] - boxMax[1], np.zeros(p[:, 1].shape)]).T, 1)
        dz = np.amax(np.vstack([boxMin[2] - p[:, 2], p[:, 2] - boxMax[2], np.zeros(p[:, 2].shape)]).T, 1)

        # convert to distance
        distances = np.vstack([dx, dy, dz]).T
        dist = np.linalg.norm(distances, axis=1)

        # Figure out the signs
        signs = np.sign(boxCenter-p)

        # Calculate unit vector and replace with
        unit = distances / dist[:, np.newaxis] * signs
        unit[np.isnan(unit)] = 0
        unit[np.isinf(unit)] = 0
        return dist, unit

    def compute_forces(self, target, obstacle, current):
        """
        Helper function for the computation of forces on every joints. Computes the sum 
        of forces (attactive, repulsive) on each joint. 

        INPUTS:
        target - 3x7 numpy array representing the desired joint/end effector positions 
        in the world frame
        obstacle - nx6 numpy array representing the obstacle box min and max positions
        in the world frame
        current- 3x7 numpy array representing the current joint/end effector positions 
        in the world frame

        OUTPUTS:
        joint_forces - 3x7 numpy array representing the force vectors on each 
        joint/end effector
        """

        ## STUDENT CODE STARTS HERE
        joint_forces = np.zeros((3, 7)) 
        
        att_f = np.zeros((3,7))
        # iterate over joints/end effector 
        if self.cnt < 1e4 or self.cnt > 2e4:
            self.min_step_size = 1e-3
            for i in range(0, target.shape[1]):
                att_f_i = self.attractive_force(target[:,i], current[:,i])
                att_f[:, i] = att_f_i
        else:
            self.min_step_size = 1e-2
            for i in range(0, target.shape[1]):
                if i == 0: 
                    pre_joint_dist = 0
                else: 
                    pre_joint_dist = np.linalg.norm(target[:,i-1]-current[:,i-1], axis=0)
                if pre_joint_dist < 1e-2:
                    att_f_i = self.attractive_force(target[:,i], current[:,i])
                    att_f[:, i] = att_f_i
                else:
                    break
        # print('att f', att_f)
        ''' 
        # verify attractive force 
        att_pos = target - current
        dir_arr = np.multiply(att_f, att_pos)
        if dir_arr.any() < 0:
            raise Exception('Attractive force direction is not correct')
        '''
        # print('ratio', np.divide(att_f, att_pos))
        # print('f', att_f)
        # pdb.set_trace()
        
        rep_f = np.zeros((3,7))
        obstacle = np.asarray(obstacle)
        # iterate over obstacles
        for i in range(obstacle.shape[0]):
            rep_f_i = np.zeros((3,7))
            c_dist, unit_vecs = self.dist_point2box(current.transpose(), obstacle[i, :])
            '''
            if i == 0:
                print('obstacle' + str(i), c_dist[-1])
                print('obstacle range', obstacle[i, :])
                end_pos = current[:, -1].transpose()
                obs0 = (obstacle[i, 0:3] + obstacle[0, 3:])/2
                print('end pos', end_pos)
                # print('end dist', end_pos - obs0)
                print('end unit', unit_vecs[-1, :])
            '''
            # iterate over joints/end effector 
            for j in range(current.shape[1]):
                rep_f_ij = self.repulsive_force(obstacle[i, :], current[:, j], unit_vecs[j, :], c_dist[j])
                rep_f_i[:, j] = rep_f_ij.flatten()
            '''
            # verify repulsive force 
            rep_pos = current - (obstacle[i, 0:3].reshape((3,1)) +\
                    obstacle[i, 3:].reshape((3,1)))/2
            dir_arr = np.multiply(rep_f_i, rep_pos)
            if dir_arr.any() < 0:
                raise Exception('Repulsive force direction is not correct')
            '''
            rep_f += rep_f_i # verify attractive force 
        

        # print(rep_f)
        # print('rep f', rep_f)
        # print('end rep f', np.linalg.norm(rep_f[:, -1], axis=0))
        joint_forces = att_f + rep_f
        ## END STUDENT CODE

        return joint_forces
    
    @staticmethod
    def compute_torques(joint_forces, q):
        """
        Helper function for converting joint forces to joint torques. Computes the sum 
        of torques on each joint.

        INPUTS:
        joint_forces - 3x7 numpy array representing the force vectors on each 
        joint/end effector
        q - 1x7 numpy array representing the current joint angles

        OUTPUTS:
        joint_torques - 1x7 numpy array representing the torques on each joint 
        """

        ## STUDENT CODE STARTS HERE

        joint_torques = np.zeros((1, 7)) 
        
        # compute jacobian matrix
        # print('J')
        J = calcJacobian(q)
        for i in range(joint_forces.shape[1]):
            J_i = J[i]
            torque_i = J_i.transpose() @ joint_forces[:, i]
            joint_torques += torque_i 
            # print(J_i)
            # print(torque_i)
        # pdb.set_trace() 
        ## END STUDENT CODE

        return joint_torques

    @staticmethod
    def q_distance(target, current):
        """
        Helper function which computes the distance between any two
        vectors.

        This data can be used to decide whether two joint sets can be
        considered equal within a certain tolerance.

        INPUTS:
        target - 1x7 numpy array representing some joint angles
        current - 1x7 numpy array representing some joint angles

        OUTPUTS:
        distance - the distance between the target and the current joint sets 

        """

        ## STUDENT CODE STARTS HERE
        target = target.reshape((1, -1)) 
        current = current.reshape((1, -1)) 
        
        distance = np.linalg.norm(target-current, axis=1)

        ## END STUDENT CODE

        return distance
 
    def are_joints_equal(self, target, current):
        """
        Helper function which computes the distance between any two
        vectors.

        This data can be used to decide whether two joint sets can be
        considered equal within a certain tolerance.

        INPUTS:
        target - 1x7 numpy array representing some joint angles
        current - 1x7 numpy array representing some joint angles

        OUTPUTS:
        distance - the distance between the target and the current joint sets 

        """

        ## STUDENT CODE STARTS HERE
        target = target.flatten()
        current = current.flatten()
        _, joints_target, _  = self.fk.forward(target)
        _, joints_current, _ = self.fk.forward(current)

        distance = np.linalg.norm(joints_target - joints_current, axis=1)
        
        state = True
        if distance.any() > 1e-4:
            state = False
        ## END STUDENT CODE

        return state

    def compute_gradient(self, q, target, map_struct):
        """
        Computes the joint gradient step to move the current joint positions to the
        next set of joint positions which leads to a closer configuration to the goal 
        configuration 

        INPUTS:
        q - 1x7 numpy array. the current joint configuration, a "best guess" so far for the final answer
        target - 1x7 numpy array containing the desired joint angles
        map_struct - a map struct containing the obstacle box min and max positions

        OUTPUTS:
        dq - 1x7 numpy array. a desired joint velocity to perform this task
        """

        ## STUDENT CODE STARTS HERE

        dq = np.zeros((1, 7))
        q_pos = target - q

        # get joint positions 
        q = np.ravel(q)
        _, current, _ = self.fk.forward(q)
        _, target, _ = self.fk.forward(target)
        #current[5, :]=  t6_current[0, 0:3]
        #target[5, :] = t6_target[0, 0:3]
        #print('pos')
        #print(target - current)
        
        #print('cpos')
        #print(current)

        # extrace obstacles
        obstacles = map_struct[0]
        
        # compute joint forces 
        joint_forces = self.compute_forces(target[1:, :].transpose(),\
                obstacles, current[1:, :].transpose())
        # print('force')
        # print(joint_forces)

        
        # compute torques 
        joint_torques = self.compute_torques(joint_forces, q)
        #print('joint torque')
        #print(joint_torques)
        #pdb.set_trace() 

        # gradient 
        torque_norm = np.linalg.norm(joint_torques, axis=1)
        if torque_norm == 0: 
            pass
        else:
            dq =  joint_torques/np.linalg.norm(joint_torques, axis=1)
        
        # print('torque', joint_torques)
        # verify joint velocity 
        q_dir = np.multiply(q_pos, dq)
        '''
        if q_dir.any() < 0:
            raise Exception('Joint velocity direction is incorrect')
        '''
        ## END STUDENT CODE

        return dq

    ###############################
    ### Potential Feild Solver  ###
    ###############################

    def plan(self, map_struct, start, goal):
        """
        Uses potential field to move the Panda robot arm from the startng configuration to
        the goal configuration.

        INPUTS:
        map_struct - a map struct containing min and max positions of obstacle boxes 
        start - 1x7 numpy array representing the starting joint angles for a configuration 
        goal - 1x7 numpy array representing the desired joint angles for a configuration

        OUTPUTS:
        q - nx7 numpy array of joint angles [q0, q1, q2, q3, q4, q5, q6]. This should contain
        all the joint angles throughout the path of the planner. The first row of q should be
        the starting joint angles and the last row of q should be the goal joint angles. 
        """

        q_path = np.array([]).reshape(0,7)
        
        q_path = np.vstack((q_path, start))
        q = start.reshape((1,7))
        while True:

            ## STUDENT CODE STARTS HERE
            
            # The following comments are hints to help you to implement the planner
            # You don't necessarily have to follow these steps to complete your code 
            
            # Compute gradient 
            dq = self.compute_gradient(q, goal, map_struct)

            # next step 
            q += self.min_step_size*dq

            # Termination Conditions
            if self.q_distance(q, goal) < 1e-2 or \
                    self.cnt > self.max_steps or \
                    self.are_joints_equal(goal, q):
                break 

            # YOU NEED TO CHECK FOR COLLISIONS WITH OBSTACLES
            # state = detectCollision(points1, points, box)
            '''
            state = False
            if state: 
                raise Exception('Step size is too large. The step collides with obstacles')
            '''
            # YOU MAY NEED TO DEAL WITH LOCAL MINIMA HERE
            # TODO: when detect a local minima, implement a random walk
                        
            q_path = np.vstack((q_path, q))

            q_diff = self.q_distance(q, goal)
            self.cnt += 1
            '''
            if self.cnt % 100 ==0:
                print('iteration' + str(self.cnt), q_diff)
                print('dq', dq)
                print('step', dq*self.min_step_size)
                print('q', q)
                print()
            '''
            #pdb.set_trace()
            ## END STUDENT CODE

        return q_path

    def visualize(self, q, goal, obstacles):
        """
        Visualizes the motion of the Panda arm during the potential field planning.

        INPUTS:
        q - nx7 numpy array of joint angles [q0, q1, q2, q3, q4, q5, q6]. This should contain
        all the joint angles throughout the path of the planner. The first row of q should be
        the starting joint angles and the last row of q should be the goal joint angles. 
        """
        fig = plt.figure()
        ax = fig.gca(projection='3d')
        ax.set_xlim(-1, 1)
        ax.set_ylim(-1, 1)
        ax.set_zlim(0, 1)
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_zlabel('z')

        def cuboid_data(center, size):
            """
               Create a data array for cuboid plotting.


               ============= ================================================
               Argument      Description
               ============= ================================================
               center        center of the cuboid, triple
               size          size of the cuboid, triple, (x_length,y_width,z_height)
               :type size: tuple, numpy.array, list
               :param size: size of the cuboid, triple, (x_length,y_width,z_height)
               :type center: tuple, numpy.array, list
               :param center: center of the cuboid, triple, (x,y,z)


              """
            # suppose axis direction: x: to left; y: to inside; z: to upper
            # get the (left, outside, bottom) point
            o = [a - b / 2 for a, b in zip(center, size)]
            # get the length, width, and height
            l, w, h = size
            x = [[o[0], o[0] + l, o[0] + l, o[0], o[0]],  # x coordinate of points in bottom surface
                 [o[0], o[0] + l, o[0] + l, o[0], o[0]],  # x coordinate of points in upper surface
                 [o[0], o[0] + l, o[0] + l, o[0], o[0]],  # x coordinate of points in outside surface
                 [o[0], o[0] + l, o[0] + l, o[0], o[0]]]  # x coordinate of points in inside surface
            y = [[o[1], o[1], o[1] + w, o[1] + w, o[1]],  # y coordinate of points in bottom surface
                 [o[1], o[1], o[1] + w, o[1] + w, o[1]],  # y coordinate of points in upper surface
                 [o[1], o[1], o[1], o[1], o[1]],          # y coordinate of points in outside surface
                 [o[1] + w, o[1] + w, o[1] + w, o[1] + w, o[1] + w]]    # y coordinate of points in inside surface
            z = [[o[2], o[2], o[2], o[2], o[2]],                        # z coordinate of points in bottom surface
                 [o[2] + h, o[2] + h, o[2] + h, o[2] + h, o[2] + h],    # z coordinate of points in upper surface
                 [o[2], o[2], o[2] + h, o[2] + h, o[2]],                # z coordinate of points in outside surface
                 [o[2], o[2], o[2] + h, o[2] + h, o[2]]]                # z coordinate of points in inside surface
            return x, y, z

        obstacles = np.asarray(obstacles)
        for i in range(obstacles.shape[0]):
            center = [(obstacles[i,0]+obstacles[i,3])/2, \
                    (obstacles[i,1]+obstacles[i,4])/2, \
                    (obstacles[i,2]+obstacles[i,5])/2]
            length = obstacles[i,3] - obstacles[i,0]
            width = obstacles[i,4] - obstacles[i,1]
            height = obstacles[i,5] - obstacles[i,2]
            X, Y, Z = cuboid_data(center, (length, width, height))
            X = np.asarray(X)
            Y = np.asarray(Y)
            Z = np.asarray(Z)
            ax.plot_surface(X, Y, Z, color='r', rstride=1, cstride=1, alpha=0.5)


        # goal configuration  
        _, joint_pose, _ = self.fk.forward(goal)
        x = joint_pose[:, 0]
        y = joint_pose[:, 1]
        z = joint_pose[:, 2]

        ax.plot(x, y, z, marker='.', markersize = 10, markerfacecolor='r', label='goal')
        
        # starting configuration  
        q0 = q[0, :]
        _, joint_pos0, _ = self.fk.forward(q0)
        x = joint_pos0[:, 0]
        y = joint_pos0[:, 1]
        z = joint_pos0[:, 2]

        line, = ax.plot(x, y, z, marker='.', markersize = 10, markerfacecolor='r', label='current')
       
        def update(q):
            _, joint_pos, _ = self.fk.forward(q)
            x = joint_pos[:, 0]
            y = joint_pos[:, 1]
            z = joint_pos[:, 2]
            
            line.set_xdata(x)  # update the data.
            line.set_ydata(y)  # update the data.
            line.set_3d_properties(z)  # update the data.

            ax.scatter(x[-1], y[-1], z[-1], c='k', marker='.')
            return line

        q_sample = q[0::50, :]
        # print(q_sample.shape)
        a = animation.FuncAnimation(fig, update, frames=q_sample, repeat=False, cache_frame_data=False)
        
        ax.legend() 
        plt.show()
        
################################
## Simple Testing Environment ##
################################

if __name__ == "__main__":

    np.set_printoptions(suppress=True,precision=6)
    lower_lim = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973]).reshape((1, 7))    # Lower joint limits in radians ** This does not include gripper
    upper_lim = np.array([ 2.8973,  1.7628,  2.8973, -0.0698,  2.8973,  np.pi,  2.8973]).reshape((1, 7))          # Upper joint limits in radians (grip in mm)
  

    planner = PotentialFieldPlanner()
    ''' 
    # test attractive force field 
    target = np.zeros((3,1))
    F = np.zeros((100, 1))
    for i in range(100):
        current = i*np.array([1, 0, 0])
        current = current.reshape((3,1))/500
        att_f = PotentialFieldPlanner.attractive_force(target, current)
        f = np.linalg.norm(att_f, axis=0)
        F[i] = f
    
    plt.figure()
    plt.plot(np.linspace(0, 0.2, 100), F)
    plt.show()
    '''  
    ''' 
    # test repulsive force field 
    map_struct = loadmap("../maps/map4.txt")
    obstacle = map_struct[0][0,:] 
    F = np.zeros((100, 1))
    for i in range(100): 
        current = (obstacle[0:3] + obstacle[3:])/2
        current[1] = obstacle[4]
        current = current.reshape((1,3))
        add = i/500*np.array([0, 1, 0]).reshape((1,3))
        current += add
        dist, unit = PotentialFieldPlanner.dist_point2box(current, obstacle)
        rep_f = PotentialFieldPlanner.repulsive_force(obstacle, current, unit, dist)
        F[i] = np.linalg.norm(rep_f, axis=0)
    
    plt.figure()
    plt.plot(np.linspace(0, 0.2, 100), F)
    plt.show()
    '''
    # with open(test_file) as tf: 
    #    test_dict = json.load(tf)
    
    # mapfile = '../maps/' + test_dict['map']
    # start = np.asarray(test_dict['start'])
    # goal = np.asarray(test_dict['goal'])
    mapfile = '../maps/map4.txt'
    
    # inputs 
    map_struct = loadmap(mapfile)
    #start = np.array([0,   0,  0,   0,    0,  0,  0.0])
    # goal =  np.array([0,   0,  0,   0,    0,  np.pi/2,  0.0])
    q1 = np.random.uniform(lower_lim[0, 0], upper_lim[0, 0], 1)[0]
    q2 = np.random.uniform(lower_lim[0, 1], upper_lim[0, 1], 1)[0]
    q3 = np.random.uniform(lower_lim[0, 2], upper_lim[0, 2], 1)[0]
    q4 = np.random.uniform(lower_lim[0, 3], upper_lim[0, 3], 1)[0]
    q5 = np.random.uniform(lower_lim[0, 4], upper_lim[0, 4], 1)[0]
    q6 = np.random.uniform(lower_lim[0, 5], upper_lim[0, 5], 1)[0]
    q7 = 0
    #goal = np.array([q1, q2, q3, q4, q5, q6, q7])
    #goal =  np.array([0,   -np.pi/3,  0,   -np.pi,    0,  3*np.pi/3,  0.0])
    # goal = np.array([-1.880829, -1.480869, -0.663466, -2.212905,  2.594657,  2.723194,  0.])
    start = np.array([0,-1,0,-2,0,1.57,0])
    goal =  np.array([-1.2, 1.57 , 1.57, -2.07, -1.57, 1.57, 0.7])
    # potential field planning
    q_path = planner.plan(deepcopy(map_struct), deepcopy(start), deepcopy(goal))
    print('goal', goal)
    planner.visualize(q_path, goal, map_struct[0])

    # show results
    # for i in range(q_path.shape[0]):
    #    error = PotentialFieldPlanner.q_distance(q_path[i, :], goal)
    #    print('iteration:',i,' q =', q_path[i, :], ' error=error'.format(error=error))

    # print("q path: ", q_path)
    