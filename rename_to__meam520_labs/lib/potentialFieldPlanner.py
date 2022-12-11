import numpy as np
from math import pi, acos
# from scipy.linalg import null_space
from copy import deepcopy
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
import matplotlib.pyplot as plt
import random


class PotentialFieldPlanner:

    # JOINT LIMITS
    lower = np.array([-2.8973,-1.7628,-2.8973,-3.0718,-2.8973,-0.0175,-2.8973])
    upper = np.array([2.8973,1.7628,2.8973,-0.0698,2.8973,3.7525,2.8973])

    center = lower + (upper - lower) / 2 # compute middle of range of motion of each joint
    fk = FK()
    max_steps = 5000
    alpha = 0.001

    def __init__(self, tol=1e-4, max_steps=500, min_step_size=1e-5):
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
        Strength_parm= 6
        x=target-current
        att_f = np.zeros((3, 1)) 
        norm=np.linalg.norm(x,axis=0)
        # print(target)
        # print(current)
        # print(np.linalg.norm(x))

        if norm<= 1e-4:
            att_f=np.zeros((3,1)).flatten()
            # print("norm<= 1e-4")
        if norm > 1e-4 and norm < 0.01:
            att_f=Strength_parm*x
            # print("norm > 1e-4 and norm < 0.01")
        if norm >= 0.01:
            att_f= x/norm  #conic well 
            # print("conic well")
        ## END STUDENT CODE
        # print(att_f)
        return att_f

    @staticmethod
    def repulsive_force(obstacle, current, unitvec=np.zeros((3,1))):
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
        force_const=0.5
        obstacle=np.reshape(obstacle, (1,len(obstacle)))
        current=np.reshape(current, (1,len(current)))
        p0= 0.05   #distance threshold
        rep_f = np.zeros((3, 1))
    
        dist,unitvec = PotentialFieldPlanner.dist_point2box(current,obstacle[0])
        # print("unitvec: ",unitvec)
        # print("dist" , dist[0])
        dist=dist[0]
        if dist <= p0 and dist!= 0:
            rep_f = force_const*(1/dist - 1/p0)*unitvec.reshape((3,1))/(dist**2)
            # print("rep_f: ",force_const*(1/dist - 1/p0)*unitvec.reshape((3,1))/(dist**2))
        if dist > p0:
            rep_f=np.zeros((3,1))
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

    @staticmethod
    def compute_forces(target, obstacle, current):
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
        att_force=np.zeros((3,7))
        total_rep_force=np.zeros((3,7))
        #step_size= 1e-2
        for i in range (0,7):
            
            att_force[:,i]= PotentialFieldPlanner.attractive_force(target[:,i],current[:,i]).flatten()
        # print(att_force)
        # raise
        for j in range (0,np.shape(obstacle)[0]):   
            rep_force=np.zeros((3,7))
            for k in range (0,7):
                rep_force[:,k]= PotentialFieldPlanner.repulsive_force(obstacle[j],current[:,k]).flatten()
            # print("rep_force: ",rep_force)
            total_rep_force+=rep_force
        # print(total_rep_force)
        # raise

        # print("Repulsive force: ", total_rep_force)
        # print("Attraction force: ", att_force)
        joint_forces=total_rep_force + att_force    
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

        joint_torques = np.zeros((7)) 
        Jac= calcJacobian(q)   #jacobian is 6x7
        jacVel=Jac[0:3,:]      #3x7
        
        for i in range (0,7):
            joint_torques[i]= jacVel[:,i].T @ joint_forces[:,i]
           
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
        ## END STUDENT CODE

        return np.linalg.norm(target-current)
    
    @staticmethod
    def compute_gradient(q, target, map_struct):
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
        fk=FK()
        dq = np.zeros((7))
        obstacle= map_struct[0]
        joint_pos_current, T0e_current= fk.forward(q)
        joint_pos_target, T0e_target= fk.forward(target)
        # print("1: ",joint_pos_target.T)   
        # print("joint_pos_target: ", joint_pos_target.T)   
        # print("joint_pos_current: ",joint_pos_current.T)   
        # print("joint_pos_target: ", joint_pos_target.T[:,1:8])   
        # print("joint_pos_current: ",joint_pos_current.T[:,1:8])   

        force=PotentialFieldPlanner.compute_forces(joint_pos_target.T[:,1:8],obstacle,joint_pos_current.T[:,1:8])
        torque = PotentialFieldPlanner.compute_torques(force,q)
        
        norm_torque= np.linalg.norm(torque)
        
        
        if norm_torque !=0:
            dq= PotentialFieldPlanner.alpha*torque/norm_torque

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

        q_current =start 
        q_path=np.vstack((q_path,q_current))
        error_list = []
        for i in range (0, PotentialFieldPlanner.max_steps):

            ## STUDENT CODE STARTS HERE
            
            # The following comments are hints to help you to implement the planner
            # You don't necessarily have to follow these steps to complete your code 
            
            # Compute gradient 

            dq=PotentialFieldPlanner.compute_gradient(q_current,goal, map_struct)
            # print("dq", dq)

            # TODO: this is how to change your joint angles 
            q_current += dq

            error = PotentialFieldPlanner.q_distance(q_current, goal)
            print('iteration:',i,' q =', q_path[i, :], ' error={error}'.format(error=error))

            error_list.append(error)
            

            # Termination Conditions
            if PotentialFieldPlanner.q_distance(goal, q_current) <0.01: 
                break
            q_path=np.vstack((q_path,q_current))

            # YOU NEED TO CHECK FOR COLLISIONS WITH OBSTACLES
            # TODO: Figure out how to use the provided function 
            if i > 5:
                history_diff = q_path[-5:-1, :] - q_path[-1, :]
                print("history_diff: ", np.linalg.norm(history_diff))
                if np.linalg.norm(history_diff) < 0.001:
                    print("LOCAL MINIMUM!!!")
                    q_current = q_path[-1, :] + random.gauss(0, 0.001)
            # YOU MAY NEED TO DEAL WITH LOCAL MINIMA HERE
            # TODO: when detect a local minima, implement a random walk
            
            ## END STUDENT CODE


        plt.plot(error_list)
        plt.show()

        return q_path

################################
## Simple Testing Environment ##
################################

if __name__ == "__main__":

    # cwd = os.getcwd()
    np.set_printoptions(suppress=True,precision=5)

    planner = PotentialFieldPlanner()
    
    # inputs 
    start = np.array([0,-1,0,-2,0,1.57,0])
    map_struct = loadmap('../maps/map4.txt')
    # print(map_struct[0])
    # start = np.array([0,-1,0,-2,0,1.57,0])
    lower_lim = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973]).reshape((1, 7))    # Lower joint limits in radians ** This does not include gripper
    upper_lim = np.array([ 2.8973,  1.7628,  2.8973, -0.0698,  2.8973,  np.pi,  2.8973]).reshape((1, 7))          # Upper joint limits in radians (grip in mm)
  
    q1 = np.random.uniform(lower_lim[0, 0], upper_lim[0, 0], 1)[0]
    q2 = np.random.uniform(lower_lim[0, 1], upper_lim[0, 1], 1)[0]
    q3 = np.random.uniform(lower_lim[0, 2], upper_lim[0, 2], 1)[0]
    q4 = np.random.uniform(lower_lim[0, 3], upper_lim[0, 3], 1)[0]
    q5 = np.random.uniform(lower_lim[0, 4], upper_lim[0, 4], 1)[0]
    q6 = np.random.uniform(lower_lim[0, 5], upper_lim[0, 5], 1)[0]
    q7 = np.random.uniform(lower_lim[0, 6], upper_lim[0, 6], 1)[0]
    goal = np.array([q1, q2, q3, q4, q5, q6, q7])
    # goal =  np.array([-1.2, 1.57 , 1.57, -2.07, -1.57, 1.57, 0.7])
    
    # potential field planning
    q_path = planner.plan(deepcopy(map_struct), deepcopy(start), deepcopy(goal))
    
    # show results
    # for i in range(q_path.shape[0]):
    #     error = PotentialFieldPlanner.q_distance(q_path[i, :], goal)
    #     print('iteration:',i,' q =', q_path[i, :], ' error={error}'.format(error=error))

    print("q current: ", start)
    print("q goal: ", goal)

    print("q path: ", q_path)