#!/usr/bin/env python

import numpy as np

from lib.detectCollision import detectCollision
from lib.loadmap import loadmap
from lib.calculateFK import FK
from copy import deepcopy
from lib.mpc import *

class Node:
    def __init__(self, q, parent=None):
        self.q = q
        self.parent = parent


def isPathCollided(q_a, q_b, obstacles, num_steps=20):
    step = (q_a - q_b) / num_steps
    for i in range(1, 1 + num_steps):
        if isRobotCollided(q_b + (i * step), obstacles):
            return True
    return False


def isRobotCollided(q, obstacles, offset=0.1):
    # get joint positions of current q
    joint_positions, T0e = FK().forward(q)
    end_effector_position = np.matmul(T0e, np.array([0,0,0,1]))[:3].reshape(1, -1)
    joint_positions = np.append(joint_positions, end_effector_position, axis=0)

    # collision if any joint is below ground
    for joint_position in joint_positions:
        if joint_position[2] < 0.0:
            return True

    # account for volume of links
    shifted_joint_positions = [joint_positions] + [
        joint_positions + [offset * x, offset * y, offset * z] \
        for x in [-1, 1] for y in [-1, 1] for z in [-1, 1]
    ]
    points_start = np.vstack([positions[:-1] for positions in shifted_joint_positions])
    points_end   = np.vstack([positions[1:]  for positions in shifted_joint_positions])

    # check self-collision between gripper and base + link 1 of robot arm
    base = np.array([-0.1, -0.1, 0, 0.1, 0.1, 0.333])
    end_effector_start = np.array([positions[-2] for positions in shifted_joint_positions])
    end_effector_end = np.array([positions[-1] for positions in shifted_joint_positions])
    self_collision = np.array(detectCollision(end_effector_start, end_effector_end, base))
    if self_collision.any():
        return True

    # check for collision with each environment obstacle
    for obstacle in obstacles:
        collisions = np.array(detectCollision(points_start, points_end, obstacle))
        if collisions.any():
            return True
    return False


def sampleFreeSpace(obstacles):
    # define joint limits
    lowerLim = np.array([-2.8973,-1.7628,-2.8973,-3.0718,-2.8973,-0.0175,-2.8973])
    upperLim = np.array([2.8973,1.7628,2.8973,-0.0698,2.8973,3.7525,2.8973])

    # randomly sample q within joint limits
    q = np.random.uniform(lowerLim, upperLim)

    # repeat sampling until q is collision free
    while isRobotCollided(q, obstacles):
        q = np.random.uniform(lowerLim, upperLim)
    return q


def getClosestNode(target, nodes):
    closest_node = -1
    closest_dist = 100000
    for idx, node in enumerate(nodes):
        dist = np.linalg.norm(target - node.q)
        if dist < closest_dist:
            closest_dist = dist
            closest_node = idx
    return closest_node


def getPrunedPath(path, obstacles):
    sub_paths = []
    for i in range(len(path) - 2):
        sub_path = path
        for j in range(i + 2, len(path)):
            if not isPathCollided(path[i], path[j], obstacles):
                sub_path = np.vstack((path[:i+1], path[j:]))
        sub_paths.append(sub_path)

    costs = np.array([np.linalg.norm(p[1:] - p[:-1]).sum() for p in sub_paths])
    pruned_path = sub_paths[np.argmin(costs)]
    ########## MPC ISSUE ####################
    # null_point = np.array([1.7708, -1.6215,  1.7325, -0.1372, -0.8897,  2.532,   0.1034])
    # print(null_point)
    # raise
    return pruned_path


def rrt(map, start, goal):
    """
    Implement RRT algorithm in this file.
    :param map:         the map struct
    :param start:       start pose of the robot (0x7).
    :param goal:        goal pose of the robot (0x7).
    :return:            returns an mx7 matrix, where each row consists of the configuration of the Panda at a point on
                        the path. The first row is start and the last row is goal. If no path is found, PATH is empty
    """

    # define hyperparameters
    num_iter = 500

    # start RRT
    T_start, T_goal = [Node(start)], [Node(goal)]
    for iter in range(num_iter):
        # sample from free configuration space
        q = sampleFreeSpace(map.obstacles)

        # get closest node to q in T_start
        idx = getClosestNode(q, T_start)
        q_a = T_start[idx].q

        # check if moving from q to q_a causes collision
        collision_a = isPathCollided(q_a, q, map.obstacles)
        
        # if q -> q_a causes no collision
        # then add q as child of q_a in T_start
        if not collision_a:
            T_start.append(Node(q, idx))

        # get closest node to q in T_goal
        idx = getClosestNode(q, T_goal)
        q_b = T_goal[idx].q

        # check if moving from q to q_b causes collision
        collision_b = isPathCollided(q_b, q, map.obstacles)

        # if q -> q_b causes no collision
        # then add q as child of q_b in T_goal
        if not collision_b:
            T_goal.append(Node(q, idx))
        
        # if q can connnect to both T_start and T_goal
        # get path from start to goal and return
        if not collision_a and not collision_b:
            # traverse up T_start to obtain path to q
            node = T_start[-1]
            path_start = [node.q]
            while node.parent is not None:
                node = T_start[node.parent]
                path_start.append(node.q)

            # traverse up T_goal to obtain path to qjob 
            node = T_goal[-1]
            path_goal = [node.q]
            while node.parent is not None:
                node = T_goal[node.parent]
                path_goal.append(node.q)

            # return pruned path
            path = np.array(path_start[::-1] + path_goal[1:])
            pruned_path = getPrunedPath(path, map.obstacles)
            print("Path found in %s iterations with %i steps, pruning removed %i node(s)." \
                  % (iter, len(pruned_path), len(path) - len(pruned_path)))
            # print(pruned_path)
            return pruned_path
    return []


if __name__ == '__main__':
    map_struct = loadmap("../maps/map1.txt")
    start = np.array([0,-1,0,-2,0,1.57,0])
    goal =  np.array([-1.2, 1.57 , 1.57, -2.07, -1.57, 1.57, 0.7])
    path = rrt(deepcopy(map_struct), deepcopy(start), deepcopy(goal))