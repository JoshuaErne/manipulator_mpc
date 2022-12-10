#!/usr/bin/env python

from heapq import heappush, heappop  # Recommended.
import numpy as np
import math
import matplotlib.pyplot as plt
import itertools

from lib.calculateFK import FK as calculateFK
from lib.detectCollision import detectCollision

def Astar(map, start, goal):
    """
    Parameters:
        maps,       struct containing map information
        start,      inital configuration array, shape=(7,)
        goal,       final configuration array, shape=(7,)
    Output:
        path,       configuration coordinates along the path  with
                    shape=(N,7). These are typically the centers of visited
                    voxels of an occupancy map. The first point must be the
                    start and the last point must be the goal. If no path
                    exists, return None.
    """
    start_backup = start
    goal_backup = goal
    path = [goal]
    start = start[[0,1,2,3,4,5]]
    goal = goal[[0,1,2,3,4,5]]

    # While not required, we have provided an occupancy map you may use or modify.
    resolution = np.array([0.3, 0.4, 0.6, 0.5, 0.6, 0.5])
    margin = 0.12
    occ_map = OccupancyMap(map, resolution, margin)

    # Retrieve the index in the occupancy grid matrix corresponding to a position in space.
    start_index = tuple(occ_map.metric_to_index(start))
    goal_index = tuple(occ_map.metric_to_index(goal))

    if not occ_map.is_valid_index(goal_index) :
        print("invalid Goal Index")
        return np.array([])
    if not occ_map.is_valid_index(start_index):
        print("invalid start Index")
        return np.array([])
    if occ_map.is_occupied_index(start_index):
        print("Occupied start Index")
        return np.array([])
    if occ_map.is_occupied_index(goal_index):
        print("Occupied Goal Index")
        return np.array([])

    # Build lookup table of step distances.
    step_distance_table = np.zeros((3,3,3,3,3,3))
    for direction in itertools.product((-1,0,1), repeat=6):
        direction = np.array(direction)
        index = tuple(direction + 1)
        step_distance_table[index] = np.linalg.norm(direction * resolution)

    def get_step_distance(direction):
        """
        Return the step distance for an index direction (i,j,k,l,m,n).
        """
        index = tuple(np.array(direction) + 1)
        return step_distance_table[index]

    gCosts = {start_index: 0}
    parents = {}
    open_list = []
    heappush(open_list, (np.linalg.norm(occ_map.index_to_metric_negative_corner(goal_index) -
                                        occ_map.index_to_metric_negative_corner(start_index)), start_index))
    closed_list = set()


    # if all the cost in g are smaller than inf, there might have a path found
    # or if the goal is closed, there might have a path found
    while open_list:  # while there is node not open and the goal is not closed
        # pick the node with smallest cost as the current node, pop it out then mark it as closed
        u = heappop(open_list)[1]
        #print(u)
        if u in closed_list:
            continue
        if u == goal_index:
            break
        closed_list.add(u)
        for direction in itertools.product((-1, 0, 1), repeat=6):
            v = tuple([u[ii]+direction[ii] for ii in range(6)])
            # determine the distance between voxel
            if occ_map.is_valid_index(v):
                if not occ_map.is_occupied_index(v):
                    # chech if the neightbor is open, is it in Q. Here we check its status with 2
                    if not v in closed_list:  # if neighbor is not closed
                        # calculate maximum distance
                        h = np.linalg.norm(occ_map.index_to_metric_negative_corner(goal_index) - occ_map.index_to_metric_negative_corner(v))
                        c = get_step_distance(direction)
                        neighbor_g  = gCosts[u] + c
                        if neighbor_g < gCosts.get(v, np.inf):
                            gCosts[v] = neighbor_g
                            parents[v] = u
                            heappush(open_list, (neighbor_g + h, v))

    print("Finished finding path")
    # if the parent of the goal node is empty, that measn no path is found
    if parents.get(goal_index, np.inf) == np.inf:  # goal not found
        path = None
        raise Exception("No Path Found. ")
    else:
        # using the parent index, trackback to form the path
        pointer = goal_index
        while pointer != start_index:
            par = parents[pointer]
            path.append(np.append(occ_map.index_to_metric_negative_corner(par), goal_backup[[6]]))
            pointer = par
        path.append(start_backup)
        path.reverse()
        path = np.array(path)

    return path


class OccupancyMap:

    def __init__(self, map, resolution, radius):
        """
        Initialize Occupancy map. This would create the grid map based on the dimension and resolutions
        defined in the Json file. It would then load the blocks and mark their corresponding grid as True
         as occupied.
        :param map: the map struct.
        """

        self.FK = calculateFK()
        # Get joint limits
        self.lowerLim = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, .25])
        self.upperLim = np.array([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525])
        self.resolution = resolution
        self.origin = self.lowerLim

        # Loading blocks into occupancy map.
        self.blocks = map.obstacles
        self.InflatedBlocks = self.BlockInflation(self.blocks, radius)

        # For each joint find all combinations
        th1_val = np.arange(self.lowerLim[0], self.upperLim[0], resolution[0])
        th2_val = np.arange(self.lowerLim[1], self.upperLim[1], resolution[1])
        th3_val = np.arange(self.lowerLim[2], self.upperLim[2], resolution[2])
        th4_val = np.arange(self.lowerLim[3], self.upperLim[3], resolution[3])
        th5_val = np.arange(self.lowerLim[4], self.upperLim[4], resolution[4])
        th6_val = np.arange(self.lowerLim[5], self.upperLim[5], resolution[5])

        # Initialize occupancy map
        self.occ = np.zeros([len(th1_val)+1, len(th2_val)+1, len(th3_val)+1, len(th4_val)+1, len(th5_val)+1,
                            len(th6_val)+1])
        self.occ_check = {}


    def metric_to_index(self, metric):
        """
        Returns the index of the voxel containing a metric point.
        Remember that this and index_to_metric and not inverses of each other!
        If the metric point lies on a voxel boundary along some coordinate,
        the returned index is the lesser index.
        """
        return np.round((np.array(metric) - np.array(self.origin))/self.resolution).astype('int')

    def index_to_metric_center(self, ind):
        """
        :param ind:
        :return:
        """
        return self.index_to_metric_negative_corner(ind) + np.array(self.resolution)/2.0

    def index_to_metric_negative_corner(self,ind):
        """
        Given the index, return the
        :param ind:
        :return:
        """
        return ind*np.array(self.resolution) + np.array(self.origin)

    def is_occupied_index(self, ind):
        """
        Check if occupied is occupied.
        :param ind:
        :return:
        """
        if not ind  in self.occ_check:
            self.process_index(ind)
        return self.occ[ind]

    def process_index(self, ind):
        """
        Calculates if the metric described by this index is occupied
        :param ind:
        :return:
        """
        beg_index = [0,1,2,3,4,5,6]
        end_index = [1,2,3,4,5,6,7]
        joint_vector = self.index_to_metric_negative_corner(ind)
        joint_pos, foo = self.FK.forward(np.append(joint_vector, [0, 0]))
        joint_pos_full=np.array(np.vstack((joint_pos,np.transpose(foo[0:3,3]))))
        beg_points = joint_pos_full[beg_index, ::]
        end_points = joint_pos_full[end_index, ::]

        for block in self.InflatedBlocks:
            is_collided = detectCollision(beg_points, end_points, block)
            if any(is_collided):
                self.occ[ind] = True
                break

        self.occ_check[ind] = True

    def is_valid_index(self, ind):
        """
        Check if the index is valide or not.
        :param ind:
        :return:
        """
        if ind[0] >= 0 and ind[0] < self.occ.shape[0]:
            if ind[1] >= 0 and ind[1] < self.occ.shape[1]:
                if ind[2] >= 0 and ind[2] < self.occ.shape[2]:
                    if ind[3] >= 0 and ind[3] < self.occ.shape[3]:
                        if ind[4] >= 0 and ind[4] < self.occ.shape[4]:
                            if ind[5] >= 0 and ind[5] < self.occ.shape[5]:
                                return True
        return False

    def is_valid_metric(self, metric):
        """
        Check it the metric is inside the boundary or not.
        :param metric:
        :return:
        """
        for i in range(6):
            if metric[i] <= self.lowerLim[i] or metric[i] >= self.upperLim[i]:
                return False
        return True

    def BlockInflation(self, blocks, radius):
        """
        Inflate the Block dimension with the radius of the robot.
        :param block:
        :param radius:
        :return:  ndarray
        """
        modifier = np.array([-radius, -radius, -radius, radius, radius, radius])
        return np.array([modifier + block for block in blocks])
