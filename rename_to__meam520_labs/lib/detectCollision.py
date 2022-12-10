#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Edited Shane Rozen-Levy 10/1/20
    Added 3d case
    Added unit tests
    Allowed for multiple inputs
"""

import numpy as np


def detectCollision (linePt1, linePt2, box):
    """
    Check if multiple lines formed from two points intercepts with the block.
    Check one at a time.
    :param linePt1: [n,3] np array where each row describes one end of a line
    :param linePt2  [n,3] np array where each row describes one end of a line
    :param box [xmin, ymin, zmin, xmax, ymax, zmax]: box
    :return: n dimensional array, true if line n is in collision with the box
    """
    n_samples = len(linePt1)
    return [detectCollisionOnce(linePt1[index], linePt2[index], box) for index in range(n_samples)]

def detectCollisionOnce(linePt1, linePt2, box):
    """
    Check if line form from two points intercepts with the per block.
    Check one at a time.
    :param linePt1 [x,y,z]:
    :param linePt2 [x,y,z]:
    :param box [xmin, ymin, zmin, xmax, ymax, zmax]:
    :return: true if collision, otherwise false
    """
    # %% Initialization
    # box = box[0]
    # Initialize all lines as collided.
    isCollided = np.ones(1)

    # Divide box into lower left point and "size"
    boxPt1 = np.array([box[0],box[1], box[2]])
    # Create point in the opposize corner of the box
    boxPt2 = np.array([box[3],box[4], box[5]])
    boxSize = boxPt2 - boxPt1
    # Find slopes vector
    lineSlope = linePt2 - linePt1
    lineSlope = [0.001 if num == 0 else num for num in lineSlope]

    # %% Begin Collision Detection

    # The parameter t = [0,1] traces out from linePt1 to linePt2

    # Return false if box is invalid or has a 0 dimension
    if min(boxSize) <= 0:
        isCollided = 0 * isCollided
        return isCollided

    # Get minimum and maximum intersection with the y-z planes of the box
    txmin = (boxPt1[0] - linePt1[0]) / lineSlope[0]
    txmax = (boxPt2[0] - linePt1[0]) / lineSlope[0]

    # Put them in order based on the parameter t
    ts = np.sort(np.array([txmin,txmax]).transpose())
    txmin = ts[0]
    txmax = ts[1]


    # Get minimum and maximum intersection with the x-z planes of the box

    tymin = (boxPt1[1] - linePt1[1]) / lineSlope[1]
    tymax = (boxPt2[1] - linePt1[1]) / lineSlope[1]

    # Put them in order based on the parameter t
    ts = np.sort(np.array([tymin, tymax]).transpose())
    tymin = ts[0]
    tymax = ts[1]
    # if we miss the box in this plane, no collision
    isCollided = np.logical_and(isCollided, np.logical_not(np.logical_or((txmin > tymax), (tymin > txmax))))

    # identify the parameters to use with z
    tmin = np.maximum.reduce([txmin, tymin])
    tmax = np.minimum.reduce([txmax, tymax])

    # Get minimum and maximum intersection with the x-z planes of the box
    tzmin = (boxPt1[2] - linePt1[2]) / lineSlope[2]
    tzmax = (boxPt2[2] - linePt1[2]) / lineSlope[2]
    # Put them in order based on the parameter t
    ts = np.sort(np.array([tzmin, tzmax]).transpose())
    tzmin = ts[0]
    tzmax = ts[1]

    # if we miss the box in this plane, no collision
    isCollided = np.logical_and(isCollided, np.logical_not(np.logical_or((tmin > tzmax), (tzmin > tmax))))

    # identify the parameters to use with z
    tmin = np.maximum.reduce([tmin, tzmin])
    tmax = np.minimum.reduce([tmax, tzmax])

    # check that the intersecion is within the link length
    isCollided = np.logical_and(isCollided, np.logical_not(np.logical_or((0 > tmax), (1 < tmin))))
    isCollided = isCollided.reshape((isCollided.shape[0],1))
    return isCollided[0,0]


def plotBox(axis, box):
    """
    :param axis: plot axis
    :param box: corners of square to be plotted
    :return: nothing
    """
    prism = Poly3DCollection([box],edgecolor='g',facecolor='g',alpha=0.5)
    axis.add_collection3d(prism)


if __name__=='__main__':
    """
    Visual unit tests for collision check. Generates random lines and check if collide with box. Output is plot of lines
    and box, color coded by collision.
    """
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    import random

    nline = 30
    box = np.array([-1, -1, -1, 1, 1, 1])
    world_length = 5
    fig = plt.figure()
    ax = Axes3D(fig)
    for i in range(nline):
        line_pt1 = np.array([[(random.random() * world_length)-world_length/2, (random.random() * world_length)-world_length/2, (random.random() * world_length)-world_length/2]])
        line_pt2 = np.array([[(random.random() * world_length)-world_length/2, (random.random() * world_length)-world_length/2, (random.random() * world_length)-world_length/2]])
        if detectCollision(line_pt1, line_pt2, box)[0]:
            ax.plot([line_pt1[0,0], line_pt2[0,0]], [line_pt1[0,1], line_pt2[0,1]], [line_pt1[0,2], line_pt2[0,2]], 'r')
        else:
            ax.plot([line_pt1[0,0], line_pt2[0,0]], [line_pt1[0,1], line_pt2[0,1]], [line_pt1[0,2], line_pt2[0,2]], 'b')
    box1 = [[box[0], box[1], box[2]],
            [box[0+3], box[1], box[2]],
            [box[0+3], box[1+3], box[2]],
            [box[0], box[1+3], box[2]]]
    box2 = [[box[0], box[1], box[2]],
            [box[0+3], box[1], box[2]],
            [box[0+3], box[1], box[2+3]],
            [box[0], box[1], box[2+3]]]
    box3 = [[box[0], box[1], box[2]],
            [box[0], box[1+3], box[2]],
            [box[0], box[1+3], box[2+3]],
            [box[0], box[1], box[2+3]]]
    box4 = [[box[0], box[1], box[2+3]],
            [box[0+3], box[1], box[2+3]],
            [box[0+3], box[1+3], box[2+3]],
            [box[0], box[1+3], box[2+3]]]
    box5 = [[box[0], box[1+3], box[2]],
            [box[0+3], box[1+3], box[2]],
            [box[0+3], box[1+3], box[2+3]],
            [box[0], box[1+3], box[2+3]]]
    box6 = [[box[0+3], box[1], box[2]],
            [box[0+3], box[1+3], box[2]],
            [box[0+3], box[1+3], box[2+3]],
            [box[0+3], box[1], box[2+3]]]
    plotBox(ax, box1)
    plotBox(ax, box2)
    plotBox(ax, box3)
    plotBox(ax, box4)
    plotBox(ax, box5)
    plotBox(ax, box6)
    plt.show()


