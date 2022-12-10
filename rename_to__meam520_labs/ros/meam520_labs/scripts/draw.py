#!/usr/bin/python

import cairo
import numpy as np
import yaml
from math import sqrt

from core.utils import trans

def m2p(m): # meters to pts
    return m / .0254 * 100

size_in_pts = m2p(.25) # m
ps = cairo.SVGSurface("jig.svg", size_in_pts, size_in_pts)
cr = cairo.Context(ps)

matrix = cairo.Matrix()
matrix.translate(size_in_pts/2, size_in_pts/2)
matrix.rotate(-np.pi/2)
matrix.scale(-1,1)
cr.set_matrix(matrix)


def circle(xy,r):
    cr.arc(m2p(xy[0]), m2p(xy[1]), m2p(r), 0, 2*np.pi)
    cr.set_source_rgb(1, 0, 0)
    cr.set_line_width(3)
    cr.stroke()

def square(corners):
    cr.move_to(m2p(corners[0][-1]),m2p(corners[1][-1]))
    for i in range(4):
        cr.line_to(m2p(corners[0][i]),m2p(corners[1][i]))
    cr.set_source_rgb(1, 0, 0)
    cr.set_line_width(3)
    cr.stroke()

def arrow(start,end):
    vector = end - start
    length = np.linalg.norm(vector)
    direction = vector / length
    # line
    cr.move_to(m2p(start[0]),m2p(start[1]))
    cr.line_to(m2p(end[0]),m2p(end[1]))
    # head
    ear1 = end + (np.array([
        [0, -1],
        [1, 0]
    ]) @ direction - direction) * length * .2
    ear2 = end + (np.array([
        [0, 1],
        [-1, 0]
    ]) @ direction - direction) * length * .2
    print(ear1)
    print(ear2)
    cr.line_to(m2p(ear1[0]),m2p(ear1[1]))
    cr.move_to(m2p(end[0]),m2p(end[1]))
    cr.line_to(m2p(ear2[0]),m2p(ear2[1]))
    cr.set_source_rgb(1, 0, 0)
    cr.set_line_width(3)
    cr.stroke()

def into(xy,r):
    x = xy[0]
    y = xy[1]
    circle(xy,r)
    cr.move_to(m2p(x + r / sqrt(2)),m2p(y + r / sqrt(2)))
    cr.line_to(m2p(x - r / sqrt(2)),m2p(y - r / sqrt(2)))
    cr.move_to(m2p(x + r / sqrt(2)),m2p(y - r / sqrt(2)))
    cr.line_to(m2p(x - r / sqrt(2)),m2p(y + r / sqrt(2)))


    cr.set_source_rgb(1, 0, 0)
    cr.set_line_width(3)
    cr.stroke()

def outof(xy,r):
    x = xy[0]
    y = xy[1]
    circle(xy,r)
    circle(xy,r/5)

    cr.set_source_rgb(1, 0, 0)
    cr.set_line_width(3)
    cr.stroke()


# outline
r = .125
# square(np.array([
#     [r,-r,-r,r],
#     [r,r,-r,-r]
# ]))
for x in [-r,r]:
    for y in [-r,r]:
        circle(np.array([x,y]),.003)


np.set_printoptions(suppress=True,precision=3)

transforms = []

with open("data.yaml", "r") as stream:
    try:
        models = yaml.safe_load(stream)
        for name, pose in models['models'].items():
            if 'static' in name:
                T = np.array(pose).reshape((4,4))
                transforms.append((name,T))
    except yaml.YAMLError as exc:
        print(exc)


r = .025
for name, T in transforms:
    print(name)
    x = T[0:2,-1]
    R = T[0:3,0:3]
    # arrow(np.array([0,0]),T[0:2,-1])
    axes = []
    for j in range(3):
        axis = np.eye(3)[j]
        if abs(np.dot(R @ axis,np.array([0,0,1]))) < 1e-3:
            axes.append(axis) # lies in plane
    print(axes)
    corners = []
    for d1, d2 in [(1,1),(-1,1),(-1,-1),(1,-1)]:
        point = T @ trans(r * d1 * axes[0] + r * d2 * axes[1])
        corners.append(point[0:2,-1])
    square(np.array(corners).transpose())
    z = R @ np.array([0,0,1])
    proj = np.dot(z,np.array([0,0,1]))
    if abs(proj) < 1e-3:
        pass
        # z lies in plane
        arrow(x - z[0:2] * r * 3/5,x + z[0:2] * r * 3/5)
    else:
        if proj > 0:
            outof(x,.005)
        else:
            into(x,.005)
        # z not in plane

arrow(np.array([0,0]),np.array([.03,0]))
arrow(np.array([0,0]),np.array([0,.03]))

arrow(np.array([-.08,0]),np.array([-.115,0]))

cr.show_page()
