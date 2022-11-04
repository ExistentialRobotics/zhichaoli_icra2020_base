#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Find furtherest intersection between a path a geometric object
Support objects: ball

Author: Zhichao Li at UCSD ERL
Date: Jul-17 2020
"""
# python built in package
import numpy as np
import numpy.linalg as npla



class DistError(Exception):
    """ User Defined Exceptions for Distance Module.
    """

    def __init__(self, *args):
        if args:
            self.msg = args[0]
        else:
            self.msg = ""

    def __str__(self):
        if self.msg:
            return "DistErr exception: {0}".format(self.msg)
        else:
            return "DistErr exception"


class FindIntrError(Exception):
    """ User Defined Exceptions for FindIntr Module.
    """

    def __init__(self, *args):
        if args:
            self.msg = args[0]
        else:
            self.msg = ""

    def __str__(self):
        if self.msg:
            return "FindIntr exception: {0}".format(self.msg)
        else:
            return "FindIntr exception"



def ball_and_path(x, r, path):
    """
    Find furthest intersection between a ball and a path in 2D/3D. (d=2/3)
    Inputs:
        x: coordinates of the ball center (d, )
        r: radius of the ball
        path: a series of waypoints (num_pts, d)
    Output
        status: running of this algorithm
                -1: failed
                0: succeeded

        x_star  : furthest intersection (d, )
        B_idx: the furthest index of path xg lies in
                Example: B_idx = 6
                 (0)               (5)                (6)
                 START---->....---> A-----x_star-----> B----> .... ---> END

    Ref: personal notes
    """
    # default return
    status, x_star, B_idx = -1, [], []
    if len(path) < 2:
        raise FindIntrError("path len is too short, >=2 waypoints is required!")
    # loop backward
    for path_idx in range(len(path) - 1):
        B = path[-path_idx - 1]
        A = path[-path_idx - 2]
        B_idx = -path_idx - 1
        dAB = npla.norm(A - B)  # segment length ptA from ptB
        u = x - A
        v = B - A
        w1 = np.inner(u, v) / dAB ** 2
        w1hat = max(min(w1, 1), 0)  # normalized to [0,1]
        dist2segAB = npla.norm(u - w1hat * v)

        if dist2segAB > r:
            continue
        else:
            # distance from x to line AB
            dist2lineAB = npla.norm(u - w1 * v)
            # print('DEBUG dist to line AB is %.2f' %(dist2lineAB))
            w2 = np.sqrt(r ** 2 - dist2lineAB ** 2) / dAB  # ratio of |v|
            w = w1 + w2
            w = max(min(w, 1), 0)
            x_star = (1 - w) * A + w * B
            status = 0
            break

    return status, x_star, B_idx


def get_furthest_intr_lineseg2circle(ptA, ptB, circle_vec, eps=1e-6):
    """ Given a line segment (ptA - > ptB) and a circle
    (center 2/3D , radius), compute the intersections of them.
    Ref:
        # https://www.geometrictools.com/Source/Distance3D.html
    Alg Main Idea:
        paremterized line using x(t) = P + tD,  0 <=t <= 1
        paremterized circle |x - C| = R^2
        solve t, and check range
    """
    status = -1
    intrs = []
    t_candidates = []
    x_star = None
    ptA = np.asarray(ptA)
    ptB = np.asarray(ptB)
    circle_vec = np.asarray(circle_vec)
    P = ptA  # segment start
    D = ptB - ptA  # segment dir
    C = circle_vec[:-1]  # circle center
    R = circle_vec[-1]  # cirlce radius
    Delta = P - C
    # form quadratic equation about t
    a = np.inner(D, D)
    if a < 1e-6:
        raise DistError("Two points are too close")
    b = 2.0 * np.inner(D, Delta)
    c = np.inner(Delta, Delta) - R ** 2
    # discriminant
    delta = b ** 2 - 4 * a * c

    if delta >= 0:
        tmp = np.sqrt(delta)
        t1 = 1.0 / (2 * a) * (-b + tmp)
        t2 = 1.0 / (2 * a) * (-b - tmp)
        if t1 >= 0 and t1 <= 1:
            x1 = P + t1 * D
            intrs.append(x1)
            t_candidates.append(t1)
        if t2 >= 0 and t2 <= 1:
            x2 = P + t2 * D
            intrs.append(x2)
            t_candidates.append(t2)

        if len(t_candidates) > 0:
            status = 0
            tmax = max(t_candidates)
            x_star = P + tmax * D
        else:
            print("ptA, ptB, circle_vec", ptA, ptB, circle_vec)
            raise FindIntrError("No intersection found ! ")

    return status, x_star



# Some examples
if __name__ == "__main__":
    # 2D example
    # sA = [0, 0]
    # sB = [3, 0]
    ball_center = np.array([0, 0])
    ball_radius = 1.0
    circle_vec = np.hstack((ball_center, ball_radius))

    path = np.array([[0, 0], [3, 0]])
    status1, x_star1, sB_idx1 = ball_and_path(ball_center, ball_radius, path=path)
    status2, x_star2 = get_furthest_intr_lineseg2circle(path[0], path[1], circle_vec=circle_vec)
    print("status1 = %d, x_star1 = %s, sB_idx1 = %d" % (status1, x_star1, sB_idx1))
    print("status2 = %d, x_star2 = %s" % (status2, x_star2))

