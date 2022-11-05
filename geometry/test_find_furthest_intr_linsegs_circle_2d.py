"""
Test geometric functions

Author: zhichao li at UCSD ERL
Date: 11/19/2021
BSD 3-Clause License
TODO_ZHL
"""
import matplotlib.patches as mp
import numpy as np
import numpy.linalg as npla
from matplotlib import pyplot as plt

from plt_common import add_waypoint_path


def pressQ_to_exist():
    """ press key `Q` or `q` to exit
    """
    while True:
        key_got = input('Press [Q] to quit\n')
        if key_got == 'q' or key_got == 'Q':
            print('Received %c, Program terminated'  %key_got)
            break
    return 0

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

    """
    # default return
    status, x_star, B_idx = -1, [], []
    # TODO_STEP3 
    return status, x_star, B_idx


# Test examples
if __name__ == "__main__":


    np.set_printoptions(formatter={'float': '{: 0.2f}'.format})

    random_circle = False
    random_path = False
    show_plots = True

    if show_plots:
      plt.ion()
    
    if random_path:
      waypoints = np.random.randint(-10, 10, size=100).reshape((-1,2))
      waypoints = np.unique(waypoints, axis=0)
    else:
      waypoints = np.array([[-5, -4], [-2.5, 0], [-5, 2], [1, 2], [3, 4], [6, 0]], dtype=np.float32)
    
    pt_num, pt_dim = waypoints.shape

    for r in range(1, 8):
      # fixed circle or random generated one
      if random_circle:
        circle_vec = np.hstack((np.random.randint(-5, 5, size=pt_dim), np.random.randint(1, 5))).astype(np.float32)
      else:
        circle_vec = np.array([0, 0, 1.0 * r])

      # find furthest intersection between ball and path
      # TODO_STEP3 test FuncA 
      # status: 0, succ, -1 failed, lpg
      # status, lpg, nav_adv_idx = FuncA(circle_vec[0:pt_dim], circle_vec[-1], path=waypoints)

      print("status = %02d, lpg = %s, nav_adv_idx = %d" % (status, lpg, nav_adv_idx))
      if show_plots:
        fig, ax = plt.subplots()
        patch_circle = mp.Circle(circle_vec[:pt_dim], circle_vec[-1], color="tab:grey", alpha=0.8)
        ax.add_patch(patch_circle)

        ax = add_waypoint_path(ax, waypoints)
        ax.grid()
        PLOT_LIMIT = 8
        ax.set_xlim([-PLOT_LIMIT, PLOT_LIMIT])
        ax.set_ylim([-PLOT_LIMIT, PLOT_LIMIT])
        ax.set_aspect("equal")
        if status >= 0:
          plt.scatter(lpg[0], lpg[1], c="red", marker="o")
        plt.pause(0.01)
        plt.show()
    pressQ_to_exist()
