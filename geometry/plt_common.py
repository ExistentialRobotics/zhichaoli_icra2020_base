#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jul  8 18:56:14 2020

Author: Zhichao Li at UCSD ERL
"""

import matplotlib.patches as mp
import numpy as np
from matplotlib import pyplot as plt

# from matplotlib import pyplot as plt
from matplotlib.collections import PatchCollection
from matplotlib.path import Path

# ===========================================================
# change legend attributes
# ax11.legend(loc='center left', bbox_to_anchor=(1, 0.7), prop={'size':14})
# ax12.legend(loc='center left', bbox_to_anchor=(1, 0.5), prop={'size':14})
# ===========================================================


def draw_path3d(ax, waypoints, lc="blue", lw=2, lstr="path", ps="-"):
    """ draw 3d path consists of waypoints (num_pts, 3)
    """
    ax.plot(waypoints[:, 0], waypoints[:, 1], waypoints[:, 2], c=lc, ls=ps, label=lstr, lw=lw)
    return ax


def show_mesh(mesh, azim_angle=-89, elev_angle=75, show_plot=False):
    """ Show mesh object (trimesh package) with matplotlib 
    """
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    ax.add_collection3d(Poly3DCollection(mesh.vertices[mesh.faces]))
    # for better visualization
    ax.set_xlim(mesh.bounds[0, 0], mesh.bounds[1, 0])  # xlimit
    ax.set_ylim(mesh.bounds[0, 1], mesh.bounds[1, 1])  # ylimit
    ax.set_zlim(mesh.bounds[0, 2], mesh.bounds[1, 2])  # zlimit
    # plt.title("mesh visualization")
    aspect_from_bounds = mesh.bounds[1] - mesh.bounds[0]
    ax.set_box_aspect(aspect=aspect_from_bounds)
    if show_plot:
        plt.show()
    return fig, ax


def get_sphere_points(ball_center=np.zeros(3), ball_radius=10):
    """ Get sphere points
    """
    # fig = plt.figure()
    # ax = fig.add_subplot(111, projection='3d')

    # Make data
    u = np.linspace(0, 2 * np.pi, 100)
    v = np.linspace(0, np.pi, 100)
    x = ball_center[0] + ball_radius * np.outer(np.cos(u), np.sin(v))
    y = ball_center[1] + ball_radius * np.outer(np.sin(u), np.sin(v))
    z = ball_center[2] + ball_radius * np.outer(np.ones(np.size(u)), np.cos(v))

    # Plot the surface
    # ax.plot_surface(x, y, z, color='b')
    # plt.show()

    return x, y, z


def zhl_default_plt_setting():
    my_default_plt_setting()


def my_default_plt_setting():
    """ Set customized default setting.
    """
    import matplotlib as mpl

    from matplotlib import pyplot as plt

    # plt.ion()
    # plt.tight_layout()
    mpl.rcParams["pdf.fonttype"] = 42
    mpl.rcParams["ps.fonttype"] = 42
    mpl.rcParams["lines.linewidth"] = 2.0
    mpl.rcParams["legend.fontsize"] = "large"
    mpl.rcParams["axes.titlesize"] = 18
    mpl.rcParams["axes.labelsize"] = 14
    mpl.rcParams["axes.grid"] = True
    # print(mpl.rcParams['axes.grid'])
    # plt.grid()
    plt.rcParams["text.usetex"] = True
    # set_numpy_print() # numpy print
    # plt.ion()
    return 0


def set_plot3d_box(ax, box_rxyz=None, box_center=None, equal_aspect=False, plot3d=False):
    """ Set canvas of ax using box radius and box center.
    """
    if box_center is None:
        box_center = np.zeros(3)

    if isinstance(box_rxyz, tuple):
        box_rxyz = np.ones(3) * box_rxyz
    else:
        box_rxyz = np.array(box_rxyz)

    ax.set_xlim([-box_rxyz[0] + box_center[0], box_rxyz[0] + box_center[0]])
    ax.set_ylim([-box_rxyz[1] + box_center[1], box_rxyz[1] + box_center[1]])
    ax.set_zlim([-box_rxyz[2] + box_center[2], box_rxyz[2] + box_center[2]])

    ax.set_box_aspect(aspect=(1, 1, 1))  # set axis equal for 3D
    # ax.set_aspect('equal')

    return ax


def set_canvas_box(ax, bd_pts):
    """ Set canvas of ax using map boundary pts.
    """
    xl, xh, yl, yh = bd_pts
    ax.set_xlim([xl, xh])
    ax.set_ylim([yl, yh])
    # ax.grid()
    ax.set_aspect("equal")
    return ax


def set_peripheral(ax, /, t_str=None, x_str=None, y_str=None, grid_on=True, plt_show=True, lg_on=False):
    """ Set plot peripherals
    """
    if t_str:
        ax.set_title(t_str, fontsize=18)
    if x_str:
        ax.set_xlabel(x_str, fontsize=15)
    if y_str:
        ax.set_ylabel(y_str, fontsize=15)
    if grid_on:
        ax.grid()
    if plt_show:
        plt.show()
    if lg_on:
        ax.legend()
    return ax


def wall_corners2_ls(corners, show_debug=False):
    """Turn all wall corners to line segments.
    """
    ls_list = []
    for i in range(len(corners) - 1):
        ls_list.append([corners[i], corners[i + 1]])
    ls_list.append([corners[-1], corners[0]])
    if show_debug is True:
        print("wall line segments list")
        print(ls_list)
    if len(ls_list) == 1:
        # prevent dimension reduction
        ls_list = [ls_list]
    return ls_list


def path2_ls(path, show_debug=False):
    """Turn a navigation path to a ordered collection of line segments.
    path (num_dim, num_pt)
    """
    ls_list = []
    end_idx = len(path) - 1
    for i in range(end_idx):
        ls_list.append([path[i], path[i + 1]])

    if show_debug is True:
        print("wall line segments list")
        print(ls_list)
    return ls_list


# def axis_equal_3D(this_ax):
#     """ set aspect ratio to equal for 3D
#     https://github.com/WISDEM/CommonSE/blob/master/src/commonse/axisEqual3D.py
#     """
#     print("set aspect ratio to equal for 3D")
#     extents = np.array([getattr(this_ax, "get_{}lim".format(dim))() for dim in "xyz"])
#     sz = extents[:, 1] - extents[:, 0]
#     centers = np.mean(extents, axis=1)
#     maxsize = max(abs(sz))
#     r = maxsize / 2.0
#     for ctr, dim in zip(centers, "xyz"):
#         getattr(this_ax, "set_{}lim".format(dim))(ctr - r, ctr + r)
def axis_equal_3D(this_ax):
    this_ax.set_box_aspect(aspect=(1, 1, 1))
    return this_ax


def create_path_patch(path_array, lc="red"):
    """Create nav_path patch using navigation global waypoints.
    """
    verts = path_array
    path_len = len(path_array)
    codes = [Path.MOVETO] + [Path.LINETO] * (path_len - 1)
    path = Path(verts, codes)
    patch_path = mp.PathPatch(path, facecolor="none", ec=lc, lw=2)
    return patch_path, verts


def add_waypoint_path(ax, waypoints_array, *, lc="black", ms=5, lw=2, lstr="nav path"):
    """ Add navigation path according to waypoints array
    """
    patch_nav_path, verts_nav_path = create_path_patch(waypoints_array, lc)
    ax.add_patch(patch_nav_path)
    ax.add_patch(patch_nav_path)
    # plot vertes of path
    xs, ys = zip(*verts_nav_path)
    ax.plot(xs, ys, "o--", lw=lw, color="black", ms=ms, label=lstr)
    return ax


def create_circle_patch(circle_array, color="tab:grey", alpha=0.8):
    """
    Given a list of same objects: (circles, ploygons, ...), create a
    patch collection object with certain color and trasparent alpha.
    Input: circle_array (num_pt * 3)
           each item [x, y, radius]
    """
    clist = []
    # prevent array size degeneration always turn it to 2D
    circle_array = np.reshape(circle_array, (-1, circle_array.shape[-1]))
    for item in circle_array:
        x, y, r = item
        circle = mp.Circle((x, y), r)
        clist.append(circle)
    patch_circles = PatchCollection(clist, color=color, alpha=alpha)
    return patch_circles


def add_arrow(ax, arr_item, *, sf=1, width=0.1, color="m"):
    """ Add a arrow defined by arr_item (4,)
    arr_item [x, y, dx, dy]
    """
    stx, sty, dx, dy = arr_item
    arrow = mp.Arrow(stx, sty, sf * dx, sf * dy, color=color, width=width)
    ax.add_patch(arrow)
    return ax


def add_vel_profile(ax, loc_path, vel_path):
    """ Add velocity profile along given path, using perpendicular
    arrow to show velocity change.
    """
    for ii in range(1, len(loc_path), 3):
        stx, sty = loc_path[ii]
        xv = vel_path[ii]
        dx, dy = -xv[1], xv[0]
        vp_arrow = mp.Arrow(stx, sty, dx, dy, color="m", width=0.1)
        ax.add_patch(vp_arrow)
    return ax


def create_arrow_patch(arrow_array, *, scale_factor=1, width=0.1, color="m"):  # scale factor
    """
    Given a list of same objects: (arrows), create a
    patch collection object with certain color and trasparent alpha.
    Input: arrow_array (num_pt * 4)
           each item [stx, sty, dx, dy]
    """
    arrow_list = []
    arrow_array = np.reshape(arrow_array, (-1, arrow_array.shape[-1]))
    for item in arrow_array:
        stx, sty, dx, dy = item
        dx = dx * scale_factor
        dy = dy * scale_factor
        this_arrow = mp.Arrow(stx, sty, dx, dy, width=width)
        arrow_list.append(this_arrow)
    patch_arrows = PatchCollection(arrow_list, color=color)
    return patch_arrows


def circle_obj_list2arr(circle_obj_list):
    """Transform circle list to cirlce array
    each row of the array is a cirlce [x,y,r]
    """
    # to list structure avoid degeneration when list reduce to 1D
    if isinstance(circle_obj_list, list):
        pass
    else:
        circle_obj_list = [circle_obj_list]

    n = len(circle_obj_list)
    circle_arr = np.zeros((n, 3))
    for ii in range(n):
        circle_obj = circle_obj_list[ii]
        [x, y], r = circle_obj.center, circle_obj.radius
        circle_arr[ii] = np.array([x, y, r])

    return circle_arr
