#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Governor System Class for sparse known env.
Author: zhichao li at UCSD ERL
Date: 06/26/2020
BSD 3-Clause License
https://github.com/zhl355/ICRA2020_RG_SDDM
"""
# python built in package
import numpy as np
from numpy import linalg as LA
from numpy.linalg import norm

import opt_solver as opt
# personal
# from gov_ellipse import MyEllipse
from my_utils import Pnorm_len, debug_print, flist1D

# float precision
np.set_printoptions(precision=4)


class GovError(Exception):
    """ User Defined Exceptions for RG.
    """

    def __init__(self, *args):
        if args:
            self.msg = args[0]
        else:
            self.msg = ''

    def __str__(self):
        if self.msg:
            return "GovError exception: {0}".format(self.msg)
        else:
            return "GovError exception"


class RbtGovSys:
    """
    A class for robot-governor system (RGS) update in simulation.
    It consists of three big parts
        1. RGS
            # xvec = [xr, xv, xg]
                - xr is the location of the robot
                - xg is the postion of governor
                - xv is the speed of the robot

            # Controller Design Paramters kv, kg, zeta
                - xr_dot = xv
                - xv_dot = -2 kv (xr-xg) - zeta xv
                - xg_dot = - kg (xg - xg_bar)

            # Numerical computation varibles:
                - eps: numerical machine eps
                - dt : discretization time interval

            # dvec = [dgO, drg, drO]
                - dgO/dgF: governor to the closest obstacle
                - drg: robot to governor
                - drO/drF: robot to to the closet obstacle

    """
    # Running status table
    GOV_HALT = 0
    GOV_NORMAL = 1
    GOV_GOAL_REACHED = 100
    GOV_GOAL_REACHED_FLAG = False
    GOV_NO_LOCAL_GOAL = -1
    ERROR_NEGATIVE_deltaE = -2

    def __init__(self, rgs_map, xvec0, eta_max0, energy_metric,
                 Pd, PV0, P_kinematic, design_paras,
                 dt=0.05, eps=1e-6):
        """ Init RGS system.
        """

        self.map = rgs_map
        self.xvec = xvec0
        self.eta_max = eta_max0
        self.energy_metric = energy_metric

        self.Pd = Pd
        self.PV = PV0                   # potential energy norm
        self.P_kinematic = P_kinematic  # kinematic

        self.design_paras = design_paras
        self.kv, self.kg, self.zeta = design_paras
        self.clock = 0

        self.dt = dt
        self.eps = eps

        self.start_pt = self.map.start_pt
        self.goal_pt = self.map.goal_pt
        self.gov_status = RbtGovSys.GOV_HALT

        # eigenvalue of P_energy
        _, eig_PV, _ = LA.svd(self.PV)
        self.start_pt = xvec0[0:2]
        # initialize distance vector
        # dvec0, _, _, _ = RbtGovSys.dist_vec(self)
        # if sum(dvec0 >= 0) < 3:
        #     dist_err_str = 'Init Error' + str(flist1D(dvec0))
        #     raise GovError(dist_err_str)


        self.gov_status = RbtGovSys.GOV_NORMAL
        # history log container
        self.xvec_log = [xvec0]
        self.dvec_log = []
        self.deltaE_log = []
        self.lpg_log = []
        self.le_rho_log = []
        self.nav_adv_idx_log = []
        self.Evec_log = []
        self.LEE_specs_log = []
        self.GDE_specs_log = []


    def dist_pt2wall(self, pt):
        """ Find distance from pt to all wall
        """
        debug_level = -100
        if self.energy_metric == 'ball':
            # find closet point in Obs Space in 2-norm
            xl, xh, yl, yh = self.map.boundary
            x, y = pt
            dist_walls = np.abs([x-xl, x - xh, y - yl, y - yh])
            mindist_idx = np.argmin(dist_walls)
            dist_wall = dist_walls[mindist_idx]
            pstar_candidates = np.array([[xl, y], [xh, y], [x, yl], [x, yh]])
            # get back the closet pt on the wall
            pstar = pstar_candidates[mindist_idx]
        else:
            # find closet point in Obs Space in Quadratic-norm
            wall_ls = self.map.wall_ls
            norm_mat = self.PV  # NEL should be rotated
            dist_wall, pstar \
                = opt.dist_pt2seglist_Pnorm(pt, wall_ls, norm_mat)

        debug_print(debug_level, 'dist_pt2wall')
        dmsg = 'Get pstar %s and dist_wall %.2f from pt %s' \
            % (pstar, dist_wall, pt)
        debug_print(debug_level, dmsg)
        return dist_wall, pstar

    def dist_pt2obs(self, pt, add_noise=True):
        """ Find distance from pt to all obstacles
        for here just to the a collection of circle obstacles
        """
        debug_level = -1
        dist2obs_vec = []
        obs = self.map.obstacles.copy()

        if self.energy_metric == 'ball':
            dist2obs_vec = norm(obs[:, :2] - pt, axis=1) - obs[:, -1]
            mindist_idx = np.argmin(dist2obs_vec)
            pstar = obs[mindist_idx, :2]
            dist_obs = dist2obs_vec[mindist_idx]

            dmsg1 = 'DEBUG dist_pt2obs %s ' % (dist2obs_vec)
            dmsg2 = 'get pstar %s and dist_obs = %.2f from pt %s' \
                % (pstar, dist_obs, pt)
            debug_print(debug_level, dmsg1)
            debug_print(debug_level, dmsg2)

        return dist_obs, pstar

    def dist_pt2F(self, pt, msg):
        """
        Find distance from pt to free space boundary
        for here we simply takv the min of the dist 2 env and obs.
        """
        debug_level = -100
        dmsg = 'dist_pt2F' + msg + str(pt)
        debug_print(debug_level, dmsg)
        dist_F = 0
        d2wall, pstar_wall = RbtGovSys.dist_pt2wall(self, pt)
        d2obs,  pstar_obs = RbtGovSys.dist_pt2obs(self, pt)

        if d2wall < 0:
            print('d2wall is %.2f from pt %s' % (d2wall, pt))
            raise GovError('Crush to wall')
        elif d2obs < 0:
            print('d2obs is %.2f from pt %s' % (d2obs, pt))
            print('Obs location')
            print(self.map.obstacles)
            raise GovError('Crush to obstacles')
        else:
            if d2obs <= d2wall:
                dist_F = d2obs
                pstar_F = pstar_obs
                msg = '!!! closest pt at Xobs dist_F %.2f' % dist_F
                debug_print(debug_level, msg)
            else:
                dist_F = d2wall
                pstar_F = pstar_wall
                msg = '!!! closest pt at wall dist_F %.2f' % dist_F
                debug_print(debug_level, msg)

        return dist_F, pstar_F

    def dist_vec(self):
        """
        Compute distances, dist_vec: [dgO, drg, drO]
            # dgO: governor to the closest obstacle
            # drg: robot to governor
            # drO: robot to to the closet obstacle
        """
        # TODO_STEP1
        # 1. retrieve system states and hyperparameter from class attributes
        # 2. compute distance dgO, drg, drO (hint: dist_pt2F)
        dvec = [dgO, drg, drO]
        self.dvec = dvec
        dmsg = 'dist_vec: [dgO, drg, drO]=[%.4f %.4f %.4f]' % (dgO, drg, drO)
        debug_print(debug_level, dmsg)
        return dvec, dgO, drg, drO

    def cpt_deltaE_ball(self):
        """
        Compute energy that can be safely added to the system.
        Energy mesured in ball sense.
        """

        # TODO_STEP2
        # 1. retrieve system states and hyperparameter from class attributes
        # 2. compute distance dgO and drg (Hint)
        # 3. compute dynamic safety margin deltaE
        return deltaE


    def find_proj_goal_ball(self, th=1):
        """
        Find local projected goal "xg_bar" using current state.
        i.e., the farthest intersection between reference path and Local safe zone

        Given current governor position, distance to free space
        and navigtaion path return the furthest possible pt
        along the Path (alpha*) in free space.

        # Key attributes from self
            xg      : center of ball  dim: 1 * D (governor position)
            deltaE  : from self, extra energy that can be safely added to the system.
        # Output
            STATUS  : status of this algorithm (GOAL_REACHED, NORMAL, HALT...)
            lpg  : local projected goal 
            nav_adv_idx : index of navagation path the system is heading to
        """

        # return governor_status, lpg, nav_adv_idx

        # TODO_STEP3
        # 1. write a function "FuncA" which can find intersections between a line segment (not unbounded line) and ball
        # 2. write a function "FuncB" to use FuncA to find furthest intersection between a ball and a path (a few waypoints in 2d array, each row is a waypoint)
        # 3. call FuncB with correct input to compute local projected goal  (hint: look at definition of local projected goal on paper )

        # nav_adv_idx: the furthest index of path lpg lies in
        # Example: nav_adv_idx = 6
        #  (0)               (5)             (6)
        #  START---->....---> A-----lpg-----> B----> .... ---> END

        # 4. determine governor_status according result step 3
        # governor_status can be {RbtGovSys.GOV_GOAL_REACHED, RbtGovSys.GOV_HALT, RbtGovSys.GOV_NORMAL}
        # GOV_GOAL_REACHED: lpg very close to final goal in reference path
        # GOV_HALT: cannot find a intersection due to numerical problem use last successful result
        # GOV_NORMAL: other cases
    
        # 5. figure out for each governor status, what other return arguments should be 
        # there are two outputs left, lpg, and nav_adv_idx


    def update_gov(self):
        """
        given current status of governor  find local projected goal xg_bar
        return the status of the main function and xg_bar

        # Input(from self)
            gov_status  : status of this govnor

        # Output(update self)
            gov_status          : new status of gov
            xg_bar              : local projected goal
            rB                  : local energy ball radius in ball way
            deltaE              : deltaE of the rgs
        """
        # get lastest gov_status, nav_adv_idx, xg_bar

        gov_status = self.gov_status
        if len(self.nav_adv_idx_log) > 0:
            nav_adv_idx = self.nav_adv_idx_log[-1]
        else:
            nav_adv_idx = 0

        rho = 0   # rho remains zero unless been changed in normal cases
        # case governor encounter error
        if gov_status < RbtGovSys.GOV_HALT:
            err_msg = 'gov_status %d' % gov_status
            raise GovError(err_msg)
        elif gov_status == RbtGovSys.GOV_HALT:
            print('Governor Halting')
        else:
            pass
        # case governor reached the goal
        if gov_status == RbtGovSys.GOV_GOAL_REACHED \
                and RbtGovSys.GOV_GOAL_REACHED_FLAG is False:
            RbtGovSys.GOV_GOAL_REACHED_FLAG = True
            print('----------- Governor Reached Goal -----------')
            gov_status = RbtGovSys.GOV_GOAL_REACHED

        """   Halt or normal running cases. """
        energy_metric = self.energy_metric

        if energy_metric == 'ball':
            # Evec = RbtGovSys.cpt_deltaE_ball(self)
            deltaE = RbtGovSys.cpt_deltaE_ball(self)
        
        # deltaE, e_plus, e_rgs, et, ev = Evec
        self.deltaE = deltaE
        
        # halt case due to simulation discretization and numerical issue
        if deltaE <= 0:
            xvec = self.xvec
            xg = xvec[-2:]
            gov_status = RbtGovSys.GOV_HALT
            xg_bar = xg

        else:
            if energy_metric == 'ball':
                rB = np.sqrt(deltaE/self.kv)
                # find new xg_bar given current configuation and nav path
                gov_status, xg_bar, nav_adv_idx = RbtGovSys.find_proj_goal_ball(self)
                rho = rB

        # for all error free cases update relevant log and gov_status
        self.gov_status = gov_status
        self.deltaE_log.append(deltaE)
        self.le_rho_log.append(rho)
        self.lpg_log.append(xg_bar)
        self.nav_adv_idx_log.append(nav_adv_idx)
        # self.Evec_log.append(Evec)
        """ UPDATE CLOCK """
        self.clock = self.clock + 1
        return gov_status, xg_bar
