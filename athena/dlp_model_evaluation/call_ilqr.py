import numpy as np
import os
import sys
current_directory = os.getcwd()
# print(current_directory)
relative_path1 = '../../algorithm/build/env_simulator'
relative_path2 = '../../algorithm/build/cilqr'

# debug
# relative_path1 = './algorithm/build/env_simulator'
# relative_path2 = './algorithm/build/cilqr'

# jupyter
# relative_path1 = '../../../algorithm/build/env_simulator'
# relative_path2 = '../../../algorithm/build/cilqr'
file_path1 = os.path.join(current_directory, relative_path1)
file_path2 = os.path.join(current_directory, relative_path2)
sys.path.append(file_path1)
sys.path.append(file_path2)
import pybind_env_simulator
import pybind_ilqr

from bokeh.plotting import figure, show
from bokeh.io import output_notebook
from bokeh.models import WheelZoomTool
import json
from IPython.display import display, HTML

sys.path.append("..")
from common.type import *

class CallILQR:
    def __init__(self, ref_pts, ego_origin):
        self.ref_pts_ = ref_pts
        self.ego_origin_ = ego_origin
        self.processed_trajs_ = []
        self.motion_ = pybind_ilqr.LateralLongitudinalMotion()
        # for plot test
        self.plot_data_ = {}

    def process(self):
        point_init = pybind_ilqr.PlanningPoint()
        point_init.position.x = self.ego_origin_.position.x
        point_init.position.y = self.ego_origin_.position.y
        point_init.theta = self.ego_origin_.theta
        point_init.velocity = self.ego_origin_.velocity
        point_init.acceleration = 0.0

        trajectory_points = []
        for pt in self.ref_pts_:
            point = pybind_env_simulator.Point2D()
            point.x = pt['x']
            point.y = pt['y']
            trajectory_points.append(point)
        self.motion_.init(trajectory_points, point_init)
        self.motion_.execute_tree()
        return

    def get_processed_trajs(self):
        traj_trees = self.motion_.get_trajectory_tree()
        print("trajectory tree branch size is: ", len(traj_trees))
        lon_debug = self.motion_.get_lon_debug_tree()
        lat_debug = self.motion_.get_lat_debug_tree()
        self.processed_trajs_ = []
        j = 0
        for traj_tree in traj_trees:  
            traj_new_list = []
            index = 0
            for traj in traj_tree:
                pt = {}
                pt['x'] = traj.position.x
                pt['y'] = traj.position.y
                pt['theta'] = traj.theta
                pt['v'] = traj.velocity
                pt['acc'] = traj.acceleration
                pt['acc_lat'] = traj.velocity * traj.omega
                pt['ref_vel'] = lon_debug[j][index].v_ref
                pt['ref_acc'] = lon_debug[j][index].a_ref
                pt['match_point'] = lat_debug[j][index].match_point
                pt['jerk'] = traj.jerk
                pt['omega_dot'] = traj.omega_dot
                index = index + 1
                traj_new_list.append(pt)
            self.processed_trajs_.append(traj_new_list)
            j = j+1        
        return self.processed_trajs_

    def get_plot_data(self):
        # generate plot data
        env = self.motion_.get_env()
        planning_origin = self.motion_.get_planning_origin()
        pts = env.get_all_center_points()
        obs = env.get_all_obstacle()
        planning_init = {}
        planning_init['x'] = planning_origin.position.x
        planning_init['y'] = planning_origin.position.y
        planning_init['theta'] = planning_origin.theta
        trajs = self.motion_.get_init_trajectory()
        traj_list = []
        for traj in trajs:
            pt = {}
            pt['x'] = traj.position.x
            pt['y'] = traj.position.y
            pt['theta'] = traj.theta
            traj_list.append(pt)
        self.plot_data_['pts'] = pts
        self.plot_data_['obs'] = obs
        self.plot_data_['planning_init'] = planning_init
        self.plot_data_['traj_list'] = traj_list      
        return self.plot_data_
    
    def get_ref_pts(self):
        return self.ref_pts_