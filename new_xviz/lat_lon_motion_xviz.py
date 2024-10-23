import bokeh
import math
from bokeh.plotting import figure, show, output_file
from bokeh.layouts import column, row
from bokeh.models import ColumnDataSource, CustomJS, Slider
import matplotlib
import matplotlib.cm as cm
import argparse
import numpy as np
from common.bag_reader import * 
from common.figure_viz import * 
from common.layer_viz import * 
from common.plot_viz import * 
import rsclpy
from bokeh.models import CustomJS
import pandas as pd

epsilon=0.05

def radian_to_degree(num_radian):
    return num_radian * 57.2957795

class XvizPlotLatLonMotion(XvizPlotBase):
    def __init__(self, bag_path, output):
        super().__init__(bag_path, output)
        
    def figure_plot(self):
        print("==== start to build data_frame")

        # 定制化data_frame
        self.build_obs_first_polygon_data_frame()
        self.build_obs_traj_point_data_frame()
        self.build_ego_polygon_data_frame()
        self.build_lane_data_frame()
        self.build_lat_lon_motion_success_traj_point_data_frame()
        self.build_lat_lon_motion_success_traj_acc_data_frame()
        self.build_lat_lon_motion_success_traj_omega_data_frame()
        self.build_lat_lon_motion_success_traj_vel_data_frame()
        self.build_lat_lon_motion_success_traj_theta_data_frame()
        self.build_lat_lon_motion_success_traj_theta_data_frame()
        self.build_lat_lon_motion_success_traj_jerk_data_frame()
        self.build_lat_lon_motion_success_traj_omega_dot_data_frame()
        self.build_lat_lon_motion_3_refline_kappa_data_frame()
        self.build_lat_lon_motion_match_point_data_frame()
        self.build_lat_lon_motion_ref_omega_data_frame()
        self.build_lat_lon_motion_lat_dis_to_ref_line_data_frame()
        self.build_lat_lon_motion_ref_lat_acc_data_frame()
        self.build_lat_lon_motion_ref_point_theta_data_frame()
        self.build_lat_lon_motion_lead_s_data_frame()
        self.build_lat_lon_motion_v_ref_data_frame()
        self.build_lat_lon_motion_a_ref_data_frame()
        self.build_osp_mgr_task_raw_routing_path_points_data_frame()
        self.build_osp_mgr_task_raw_routing_path_omega_data_frame()
        self.build_freespace_mgr_road_edges_points_data_frame()
        self.build_osp_mgr_task_motion_tree_points_data_frame()
        self.build_osp_mgr_task_motion_tree_speed_limit_data_frame()
        self.build_osp_mgr_task_raw_routing_path_vel_data_frame()
        # self.build_fake_sdmap_path_data_frame()
        # self.build_fake_sdmap_intersection_point_data_frame()
        self.build_nn_traj_point_data_frame()
        self.build_nn_traj_v_data_frame()
        # self.build_occ_data_frame()

        print("==== start to set main_figure")

        # 主图, 包含 obs_polygon, obs_traj, ego_polygon, lane
        main_figure_viz = FigureViz('Main_Figure', 'X', 'Y', width=750, height =900)
        self.add_layer_to_figure(main_figure_viz, args=dict(data_key='obs_first_polygon', \
                                                                plot_type='multi_polygon_id', color='blue',  \
                                                                label='obs_first_polygon', fill_alpha=0, \
                                                                line_alpha=0.4, line_width=4))
        self.add_layer_to_figure(main_figure_viz, args=dict(data_key='obs_traj_point', \
                                                                plot_type='scatter', color='black',  \
                                                                label='obs_traj_point', \
                                                                line_alpha=0.4, line_width=3))
        self.add_layer_to_figure(main_figure_viz, args=dict(data_key='ego_polygon', \
                                                                plot_type='multi_polygon', color='red',  \
                                                                label='ego_polygon', fill_alpha=0, \
                                                                line_alpha=0.4, line_width=4))
        self.add_layer_to_figure(main_figure_viz, args=dict(data_key='ego_lane_center_point', \
                                                                plot_type='scatter', color='blue',  \
                                                                label='ego_lane_center_point', \
                                                                line_alpha=0.2, line_width=4))
        self.add_layer_to_figure(main_figure_viz, args=dict(data_key='ego_raw_lane_center_point', \
                                                                plot_type='scatter', color='red',  \
                                                                label='ego_raw_lane_center_point', \
                                                                line_alpha=0.6, line_width=6))
        self.add_layer_to_figure(main_figure_viz, args=dict(data_key='ego_lane_left_bound_point', \
                                                                plot_type='line_scatter', color='Green',  \
                                                                label='ego_lane_left_bound', \
                                                                line_alpha=0.2, line_width=4))
        self.add_layer_to_figure(main_figure_viz, args=dict(data_key='ego_lane_right_bound_point', \
                                                                plot_type='line_scatter', color='coral',  \
                                                                label='ego_lane_right_bound', \
                                                                line_alpha=0.2, line_width=4))
        self.add_layer_to_figure(main_figure_viz, args=dict(data_key='left_lane_center_point', \
                                                                plot_type='scatter', color='hotpink',  \
                                                                label='left_lane_center_point', \
                                                                line_alpha=0.2, line_width=4))
        self.add_layer_to_figure(main_figure_viz, args=dict(data_key='ego_raw_left_lane_center_point', \
                                                                plot_type='scatter', color='black',  \
                                                                label='raw_left_lane_center_point', \
                                                                line_alpha=0.2, line_width=4))
        self.add_layer_to_figure(main_figure_viz, args=dict(data_key='right_lane_center_point', \
                                                                plot_type='scatter', color='indianred',  \
                                                                label='right_lane_center_point', \
                                                                line_alpha=0.2, line_width=4))
        self.add_layer_to_figure(main_figure_viz, args=dict(data_key='ego_raw_right_lane_center_point', \
                                                                plot_type='scatter', color='green',  \
                                                                label='raw_right_lane_center_point', \
                                                                line_alpha=0.2, line_width=4))
        self.add_layer_to_figure(main_figure_viz, args=dict(data_key='lat_lon_motion_success_traj_point', \
                                                                plot_type='scatter', color='green',  \
                                                                label='lat_lon_motion_success_traj_point', \
                                                                line_alpha=1, line_width=6))
        self.add_layer_to_figure(main_figure_viz, args=dict(data_key='lat_lon_motion_match_point', \
                                                                plot_type='scatter', color='darkgoldenrod',  \
                                                                label='lat_lon_motion_match_point', \
                                                                line_alpha=1, line_width=6))
        self.add_layer_to_figure(main_figure_viz, args=dict(data_key='raw_routing_pts', \
                                                                plot_type='scatter', color='purple',  \
                                                                label='raw_routing_path_points', \
                                                                line_alpha=1.0, line_width=6))     
        self.add_layer_to_figure(main_figure_viz, args=dict(data_key='motion_tree_points', \
                                                                plot_type='scatter', color='Cyan',  \
                                                                label='motion_tree_points', \
                                                                line_alpha=1.0, line_width=6))  
        self.add_layer_to_figure(main_figure_viz, args=dict(data_key='raw_road_edges_pts', \
                                                                plot_type='scatter', color="#63E398",  \
                                                                label='road_edges_points', \
                                                                line_alpha=1.0, line_width=6))
        # self.add_layer_to_figure(main_figure_viz, args=dict(data_key='fake_sdmap_path_point', \
        #                                                         plot_type='scatter', color='black',  \
        #                                                         label='fake_sdmap_path', \
        #                                                         line_alpha=1, line_width=6))
        # self.add_layer_to_figure(main_figure_viz, args=dict(data_key='fake_sdmap_intersection_point', \
        #                                                         plot_type='scatter', color='blue',  \
        #                                                         label='fake_sdmap_intersection_point', \
        #                                                         line_alpha=1, line_width=6))
        self.add_layer_to_figure(main_figure_viz, args=dict(data_key='nn_traj_point', \
                                                                plot_type='scatter', color='blue',  \
                                                                label='nn_traj_point', \
                                                                line_alpha=1, line_width=6))
        # self.add_layer_to_figure(main_figure_viz, args=dict(data_key='occ_info', \
        #                                                         plot_type='rect', color='blue',  \
        #                                                         label='occ_info', fill_alpha=0, \
        #                                                         line_alpha=0.4, line_width=4))
        self.figs_['main_fig'] = main_figure_viz.plot()
        callback_main_fig = main_figure_viz.get_callback_list()   

        print("==== start to set nop_count")     

        # nop count相关
        nop_count_figure_viz = FigureViz('Nop_Counter', 'Index', 'Data', y_range=None, 
                                         width=1100 ,height = 150, match_aspect = False)
        self.add_layer_to_figure(nop_count_figure_viz, args=dict(data_key='nop_counter', \
                                                                plot_type='single_point', \
                                                                line_alpha=1, line_width=1,label='nop_counter'))
        self.figs_['nop_count'] = nop_count_figure_viz.plot()
        callback_nop_count = nop_count_figure_viz.get_callback_list()

        # motion_res_jerk
        motion_res_jerk_figure_viz = FigureViz('Motion_Res_Jerk', 'Index', 'Data/m/s^3', x_range = [-1,31],y_range = [-6, 5], 
                                         width=550 ,height = 300, match_aspect = False)
        self.add_layer_to_figure(motion_res_jerk_figure_viz, args=dict(data_key='lat_lon_motion_success_traj_jerk', \
                                                                plot_type='index_line_circle', color='blue',  \
                                                                label='traj_jerk'))
        self.figs_['motion_res_jerk'] = motion_res_jerk_figure_viz.plot()
        callback_jerk = motion_res_jerk_figure_viz.get_callback_list()

        # motion res traj acc and ref acc 相关
        motion_res_acc_figure_viz = FigureViz('Motion_Res_Acc', 'Index', 'Data/m/s^2', x_range = [-1,31],y_range = [-6, 3], 
                                         width=550 ,height = 300, match_aspect = False)
        self.add_layer_to_figure(motion_res_acc_figure_viz, args=dict(data_key='lat_lon_motion_success_traj_acc', \
                                                                plot_type='index_line_circle', color='blue',  \
                                                                label='traj_acc'))
        self.add_layer_to_figure(motion_res_acc_figure_viz, args=dict(data_key='lat_lon_motion_a_ref', \
                                                                plot_type='index_line_circle', color='red',  \
                                                                label='ref_acc'))
        self.figs_['motion_res_acc'] = motion_res_acc_figure_viz.plot()
        callback_acc = motion_res_acc_figure_viz.get_callback_list()

        # motion res traj vel and ref vel 相关
        motion_res_vel_figure_viz = FigureViz('Motion_Res_Vel', 'Index', 'Data/m/s', x_range = [-1,31],y_range = [-1, 25], 
                                         width=550 ,height = 300, match_aspect = False)
        self.add_layer_to_figure(motion_res_vel_figure_viz, args=dict(data_key='lat_lon_motion_success_traj_vel', \
                                                                plot_type='index_line_circle', color='blue',  \
                                                                label='traj_vel'))
        self.add_layer_to_figure(motion_res_vel_figure_viz, args=dict(data_key='lat_lon_motion_v_ref', \
                                                                plot_type='index_line_circle', color='red',  \
                                                                label='ref_vel'))
        self.add_layer_to_figure(motion_res_vel_figure_viz, args=dict(data_key='nn_traj_v', \
                                                                plot_type='index_line_circle', color='black',  \
                                                            label='nn_traj_v'))
        self.add_layer_to_figure(motion_res_vel_figure_viz, args=dict(data_key='raw_routing_path_vel', \
                                                                plot_type='index_line_circle', color='Cyan',  \
                                                            label='raw_routing_path_vel'))
        
        self.figs_['motion_res_vel'] = motion_res_vel_figure_viz.plot()
        callback_vel = motion_res_vel_figure_viz.get_callback_list()

        # refline curvature
        refile_curvature_figure_viz = FigureViz('RefLine_Curvature', 'Index', 'Data/m^-1', x_range = [-1,150],y_range = [-0.01, 0.05], 
                                         width=550 ,height = 300, match_aspect = False)
        self.add_layer_to_figure(refile_curvature_figure_viz, args=dict(data_key='ego_left_refline_curvature', \
                                                                plot_type='index_line_circle', color='red',  \
                                                                label='left_ref_curva'))
        self.add_layer_to_figure(refile_curvature_figure_viz, args=dict(data_key='ego_current_refline_curvature', \
                                                                plot_type='index_line_circle', color='blue',  \
                                                                label='cur_ref_curva'))
        self.add_layer_to_figure(refile_curvature_figure_viz, args=dict(data_key='ego_right_refline_curvature', \
                                                                plot_type='index_line_circle', color='green',  \
                                                                label='right_ref_curva'))
        self.figs_['refline_curvature'] = refile_curvature_figure_viz.plot()
        callback_refline_curvature = refile_curvature_figure_viz.get_callback_list()
        
        # omega_dot
        motion_res_omega_dot_figure_viz = FigureViz('Motion_Res_Omega_Dot', 'Index', 'Data/degree/s^2', x_range = [-1,31],y_range = [-1.5, 1.5], 
                                         width=550 ,height = 300, match_aspect = False)
        self.add_layer_to_figure(motion_res_omega_dot_figure_viz, args=dict(data_key='lat_lon_motion_success_traj_omega_dot', \
                                                                plot_type='index_line_circle', color='blue',  \
                                                                label='traj_omega_dot'))
        self.figs_['motion_res_omega_dot'] = motion_res_omega_dot_figure_viz.plot()
        callback_omega_dot = motion_res_omega_dot_figure_viz.get_callback_list()

        # motion res traj omega and ref_omega 相关
        motion_res_omega_figure_viz = FigureViz('Motion_Res_Omega', 'Index', 'Data/degree/s', x_range = [-1,31],y_range = [-10, 10], 
                                         width=550 ,height = 300, match_aspect = False)
        self.add_layer_to_figure(motion_res_omega_figure_viz, args=dict(data_key='lat_lon_motion_success_traj_omega', \
                                                                plot_type='index_line_circle', color='blue',  \
                                                                label='traj_omega'))
        self.add_layer_to_figure(motion_res_omega_figure_viz, args=dict(data_key='lat_lon_motion_ref_omega', \
                                                                plot_type='index_line_circle', color='red',  \
                                                                label='ref_omega'))
        self.figs_['motion_res_omega'] = motion_res_omega_figure_viz.plot()
        callback_omega = motion_res_omega_figure_viz.get_callback_list()

        # motion res traj theta and ref_point_theta 相关
        motion_res_theta_figure_viz = FigureViz('Motion_Res_Theta', 'Index', 'Data/degree', x_range = [-1,31],y_range = [-200, 200], 
                                         width=550 ,height = 300, match_aspect = False)
        self.add_layer_to_figure(motion_res_theta_figure_viz, args=dict(data_key='lat_lon_motion_success_traj_theta', \
                                                                plot_type='index_line_circle', color='blue',  \
                                                                label='traj_theta'))
        self.add_layer_to_figure(motion_res_theta_figure_viz, args=dict(data_key='lat_lon_motion_ref_point_theta', \
                                                                plot_type='index_line_circle', color='red',  \
                                                                label='ref_point_theta'))
        self.figs_['motion_res_theta'] = motion_res_theta_figure_viz.plot()
        callback_theta = motion_res_theta_figure_viz.get_callback_list()

        print("==== start to set lat_dis_to_ref_line")

        # lat_dis_to_ref_line
        lat_dis_to_ref_line_viz = FigureViz('Lat_Dis_To_Ref_Line', 'Index', 'Data/m', x_range = [-1,31],y_range = [-3, 3], 
                                         width=550 ,height = 300, match_aspect = False)
        self.add_layer_to_figure(lat_dis_to_ref_line_viz, args=dict(data_key='lat_lon_motion_lat_dis_to_ref_line', \
                                                                plot_type='index_line_circle', color='blue',  \
                                                                label='lat_dis_to_ref_line'))
        self.figs_['lat_dis_to_ref_line'] = lat_dis_to_ref_line_viz.plot()
        callback_lat_dis_to_ref_line = lat_dis_to_ref_line_viz.get_callback_list()

        print("==== start to set ref_lat_acc_figure")

        # ref_lat_acc
        ref_lat_acc_figure_viz = FigureViz('Ref_Lat_Acc', 'Index', 'Data/m/s^2', x_range = [-1,31],y_range = [-2.0, 2.0], 
                                         width=550 ,height = 300, match_aspect = False)
        self.add_layer_to_figure(ref_lat_acc_figure_viz, args=dict(data_key='lat_lon_motion_ref_lat_acc', \
                                                                plot_type='index_line_circle', color='blue',  \
                                                                label='ref_lat_acc'))
        self.figs_['ref_lat_acc'] = ref_lat_acc_figure_viz.plot()
        callback_ref_lat_acc = ref_lat_acc_figure_viz.get_callback_list()

        print("==== start to set lead_s_figure")

        # lead_s
        lead_s_figure_viz = FigureViz('Lead_S', 'Index', 'Data/m', x_range = [-1,31],y_range = [-30, 200], 
                                         width=550 ,height = 300, match_aspect = False)
        self.add_layer_to_figure(lead_s_figure_viz, args=dict(data_key='lat_lon_motion_lead_s', \
                                                                plot_type='index_line_circle', color='blue',  \
                                                                label='lead_s'))
        self.figs_['lead_s'] = lead_s_figure_viz.plot()
        callback_lead_s = lead_s_figure_viz.get_callback_list()

        print("==== start to set osp_routing_path_omega_figure")   

         # osp routing path omega
        osp_routing_path_omega_figure_viz = FigureViz(
            "OSP_RoutingPath_Omega",
            "Index",
            "Data/degree/s",
            x_range=[-1, 31],
            y_range=[-10, 10],
            width=550,
            height=300,
            match_aspect=False,
        )
        self.add_layer_to_figure(osp_routing_path_omega_figure_viz, args=dict(data_key='osp_routing_path_points_omega', \
                                                                plot_type='index_line_circle', color='blue',  \
                                                                label='osp_routing_path_omega'))
        self.figs_["osp_rp_omega"] = osp_routing_path_omega_figure_viz.plot()
        callback_osp_rt_omega = osp_routing_path_omega_figure_viz.get_callback_list()

        print("==== start to set osp_motion_tree_speed_limit_figure")   
               
        # osp motion tree speed limit
        osp_motion_tree_speed_limit_figure_viz = FigureViz(
            "OSP_MotionTree_SpdLimit",
            "Index",
            "Data/m/s",
            x_range = [-1,31],
            y_range = [-1, 35],
            width=550,
            height=300,
            match_aspect=False,
        )
        self.add_layer_to_figure(osp_motion_tree_speed_limit_figure_viz, args=dict(data_key='osp_motion_tree_spdlimit_seq', \
                                                                plot_type='index_line_circle', color='blue',  \
                                                                label='osp_moiton_tree_spd_limit'))
        self.figs_["osp_mt_speed_limit"] = osp_motion_tree_speed_limit_figure_viz.plot()
        callback_osp_motion_tree_spd_limit = osp_motion_tree_speed_limit_figure_viz.get_callback_list()

        all_callback = [*callback_main_fig, *callback_nop_count, *callback_acc, 
                *callback_omega, *callback_vel, *callback_theta,
                *callback_omega_dot, *callback_refline_curvature, *callback_jerk,
                 *callback_lat_dis_to_ref_line,
                *callback_ref_lat_acc, 
                *callback_lead_s,
                *callback_osp_rt_omega,
                *callback_osp_motion_tree_spd_limit
                ]

        # 创建滑块        
        self.add_slider()
        self.time_slider_.js_on_change('value', *all_callback)
        
        self.activate_figure_option()
        self.add_check_group()

        print("==== start to show2html") 

        self.show2html()
        return
    
    def build_obs_first_polygon_data_frame(self):
        data_frame_new = []
        data_frame = self.get_data_frame_at_datakey('obstacle_debug_pred_first_polygon')   
        data_frame_ids = self.get_data_frame_at_datakey('obstacle_debug_id')
        for one_frame, one_frame_id in zip(data_frame, data_frame_ids):
            one_frame_new = {}
            one_frame_new['t'] = one_frame['t']
            one_frame_new['index'] = one_frame['index']

            one_data = one_frame['data']
            one_data_id = one_frame_id['data']
            xs = []
            ys = []
            ids = []
            for one_obstacle_polygon, one_data_id  in zip(one_data, one_data_id):
                x = []
                y = []
                for point in one_obstacle_polygon:
                    x.append(round(point['x'],2))
                    y.append(round(point['y'],2))
                xs.append(x)
                ys.append(y)
                ids.append(one_data_id)
            one_data_new = {
                    "xs": xs,
                    "ys": ys,
                    "obs_id": ids
                    }
            one_frame_new['data'] = one_data_new
            data_frame_new.append(one_frame_new)
        self.set_data_frame_at_datakey('polygon_channel', 'obs_first_polygon', data_frame_new)
    
    def build_obs_traj_point_data_frame(self):
        data_frame_new = []
        data_frame = self.get_data_frame_at_datakey('obstacle_debug_pred_traj')
        
        for one_frame in data_frame:
            one_frame_new = {}
            one_frame_new['t'] = one_frame['t']
            one_frame_new['index'] = one_frame['index']

            one_data = one_frame['data']
            x = []
            y = []
            for one_obstacle_traj in one_data:
                for point in one_obstacle_traj:
                    x.append(point['x'])
                    y.append(point['y'])
            one_data_new = {
                    "x": x,
                    "y": y
                    }
            one_frame_new['data'] = one_data_new
            data_frame_new.append(one_frame_new)
        self.set_data_frame_at_datakey('traj_channel', 'obs_traj_point', data_frame_new)
    
    def build_lat_lon_motion_success_traj_point_data_frame(self):
        data_frame_new = []
        data_frame = self.get_data_frame_at_datakey('lat_lon_motion_debug')
        
        for one_frame in data_frame:
            one_frame_new = {}
            one_frame_new['t'] = one_frame['t']
            one_frame_new['index'] = one_frame['index']

            x = []
            y = []
            if 'traj_tree' in one_frame['data']:
                traj_tree = one_frame['data']['traj_tree']
                if len(traj_tree) == 0:
                    pass
                else:
                    for point in traj_tree[0]:
                        x.append(point['position']['x'])
                        y.append(point['position']['y'])
            one_data_new = {
                    "x": x,
                    "y": y
                    }
            one_frame_new['data'] = one_data_new
            data_frame_new.append(one_frame_new)
        self.set_data_frame_at_datakey('traj_channel', 'lat_lon_motion_success_traj_point', data_frame_new)

    def build_fake_sdmap_path_data_frame(self):
        data_frame_new = []
        data_frame = self.get_data_frame_at_datakey('fake_sdmap_debug')
        
        for one_frame in data_frame:
            one_frame_new = {}
            one_frame_new['t'] = one_frame['t']
            one_frame_new['index'] = one_frame['index']

            x = []
            y = []
            if 'path_points' in one_frame['data']:
                path_points = one_frame['data']['path_points']
                for point in path_points:
                    x.append(point['x'])
                    y.append(point['y'])
            one_data_new = {
                    "x": x,
                    "y": y
                    }
            one_frame_new['data'] = one_data_new
            data_frame_new.append(one_frame_new)
        self.set_data_frame_at_datakey('', 'fake_sdmap_path_point', data_frame_new)

    def build_nn_traj_point_data_frame(self):
        data_frame_new = []
        data_frame = self.get_data_frame_at_datakey('nn_traj_debug')
        
        for one_frame in data_frame:
            one_frame_new = {}
            one_frame_new['t'] = one_frame['t']
            one_frame_new['index'] = one_frame['index']

            x = []
            y = []
            if one_frame['data'] == None:
                continue
            if 'nn_traj' in one_frame['data']:
                nn_traj = one_frame['data']['nn_traj']
                for point in nn_traj:
                    x.append(point['position']['x'])
                    y.append(point['position']['y'])
            one_data_new = {
                    "x": x,
                    "y": y
                    }
            one_frame_new['data'] = one_data_new
            data_frame_new.append(one_frame_new)
        self.set_data_frame_at_datakey('', 'nn_traj_point', data_frame_new)
        
    def build_nn_traj_v_data_frame(self):
        data_frame_new = []
        data_frame = self.get_data_frame_at_datakey('nn_traj_debug')
        
        for one_frame in data_frame:
            one_frame_new = {}
            one_frame_new['t'] = one_frame['t']
            one_frame_new['index'] = one_frame['index']

            x = []
            if one_frame['data'] == None:
                continue
            if 'nn_traj' in one_frame['data']:
                nn_traj = one_frame['data']['nn_traj']
                for point in nn_traj:
                    x.append(point['v'])
            one_frame_new['data'] = x
            data_frame_new.append(one_frame_new)
        self.set_data_frame_at_datakey('', 'nn_traj_v', data_frame_new)
    
    def build_fake_sdmap_intersection_point_data_frame(self):
        data_frame_new = []
        data_frame = self.get_data_frame_at_datakey('fake_sdmap_debug')
        
        for one_frame in data_frame:
            one_frame_new = {}
            one_frame_new['t'] = one_frame['t']
            one_frame_new['index'] = one_frame['index']

            x = []
            y = []
            if 'intersection_points' in one_frame['data']:
                intersection_points = one_frame['data']['intersection_points']
                for point in intersection_points:
                    x.append(point['x'])
                    y.append(point['y'])
            one_data_new = {
                    "x": x,
                    "y": y
                    }
            one_frame_new['data'] = one_data_new
            data_frame_new.append(one_frame_new)
        self.set_data_frame_at_datakey('', 'fake_sdmap_intersection_point', data_frame_new)
    
    def build_lat_lon_motion_success_traj_acc_data_frame(self):
        data_frame_new = []
        data_frame = self.get_data_frame_at_datakey('lat_lon_motion_debug')
        
        for one_frame in data_frame:
            one_frame_new = {}
            one_frame_new['t'] = one_frame['t']
            one_frame_new['index'] = one_frame['index']

            x = []
            if 'traj_tree' in one_frame['data']:
                traj_tree = one_frame['data']['traj_tree']
                if len(traj_tree) == 0:
                    pass
                else:
                    for point in traj_tree[0]:
                        x.append(point['acceleration'])
            one_frame_new['data'] = x
            data_frame_new.append(one_frame_new)
        self.set_data_frame_at_datakey('acc_channel', 'lat_lon_motion_success_traj_acc', data_frame_new)
    
    def build_lat_lon_motion_success_traj_vel_data_frame(self):
        data_frame_new = []
        data_frame = self.get_data_frame_at_datakey('lat_lon_motion_debug')
        
        for one_frame in data_frame:
            one_frame_new = {}
            one_frame_new['t'] = one_frame['t']
            one_frame_new['index'] = one_frame['index']

            x = []
            if 'traj_tree' in one_frame['data']:
                traj_tree = one_frame['data']['traj_tree']
                if len(traj_tree) == 0:
                    pass
                else:
                    for point in traj_tree[0]:
                        x.append(point['velocity'])
            one_frame_new['data'] = x
            data_frame_new.append(one_frame_new)
        self.set_data_frame_at_datakey('acc_channel', 'lat_lon_motion_success_traj_vel', data_frame_new)
    
    def build_lat_lon_motion_success_traj_omega_data_frame(self):
        data_frame_new = []
        data_frame = self.get_data_frame_at_datakey('lat_lon_motion_debug')
        
        for one_frame in data_frame:
            one_frame_new = {}
            one_frame_new['t'] = one_frame['t']
            one_frame_new['index'] = one_frame['index']

            x = []
            if 'traj_tree' in one_frame['data']:
                traj_tree = one_frame['data']['traj_tree']
                if len(traj_tree) == 0:
                    pass
                else:
                    for point in traj_tree[0]:
                        x.append(radian_to_degree(point['omega']))
            one_frame_new['data'] = x
            data_frame_new.append(one_frame_new)
        self.set_data_frame_at_datakey('omega_channel', 'lat_lon_motion_success_traj_omega', data_frame_new)
    
    def build_lat_lon_motion_success_traj_theta_data_frame(self):
        data_frame_new = []
        data_frame = self.get_data_frame_at_datakey('lat_lon_motion_debug')
        
        for one_frame in data_frame:
            one_frame_new = {}
            one_frame_new['t'] = one_frame['t']
            one_frame_new['index'] = one_frame['index']

            x = []
            if 'traj_tree' in one_frame['data']:
                traj_tree = one_frame['data']['traj_tree']
                if len(traj_tree) == 0:
                    pass
                else:
                    for point in traj_tree[0]:
                        x.append(radian_to_degree(point['theta']))
            one_frame_new['data'] = x
            data_frame_new.append(one_frame_new)
        self.set_data_frame_at_datakey('acc_channel', 'lat_lon_motion_success_traj_theta', data_frame_new)

    def build_lat_lon_motion_success_traj_jerk_data_frame(self):
        data_frame_new = []
        data_frame = self.get_data_frame_at_datakey('lat_lon_motion_debug')
        
        for one_frame in data_frame:
            one_frame_new = {}
            one_frame_new['t'] = one_frame['t']
            one_frame_new['index'] = one_frame['index']

            x = []
            if 'traj_tree' in one_frame['data']:
                traj_tree = one_frame['data']['traj_tree']
                if len(traj_tree) == 0:
                    pass
                else:
                    for point in traj_tree[0]:
                        x.append(point['jerk'])
            one_frame_new['data'] = x
            data_frame_new.append(one_frame_new)
        self.set_data_frame_at_datakey('', 'lat_lon_motion_success_traj_jerk', data_frame_new)
    
    def build_lat_lon_motion_success_traj_omega_dot_data_frame(self):
        data_frame_new = []
        data_frame = self.get_data_frame_at_datakey('lat_lon_motion_debug')
        
        for one_frame in data_frame:
            one_frame_new = {}
            one_frame_new['t'] = one_frame['t']
            one_frame_new['index'] = one_frame['index']

            x = []
            if 'traj_tree' in one_frame['data']:
                traj_tree = one_frame['data']['traj_tree']
                if len(traj_tree) == 0:
                    pass
                else:
                    for point in traj_tree[0]:
                        x.append(radian_to_degree(point['omega_dot']))
            one_frame_new['data'] = x
            data_frame_new.append(one_frame_new)
        self.set_data_frame_at_datakey('', 'lat_lon_motion_success_traj_omega_dot', data_frame_new)

    def build_lat_lon_motion_3_refline_kappa_data_frame(self):
        self.build_lat_lon_motion_left_refline_kappa_data_frame()
        self.build_lat_lon_motion_current_refline_kappa_data_frame()
        self.build_lat_lon_motion_right_refline_kappa_data_frame()
    
    
    def build_lat_lon_motion_left_refline_kappa_data_frame(self):
        data_frame = self.get_data_frame_at_datakey('lane_debug')
        data_frame_new = []
        for one_frame in data_frame:
            one_frame_new = {}
            one_frame_new['t'] = one_frame['t']
            one_frame_new['index'] = one_frame['index']

            one_data = one_frame['data']
            x = []
            for lane_debug in one_data:
                if lane_debug['id'] == -1:
                    for point in lane_debug['lane_points']:
                        x.append(point['curvature'])
            one_frame_new['data'] = x
            data_frame_new.append(one_frame_new)
        # pprint(data_frame_new)    
        self.set_data_frame_at_datakey('', 'ego_left_refline_curvature', data_frame_new)
        
    def build_lat_lon_motion_current_refline_kappa_data_frame(self):
        data_frame = self.get_data_frame_at_datakey('lane_debug')
        data_frame_new = []
        for one_frame in data_frame:
            one_frame_new = {}
            one_frame_new['t'] = one_frame['t']
            one_frame_new['index'] = one_frame['index']

            one_data = one_frame['data']
            x = []
            for lane_debug in one_data:
                if lane_debug['id'] == 0:
                    for point in lane_debug['lane_points']:
                        x.append(point['curvature'])
            one_frame_new['data'] = x
            data_frame_new.append(one_frame_new)
        # pprint(data_frame_new)    
        self.set_data_frame_at_datakey('', 'ego_current_refline_curvature', data_frame_new)
        
    def build_lat_lon_motion_right_refline_kappa_data_frame(self):
        data_frame = self.get_data_frame_at_datakey('lane_debug')
        data_frame_new = []
        for one_frame in data_frame:
            one_frame_new = {}
            one_frame_new['t'] = one_frame['t']
            one_frame_new['index'] = one_frame['index']

            one_data = one_frame['data']
            x = []
            for lane_debug in one_data:
                if lane_debug['id'] == 1:
                    for point in lane_debug['lane_points']:
                        x.append(point['curvature'])
            one_frame_new['data'] = x
            data_frame_new.append(one_frame_new)
        # pprint(data_frame_new)    
        self.set_data_frame_at_datakey('', 'ego_right_refline_curvature', data_frame_new)

    def build_lat_lon_motion_match_point_data_frame(self):
        data_frame_new = []
        data_frame = self.get_data_frame_at_datakey('lat_lon_motion_debug')
        
        for one_frame in data_frame:
            one_frame_new = {}
            one_frame_new['t'] = one_frame['t']
            one_frame_new['index'] = one_frame['index']

            x = []
            y = []
            if 'lat_debug_tree' in one_frame['data']:
                lat_debug_tree = one_frame['data']['lat_debug_tree']
                if len(lat_debug_tree) == 0:
                    pass
                else:
                    for point in lat_debug_tree[0]:
                        x.append(point['match_point']['x'])
                        y.append(point['match_point']['y'])
            one_data_new = {
                    "x": x,
                    "y": y
                    }
            one_frame_new['data'] = one_data_new
            data_frame_new.append(one_frame_new)
        self.set_data_frame_at_datakey('', 'lat_lon_motion_match_point', data_frame_new)
    
    def build_lat_lon_motion_ref_omega_data_frame(self):
        data_frame_new = []
        data_frame = self.get_data_frame_at_datakey('lat_lon_motion_debug')
        
        for one_frame in data_frame:
            one_frame_new = {}
            one_frame_new['t'] = one_frame['t']
            one_frame_new['index'] = one_frame['index']

            x = []
            if 'lat_debug_tree' in one_frame['data']:
                lat_debug_tree = one_frame['data']['lat_debug_tree']
                if len(lat_debug_tree) == 0:
                    pass
                else:
                    for point in lat_debug_tree[0]:
                        x.append(radian_to_degree(point['ref_omega']))
            one_frame_new['data'] = x
            data_frame_new.append(one_frame_new)
        self.set_data_frame_at_datakey('', 'lat_lon_motion_ref_omega', data_frame_new)
    
    def build_lat_lon_motion_lat_dis_to_ref_line_data_frame(self):
        data_frame_new = []
        data_frame = self.get_data_frame_at_datakey('lat_lon_motion_debug')
        
        for one_frame in data_frame:
            one_frame_new = {}
            one_frame_new['t'] = one_frame['t']
            one_frame_new['index'] = one_frame['index']

            x = []
            if 'lat_debug_tree' in one_frame['data']:
                lat_debug_tree = one_frame['data']['lat_debug_tree']
                if len(lat_debug_tree) == 0:
                    pass
                else:
                    for point in lat_debug_tree[0]:
                        x.append(point['lat_dis_to_ref_line'])
            one_frame_new['data'] = x
            data_frame_new.append(one_frame_new)
        self.set_data_frame_at_datakey('', 'lat_lon_motion_lat_dis_to_ref_line', data_frame_new)
    
    def build_lat_lon_motion_ref_lat_acc_data_frame(self):
        data_frame_new = []
        data_frame = self.get_data_frame_at_datakey('lat_lon_motion_debug')
        
        for one_frame in data_frame:
            one_frame_new = {}
            one_frame_new['t'] = one_frame['t']
            one_frame_new['index'] = one_frame['index']

            x = []
            if 'lat_debug_tree' in one_frame['data']:
                lat_debug_tree = one_frame['data']['lat_debug_tree']
                if len(lat_debug_tree) == 0:
                    pass
                else:
                    for point in lat_debug_tree[0]:
                        x.append(point['ref_lat_acc'])
            one_frame_new['data'] = x
            data_frame_new.append(one_frame_new)
        self.set_data_frame_at_datakey('', 'lat_lon_motion_ref_lat_acc', data_frame_new)
    
    def build_lat_lon_motion_ref_point_theta_data_frame(self):
        data_frame_new = []
        data_frame = self.get_data_frame_at_datakey('lat_lon_motion_debug')
        
        for one_frame in data_frame:
            one_frame_new = {}
            one_frame_new['t'] = one_frame['t']
            one_frame_new['index'] = one_frame['index']

            x = []
            if 'lat_debug_tree' in one_frame['data']:
                lat_debug_tree = one_frame['data']['lat_debug_tree']
                if len(lat_debug_tree) == 0:
                    pass
                else:
                    for point in lat_debug_tree[0]:
                        x.append(radian_to_degree(point['ref_point_theta']))
            one_frame_new['data'] = x
            data_frame_new.append(one_frame_new)
        self.set_data_frame_at_datakey('', 'lat_lon_motion_ref_point_theta', data_frame_new)
    
    def build_lat_lon_motion_lead_s_data_frame(self):
        data_frame_new = []
        data_frame = self.get_data_frame_at_datakey('lat_lon_motion_debug')
        
        for one_frame in data_frame:
            one_frame_new = {}
            one_frame_new['t'] = one_frame['t']
            one_frame_new['index'] = one_frame['index']

            x = []
            if 'lon_debug_tree' in one_frame['data']:
                lon_debug_tree = one_frame['data']['lon_debug_tree']
                if len(lon_debug_tree) == 0:
                    pass
                else:
                    for point in lon_debug_tree[0]:
                        x.append(point['min_s'])
            one_frame_new['data'] = x
            data_frame_new.append(one_frame_new)
        self.set_data_frame_at_datakey('', 'lat_lon_motion_lead_s', data_frame_new)
    
    def build_lat_lon_motion_v_ref_data_frame(self):
        data_frame_new = []
        data_frame = self.get_data_frame_at_datakey('lat_lon_motion_debug')
        
        for one_frame in data_frame:
            one_frame_new = {}
            one_frame_new['t'] = one_frame['t']
            one_frame_new['index'] = one_frame['index']

            x = []
            if 'lon_debug_tree' in one_frame['data']:
                lon_debug_tree = one_frame['data']['lon_debug_tree']
                if len(lon_debug_tree) == 0:
                    pass
                else:
                    for point in lon_debug_tree[0]:
                        x.append(point['v_ref'])
            one_frame_new['data'] = x
            data_frame_new.append(one_frame_new)
        self.set_data_frame_at_datakey('', 'lat_lon_motion_v_ref', data_frame_new)
    
    def build_lat_lon_motion_a_ref_data_frame(self):
        data_frame_new = []
        data_frame = self.get_data_frame_at_datakey('lat_lon_motion_debug')
        
        for one_frame in data_frame:
            one_frame_new = {}
            one_frame_new['t'] = one_frame['t']
            one_frame_new['index'] = one_frame['index']

            x = []
            if 'lon_debug_tree' in one_frame['data']:
                lon_debug_tree = one_frame['data']['lon_debug_tree']
                if len(lon_debug_tree) == 0:
                    pass
                else:
                    for point in lon_debug_tree[0]:
                        x.append(point['a_ref'])
            one_frame_new['data'] = x
            data_frame_new.append(one_frame_new)
        self.set_data_frame_at_datakey('', 'lat_lon_motion_a_ref', data_frame_new)
    
    def build_osp_mgr_task_raw_routing_path_points_data_frame(self):
        data_frame_new = []
        data_frame = self.get_data_frame_at_datakey(
            'osp_raw_routing_path_points')

        for one_frame in data_frame:
            one_frame_new = {}
            one_frame_new['t'] = one_frame['t']
            one_frame_new['index'] = one_frame['index']

            x = []
            y = []
            
            if one_frame['data'] is not None: 
                for data in one_frame['data']:
                    x.append(data['x'])
                    y.append(data['y'])
            one_data_new = {
                    "x": x,
                    "y": y
                    }   
            one_frame_new['data'] = one_data_new
            data_frame_new.append(one_frame_new)
        self.set_data_frame_at_datakey(
            '', 'raw_routing_pts', data_frame_new)
        
    def build_osp_mgr_task_motion_tree_points_data_frame(self):
        data_frame_new = []
        data_frame = self.get_data_frame_at_datakey(
            'osp_motion_tree_points')

        for one_frame in data_frame:
            one_frame_new = {}
            one_frame_new['t'] = one_frame['t']
            one_frame_new['index'] = one_frame['index']

            x = []
            y = []
            
            if one_frame['data'] is not None: 
                for data in one_frame['data']:
                    x.append(data['x'])
                    y.append(data['y'])
            one_data_new = {
                    "x": x,
                    "y": y
                    }   
            one_frame_new['data'] = one_data_new
            data_frame_new.append(one_frame_new)
        self.set_data_frame_at_datakey(
            '', 'motion_tree_points', data_frame_new)
        
    def build_osp_mgr_task_raw_routing_path_omega_data_frame(self):
        data_frame_new = []
        data_frame = self.get_data_frame_at_datakey("osp_raw_routing_path_omega")

        for one_frame in data_frame:
            one_frame_new = {}
            one_frame_new["t"] = one_frame["t"]
            one_frame_new["index"] = one_frame["index"]
            x = []
            if one_frame['data'] is not None: 
                for point in one_frame["data"]:
                    x.append(radian_to_degree(point))
            one_frame_new["data"] = x
            data_frame_new.append(one_frame_new)
        self.set_data_frame_at_datakey(
            "", "osp_routing_path_points_omega", data_frame_new
        )
        
    def build_osp_mgr_task_motion_tree_speed_limit_data_frame(self):
        data_frame_new = []
        data_frame = self.get_data_frame_at_datakey("osp_motion_tree_speed_limit_seq")

        for one_frame in data_frame:
            one_frame_new = {}
            one_frame_new["t"] = one_frame["t"]
            one_frame_new["index"] = one_frame["index"]
            x = []
            if one_frame['data'] is not None: 
                for point in one_frame["data"]:
                    x.append(point)
            one_frame_new["data"] = x
            data_frame_new.append(one_frame_new)
        self.set_data_frame_at_datakey(
            "", "osp_motion_tree_spdlimit_seq", data_frame_new
        )
        
    def build_osp_mgr_task_raw_routing_path_vel_data_frame(self):
        data_frame_new = []
        data_frame = self.get_data_frame_at_datakey("osp_raw_routing_path_vel")

        for one_frame in data_frame:
            one_frame_new = {}
            one_frame_new["t"] = one_frame["t"]
            one_frame_new["index"] = one_frame["index"]
            x = []
            if one_frame['data'] is not None: 
                for point in one_frame["data"]:
                    x.append(point)
            one_frame_new["data"] = x
            data_frame_new.append(one_frame_new)
        self.set_data_frame_at_datakey(
            "", "raw_routing_path_vel", data_frame_new
        )
 
    def build_freespace_mgr_road_edges_points_data_frame(self):
        data_frame_new = []
        data_frame = self.get_data_frame_at_datakey(
            'fs_road_edges_points')
        for one_frame in data_frame:
            one_frame_new = {}
            one_frame_new['t'] = one_frame['t']
            one_frame_new['index'] = one_frame['index']

            x = []
            y = []
            
            if one_frame['data'] is not None: 
                for data in one_frame['data']:
                    x.append(data['x'])
                    y.append(data['y'])
            one_data_new = {
                    "x": x,
                    "y": y
                    }   
            one_frame_new['data'] = one_data_new
            data_frame_new.append(one_frame_new)
        self.set_data_frame_at_datakey(
            '', 'raw_road_edges_pts', data_frame_new)
                       
    def build_ego_polygon_data_frame(self):
        data_frame_new = []
        data_frame = self.get_data_frame_at_datakey('ego_info_debug')
        for one_frame in data_frame:
            one_frame_new = {}
            one_frame_new['t'] = one_frame['t']
            one_frame_new['index'] = one_frame['index']

            one_data = one_frame['data']
            xs = []
            ys = []
            ego_polygon_x, ego_polygon_y =  \
                                self.build_polygon_by_pos_and_heading(one_data['veh_position']['x'], \
                                                                    one_data['veh_position']['y'], \
                                                                    one_data['veh_yaw'])
            xs.append(ego_polygon_x)
            ys.append(ego_polygon_y)
            one_data_new = {
                    "xs": xs,
                    "ys": ys
                    }
            one_frame_new['data'] = one_data_new
            data_frame_new.append(one_frame_new)
        self.set_data_frame_at_datakey('polygon_channel', 'ego_polygon', data_frame_new)

    def build_polygon_by_pos_and_heading(self, geometry_center_x, geometry_center_y, yaw, ego_width = 2.1, ego_length = 4.886):
        x = []
        y = []

        half_width = 0.5 * ego_width
        half_length = 0.5 * ego_length
        dist_center_to_corner = math.sqrt(half_width**2 + half_length**2)
        angel_center_to_corner = math.atan(half_width / half_length)

        left_front_corner_x = geometry_center_x + dist_center_to_corner * \
                                math.cos(angel_center_to_corner + yaw)
        left_front_corner_y = geometry_center_y + dist_center_to_corner * \
                                math.sin(angel_center_to_corner + yaw)
        
        right_front_corner_x = geometry_center_x + dist_center_to_corner * \
                                math.cos(angel_center_to_corner - yaw)
        right_front_corner_y = geometry_center_y - dist_center_to_corner * \
                                math.sin(angel_center_to_corner - yaw)
        
        left_rear_corner_x = geometry_center_x - dist_center_to_corner * \
                                math.cos(angel_center_to_corner - yaw)
        left_rear_corner_y = geometry_center_y + dist_center_to_corner * \
                                math.sin(angel_center_to_corner - yaw)
        
        right_rear_corner_x = geometry_center_x - dist_center_to_corner * \
                                math.cos(angel_center_to_corner + yaw)
        right_rear_corner_y = geometry_center_y - dist_center_to_corner * \
                                math.sin(angel_center_to_corner + yaw)
        
        x.append(left_front_corner_x)
        x.append(right_front_corner_x)
        x.append(right_rear_corner_x)
        x.append(left_rear_corner_x)
        y.append(left_front_corner_y)
        y.append(right_front_corner_y)
        y.append(right_rear_corner_y)
        y.append(left_rear_corner_y)
        return x, y

    def build_lane_data_frame(self):
        self.build_ego_lane_center_point_data_frame()
        self.build_ego_lane_raw_center_point_data_frame()
        self.build_left_lane_raw_center_point_data_frame()
        self.build_right_lane_raw_center_point_data_frame()
        self.build_ego_lane_left_bound_point_data_frame()
        self.build_ego_lane_right_bound_point_data_frame()
        self.build_left_lane_center_point_data_frame()
        self.build_right_lane_center_point_data_frame()
    
    def build_ego_lane_center_point_data_frame(self):
        data_frame = self.get_data_frame_at_datakey('lane_debug')
        data_frame_new = []
        for one_frame in data_frame:
            one_frame_new = {}
            one_frame_new['t'] = one_frame['t']
            one_frame_new['index'] = one_frame['index']

            one_data = one_frame['data']
            x = []
            y = []
            for lane_debug in one_data:
                if lane_debug['id'] == 0:
                    for point in lane_debug['lane_points']:
                        x.append(point['center_point']['x'])
                        y.append(point['center_point']['y'])
            one_data_new = {
                    "x": x,
                    "y": y
                    }
            one_frame_new['data'] = one_data_new
            data_frame_new.append(one_frame_new)
        self.set_data_frame_at_datakey('point_channel', 'ego_lane_center_point', data_frame_new)
    
    def build_ego_lane_raw_center_point_data_frame(self):
        data_frame = self.get_data_frame_at_datakey('lane_debug')
        data_frame_new = []
        for one_frame in data_frame:
            one_frame_new = {}
            one_frame_new['t'] = one_frame['t']
            one_frame_new['index'] = one_frame['index']

            one_data = one_frame['data']
            x = []
            y = []
            for lane_debug in one_data:
                if lane_debug['id'] == 0:
                    for point in lane_debug['raw_points']:
                        x.append(point['x'])
                        y.append(point['y'])
            one_data_new = {
                    "x": x,
                    "y": y
                    }
            one_frame_new['data'] = one_data_new
            data_frame_new.append(one_frame_new)
        self.set_data_frame_at_datakey('point_channel', 'ego_raw_lane_center_point', data_frame_new)
        
    def build_left_lane_raw_center_point_data_frame(self):
        data_frame = self.get_data_frame_at_datakey('lane_debug')
        data_frame_new = []
        for one_frame in data_frame:
            one_frame_new = {}
            one_frame_new['t'] = one_frame['t']
            one_frame_new['index'] = one_frame['index']

            one_data = one_frame['data']
            x = []
            y = []
            for lane_debug in one_data:
                if lane_debug['id'] == -1:
                    for point in lane_debug['raw_points']:
                        x.append(point['x'])
                        y.append(point['y'])
            one_data_new = {
                    "x": x,
                    "y": y
                    }
            one_frame_new['data'] = one_data_new
            data_frame_new.append(one_frame_new)
        self.set_data_frame_at_datakey('point_channel', 'ego_raw_left_lane_center_point', data_frame_new)
        
    def build_right_lane_raw_center_point_data_frame(self):
        data_frame = self.get_data_frame_at_datakey('lane_debug')
        data_frame_new = []
        for one_frame in data_frame:
            one_frame_new = {}
            one_frame_new['t'] = one_frame['t']
            one_frame_new['index'] = one_frame['index']

            one_data = one_frame['data']
            x = []
            y = []
            for lane_debug in one_data:
                if lane_debug['id'] == 1:
                    for point in lane_debug['raw_points']:
                        x.append(point['x'])
                        y.append(point['y'])
            one_data_new = {
                    "x": x,
                    "y": y
                    }
            one_frame_new['data'] = one_data_new
            data_frame_new.append(one_frame_new)
        self.set_data_frame_at_datakey('point_channel', 'ego_raw_right_lane_center_point', data_frame_new)

    def build_ego_lane_left_bound_point_data_frame(self):
        data_frame = self.get_data_frame_at_datakey('lane_debug')
        data_frame_new = []
        for one_frame in data_frame:
            one_frame_new = {}
            one_frame_new['t'] = one_frame['t']
            one_frame_new['index'] = one_frame['index']

            one_data = one_frame['data']
            x = []
            y = []
            for lane_debug in one_data:
                if lane_debug['id'] == 0:
                    for lane_segment in lane_debug['lane_segments']:
                        for tmp_point in lane_segment['left_line_points']:
                            x.append(tmp_point['pos']['x'])
                            y.append(tmp_point['pos']['y'])
            one_data_new = {
                    "x": x,
                    "y": y
                    }
            one_frame_new['data'] = one_data_new
            data_frame_new.append(one_frame_new)
        self.set_data_frame_at_datakey('point_channel', 'ego_lane_left_bound_point', data_frame_new)

    def build_ego_lane_right_bound_point_data_frame(self):
        data_frame = self.get_data_frame_at_datakey('lane_debug')
        data_frame_new = []
        for one_frame in data_frame:
            one_frame_new = {}
            one_frame_new['t'] = one_frame['t']
            one_frame_new['index'] = one_frame['index']

            one_data = one_frame['data']
            x = []
            y = []
            for lane_debug in one_data:
                if lane_debug['id'] == 0:
                    for lane_segment in lane_debug['lane_segments']:
                        for tmp_point in lane_segment['right_line_points']:
                            x.append(tmp_point['pos']['x'])
                            y.append(tmp_point['pos']['y'])
            one_data_new = {
                    "x": x,
                    "y": y
                    }
            one_frame_new['data'] = one_data_new
            data_frame_new.append(one_frame_new)
        self.set_data_frame_at_datakey('point_channel', 'ego_lane_right_bound_point', data_frame_new)

    def build_left_lane_center_point_data_frame(self):
        data_frame = self.get_data_frame_at_datakey('lane_debug')
        data_frame_new = []
        for one_frame in data_frame:
            one_frame_new = {}
            one_frame_new['t'] = one_frame['t']
            one_frame_new['index'] = one_frame['index']

            one_data = one_frame['data']
            x = []
            y = []
            for lane_debug in one_data:
                if lane_debug['id'] == -1:
                    for point in lane_debug['lane_points']:
                        x.append(point['center_point']['x'])
                        y.append(point['center_point']['y'])
            one_data_new = {
                    "x": x,
                    "y": y
                    }
            one_frame_new['data'] = one_data_new
            data_frame_new.append(one_frame_new)
        self.set_data_frame_at_datakey('point_channel', 'left_lane_center_point', data_frame_new)

    def build_right_lane_center_point_data_frame(self):
        data_frame = self.get_data_frame_at_datakey('lane_debug')
        data_frame_new = []
        for one_frame in data_frame:
            one_frame_new = {}
            one_frame_new['t'] = one_frame['t']
            one_frame_new['index'] = one_frame['index']

            one_data = one_frame['data']
            x = []
            y = []
            for lane_debug in one_data:
                if lane_debug['id'] == 1:
                    for point in lane_debug['lane_points']:
                        x.append(point['center_point']['x'])
                        y.append(point['center_point']['y'])
            one_data_new = {
                    "x": x,
                    "y": y
                    }
            one_frame_new['data'] = one_data_new
            data_frame_new.append(one_frame_new)
        self.set_data_frame_at_datakey('point_channel', 'right_lane_center_point', data_frame_new)
    
    def build_ego_lane_center_dist_to_left_data_frame(self):
        data_frame = self.get_data_frame_at_datakey('lane_debug')
        data_frame_new = []
        for one_frame in data_frame:
            one_frame_new = {}
            one_frame_new['t'] = one_frame['t']
            one_frame_new['index'] = one_frame['index']

            one_data = one_frame['data']
            x = []
            y = []
            for lane_debug in one_data:
                if lane_debug['id'] == 0:
                    for point in lane_debug['lane_points']:
                        x.append(point['left_boundry']['dist_to_boundry'])
            y = (list(range(len(x))))
            one_data_new = {
                    "x": x,
                    "y": y
                    }
            one_frame_new['data'] = one_data_new
            data_frame_new.append(one_frame_new)
        self.set_data_frame_at_datakey('point_channel', 'ego_lane_center_dist_to_left', data_frame_new)
        
    def transform(self,x0, y0, theta0, x_prime, y_prime, theta_prime):
        # 将自车的航向角转换为旋转矩阵
        cos_theta0 = math.cos(theta0)
        sin_theta0 = math.sin(theta0)

        # 计算在local系中的位置
        x = x0 + x_prime * sin_theta0 + y_prime * cos_theta0
        y = y0 - x_prime * cos_theta0 + y_prime * sin_theta0

        # 计算在local系中的航向角
        theta = theta0 + theta_prime

        return x, y, theta

    def find_data_by_time(self,source_list, target_time):
        for entry in source_list:
            if abs(entry['t'] - target_time) < epsilon:
                return entry['data']
        return None
        
    def build_occ_data_frame(self):
        occ_raw_data_list = self.get_data_frame_at_datakey('occupancyData')
        # ego_info_list = self.get_data_frame_at_datakey('ego_info_debug')
        ego_info_list = self.get_data_frame_at_datakey('loc_data')
        grid_size_default=0.4
    
        # transform occ_raw_data_list to local coordinate system
        data_frame_new = []
        for occ_item in occ_raw_data_list:
            one_frame_new = {}
            one_frame_new['t'] = occ_item['t']
            one_frame_new['index'] = occ_item['index']
            x_local=[]
            y_local=[]
            angle_local=[]
            state_local=[]
            match_data=self.find_data_by_time(ego_info_list,occ_item['t'])
            if match_data!=None:
                match_x=match_data['positionFlu']['x']
                match_y=match_data['positionFlu']['y']
                match_theta=match_data['yaw']
                # print(f"match_loc_x:{match_x},match_loc_y:{match_y},match_loc_theta:{match_theta}")
                
                for i in range(occ_item['data']['length']):
                    for j in range(occ_item['data']['width']):
                        x_ego=occ_item['data']['width'] *0.5*grid_size_default - j*grid_size_default -0.5*grid_size_default
                        y_ego=i * grid_size_default + grid_size_default / 2
                        # print(f"x_ego:{x_ego},y_ego:{y_ego}")
                        x_local_tmp,y_local_tmp,theta_local_tmp=self.transform(match_x,match_y,match_theta,x_ego,y_ego,0.0)
                        # print(f"trans_x_loc:{x_local_tmp},trans_y_loc:{y_local_tmp}")     
                        theta_local_tmp = theta_local_tmp-np.pi/2.0
                        state_tmp=occ_item['data']['occTypeList'][i*occ_item['data']['width']+j] 
                        if state_tmp == 0:
                            continue
                        x_local.append(x_local_tmp)
                        y_local.append(y_local_tmp)
                        angle_local.append(theta_local_tmp)
                        state_local.append(state_tmp)
                one_data_new = {
                    "x_local": x_local,
                    "y_local": y_local,
                    "theta_local":angle_local,
                    "occTypeList":state_local,
                    "width":occ_item['data']['xPartitionInfo']['steps'][0],
                    "height":occ_item['data']['yPartitionInfo']['steps'][0]
                    }   
                one_frame_new['data'] = one_data_new
                data_frame_new.append(one_frame_new)
            else:
                print('can not find ego location info!')
        self.set_data_frame_at_datakey('', 'occ_info', data_frame_new)
    
    def build_ego_lane_center_dist_to_right_data_frame(self):
        data_frame = self.get_data_frame_at_datakey('lane_debug')
        data_frame_new = []
        for one_frame in data_frame:
            one_frame_new = {}
            one_frame_new['t'] = one_frame['t']
            one_frame_new['index'] = one_frame['index']

            one_data = one_frame['data']
            x = []
            y = []
            for lane_debug in one_data:
                if lane_debug['id'] == 0:
                    for point in lane_debug['lane_points']:
                        x.append(point['right_boundry']['dist_to_boundry'])
            y = (list(range(len(x))))
            one_data_new = {
                    "x": x,
                    "y": y
                    }
            one_frame_new['data'] = one_data_new
            data_frame_new.append(one_frame_new)
        self.set_data_frame_at_datakey('point_channel', 'ego_lane_center_dist_to_right', data_frame_new)
    
    def add_check_group(self):
        for fig_name in ['motion_res_jerk', 'motion_res_acc','motion_res_vel','lead_s','osp_rp_omega','osp_mt_speed_limit',
                     'motion_res_omega_dot','motion_res_omega','motion_res_theta','lat_dis_to_ref_line','ref_lat_acc',
                     'refline_curvature']:
            fig = self.figs_[fig_name]
            checkbox_group = CheckboxButtonGroup(labels=[fig_name], active=[0], width=500)
            callback = CustomJS(args=dict(fig=fig, checkbox_group=checkbox_group), code="""
                fig.visible = checkbox_group.active.includes(0);
            """)
            checkbox_group.js_on_change('active', callback)
            self.checkbox_groups_[fig_name] = checkbox_group  # Add the checkbox group to the dictionary
        
    def show2html(self):
        output_file(self.output_)
        left = column(self.checkbox_groups_['motion_res_jerk'],self.figs_['motion_res_jerk'],
                      self.checkbox_groups_['motion_res_acc'],self.figs_['motion_res_acc'],
                      self.checkbox_groups_['motion_res_vel'],self.figs_['motion_res_vel'],
                      self.checkbox_groups_['lead_s'],self.figs_['lead_s'],
                      self.checkbox_groups_['osp_rp_omega'],self.figs_['osp_rp_omega'],
                      self.checkbox_groups_['osp_mt_speed_limit'], self.figs_['osp_mt_speed_limit'])
        right = column(self.checkbox_groups_['motion_res_omega_dot'],self.figs_['motion_res_omega_dot'],
                       self.checkbox_groups_['refline_curvature'],self.figs_['refline_curvature'],
                       self.checkbox_groups_['motion_res_omega'],self.figs_['motion_res_omega'],
                       self.checkbox_groups_['motion_res_theta'],self.figs_['motion_res_theta'],
                       self.checkbox_groups_['lat_dis_to_ref_line'],self.figs_['lat_dis_to_ref_line'],
                       self.checkbox_groups_['ref_lat_acc'],self.figs_['ref_lat_acc'])
        row1 = row(left, right)
        col1 = column(self.figs_['nop_count'],row1)
        row2 = row(self.figs_['main_fig'], col1)
        show(column(self.time_slider_,row2))
        return

        

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='parse rsclbag',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-s',
                        '--source',
                        dest='source',
                        help='specify source rsclbag',
                        required=True)
    parser.add_argument('-o',
                        '--output',
                        dest='output',
                        help='specify output html',
                        required=True)
    args = parser.parse_args()

    xpilot_plot = XvizPlotLatLonMotion(args.source, args.output)

    xpilot_plot.add_required_channel(XDEBUG_CHANNEL)
    xpilot_plot.add_required_channel(PLANNING_CHANNEL)
    xpilot_plot.add_required_channel(LOCALIZATION_CHANNEL)
    xpilot_plot.add_required_channel(OCC_CHANNEL)
    xpilot_plot.read_msg()

    xpilot_plot.figure_plot()