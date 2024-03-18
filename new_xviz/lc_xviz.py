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

def radian_to_degree(num_radian):
    return num_radian * 57.2957795

lon_search_max_cost = 1e20

def is_lon_search_cost_max(cost):
    if cost > lon_search_max_cost:
        return True
    return False

def clip_lc_lon_search_cost(cost):
    if is_lon_search_cost_max(cost):
        return lon_search_max_cost
    return cost

class XvizPlotLc(XvizPlotBase):
    def __init__(self, bag_path, output):
        super().__init__(bag_path, output)
        
    def figure_plot(self):
        # 定制化data_frame
        self.build_obs_first_polygon_data_frame()
        self.build_obs_traj_point_data_frame()
        self.build_ego_polygon_data_frame()
        self.build_lane_data_frame()
        self.build_traj_point_data_frame()
        self.build_traj_s_theta_data_frame()
        self.build_traj_index_theta_error_data_frame()
        self.build_traj_index_delta_data_frame()
        self.build_lc_lat_search_sd_bound_data_frame()
        self.build_lc_lat_search_sd_range_data_frame()
        self.build_lc_lat_search_sd_goal_data_frame()
        self.build_lc_lat_search_sd_ego_data_frame()
        self.build_lc_lat_search_sd_ref_data_frame()
        self.build_lc_lon_search_st_obs_polygon_data_frame()
        self.build_lc_lon_search_st_res_data_frame()
        self.build_lc_lon_search_st_sample_point_data_frame()
        self.build_lc_lon_search_vt_res_data_frame()
        self.build_lc_lon_search_vt_limit_data_frame()


        # 主图, 包含 obs_polygon, obs_traj, ego_polygon, lane
        main_figure_viz = FigureViz('Main_Figure', 'X', 'Y', width=750, height =900)
        self.add_layer_to_figure(main_figure_viz, args=dict(data_key='obs_first_polygon', \
                                                                plot_type='multi_polygon', color='blue',  \
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
        self.add_layer_to_figure(main_figure_viz, args=dict(data_key='right_lane_center_point', \
                                                                plot_type='scatter', color='indianred',  \
                                                                label='right_lane_center_point', \
                                                                line_alpha=0.2, line_width=4))
        self.add_layer_to_figure(main_figure_viz, args=dict(data_key='traj_point', \
                                                                plot_type='scatter', color='green',  \
                                                                label='traj_point', \
                                                                line_alpha=1, line_width=6))
        self.add_layer_to_figure(main_figure_viz, args=dict(data_key='traj_point_ref', \
                                                                plot_type='scatter', color='darkgoldenrod',  \
                                                                label='traj_point_ref', \
                                                                line_alpha=1, line_width=6))
        self.figs_['main_fig'] = main_figure_viz.plot()
        callback_main_fig = main_figure_viz.get_callback_list()        
        

        # nop count相关
        nop_count_figure_viz = FigureViz('Nop_Counter', 'Index', 'Data', y_range=None, 
                                         width=600 ,height = 150, match_aspect = False)
        self.add_layer_to_figure(nop_count_figure_viz, args=dict(data_key='nop_counter', \
                                                                plot_type='single_point'))
        self.figs_['nop_count'] = nop_count_figure_viz.plot()
        callback_nop_count = nop_count_figure_viz.get_callback_list()

        # 轨迹原点重规划相关
        traj_update_figure_viz = FigureViz('Origin_Update', 'Index', 'Data', y_range=[-1, 2], 
                                           width=600 ,height = 150)
        self.add_layer_to_figure(traj_update_figure_viz, args=dict(data_key='updatedByVehicleStatus', \
                                                                plot_type='single_point'))
        self.figs_['traj_update'] = traj_update_figure_viz.plot()
        callback_traj_update = traj_update_figure_viz.get_callback_list()

        # traj index-d
        fig_traj_d = FigureViz('Traj_Index_D', 'Index', 'D/m', width=600, height =200,y_range =[-4, 4])
        self.add_layer_to_figure(fig_traj_d, args=dict(data_key='traj_ref_d', \
                                                                plot_type='index_circle', color='blue',  \
                                                                label='traj_d_ref', \
                                                                line_alpha=0.2, line_width=4))
        self.add_layer_to_figure(fig_traj_d, args=dict(data_key='traj_d', \
                                                                plot_type='index_circle', color='red',  \
                                                                label='traj_d', \
                                                                line_alpha=0.2, line_width=4))
        self.figs_['traj_d'] = fig_traj_d.plot()
        callback_traj_d = fig_traj_d.get_callback_list()

        # traj s-theta
        fig_traj_theta = FigureViz('Traj_S_Theta', 'S/m', 'Theta/degree', width=600, height =200,
                                   y_range = None)
        self.add_layer_to_figure(fig_traj_theta, args=dict(data_key='traj_s_theta', \
                                                                plot_type='scatter', color='red',  \
                                                                label='traj_theta', \
                                                                line_alpha=0.2, line_width=4))
        self.figs_['traj_theta'] = fig_traj_theta.plot()
        callback_traj_theta = fig_traj_theta.get_callback_list()    
        
        # traj index-theta_error_ref
        fig_traj_theta_error_ref = FigureViz('Traj_Index_Theta_Error_Ref', 'Index', 'Theta/degree', width=600, height =200)
        self.add_layer_to_figure(fig_traj_theta_error_ref, args=dict(data_key='traj_index_theta_err_ref', \
                                                                plot_type='index_circle', color='blue',  \
                                                                label='traj_theta_error_ref', \
                                                                line_alpha=0.2, line_width=4))
        self.figs_['traj_theta_error_ref'] = fig_traj_theta_error_ref.plot()
        callback_traj_theta_error_ref = fig_traj_theta_error_ref.get_callback_list()  
        
        # traj index-delta_ref index-delta
        fig_traj_delta = FigureViz('Traj_Index_Delta', 'Index', 'Delta/degree', width=600, height =200,y_range = None)
        self.add_layer_to_figure(fig_traj_delta, args=dict(data_key='traj_index_delta_ref', \
                                                                plot_type='index_circle', color='blue',  \
                                                                label='traj_delta_ref', \
                                                                line_alpha=0.2, line_width=4))
        self.add_layer_to_figure(fig_traj_delta, args=dict(data_key='traj_index_delta', \
                                                                plot_type='index_circle', color='red',  \
                                                                label='traj_delta', \
                                                                line_alpha=0.2, line_width=4))
        self.figs_['traj_delta'] = fig_traj_delta.plot()
        callback_traj_delta = fig_traj_delta.get_callback_list()  

        # 优化s相关
        longi_s_figure_viz = FigureViz('Longi_Opti_S', 'Index', 'Data', x_range = [-5,35], y_range =[-100, 1200],
                                         width=600 ,height = 200)
        self.add_layer_to_figure(longi_s_figure_viz, args=dict(data_key='optimizer_s_out', \
                                                                plot_type='index_line_circle', color='red',  \
                                                                label='s_out'))
        self.add_layer_to_figure(longi_s_figure_viz, args=dict(data_key='optimizer_s_ref', \
                                                                plot_type='index_line_circle', color='blue',  \
                                                                label='s_ref'))
        self.add_layer_to_figure(longi_s_figure_viz, args=dict(data_key='optimizer_s_upper', \
                                                                plot_type='index_circle', color='black',  \
                                                                label='s_upper'))
        self.figs_['longi_optimization_s'] = longi_s_figure_viz.plot()
        callback_s = longi_s_figure_viz.get_callback_list()
        
        # 优化vel相关
        longi_vel_figure_viz = FigureViz('Longi_Opti_Vel', 'Index', 'Data', x_range = [-5,35], y_range =[-5, 30],
                                         width=600 ,height = 200)
        self.add_layer_to_figure(longi_vel_figure_viz, args=dict(data_key='optimizer_v_out', \
                                                                plot_type='index_line_circle', color='red',  \
                                                                label='vel_out'))
        self.add_layer_to_figure(longi_vel_figure_viz, args=dict(data_key='optimizer_v_ref', \
                                                                plot_type='index_line_circle', color='blue',  \
                                                                label='vel_ref'))
        self.add_layer_to_figure(longi_vel_figure_viz, args=dict(data_key='optimizer_v_upper', \
                                                                plot_type='index_circle', color='black',  \
                                                                label='vel_upper'))
        self.figs_['longi_optimization_vel'] = longi_vel_figure_viz.plot()
        callback_vel = longi_vel_figure_viz.get_callback_list()
        
        # 优化acc相关
        longi_acc_figure_viz = FigureViz('Longi_Opti_Acc', 'Index', 'Data', x_range = [-5,35],y_range = [-6, 3], 
                                         width=600 ,height = 200)
        self.add_layer_to_figure(longi_acc_figure_viz, args=dict(data_key='optimizer_a_out', \
                                                                plot_type='index_line_circle', color='red',  \
                                                                label='acc_out'))
        self.add_layer_to_figure(longi_acc_figure_viz, args=dict(data_key='optimizer_a_ref', \
                                                                plot_type='index_line_circle', color='blue',  \
                                                                label='acc_ref'))
        self.add_layer_to_figure(longi_acc_figure_viz, args=dict(data_key='optimizer_a_upper', \
                                                                plot_type='index_circle', color='black',  \
                                                                label='acc_upper'))
        self.add_layer_to_figure(longi_acc_figure_viz, args=dict(data_key='optimizer_a_lower', \
                                                                plot_type='index_circle', color='black',  \
                                                                label='acc_lower'))
        self.figs_['longi_optimization_acc'] = longi_acc_figure_viz.plot()
        callback_acc = longi_acc_figure_viz.get_callback_list()

        # 优化jerk相关
        longi_jerk_figure_viz = FigureViz('Longi_Opti_Jerk', 'Index', 'Data', x_range = [-5,35],y_range = [-5, 5],
                                         width=600 ,height = 200)
        self.add_layer_to_figure(longi_jerk_figure_viz, args=dict(data_key='optimizer_jerk_out', \
                                                                plot_type='index_line_circle', color='red',  \
                                                                label='jerk_out'))
        self.add_layer_to_figure(longi_jerk_figure_viz, args=dict(data_key='optimizer_jerk_lower', \
                                                                plot_type='index_circle', color='black',  \
                                                                label='jerk_lower'))
        self.add_layer_to_figure(longi_jerk_figure_viz, args=dict(data_key='optimizer_jerk_upper', \
                                                                plot_type='index_circle', color='black',  \
                                                                label='jerk_upper'))
        self.figs_['longi_optimization_jerk'] = longi_jerk_figure_viz.plot()
        callback_jerk = longi_jerk_figure_viz.get_callback_list()

        # lc lat search sd
        fig_lc_lat_search = FigureViz('Lc_Lat_Search_S_D', 'S/m', 'D/m', width=600, height =600, x_range=[10, -10])
        self.add_layer_to_figure(fig_lc_lat_search, args=dict(data_key='lat_search_sd_bound_point', \
                                                                plot_type='scatter', color='green',  \
                                                                label='sd_bound', \
                                                                line_alpha=1, line_width=6))
        self.add_layer_to_figure(fig_lc_lat_search, args=dict(data_key='lat_search_sd_range_point', \
                                                                plot_type='scatter', color='blue',  \
                                                                label='sd_range', \
                                                                line_alpha=1, line_width=6))
        self.add_layer_to_figure(fig_lc_lat_search, args=dict(data_key='lat_search_sd_goal_point', \
                                                                plot_type='scatter', color='yellow',  \
                                                                label='sd_goal', \
                                                                line_alpha=1, line_width=6))
        self.add_layer_to_figure(fig_lc_lat_search, args=dict(data_key='lat_search_sd_ego_point', \
                                                                plot_type='scatter', color='red',  \
                                                                label='sd_ego', \
                                                                line_alpha=1, line_width=8))
        self.add_layer_to_figure(fig_lc_lat_search, args=dict(data_key='lat_search_sd_ref_point', \
                                                                plot_type='scatter', color='black',  \
                                                                label='sd_ref', \
                                                                line_alpha=1, line_width=4))
        self.figs_['lc_lat_search'] = fig_lc_lat_search.plot()
        callback_lc_lat_search = fig_lc_lat_search.get_callback_list()       

        # lc lon search st
        fig_lc_lon_search_st = FigureViz('Lc_Lon_Search_T_S', 'T/s', 'S/m', width=600, height =600, x_range=[-1, 7])
        self.add_layer_to_figure(fig_lc_lon_search_st, args=dict(data_key='lc_lon_search_st_obs_polygon', \
                                                                plot_type='multi_polygon_lc_lon_search_obs', color='hotpink',  \
                                                                label='obs_polygon', fill_alpha=0.6, \
                                                                fill_color='red',
                                                                line_alpha=0.4, size=4))
        self.add_layer_to_figure(fig_lc_lon_search_st, args=dict(data_key='lc_lon_search_st_res', \
                                                        plot_type='line_scatter', color='coral',  \
                                                        label='st_res', \
                                                        line_alpha=0.2, size=6))
        self.add_layer_to_figure(fig_lc_lon_search_st, args=dict(data_key='lc_lon_search_st_sample_point', \
                                                        plot_type='scatter_lc_lon_search_sample', \
                                                        label='st_sample_point', \
                                                        line_alpha=1, size=10))
        self.figs_['lc_lon_search_st'] = fig_lc_lon_search_st.plot()
        callback_lc_lon_search_st = fig_lc_lon_search_st.get_callback_list()  

        # lc lon search vt
        fig_lc_lon_search_vt = FigureViz('Lc_Lon_Search_T_V', 'T/s', 'V/m/s', width=600, height =600, x_range=[-1, 7])
        self.add_layer_to_figure(fig_lc_lon_search_vt, args=dict(data_key='lc_lon_search_vt_res', \
                                                        plot_type='line_scatter', color='coral',  \
                                                        label='vt_res', \
                                                        line_alpha=0.2, size=6))
        self.add_layer_to_figure(fig_lc_lon_search_vt, args=dict(data_key='lc_lon_search_vt_limit', \
                                                        plot_type='scatter', color='blue',  \
                                                        label='vt_limit', \
                                                        line_alpha=0.2, size=6))
        self.figs_['lc_lon_search_vt'] = fig_lc_lon_search_vt.plot()
        callback_lc_lon_search_vt = fig_lc_lon_search_vt.get_callback_list()       

        # 汇合所有的callback
        all_callback = [*callback_main_fig, *callback_nop_count, 
                        *callback_traj_update, 
                        *callback_traj_d, *callback_traj_theta,
                        *callback_traj_theta_error_ref, *callback_traj_delta,
                        *callback_s, *callback_vel,
                        *callback_acc, *callback_jerk,
                        *callback_lc_lat_search,
                        *callback_lc_lon_search_st,
                        *callback_lc_lon_search_vt,
                        ]

        # 创建滑块        
        self.add_slider()
        self.time_slider_.js_on_change('value', *all_callback)
        
        self.activate_figure_option()
        self.show2html()
        return
    
    def build_obs_first_polygon_data_frame(self):
        data_frame_new = []
        data_frame = self.get_data_frame_at_datakey('obstacle_debug_pred_first_polygon')
        for one_frame in data_frame:
            one_frame_new = {}
            one_frame_new['t'] = one_frame['t']
            one_frame_new['index'] = one_frame['index']

            one_data = one_frame['data']
            xs = []
            ys = []
            for one_obstacle_polygon in one_data:
                x = []
                y = []
                for point in one_obstacle_polygon:
                    x.append(point['x'])
                    y.append(point['y'])
                xs.append(x)
                ys.append(y)
            one_data_new = {
                    "xs": xs,
                    "ys": ys
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

    def build_traj_point_data_frame(self):
        data_frame_traj_x = self.get_data_frame_at_datakey('traj_x', XDEBUG_CHANNEL)
        data_frame_traj_y = self.get_data_frame_at_datakey('traj_y', XDEBUG_CHANNEL)

        data_frame_new = []
        for one_frame_x,one_frame_y in zip(data_frame_traj_x,data_frame_traj_y):
            one_frame_new = {}
            one_frame_new['t'] = one_frame_x['t']
            one_frame_new['index'] = one_frame_x['index']

            one_data_x = one_frame_x['data']
            one_data_y = one_frame_y['data']
            x = []
            y = []
            for tmp_x,tmp_y in zip(one_data_x,one_data_y):
                x.append(tmp_x)
                y.append(tmp_y)
            one_data_new = {
                    "x": x,
                    "y": y
                    }
            # print(one_data_new)
            one_frame_new['data'] = one_data_new
            data_frame_new.append(one_frame_new)
        self.set_data_frame_at_datakey('', 'traj_point', data_frame_new)

        data_frame_traj_r_ref_x = self.get_data_frame_at_datakey('traj_r_ref_x')
        data_frame_traj_r_ref_y = self.get_data_frame_at_datakey('traj_r_ref_y')
        data_frame_new = []
        for one_frame_x,one_frame_y in zip(data_frame_traj_r_ref_x,data_frame_traj_r_ref_y):
            one_frame_new = {}
            one_frame_new['t'] = one_frame_x['t']
            one_frame_new['index'] = one_frame_x['index']

            one_data_x = one_frame_x['data']
            one_data_y = one_frame_y['data']
            x = []
            y = []
            for tmp_x,tmp_y in zip(one_data_x,one_data_y):
                x.append(tmp_x)
                y.append(tmp_y)
            one_data_new = {
                    "x": x,
                    "y": y
                    }
            one_frame_new['data'] = one_data_new
            data_frame_new.append(one_frame_new)
        self.set_data_frame_at_datakey('', 'traj_point_ref', data_frame_new)

    def build_traj_s_theta_data_frame(self):
        data_frame_traj_s = self.get_data_frame_at_datakey('traj_s')
        data_frame_traj_theta = self.get_data_frame_at_datakey('traj_theta')
        data_frame_new = []
        for one_frame_s,one_frame_theta in zip(data_frame_traj_s,data_frame_traj_theta):
            one_frame_new = {}
            one_frame_new['t'] = one_frame_s['t']
            one_frame_new['index'] = one_frame_s['index']

            one_data_s = one_frame_s['data']
            one_data_theta = one_frame_theta['data']
            x = []
            y = []
            for s,theta in zip(one_data_s,one_data_theta):
                x.append(s)
                y.append(radian_to_degree(theta))
            one_data_new = {
                    "x": x,
                    "y": y
                    }
            one_frame_new['data'] = one_data_new
            data_frame_new.append(one_frame_new)
        self.set_data_frame_at_datakey('', 'traj_s_theta', data_frame_new)

    def build_traj_index_theta_error_data_frame(self):
        data_frame_traj_theta_err_ref = self.get_data_frame_at_datakey('traj_theta_error_ref')
        data_frame_new = []
        for one_frame_d in data_frame_traj_theta_err_ref:
            one_frame_new = {}
            one_frame_new['t'] = one_frame_d['t']
            one_frame_new['index'] = one_frame_d['index']

            one_data_d = one_frame_d['data']
            x = []
            for d in one_data_d:
                x.append(radian_to_degree(d))
            one_data_new = x
            one_frame_new['data'] = one_data_new
            data_frame_new.append(one_frame_new)
        self.set_data_frame_at_datakey('', 'traj_index_theta_err_ref', data_frame_new)

    def build_traj_index_delta_data_frame(self):
        data_frame_traj_delta = self.get_data_frame_at_datakey('traj_delta')
        data_frame_new = []
        for one_frame_d in data_frame_traj_delta:
            one_frame_new = {}
            one_frame_new['t'] = one_frame_d['t']
            one_frame_new['index'] = one_frame_d['index']

            one_data_d = one_frame_d['data']
            x = []
            for d in one_data_d:
                x.append(radian_to_degree(d))
            one_data_new = x
            one_frame_new['data'] = one_data_new
            data_frame_new.append(one_frame_new)
        self.set_data_frame_at_datakey('', 'traj_index_delta', data_frame_new)

        data_frame_traj_d_ref = self.get_data_frame_at_datakey('traj_delta_ref')
        data_frame_new = []
        for one_frame_d in data_frame_traj_d_ref:
            one_frame_new = {}
            one_frame_new['t'] = one_frame_d['t']
            one_frame_new['index'] = one_frame_d['index']

            one_data_d = one_frame_d['data']
            x = []
            for d in one_data_d:
                x.append(radian_to_degree(d))
            one_data_new = x
            one_frame_new['data'] = one_data_new
            data_frame_new.append(one_frame_new)
        self.set_data_frame_at_datakey('', 'traj_index_delta_ref', data_frame_new)

    def build_lc_lat_search_sd_bound_data_frame(self):
        data_frame_lc_lat_search_debug = self.get_data_frame_at_datakey('lc_lat_search_debug')

        data_frame_new = []
        for one_frame in data_frame_lc_lat_search_debug:
            one_frame_new = {}
            one_frame_new['t'] = one_frame['t']
            one_frame_new['index'] = one_frame['index']

            one_data = one_frame['data']
            x = []
            y = []
            if 'ds_bound' in one_data:
                ds_bound = one_data['ds_bound']
                if 's_seq' in ds_bound and 'd_low_seq' in ds_bound and \
                    'd_high_seq' in ds_bound and \
                    len(ds_bound['s_seq']) == len(ds_bound['d_low_seq']) and \
                    len(ds_bound['s_seq']) == len(ds_bound['d_high_seq']):
                    for i in range(len(ds_bound['s_seq'])):
                        x.append(ds_bound['s_seq'][i])
                        y.append(ds_bound['d_low_seq'][i])
                    for i in range(len(ds_bound['s_seq'])):
                        x.append(ds_bound['s_seq'][i])
                        y.append(ds_bound['d_high_seq'][i])
            one_data_new = {
                    "x": y,
                    "y": x
                    }
            one_frame_new['data'] = one_data_new
            data_frame_new.append(one_frame_new)
        self.set_data_frame_at_datakey('', 'lat_search_sd_bound_point', data_frame_new)
    
    def build_lc_lat_search_sd_range_data_frame(self):
        data_frame_lc_lat_search_debug = self.get_data_frame_at_datakey('lc_lat_search_debug')

        data_frame_new = []
        for one_frame in data_frame_lc_lat_search_debug:
            one_frame_new = {}
            one_frame_new['t'] = one_frame['t']
            one_frame_new['index'] = one_frame['index']

            one_data = one_frame['data']
            x = []
            y = []
            if 'ds_range' in one_data and 'ds_bound' in one_data:
                ds_range = one_data['ds_range']
                ds_bound = one_data['ds_bound']
                if 's_seq' in ds_bound and 'd_low' in ds_range and \
                    'd_high' in ds_range:
                    for i in range(len(ds_bound['s_seq'])):
                        x.append(ds_bound['s_seq'][i])
                        y.append(ds_range['d_low'])
                    for i in range(len(ds_bound['s_seq'])):
                        x.append(ds_bound['s_seq'][i])
                        y.append(ds_range['d_high'])
            one_data_new = {
                    "x": y,
                    "y": x
                    }
            one_frame_new['data'] = one_data_new
            data_frame_new.append(one_frame_new)
        self.set_data_frame_at_datakey('', 'lat_search_sd_range_point', data_frame_new)
    
    def build_lc_lat_search_sd_goal_data_frame(self):
        data_frame_lc_lat_search_debug = self.get_data_frame_at_datakey('lc_lat_search_debug')

        data_frame_new = []
        for one_frame in data_frame_lc_lat_search_debug:
            one_frame_new = {}
            one_frame_new['t'] = one_frame['t']
            one_frame_new['index'] = one_frame['index']

            one_data = one_frame['data']
            x = []
            y = []
            if 'lateral_goal' in one_data:
                lateral_goal = one_data['lateral_goal']
                for pt in lateral_goal:
                    x.append(pt['x'])
                    y.append(pt['y'])
            one_data_new = {
                    "x": y,
                    "y": x
                    }
            one_frame_new['data'] = one_data_new
            data_frame_new.append(one_frame_new)
        self.set_data_frame_at_datakey('', 'lat_search_sd_goal_point', data_frame_new)
    
    def build_lc_lat_search_sd_ego_data_frame(self):
        data_frame_lc_lat_search_debug = self.get_data_frame_at_datakey('lc_lat_search_debug')

        data_frame_new = []
        for one_frame in data_frame_lc_lat_search_debug:
            one_frame_new = {}
            one_frame_new['t'] = one_frame['t']
            one_frame_new['index'] = one_frame['index']

            one_data = one_frame['data']
            x = []
            y = []
            if 'origin_pos' in one_data:
                origin_pos = one_data['origin_pos']
                x.append(origin_pos['x'])
                y.append(origin_pos['y'])
            one_data_new = {
                    "x": y,
                    "y": x
                    }
            one_frame_new['data'] = one_data_new
            data_frame_new.append(one_frame_new)
        self.set_data_frame_at_datakey('', 'lat_search_sd_ego_point', data_frame_new)
    
    def build_lc_lat_search_sd_ref_data_frame(self):
        data_frame_lc_lat_search_debug = self.get_data_frame_at_datakey('lc_lat_search_debug')

        data_frame_new = []
        for one_frame in data_frame_lc_lat_search_debug:
            one_frame_new = {}
            one_frame_new['t'] = one_frame['t']
            one_frame_new['index'] = one_frame['index']

            one_data = one_frame['data']
            x = []
            y = []
            if 'ref_points' in one_data:
                ref_points = one_data['ref_points']
                for pt in ref_points:
                    x.append(pt['x'])
                    y.append(pt['y'])
            one_data_new = {
                    "x": y,
                    "y": x
                    }
            one_frame_new['data'] = one_data_new
            data_frame_new.append(one_frame_new)
        self.set_data_frame_at_datakey('', 'lat_search_sd_ref_point', data_frame_new)

    def build_lc_lon_search_st_obs_polygon_data_frame(self):
        data_frame_new = []
        data_frame = self.get_data_frame_at_datakey('lc_lon_search_debug')
        for one_frame in data_frame:
            one_frame_new = {}
            one_frame_new['t'] = one_frame['t']
            one_frame_new['index'] = one_frame['index']

            one_data = one_frame['data']
            xs = []
            ys = []
            obs_id = []
            if 'st_obstacle' in one_data:
                st_obstacle = one_data['st_obstacle']
                for obs in st_obstacle:
                    x = []
                    y = []
                    for pt in obs['polygon']:
                        x.append(pt['x'])
                        y.append(pt['y'])
                    xs.append(x)
                    ys.append(y)

                    obs_id.append(obs['id'])
            one_data_new = {
                    "xs": xs,
                    "ys": ys,
                    "obs_id": obs_id,
                    }
            one_frame_new['data'] = one_data_new
            data_frame_new.append(one_frame_new)
        self.set_data_frame_at_datakey('', 'lc_lon_search_st_obs_polygon', data_frame_new)

    def build_lc_lon_search_st_res_data_frame(self):
        data_frame_new = []
        data_frame = self.get_data_frame_at_datakey('lc_lon_search_debug')
        for one_frame in data_frame:
            one_frame_new = {}
            one_frame_new['t'] = one_frame['t']
            one_frame_new['index'] = one_frame['index']

            one_data = one_frame['data']
            x = []
            y = []
            if 'st_path' in one_data:
                st_path = one_data['st_path']
                for pt in st_path:
                    x.append(pt['t'])
                    y.append(pt['s'])
            one_data_new = {
                    "x": x,
                    "y": y
                    }
            one_frame_new['data'] = one_data_new
            data_frame_new.append(one_frame_new)
        self.set_data_frame_at_datakey('', 'lc_lon_search_st_res', data_frame_new)

    def build_lc_lon_search_st_sample_point_data_frame(self):
        data_frame_new = []
        data_frame = self.get_data_frame_at_datakey('lc_lon_search_debug')
        for one_frame in data_frame:
            one_frame_new = {}
            one_frame_new['t'] = one_frame['t']
            one_frame_new['index'] = one_frame['index']

            one_data = one_frame['data']
            x = []
            y = []
            a = []
            v = []
            cost = []
            smooth_cost = []
            obs_cost = []
            vel_guidance_cost = []
            color = []
            is_cost_max = []
            if 'sample_points' in one_data:
                sample_points = one_data['sample_points']
                for pt in sample_points:
                    if is_lon_search_cost_max(pt['smooth_cost']):
                        continue
                    x.append(pt['point']['t'])
                    y.append(pt['point']['s'])
                    v.append(pt['point']['v'])
                    a.append(pt['point']['a'])
                    cost.append(clip_lc_lon_search_cost(pt['cost']))
                    is_cost_max.append(is_lon_search_cost_max(pt['cost']))
                    smooth_cost.append(clip_lc_lon_search_cost(pt['smooth_cost']))
                    obs_cost.append(clip_lc_lon_search_cost(pt['obstacle_cost']))
                    vel_guidance_cost.append(clip_lc_lon_search_cost(pt['vel_guidance_cost']))
                if len(cost) > 0:
                    minima = 0.0
                    maxima = 1e5
                    tmp_pts = [c for i, c in enumerate(cost) if not is_cost_max[i]]
                    if len(tmp_pts) > 0:
                        minima = min(tmp_pts)
                        maxima = max(tmp_pts)
                    norm = matplotlib.colors.Normalize(vmin=minima, vmax=maxima, clip=True)
                    mapper = cm.ScalarMappable(norm=norm, cmap=cm.get_cmap("Wistia", 256))
                    for i, c in enumerate(cost):
                        if is_cost_max[i]:
                            color.append(
                                matplotlib.colors.rgb2hex(mapper.to_rgba(maxima))
                            )
                        else:
                            color.append(
                                matplotlib.colors.rgb2hex(mapper.to_rgba(c))
                            )
            one_data_new = {
                    "x": x,
                    "y": y,
                    "v": v,
                    "a": a,
                    "cost": cost,
                    "smooth_cost": smooth_cost,
                    "obs_cost": obs_cost,
                    "vel_guidance_cost": vel_guidance_cost,
                    'color': color,
                    }
            one_frame_new['data'] = one_data_new
            data_frame_new.append(one_frame_new)
        self.set_data_frame_at_datakey('', 'lc_lon_search_st_sample_point', data_frame_new)
    
    def build_lc_lon_search_vt_res_data_frame(self):
        data_frame_new = []
        data_frame = self.get_data_frame_at_datakey('lc_lon_search_debug')
        for one_frame in data_frame:
            one_frame_new = {}
            one_frame_new['t'] = one_frame['t']
            one_frame_new['index'] = one_frame['index']

            one_data = one_frame['data']
            x = []
            y = []
            if 'st_path' in one_data:
                st_path = one_data['st_path']
                for pt in st_path:
                    x.append(pt['t'])
                    y.append(pt['v'])
            one_data_new = {
                    "x": x,
                    "y": y
                    }
            one_frame_new['data'] = one_data_new
            data_frame_new.append(one_frame_new)
        self.set_data_frame_at_datakey('', 'lc_lon_search_vt_res', data_frame_new)
    
    def build_lc_lon_search_vt_limit_data_frame(self):
        data_frame_new = []
        data_frame = self.get_data_frame_at_datakey('lc_lon_search_debug')
        for one_frame in data_frame:
            one_frame_new = {}
            one_frame_new['t'] = one_frame['t']
            one_frame_new['index'] = one_frame['index']

            one_data = one_frame['data']
            x = []
            y = []
            if 'vt_limit' in one_data:
                vt_limit = one_data['vt_limit']
                for pt in vt_limit:
                    x.append(pt['x'])
                    y.append(pt['y'])
            one_data_new = {
                    "x": x,
                    "y": y
                    }
            one_frame_new['data'] = one_data_new
            data_frame_new.append(one_frame_new)
        self.set_data_frame_at_datakey('', 'lc_lon_search_vt_limit', data_frame_new)

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

    def show2html(self):
        output_file(self.output_)
        show(column(self.time_slider_,
                    row(column(self.figs_['main_fig'],
                                ), 
                        column(self.figs_['nop_count'],
                                self.figs_['traj_d'],
                                self.figs_['traj_theta'],
                                self.figs_['traj_theta_error_ref'],
                                self.figs_['traj_delta'],
                                self.figs_['lc_lat_search'],
                               ),
                        column(
                                self.figs_['traj_update'],
                                self.figs_['longi_optimization_s'],
                                self.figs_['longi_optimization_vel'],
                                self.figs_['longi_optimization_acc'],
                                self.figs_['longi_optimization_jerk'],
                                self.figs_['lc_lon_search_st'],
                                self.figs_['lc_lon_search_vt'],
                        )
                    )
                    )
            )
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

    xpilot_plot = XvizPlotLc(args.source, args.output)

    xpilot_plot.add_required_channel(XDEBUG_CHANNEL)
    xpilot_plot.add_required_channel(PLANNING_CHANNEL)
    xpilot_plot.read_msg()

    xpilot_plot.figure_plot()