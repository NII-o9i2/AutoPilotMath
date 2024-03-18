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

class XvizPlotLatLonDecider(XvizPlotBase):
    def __init__(self, bag_path, output):
        super().__init__(bag_path, output)
        
    def figure_plot(self):
        # 定制化data_frame
        self.build_obs_first_polygon_data_frame()
        self.build_obs_traj_point_data_frame()
        self.build_obs_traj_polygon_data_frame()
        self.build_obs_traj_ellipse_data_frame()
        self.build_ego_polygon_data_frame()
        self.build_lane_data_frame()
        self.build_lat_lon_decider_success_traj_point_data_frame()
        self.build_lat_lon_decider_failed_traj_point_data_frame()
        self.build_lat_lon_decider_failed_ego_circle_point_data_frame()
        self.build_lat_lon_decider_success_traj_acc_data_frame()
        self.build_lat_lon_decider_success_traj_omega_data_frame()
        self.build_lat_lon_decider_success_traj_vel_data_frame()
        self.build_lat_lon_decider_success_traj_theta_data_frame()
        # self.build_ego_lane_center_dist_to_left_data_frame()
        # self.build_ego_lane_center_dist_to_right_data_frame()

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
        self.add_layer_to_figure(main_figure_viz, args=dict(data_key='obs_traj_polygon', \
                                                                plot_type='multi_polygon', color='blue',  \
                                                                label='obs_traj_polygon', fill_alpha=0.1, \
                                                                line_alpha=0.1, line_width=1))
        self.add_layer_to_figure(main_figure_viz, args=dict(data_key='obs_traj_ellipse', \
                                                                plot_type='multi_ellipse', color='purple',  \
                                                                label='obs_traj_ellipse', fill_alpha=0.1, \
                                                                line_alpha=0.1, line_width=1))
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
        self.add_layer_to_figure(main_figure_viz, args=dict(data_key='lat_lon_decider_success_traj_point', \
                                                                plot_type='scatter', color='greenyellow',  \
                                                                label='lat_lon_decider_success_traj_point', \
                                                                line_alpha=0.8, line_width=6))
        self.add_layer_to_figure(main_figure_viz, args=dict(data_key='lat_lon_decider_failed_traj_point', \
                                                                plot_type='scatter', color='crimson',  \
                                                                label='lat_lon_decider_failed_traj_point', \
                                                                line_alpha=0.5, line_width=3))
        self.add_layer_to_figure(main_figure_viz, args=dict(data_key='lat_lon_decider_failed_ego_circle_point', \
                                                                plot_type='scatter', color='mediumslateblue',  \
                                                                label='lat_lon_decider_failed_ego_circle_point', \
                                                                line_alpha=0.5, line_width=3))                                                   
        self.figs_['main_fig'] = main_figure_viz.plot()
        callback_main_fig = main_figure_viz.get_callback_list()        

        # nop count相关
        nop_count_figure_viz = FigureViz('Nop_Counter', 'Index', 'Data', y_range=None, 
                                         width=600 ,height = 150, match_aspect = False)
        self.add_layer_to_figure(nop_count_figure_viz, args=dict(data_key='nop_counter', \
                                                                plot_type='single_point'))
        self.figs_['nop_count'] = nop_count_figure_viz.plot()
        callback_nop_count = nop_count_figure_viz.get_callback_list()

        # search res traj acc相关
        search_res_acc_figure_viz = FigureViz('Search_Res_Acc', 'Index', 'Data/m/s^2', x_range = [-1,30],y_range = [-6, 3], 
                                         width=600 ,height = 200, match_aspect = False)
        self.add_layer_to_figure(search_res_acc_figure_viz, args=dict(data_key='lat_lon_decider_success_traj_acc', \
                                                                plot_type='index_line_circle', color='blue',  \
                                                                label='traj_acc'))
        self.figs_['search_res_acc'] = search_res_acc_figure_viz.plot()
        callback_acc = search_res_acc_figure_viz.get_callback_list()

        # search res traj vel相关
        search_res_vel_figure_viz = FigureViz('Search_Res_Vel', 'Index', 'Data/m/s', x_range = [-1,30],y_range = [-1, 25], 
                                         width=600 ,height = 200, match_aspect = False)
        self.add_layer_to_figure(search_res_vel_figure_viz, args=dict(data_key='lat_lon_decider_success_traj_vel', \
                                                                plot_type='index_line_circle', color='blue',  \
                                                                label='traj_vel'))
        self.figs_['search_res_vel'] = search_res_vel_figure_viz.plot()
        callback_vel = search_res_vel_figure_viz.get_callback_list()

        # search res traj omega相关
        search_res_omega_figure_viz = FigureViz('Search_Res_Omega', 'Index', 'Data/radian/s', x_range = [-1,30],y_range = [-0.5, 0.5], 
                                         width=600 ,height = 200, match_aspect = False)
        self.add_layer_to_figure(search_res_omega_figure_viz, args=dict(data_key='lat_lon_decider_success_traj_omega', \
                                                                plot_type='index_line_circle', color='blue',  \
                                                                label='traj_omega'))
        self.figs_['search_res_omega'] = search_res_omega_figure_viz.plot()
        callback_omega = search_res_omega_figure_viz.get_callback_list()

        # search res traj theta相关
        search_res_theta_figure_viz = FigureViz('Search_Res_Theta', 'Index', 'Data/radian', x_range = [-1,30],y_range = [-3.14, 3.14], 
                                         width=600 ,height = 200, match_aspect = False)
        self.add_layer_to_figure(search_res_theta_figure_viz, args=dict(data_key='lat_lon_decider_success_traj_theta', \
                                                                plot_type='index_line_circle', color='blue',  \
                                                                label='traj_theta'))
        self.figs_['search_res_theta'] = search_res_theta_figure_viz.plot()
        callback_theta = search_res_theta_figure_viz.get_callback_list()

        # 汇合所有的callback
        all_callback = [*callback_main_fig, *callback_nop_count, *callback_acc, 
                        *callback_omega, *callback_vel, *callback_theta]

        # 创建滑块        
        self.add_slider()
        self.time_slider_.js_on_change('value', *all_callback)
        
        self.activate_figure_option()
        self.show2html()
        return
    
    def build_obs_first_polygon_data_frame(self):
        data_frame_new = []
        data_frame = self.get_data_frame_at_datakey('lat_lon_decider_debug')
        for one_frame in data_frame:
            one_frame_new = {}
            one_frame_new['t'] = one_frame['t']
            one_frame_new['index'] = one_frame['index']
            
            xs = []
            ys = []
            one_data = one_frame['data']['obstacle_set']
            for one_obs in one_data:
                if len(one_obs['trajectory']) == 0:
                    continue
                pt = one_obs['trajectory'][0]
                polygon_x, polygon_y =  \
                                self.build_polygon_by_pos_and_heading(pt['position']['x'], \
                                                                    pt['position']['y'], \
                                                                    pt['theta'], one_obs['width'],
                                                                    one_obs['length'])
                xs.append(polygon_x)
                ys.append(polygon_y)

            one_data_new = {
                    "xs": xs,
                    "ys": ys
                    }
            one_frame_new['data'] = one_data_new
            data_frame_new.append(one_frame_new)
        self.set_data_frame_at_datakey('polygon_channel', 'obs_first_polygon', data_frame_new)
    
    def build_obs_traj_point_data_frame(self):
        data_frame_new = []
        data_frame = self.get_data_frame_at_datakey('lat_lon_decider_debug')
        
        for one_frame in data_frame:
            one_frame_new = {}
            one_frame_new['t'] = one_frame['t']
            one_frame_new['index'] = one_frame['index']


            x = []
            y = []
            one_data = one_frame['data']['obstacle_set']
            for one_obs in one_data:
                if len(one_obs['trajectory']) == 0:
                    continue
                for point in one_obs['trajectory']:
                    x.append(point['position']['x'])
                    y.append(point['position']['y'])
            one_data_new = {
                    "x": x,
                    "y": y
                    }
            one_frame_new['data'] = one_data_new
            data_frame_new.append(one_frame_new)
        self.set_data_frame_at_datakey('traj_channel', 'obs_traj_point', data_frame_new)
    
    def build_obs_traj_polygon_data_frame(self):
        data_frame_new = []
        data_frame = self.get_data_frame_at_datakey('lat_lon_decider_debug')
        for one_frame in data_frame:
            one_frame_new = {}
            one_frame_new['t'] = one_frame['t']
            one_frame_new['index'] = one_frame['index']
            
            xs = []
            ys = []
            one_data = one_frame['data']['obstacle_set']
            for one_obs in one_data:
                if len(one_obs['trajectory']) == 0:
                    continue
                for point in one_obs['trajectory']:
                    polygon_x, polygon_y =  \
                                    self.build_polygon_by_pos_and_heading(point['position']['x'], \
                                                                        point['position']['y'], \
                                                                        point['theta'], one_obs['width'],
                                                                        one_obs['length'])
                    xs.append(polygon_x)
                    ys.append(polygon_y)

            one_data_new = {
                    "xs": xs,
                    "ys": ys
                    }
            one_frame_new['data'] = one_data_new
            data_frame_new.append(one_frame_new)
        self.set_data_frame_at_datakey('polygon_channel', 'obs_traj_polygon', data_frame_new)
    
    def build_obs_traj_ellipse_data_frame(self):
        data_frame_new = []
        data_frame = self.get_data_frame_at_datakey('lat_lon_decider_debug')
        for one_frame in data_frame:
            one_frame_new = {}
            one_frame_new['t'] = one_frame['t']
            one_frame_new['index'] = one_frame['index']
            
            x = []
            y = []
            width = []
            height = []
            angle = []
            one_data = one_frame['data']['obstacle_set']
            for one_obs in one_data:
                if len(one_obs['trajectory']) == 0:
                    continue
                for point in one_obs['trajectory']:
                    x.append(point['position']['x'])
                    y.append(point['position']['y'])
                    width.append(one_obs['ellipse_a']*2)
                    height.append(one_obs['ellipse_b']*2)
                    angle.append(point['theta'])

            one_data_new = {
                    "x": x,
                    "y": y,
                    "width": width,
                    "height": height,
                    "angle": angle
                    }
            one_frame_new['data'] = one_data_new
            data_frame_new.append(one_frame_new)
        self.set_data_frame_at_datakey('polygon_channel', 'obs_traj_ellipse', data_frame_new)

    def build_lat_lon_decider_success_traj_point_data_frame(self):
        data_frame_new = []
        data_frame = self.get_data_frame_at_datakey('lat_lon_decider_debug')
        
        for one_frame in data_frame:
            one_frame_new = {}
            one_frame_new['t'] = one_frame['t']
            one_frame_new['index'] = one_frame['index']


            x = []
            y = []
            success_path = one_frame['data']['success_path']
            for point in success_path:
                x.append(point['position']['x'])
                y.append(point['position']['y'])
            one_data_new = {
                    "x": x,
                    "y": y
                    }
            one_frame_new['data'] = one_data_new
            data_frame_new.append(one_frame_new)
        self.set_data_frame_at_datakey('traj_channel', 'lat_lon_decider_success_traj_point', data_frame_new)
    
    def build_lat_lon_decider_failed_traj_point_data_frame(self):
        data_frame_new = []
        data_frame = self.get_data_frame_at_datakey('lat_lon_decider_debug')
        
        for one_frame in data_frame:
            one_frame_new = {}
            one_frame_new['t'] = one_frame['t']
            one_frame_new['index'] = one_frame['index']


            x = []
            y = []
            failed_path_set = one_frame['data']['failed_path_set']
            for failed_path in failed_path_set:
                for point in failed_path:
                    x.append(point['position']['x'])
                    y.append(point['position']['y'])
            one_data_new = {
                    "x": x,
                    "y": y
                    }
            one_frame_new['data'] = one_data_new
            data_frame_new.append(one_frame_new)
        self.set_data_frame_at_datakey('traj_channel', 'lat_lon_decider_failed_traj_point', data_frame_new)
    
    def build_lat_lon_decider_failed_ego_circle_point_data_frame(self):
        data_frame_new = []
        data_frame = self.get_data_frame_at_datakey('lat_lon_decider_debug')

        ratio = [3, 1, -1, -3]
        l_4 = 0.125 * 4.886
        l_body_center_to_rear_wheel_center_ = 1.405
        
        for one_frame in data_frame:
            one_frame_new = {}
            one_frame_new['t'] = one_frame['t']
            one_frame_new['index'] = one_frame['index']

            x = []
            y = []
            failed_path_set = one_frame['data']['failed_path_set']
            for failed_path in failed_path_set:
                if len(failed_path) == 0:
                    continue
                point = failed_path[-1]
                cos_theta =  math.cos(point['theta'])
                sin_theta =  math.sin(point['theta'])
                body_center_x = point['position']['x'] + l_body_center_to_rear_wheel_center_ * cos_theta
                body_center_y = point['position']['y'] + l_body_center_to_rear_wheel_center_ * sin_theta
                for r in ratio:
                        x.append(body_center_x + r * l_4 * cos_theta)
                        y.append(body_center_y + r * l_4 * sin_theta)
            one_data_new = {
                    "x": x,
                    "y": y
                    }
            one_frame_new['data'] = one_data_new
            data_frame_new.append(one_frame_new)
        self.set_data_frame_at_datakey('traj_channel', 'lat_lon_decider_failed_ego_circle_point', data_frame_new)

    def build_lat_lon_decider_success_traj_acc_data_frame(self):
        data_frame_new = []
        data_frame = self.get_data_frame_at_datakey('lat_lon_decider_debug')
        
        for one_frame in data_frame:
            one_frame_new = {}
            one_frame_new['t'] = one_frame['t']
            one_frame_new['index'] = one_frame['index']

            x = []
            success_path = one_frame['data']['success_path']
            for point in success_path:
                x.append(point['acceleration'])
            one_frame_new['data'] = x
            data_frame_new.append(one_frame_new)
        self.set_data_frame_at_datakey('acc_channel', 'lat_lon_decider_success_traj_acc', data_frame_new)
    
    def build_lat_lon_decider_success_traj_vel_data_frame(self):
        data_frame_new = []
        data_frame = self.get_data_frame_at_datakey('lat_lon_decider_debug')
        
        for one_frame in data_frame:
            one_frame_new = {}
            one_frame_new['t'] = one_frame['t']
            one_frame_new['index'] = one_frame['index']

            x = []
            success_path = one_frame['data']['success_path']
            for point in success_path:
                x.append(point['velocity'])
            one_frame_new['data'] = x
            data_frame_new.append(one_frame_new)
        self.set_data_frame_at_datakey('acc_channel', 'lat_lon_decider_success_traj_vel', data_frame_new)
    
    def build_lat_lon_decider_success_traj_omega_data_frame(self):
        data_frame_new = []
        data_frame = self.get_data_frame_at_datakey('lat_lon_decider_debug')
        
        for one_frame in data_frame:
            one_frame_new = {}
            one_frame_new['t'] = one_frame['t']
            one_frame_new['index'] = one_frame['index']

            x = []
            success_path = one_frame['data']['success_path']
            for point in success_path:
                x.append(point['omega'])
            one_frame_new['data'] = x
            data_frame_new.append(one_frame_new)
        self.set_data_frame_at_datakey('omega_channel', 'lat_lon_decider_success_traj_omega', data_frame_new)
    
    def build_lat_lon_decider_success_traj_theta_data_frame(self):
        data_frame_new = []
        data_frame = self.get_data_frame_at_datakey('lat_lon_decider_debug')
        
        for one_frame in data_frame:
            one_frame_new = {}
            one_frame_new['t'] = one_frame['t']
            one_frame_new['index'] = one_frame['index']

            x = []
            success_path = one_frame['data']['success_path']
            for point in success_path:
                x.append(point['theta'])
            one_frame_new['data'] = x
            data_frame_new.append(one_frame_new)
        self.set_data_frame_at_datakey('acc_channel', 'lat_lon_decider_success_traj_theta', data_frame_new)

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
    
    def show2html(self):
        output_file(self.output_)
        show(column(self.time_slider_,
                    row(column(self.figs_['main_fig'],
                                # self.figs_['lane_center_dist_to_boundary']
                                ), 
                        column(self.figs_['nop_count'],
                               self.figs_['search_res_acc'],
                               self.figs_['search_res_vel'],
                               self.figs_['search_res_omega'],
                               self.figs_['search_res_theta']))))
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

    xpilot_plot = XvizPlotLatLonDecider(args.source, args.output)

    xpilot_plot.add_required_channel(XDEBUG_CHANNEL)
    xpilot_plot.add_required_channel(PLANNING_CHANNEL)
    xpilot_plot.read_msg()

    xpilot_plot.figure_plot()