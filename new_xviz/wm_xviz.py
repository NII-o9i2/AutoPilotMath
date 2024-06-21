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

class XvizPlotWM(XvizPlotBase):
    def __init__(self, bag_path, output):
        super().__init__(bag_path, output)
        
    def figure_plot(self):
        # 定制化data_frame
        self.build_obstacle_polygon_data_frame()
        self.build_obstacle_traj_data_frame()
        self.build_ego_polygon_data_frame()
        self.build_lane_data_frame()

        # 主图, 包含 obs_polygon, obs_traj, ego_polygon, lane
        main_figure_viz = FigureViz('Main_Figure', 'X', 'Y', width=1000, height =900)
        self.add_layer_to_figure(main_figure_viz, args=dict(data_key='obstacle_polygon', \
                                                                plot_type='multi_polygon', color='blue',  \
                                                                label='obstacle_polygon', fill_alpha=0, \
                                                                line_alpha=0.4, line_width=4))
        self.add_layer_to_figure(main_figure_viz, args=dict(data_key='obstacle_traj', \
                                                                plot_type='scatter', color='black',  \
                                                                label='obstacle_traj', \
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
        self.add_layer_to_figure(main_figure_viz, args=dict(data_key='last_ego_raw_lane_center_point', \
                                                                plot_type='line_scatter', color='limegreen',  \
                                                                label='last_ego_raw_lane_center_point', \
                                                                line_alpha=0.4, line_width=4))    
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
        # self.add_layer_to_figure(main_figure_viz, args=dict(data_key='raw_road_edge_seq', \
        #                                                     plot_type='scatter', color='black',  \
        #                                                     label='raw_road_edge', \
        #                                                     line_alpha=1, line_width=4))
        # self.add_layer_to_figure(main_figure_viz, args=dict(data_key='raw_convex_hull_seq', \
        #                                                     plot_type='multi_polygon', color='black',  \
        #                                                     label='raw_convex_hull', \
        #                                                     line_alpha=0.4, line_width=4))

        self.figs_['main_fig'] = main_figure_viz.plot()
        callback_main_fig = main_figure_viz.get_callback_list()        

        # nop count相关
        nop_count_figure_viz = FigureViz('Nop_Counter', 'Index', 'Data', y_range=None, 
                                         width=400 ,height = 400)
        self.add_layer_to_figure(nop_count_figure_viz, args=dict(data_key='nop_counter', \
                                                                plot_type='single_point', \
                                                                    line_alpha=1, line_width=1))
        self.figs_['nop_count'] = nop_count_figure_viz.plot()
        callback_nop_count = nop_count_figure_viz.get_callback_list()

        # lane center dist to bound相关
        lane_center_dist_to_boundary_viz = FigureViz('Nop_Counter', 'Dist', 'Index', \
                                                    width=400 ,height = 400)
        self.add_layer_to_figure(lane_center_dist_to_boundary_viz, args=dict(data_key='ego_lane_center_dist_to_left', \
                                                                plot_type='line_scatter', color='Green',  \
                                                                label='ego_lane_center_dist_to_left', \
                                                                line_alpha=0.2, line_width=4))
        self.add_layer_to_figure(lane_center_dist_to_boundary_viz, args=dict(data_key='ego_lane_center_dist_to_right', \
                                                                plot_type='line_scatter', color='coral',  \
                                                                label='ego_lane_center_dist_to_right', \
                                                                line_alpha=0.2, line_width=4))
        self.figs_['lane_center_dist_to_boundary'] = lane_center_dist_to_boundary_viz.plot()
        callback_lane_center_dist_to_boundary = lane_center_dist_to_boundary_viz.get_callback_list()
        
        # ego to diff refline min dis相关
        car_dis_figure_viz = FigureViz('Car_To_RefLine_Min_Dis', 'Index', 'Data', y_range=[-2, 8], 
                                         width=800 ,height = 500)
        self.add_layer_to_figure(car_dis_figure_viz, args=dict(data_key='car_to_ego_lane_refline_min_dis', \
                                                                plot_type='single_point', \
                                                                color='Green',  \
                                                                label='car_to_ego_lane_min_dis', \
                                                                line_alpha=0.5, line_width=4))
        self.add_layer_to_figure(car_dis_figure_viz, args=dict(data_key='car_to_left_lane_refline_min_dis', \
                                                                plot_type='single_point', \
                                                                color='red',  \
                                                                label='car_to_left_lane_min_dis', \
                                                                line_alpha=0.5, line_width=4))
        self.add_layer_to_figure(car_dis_figure_viz, args=dict(data_key='car_to_right_lane_refline_min_dis', \
                                                                plot_type='single_point', \
                                                                color='yellow',  \
                                                                label='car_to_right_lane_min_dis', \
                                                                line_alpha=0.5, line_width=4))
        self.figs_['car_min_dis_to_refline'] = car_dis_figure_viz.plot()
        callback_car_min_dis = car_dis_figure_viz.get_callback_list()

        # 汇合所有的callback
        all_callback = [*callback_main_fig, *callback_nop_count, 
                        *callback_lane_center_dist_to_boundary, *callback_car_min_dis]

        # 创建滑块        
        self.add_slider()
        self.time_slider_.js_on_change('value', *all_callback)
        
        self.activate_figure_option()
        self.show2html()
        return
    
    def build_obstacle_polygon_data_frame(self):
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
        self.set_data_frame_at_datakey('polygon_channel', 'obstacle_polygon', data_frame_new)

    def build_obstacle_traj_data_frame(self):
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
        self.set_data_frame_at_datakey('traj_channel', 'obstacle_traj', data_frame_new)
    
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

    def build_polygon_by_pos_and_heading(self, geometry_center_x, geometry_center_y, yaw):
        x = []
        y = []

        ego_width = 2.2
        ego_length = 5.0
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
        self.build_ego_lane_center_dist_to_left_data_frame()
        self.build_ego_lane_center_dist_to_right_data_frame()
        self.build_car_to_ego_lane_refline_min_dis_data_frame()
        self.build_car_to_left_lane_refline_min_dis_data_frame()
        self.build_car_to_right_lane_refline_min_dis_data_frame()
    
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
        self.generate_last_ego_raw_lane_center_point(data_frame_new)
        self.set_data_frame_at_datakey('point_channel', 'ego_raw_lane_center_point', data_frame_new)
        
    def generate_last_ego_raw_lane_center_point(self, data_frame_new):
        last_ego_raw_lane_center_point = [{'data': data_frame_new[0]['data'], 
                                        'index': data_frame_new[0]['index'], 
                                        't': data_frame_new[0]['t']}]
        for i in range(1, len(data_frame_new)):
            frame = {
                'data': data_frame_new[i - 1]['data'],
                'index': data_frame_new[i]['index'],
                't': data_frame_new[i]['t']
            }
            last_ego_raw_lane_center_point.append(frame)
        # print(last_ego_raw_lane_center_point)
        self.set_data_frame_at_datakey('point_channel', 
                                       'last_ego_raw_lane_center_point', last_ego_raw_lane_center_point)   
    
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

    def build_car_to_ego_lane_refline_min_dis_data_frame(self):
        data_frame = self.get_data_frame_at_datakey('lane_debug')
        data_frame_new = []
        for one_frame in data_frame:
            one_frame_new = {}
            one_frame_new['t'] = one_frame['t']
            one_frame_new['index'] = one_frame['index']

            one_data = one_frame['data']
            ego_to_lane_min_dis_list = []
            for lane_debug in one_data:
                if lane_debug['id'] == 0:
                    ego_to_lane_min_dis_list.append(lane_debug['ego_to_lane_min_dis'])
                    # pprint(ego_to_lane_min_dis_list)
            one_frame_new['data'] = ego_to_lane_min_dis_list
            data_frame_new.append(one_frame_new)
        self.set_data_frame_at_datakey('point_channel', 'car_to_ego_lane_refline_min_dis', data_frame_new)
        
    def build_car_to_left_lane_refline_min_dis_data_frame(self):
        data_frame = self.get_data_frame_at_datakey('lane_debug')
        data_frame_new = []
        for one_frame in data_frame:
            one_frame_new = {}
            one_frame_new['t'] = one_frame['t']
            one_frame_new['index'] = one_frame['index']

            one_data = one_frame['data']
            ego_to_lane_min_dis_list = []
            for lane_debug in one_data:
                if lane_debug['id'] == -1:
                    ego_to_lane_min_dis_list.append(lane_debug['ego_to_lane_min_dis'])
                    # pprint(ego_to_lane_min_dis_list)
            one_frame_new['data'] = ego_to_lane_min_dis_list
            data_frame_new.append(one_frame_new)
        self.set_data_frame_at_datakey('point_channel', 'car_to_left_lane_refline_min_dis', data_frame_new) 
        
    def build_car_to_right_lane_refline_min_dis_data_frame(self):
        data_frame = self.get_data_frame_at_datakey('lane_debug')
        data_frame_new = []
        for one_frame in data_frame:
            one_frame_new = {}
            one_frame_new['t'] = one_frame['t']
            one_frame_new['index'] = one_frame['index']

            one_data = one_frame['data']
            ego_to_lane_min_dis_list = []
            for lane_debug in one_data:
                if lane_debug['id'] == 1:
                    ego_to_lane_min_dis_list.append(lane_debug['ego_to_lane_min_dis'])
                    # pprint(ego_to_lane_min_dis_list)
            one_frame_new['data'] = ego_to_lane_min_dis_list
            data_frame_new.append(one_frame_new)
        self.set_data_frame_at_datakey('point_channel', 'car_to_right_lane_refline_min_dis', data_frame_new)      
            
    def show2html(self):
        output_file(self.output_)
        
        row1 = row(self.figs_['nop_count'], self.figs_['lane_center_dist_to_boundary'])
        col1 = column(self.figs_['car_min_dis_to_refline'], row1)
        row2 = row(self.figs_['main_fig'], col1)
        col2 = column(self.time_slider_, row2)
        
        show(col2)
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

    xpilot_plot = XvizPlotWM(args.source, args.output)

    xpilot_plot.add_required_channel(XDEBUG_CHANNEL)
    xpilot_plot.add_required_channel(PLANNING_CHANNEL)
    xpilot_plot.read_msg()

    xpilot_plot.figure_plot()