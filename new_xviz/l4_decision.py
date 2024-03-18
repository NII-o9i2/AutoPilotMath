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
        # self.build_freespace_data_frame()
        self.build_current_lane_center_point_data_frame()
        self.build_target_lane_center_point_data_frame()
        self.build_nav_point_data_frame()
        self.build_obstacle_traj_data_frame()
        self.build_obstacle_polygon_data_frame()
        self.build_stop_point_data_frame()
        self.build_ego_polygon_data_frame()
        self.build_vru_stop_point_data_frame()
        self.build_ego_traj_point_data_frame()

        # 主图
        main_figure_viz = FigureViz('Main_Figure', 'X', 'Y', width=1000, height =900)
        self.add_layer_to_figure(main_figure_viz, args=dict(data_key='ego_lane_center_point', \
                                                                plot_type='scatter', color='red',  \
                                                                label='ego_lane_center_point', \
                                                                line_alpha=0.4, line_width=3))
        
        self.add_layer_to_figure(main_figure_viz, args=dict(data_key='target_lane_center_point', \
                                                                plot_type='scatter', color='green',  \
                                                                label='target_lane_center_point', \
                                                                line_alpha=0.4, line_width=3))
        
        self.add_layer_to_figure(main_figure_viz, args=dict(data_key='nav_point', \
                                                                plot_type='scatter', color='blue',  \
                                                                label='nav_point', \
                                                                line_alpha=0.4, line_width=10))
        
        self.add_layer_to_figure(main_figure_viz, args=dict(data_key='obstacle_traj', \
                                                                plot_type='scatterheading', color='black',  \
                                                                label='obstacle_traj', \
                                                                line_alpha=0.4, line_width=3))
        
        self.add_layer_to_figure(main_figure_viz, args=dict(data_key='obstacle_polygon', \
                                                                plot_type='multi_polygon_id', color='blue',  \
                                                                label='obstacle_polygon', fill_alpha=0, \
                                                                line_alpha=0.4, line_width=4))

        self.add_layer_to_figure(main_figure_viz, args=dict(data_key='stop_point', \
                                                                plot_type='scatter', color='orange',  \
                                                                label='stop_point', \
                                                                line_alpha=0.4, line_width=10))

        self.add_layer_to_figure(main_figure_viz, args=dict(data_key='ego_polygon', \
                                                        plot_type='multi_polygon', color='red',  \
                                                        label='ego_polygon', fill_alpha=0, \
                                                        line_alpha=0.4, line_width=4))

        self.add_layer_to_figure(main_figure_viz, args=dict(data_key='vru_stop_point', \
                                                                plot_type='scatter', color='red',  \
                                                                label='vru_stop_point', \
                                                                line_alpha=0.4, line_width=10))
        
        self.add_layer_to_figure(main_figure_viz, args=dict(data_key='ego_traj_predict', \
                                                        plot_type='scatter', color='purple',  \
                                                        label='ego_pred_traj', \
                                                        line_alpha=0.4, line_width=3))
        
        self.figs_['main_fig'] = main_figure_viz.plot()
        callback_main_fig = main_figure_viz.get_callback_list()        

        # nop count相关
        nop_count_figure_viz = FigureViz('Nop_Counter', 'Index', 'Data', y_range=None, 
                                         width=400 ,height = 400)
        self.add_layer_to_figure(nop_count_figure_viz, args=dict(data_key='dm_counter', \
                                                                 plot_type='single_point', \
                                                                 label='dm_count'   ))
        self.figs_['dm_count'] = nop_count_figure_viz.plot()
        callback_nop_count = nop_count_figure_viz.get_callback_list()


        # 汇合所有的callback
        all_callback = [*callback_main_fig, *callback_nop_count]

        # 创建滑块        
        self.add_slider()
        self.time_slider_.js_on_change('value', *all_callback)
        
        self.activate_figure_option()
        self.show2html()
        return
        
    def build_freespace_data_frame(self):
        data_frame_new = []
        data_frame = self.get_data_frame_at_datakey('freespace_list')
        for one_frame in data_frame:
            one_frame_new = {}
            one_frame_new['t'] = one_frame['t']
            one_frame_new['index'] = one_frame['index']

            one_data = one_frame['data']
            x = []
            y = []
            for one_freespace in one_data:
                for point in one_freespace:
                    x.append(point['x'])
                    y.append(point['y'])
            one_data_new = {
                    "x": x,
                    "y": y
                    }
            one_frame_new['data'] = one_data_new
            data_frame_new.append(one_frame_new)
        self.set_data_frame_at_datakey('freespace_channel', 'freespace', data_frame_new)
        
    def build_target_lane_center_point_data_frame(self):
        data_frame = self.get_data_frame_at_datakey('targetReflineEnu')
        data_frame_new = []
        for one_frame in data_frame:
            one_frame_new = {}
            one_frame_new['t'] = one_frame['t']
            one_frame_new['index'] = one_frame['index']

            one_data = one_frame['data']
            x = []
            y = []
            if (len(one_data)>0):
                for point in one_data:
                    x.append(point['x'])
                    y.append(point['y'])
            one_data_new = {
                    "x": x,
                    "y": y
                    }
            one_frame_new['data'] = one_data_new
            data_frame_new.append(one_frame_new)
        self.set_data_frame_at_datakey('target_lane_channel', 'target_lane_center_point', data_frame_new)
        

    def build_current_lane_center_point_data_frame(self):
        data_frame = self.get_data_frame_at_datakey('currentReflineEnu')
        data_frame_new = []
        for one_frame in data_frame:
            one_frame_new = {}
            one_frame_new['t'] = one_frame['t']
            one_frame_new['index'] = one_frame['index']

            one_data = one_frame['data']
            x = []
            y = []
            if (len(one_data)>1):
                for point in one_data:
                    x.append(point['x'])
                    y.append(point['y'])
            one_data_new = {
                    "x": x,
                    "y": y
                    }
            one_frame_new['data'] = one_data_new
            data_frame_new.append(one_frame_new)
        self.set_data_frame_at_datakey('current_lane_channel', 'ego_lane_center_point', data_frame_new)
        
    def build_nav_point_data_frame(self):
        data_frame = self.get_data_frame_at_datakey('positionEnu')
        data_frame_new = []
        for one_frame in data_frame:
            one_frame_new = {}
            one_frame_new['t'] = one_frame['t']
            one_frame_new['index'] = one_frame['index']

            one_data = one_frame['data']
            x = []
            y = []

            x.append(one_data['x'])
            y.append(one_data['y'])
            one_data_new = {
                "x": x,
                "y": y
            }
            one_frame_new['data'] = one_data_new
            data_frame_new.append(one_frame_new)
        self.set_data_frame_at_datakey(
            'point_channel', 'nav_point', data_frame_new)
      
    def build_stop_point_data_frame(self):
        data_frame = self.get_data_frame_at_datakey('stoppoint')
        data_frame_new = []
        for one_frame in data_frame:
            one_frame_new = {}
            one_frame_new['t'] = one_frame['t']
            one_frame_new['index'] = one_frame['index']

            one_data = one_frame['data']
            x = []
            y = []

            x.append(one_data['x'])
            y.append(one_data['y'])
            one_data_new = {
                "x": x,
                "y": y
            }
            one_frame_new['data'] = one_data_new
            data_frame_new.append(one_frame_new)
        self.set_data_frame_at_datakey(
            'point_channel', 'stop_point', data_frame_new)
        
    def build_vru_stop_point_data_frame(self):
        data_frame = self.get_data_frame_at_datakey('vrustoppoint')
        data_frame_new = []
        for one_frame in data_frame:
            one_frame_new = {}
            one_frame_new['t'] = one_frame['t']
            one_frame_new['index'] = one_frame['index']

            one_data = one_frame['data']
            x = []
            y = []

            x.append(one_data['x'])
            y.append(one_data['y'])
            one_data_new = {
                "x": x,
                "y": y
            }
            one_frame_new['data'] = one_data_new
            data_frame_new.append(one_frame_new)
        self.set_data_frame_at_datakey(
            'point_channel', 'vru_stop_point', data_frame_new)
      
    def build_obstacle_polygon_data_frame(self):
        data_frame_new = []
        data_frame = self.get_data_frame_at_datakey('prediction_traj_polygon')
        # pprint(data_frame)
        for one_frame in data_frame:
            one_frame_new = {}
            one_frame_new['t'] = one_frame['t']
            one_frame_new['index'] = one_frame['index']

            one_data = one_frame['data']
            # pprint(one_data)
            xs = []
            ys = []
            id = []
            obs_type = []
            for one_obstacle_polygon in one_data:
                x = []
                y = []
                i_d = []
                obstype = []
                for point in one_obstacle_polygon:
                    x.append(point['x'])
                    y.append(point['y'])
                    i_d.append(point['id'])
                    obstype.append(point['obs_type'])
                xs.append(x)
                ys.append(y)
                id.append(i_d)
                obs_type.append(obstype)
                
            one_data_new = {
                    "xs": xs,
                    "ys": ys,
                    "id": id,
                    "obs_type": obs_type
                    }

            one_frame_new['data'] = one_data_new
            data_frame_new.append(one_frame_new)
            # pprint(data_frame_new)
        self.set_data_frame_at_datakey('polygon_channel', 'obstacle_polygon', data_frame_new)
      
    def build_obstacle_traj_data_frame(self):
        data_frame_new = []
        data_frame = self.get_data_frame_at_datakey('prediction_traj_traj')
        # pprint(data_frame)
        for one_frame in data_frame:
            one_frame_new = {}
            one_frame_new['t'] = one_frame['t']
            one_frame_new['index'] = one_frame['index']

            one_data = one_frame['data']
            x = []
            y = []
            heading = []
            for one_obstacle_traj in one_data:
                for point in one_obstacle_traj:
                    x.append(point['x'])
                    y.append(point['y'])
                    heading.append(point['heading'])
            one_data_new = {
                    "x": x,
                    "y": y,
                    "heading": heading,
                    }
            one_frame_new['data'] = one_data_new
            data_frame_new.append(one_frame_new)
        self.set_data_frame_at_datakey('traj_channel', 'obstacle_traj', data_frame_new)


    def build_ego_polygon_data_frame(self):
        data_frame_new = []
        position_data_frame = self.get_data_frame_at_datakey('positionEnu')
        yaw_data_frame = self.get_data_frame_at_datakey('veh_yaw')
        yaw_data_index = {frame['t']: frame['data'] for frame in yaw_data_frame}

        for one_frame in position_data_frame:
            one_frame_new = {}
            one_frame_new['t'] = one_frame['t']
            one_frame_new['index'] = one_frame['index']

            one_data = one_frame['data']
            if one_frame['t'] in yaw_data_index:
                veh_yaw = yaw_data_index[one_frame['t']]
            else:
                continue

            xs = []
            ys = []
            ego_polygon_x, ego_polygon_y = self.build_polygon_by_pos_and_heading(one_data['x'], 
                                                                                 one_data['y'], 
                                                                                 veh_yaw)
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
        
    def build_ego_traj_point_data_frame(self):
        data_frame = self.get_data_frame_at_datakey('ego_traj')
        data_frame_new = []
        for one_frame in data_frame:
            one_frame_new = {}
            one_frame_new['t'] = one_frame['t']
            one_frame_new['index'] = one_frame['index']

            one_data = one_frame['data']
            x = []
            y = []
            if (len(one_data)>0):
                for point in one_data:
                    x.append(point['x'])
                    y.append(point['y'])
            one_data_new = {
                    "x": x,
                    "y": y
                    }
            one_frame_new['data'] = one_data_new
            data_frame_new.append(one_frame_new)
        # pprint(data_frame_new)
        self.set_data_frame_at_datakey('ego_traj_channel', 'ego_traj_predict', data_frame_new)

    def show2html(self):
        output_file(self.output_)
        show(column(self.time_slider_,
                    row(self.figs_['main_fig'], self.figs_['dm_count'])))
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

    # xpilot_plot.add_required_channel(XDEBUG_CHANNEL)
    # xpilot_plot.add_required_channel(PLANNING_CHANNEL)
    # xpilot_plot.add_required_channel(PERCEPTION_ROAD_GEOMETRY_CHANNEL)
    # xpilot_plot.add_required_channel(SCENEMAPPING_LOCALMAP_CHANNEL)
    xpilot_plot.add_required_channel(L4_DECISION_CHANNEL)
    xpilot_plot.add_required_channel(NAV_CHANNEL)
    xpilot_plot.add_required_channel(PREDICTION_CHANNEL)
    xpilot_plot.add_required_channel(DECISION_DEBUG_CHANNEL)
    xpilot_plot.read_msg()

    xpilot_plot.figure_plot()