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
        self.build_ego_lane_center_point_data_frame()
        self.build_left_lane_center_point_data_frame()
        self.build_right_lane_center_point_data_frame()

        # 主图
        main_figure_viz = FigureViz('Main_Figure', 'X', 'Y', width=1500, height =800)
        self.add_layer_to_figure(main_figure_viz, args=dict(data_key='left_lane_center_point', \
                                                                plot_type='scatter', color='red',  \
                                                                label='left_lane_center_point', \
                                                                line_alpha=0.4, line_width=3))
        
        self.add_layer_to_figure(main_figure_viz, args=dict(data_key='ego_lane_center_point', \
                                                                plot_type='scatter', color='green',  \
                                                                label='ego_lane_center_point', \
                                                                line_alpha=0.4, line_width=3))
        
        self.add_layer_to_figure(main_figure_viz, args=dict(data_key='right_lane_center_point', \
                                                                plot_type='scatter', color='blue',  \
                                                                label='right_lane_center_point', \
                                                                line_alpha=0.4, line_width=3))

        self.figs_['main_fig'] = main_figure_viz.plot()
        callback_main_fig = main_figure_viz.get_callback_list()        

        # 汇合所有的callback
        all_callback = [*callback_main_fig]

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
        
    def build_left_lane_center_point_data_frame(self):
        data_frame = self.get_data_frame_at_datakey('lane_list')
        data_frame_new = []
        for one_frame in data_frame:
            one_frame_new = {}
            one_frame_new['t'] = one_frame['t']
            one_frame_new['index'] = one_frame['index']

            one_data = one_frame['data']
            x = []
            y = []
            if (len(one_data)>0):
                for point in one_data[0]['centerPoints']:
                    x.append(point['x'])
                    y.append(point['y'])
            one_data_new = {
                    "x": x,
                    "y": y
                    }
            one_frame_new['data'] = one_data_new
            data_frame_new.append(one_frame_new)
        self.set_data_frame_at_datakey('point_channel', 'left_lane_center_point', data_frame_new)
        

    def build_ego_lane_center_point_data_frame(self):
        data_frame = self.get_data_frame_at_datakey('lane_list')
        data_frame_new = []
        for one_frame in data_frame:
            one_frame_new = {}
            one_frame_new['t'] = one_frame['t']
            one_frame_new['index'] = one_frame['index']

            one_data = one_frame['data']
            x = []
            y = []
            if (len(one_data)>1):
                for point in one_data[1]['centerPoints']:
                    x.append(point['x'])
                    y.append(point['y'])
            one_data_new = {
                    "x": x,
                    "y": y
                    }
            one_frame_new['data'] = one_data_new
            data_frame_new.append(one_frame_new)
        self.set_data_frame_at_datakey('point_channel', 'ego_lane_center_point', data_frame_new)
        
    def build_right_lane_center_point_data_frame(self):
        data_frame = self.get_data_frame_at_datakey('lane_list')
        data_frame_new = []
        for one_frame in data_frame:
            one_frame_new = {}
            one_frame_new['t'] = one_frame['t']
            one_frame_new['index'] = one_frame['index']

            one_data = one_frame['data']
            x = []
            y = []
            if (len(one_data)>2):
                for point in one_data[2]['centerPoints']:
                    x.append(point['x'])
                    y.append(point['y'])
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
                    row(self.figs_['main_fig'])))
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
    xpilot_plot.add_required_channel(SCENEMAPPING_LOCALMAP_CHANNEL)
    xpilot_plot.read_msg()

    xpilot_plot.figure_plot()