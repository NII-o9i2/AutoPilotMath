import bokeh
import math
from bokeh.io import curdoc
from bokeh.plotting import figure, show, output_file , output_notebook
from bokeh.models import WheelZoomTool
from bokeh.layouts import column, row
from bokeh.models import ColumnDataSource, CustomJS, Slider
from bokeh.models import HoverTool
from bokeh.models import LinearColorMapper
from bokeh.colors import HSL
from bokeh.palettes import Magma256
from bokeh.transform import linear_cmap
import matplotlib
import matplotlib.cm as cm
import argparse
import numpy as np
from common.bag_reader import * 
from common.figure_viz import * 
from common.layer_viz import * 
from common.plot_viz import * 
import rsclpy
from bokeh.models import CustomJS, CheckboxButtonGroup
import pandas as pd

class PlotXpilotViz(XvizPlotBase):
    def __init__(self, bag_path, output):
        super().__init__(bag_path, output)
        
    def figure_plot(self):
        # nop count相关
        nop_count_figure_viz = FigureViz('Nop_Counter', 'Index', 'Data', y_range=None, 
                                         width=600 ,height = 200)
        self.add_layer_to_figure(nop_count_figure_viz, args=dict(data_key='nop_counter', \
                                                                plot_type='single_point'))
        self.figs_['nop_count'] = nop_count_figure_viz.plot()
        callback_nop_count = nop_count_figure_viz.get_callback_list()
        
        # 轨迹重规划相关
        traj_update_figure_viz = FigureViz('Traj_Update', 'Index', 'Data', y_range=None, 
                                           width=600 ,height = 200)
        self.add_layer_to_figure(traj_update_figure_viz, args=dict(data_key='updatedByVehicleStatus', \
                                                                plot_type='single_point'))
        self.figs_['traj_update'] = traj_update_figure_viz.plot()
        callback_traj_update = traj_update_figure_viz.get_callback_list()
        
        # 优化s相关
        longi_s_figure_viz = FigureViz('Longi_Opti_S', 'Index', 'Data', x_range = [-5,35], y_range =[-100, 1200],
                                         width=600 ,height = 400)
        self.add_layer_to_figure(longi_s_figure_viz, args=dict(data_key='optimizer_s_out', \
                                                                plot_type='index_line_circle', color='blue',  \
                                                                label='s_out'))
        self.add_layer_to_figure(longi_s_figure_viz, args=dict(data_key='optimizer_s_ref', \
                                                                plot_type='index_line_circle', color='red',  \
                                                                label='s_ref'))
        self.add_layer_to_figure(longi_s_figure_viz, args=dict(data_key='optimizer_s_upper', \
                                                                plot_type='index_circle', color='black',  \
                                                                label='s_upper'))
        self.figs_['longi_optimization_s'] = longi_s_figure_viz.plot()
        callback_s = longi_s_figure_viz.get_callback_list()
        
        # 优化vel相关
        longi_vel_figure_viz = FigureViz('Longi_Opti_Vel', 'Index', 'Data', x_range = [-5,35], y_range =[-5, 30],
                                         width=600 ,height = 400)
        self.add_layer_to_figure(longi_vel_figure_viz, args=dict(data_key='optimizer_v_out', \
                                                                plot_type='index_line_circle', color='blue',  \
                                                                label='vel_out'))
        self.add_layer_to_figure(longi_vel_figure_viz, args=dict(data_key='optimizer_v_ref', \
                                                                plot_type='index_line_circle', color='red',  \
                                                                label='vel_ref'))
        self.add_layer_to_figure(longi_vel_figure_viz, args=dict(data_key='optimizer_v_upper', \
                                                                plot_type='index_circle', color='black',  \
                                                                label='vel_upper'))
        self.figs_['longi_optimization_vel'] = longi_vel_figure_viz.plot()
        callback_vel = longi_vel_figure_viz.get_callback_list()
        
        # 优化acc相关
        longi_acc_figure_viz = FigureViz('Longi_Opti_Acc', 'Index', 'Data', x_range = [-5,35],y_range = [-6, 3], 
                                         width=600 ,height = 400)
        self.add_layer_to_figure(longi_acc_figure_viz, args=dict(data_key='optimizer_a_out', \
                                                                plot_type='index_line_circle', color='blue',  \
                                                                label='acc_out'))
        self.add_layer_to_figure(longi_acc_figure_viz, args=dict(data_key='optimizer_a_ref', \
                                                                plot_type='index_line_circle', color='red',  \
                                                                label='acc_ref'))
        self.add_layer_to_figure(longi_acc_figure_viz, args=dict(data_key='optimizer_a_upper', \
                                                                plot_type='index_circle', color='black',  \
                                                                label='acc_upper'))
        self.add_layer_to_figure(longi_acc_figure_viz, args=dict(data_key='optimizer_a_lower', \
                                                                plot_type='index_line_circle', color='black',  \
                                                                label='acc_lower'))
        self.figs_['longi_optimization_acc'] = longi_acc_figure_viz.plot()
        callback_acc = longi_acc_figure_viz.get_callback_list()

        # 优化jerk相关
        longi_jerk_figure_viz = FigureViz('Longi_Opti_Jerk', 'Index', 'Data', x_range = [-5,35],y_range = [-5, 5],
                                         width=600 ,height = 400)
        self.add_layer_to_figure(longi_jerk_figure_viz, args=dict(data_key='optimizer_jerk_out', \
                                                                plot_type='index_line_circle', color='blue',  \
                                                                label='vel_out'))
        self.add_layer_to_figure(longi_jerk_figure_viz, args=dict(data_key='optimizer_jerk_lower', \
                                                                plot_type='index_line_circle', color='black',  \
                                                                label='vel_ref'))
        self.add_layer_to_figure(longi_jerk_figure_viz, args=dict(data_key='optimizer_jerk_upper', \
                                                                plot_type='index_circle', color='black',  \
                                                                label='vel_upper'))
        self.figs_['longi_optimization_jerk'] = longi_jerk_figure_viz.plot()
        callback_jerk = longi_jerk_figure_viz.get_callback_list()


        # 汇合所有的callback
        all_callback = [*callback_nop_count, *callback_traj_update,
                        *callback_s, *callback_vel,
                        *callback_acc, *callback_jerk]

        # 创建滑块        
        self.add_slider()
        self.time_slider_.js_on_change('value', *all_callback)
        
        self.activate_figure_option()
        self.show2html()
        return
    
    def show2html(self):
        output_file(self.output_)
        show(column(self.time_slider_,
                    row(self.figs_['nop_count'],
                        self.figs_['traj_update']),
                    row(self.figs_['longi_optimization_s'],
                        self.figs_['longi_optimization_vel']),
                    row(self.figs_['longi_optimization_acc'],
                        self.figs_['longi_optimization_jerk'])))
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

    xpilot_plot = PlotXpilotViz(args.source, args.output)

    xpilot_plot.add_required_channel(XDEBUG_CHANNEL)
    xpilot_plot.add_required_channel(PLANNING_CHANNEL)
    xpilot_plot.read_msg()

    xpilot_plot.figure_plot()