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

class XvizPlotLat(XvizPlotBase):
    def __init__(self, bag_path, output):
        super().__init__(bag_path, output)
        
    def figure_plot(self):
        self.build_traj_s_theta_data_frame()
        # traj index-d index-d_ref
        traj_d = FigureViz('traj_d', 'index', 'd/m', width=800, height =800,y_range =[-4, 4])
        self.add_layer_to_figure(traj_d, args=dict(data_key='traj_ref_d', \
                                                                plot_type='index_circle', color='blue',  \
                                                                label='traj_ref_d', \
                                                                line_alpha=0.2, line_width=4))
        self.add_layer_to_figure(traj_d, args=dict(data_key='traj_d', \
                                                                plot_type='index_circle', color='red',  \
                                                                label='traj_d', \
                                                                line_alpha=0.2, line_width=4))
        self.figs_['traj_d'] = traj_d.plot()
        callback_traj_d = traj_d.get_callback_list()    
        
        # traj index-theta_error_ref
        traj_theta_error_ref = FigureViz('traj_theta_error_ref', 'index', 'theta/rad', width=800, height =800)
        self.add_layer_to_figure(traj_theta_error_ref, args=dict(data_key='traj_theta_error_ref', \
                                                                plot_type='index_circle', color='blue',  \
                                                                label='traj_theta_error_ref', \
                                                                line_alpha=0.2, line_width=4))
        self.figs_['traj_theta_error_ref'] = traj_theta_error_ref.plot()
        callback_traj_theta_error_ref = traj_theta_error_ref.get_callback_list()  
        
        # traj index-delta_ref index-delta_out
        traj_delta = FigureViz('traj_delta', 'index', 'delta/rad', width=800, height =800,y_range = [-0.1,0.1])
        self.add_layer_to_figure(traj_delta, args=dict(data_key='traj_delta_ref', \
                                                                plot_type='index_circle', color='blue',  \
                                                                label='traj_delta_ref', \
                                                                line_alpha=0.2, line_width=4))
        self.add_layer_to_figure(traj_delta, args=dict(data_key='traj_delta', \
                                                                plot_type='index_circle', color='red',  \
                                                                label='traj_delta', \
                                                                line_alpha=0.2, line_width=4))
        self.figs_['traj_delta'] = traj_delta.plot()
        callback_traj_delta = traj_delta.get_callback_list()  
        
        # traj index-theta_out
        traj_theta_data = self.get_data_frame_at_datakey('traj_theta')
        theta_list=[]
        for tmp_data in traj_theta_data:
            theta_list.append(tmp_data['data'])
        if theta_list:
            max_val = max(theta_list)
            min_val = min(theta_list)
        else:
            max_val = None
            min_val = None
        traj_theta_out = FigureViz('traj_theta_out', 's/m', 'theta/rad', width=800, height =800,
                                   y_range = None)
        self.add_layer_to_figure(traj_theta_out, args=dict(data_key='traj_s_theta', \
                                                                plot_type='scatter', color='blue',  \
                                                                label='traj_theta', \
                                                                line_alpha=0.2, line_width=4))
        self.figs_['traj_theta_out'] = traj_theta_out.plot()
        callback_traj_theta_out = traj_theta_out.get_callback_list()       

        # nop count相关
        nop_count_figure_viz = FigureViz('Nop_Counter', 'Index', 'Data', y_range=None, 
                                         width=800, height =800)
        self.add_layer_to_figure(nop_count_figure_viz, args=dict(data_key='nop_counter', \
                                                                plot_type='single_point', \
                                                                line_alpha=1, line_width=1))
        self.figs_['nop_count'] = nop_count_figure_viz.plot()
        callback_nop_count = nop_count_figure_viz.get_callback_list()

        # 汇合所有的callback
        all_callback = [*callback_traj_d, *callback_traj_theta_error_ref,*callback_traj_delta,*callback_traj_theta_out,*callback_nop_count]

        # 创建滑块        
        self.add_slider()
        self.time_slider_.js_on_change('value', *all_callback)
        
        self.activate_figure_option()
        self.show2html()
        return
    
    def build_traj_s_theta_data_frame(self):
        data_frame_traj_s = self.get_data_frame_at_datakey('traj_s')
        data_frame_traj_theta = self.get_data_frame_at_datakey('traj_theta')
        data_frame_traj_ref_d = self.get_data_frame_at_datakey('traj_ref_d')
        data_frame_new = []
        for one_frame_s,one_frame_theta in zip(data_frame_traj_s,data_frame_traj_theta):
            one_frame_new = {}
            one_frame_new['t'] = one_frame_s['t']
            one_frame_new['index'] = one_frame_s['index']

            one_data_s = one_frame_s['data']
            one_data_theta = one_frame_theta['data']
            x = []
            y = []
            for s,d_ref in zip(one_data_s,one_data_theta):
                x.append(s)
                y.append(d_ref)
            one_data_new = {
                    "x": x,
                    "y": y
                    }
            one_frame_new['data'] = one_data_new
            data_frame_new.append(one_frame_new)
        self.set_data_frame_at_datakey('traj_channel', 'traj_s_theta', data_frame_new)

    def show2html(self):
        output_file(self.output_) 
        row1 = row(column(self.figs_['traj_theta_out'], 
                          self.figs_['traj_d']),
                   column(self.figs_['traj_delta'],
                          self.figs_['nop_count']),
                   column(self.figs_['traj_theta_error_ref']))
        layout = column(self.time_slider_, row1)
        
        show(layout)
        
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

    xpilot_plot = XvizPlotLat(args.source, args.output)

    xpilot_plot.add_required_channel(XDEBUG_CHANNEL)
    xpilot_plot.add_required_channel(PLANNING_CHANNEL)
    xpilot_plot.read_msg()

    xpilot_plot.figure_plot()