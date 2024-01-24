import os
import sys
from bokeh.palettes import Magma, Viridis
sys.path.append(os.getcwd() + '/py_lib')
from cilqr_param import *
sys.path.append('../../new_xviz')
from common.figure_viz import * 
from common.layer_viz import * 
from common.plot_util import * 


def plot_iter_stat_list(motion):
    iter_stat_list = motion.get_iter_stat_list()

    # build data frame
    count = 0
    count_list = []
    data_frame_omega = []
    data_frame_acc = []
    data_frame_pos = []
    for iter_stat in iter_stat_list:
        count_list.append(count)

        one_frame_omega = {}
        one_frame_omega['t'] = count
        one_frame_omega['index'] = count
        one_frame_omega_data = []
        for pt in iter_stat.trajectory:
            one_frame_omega_data.append(pt.omega * 180.0 / math.pi)
        one_frame_omega['data'] = one_frame_omega_data
        data_frame_omega.append(one_frame_omega)

        one_frame_acc = {}
        one_frame_acc['t'] = count
        one_frame_acc['index'] = count
        one_frame_acc_data = []
        for pt in iter_stat.trajectory:
            one_frame_acc_data.append(pt.acceleration)
        one_frame_acc['data'] = one_frame_acc_data
        data_frame_acc.append(one_frame_acc)

        one_frame_pos = {}
        one_frame_pos['t'] = count
        one_frame_pos['index'] = count
        temp_x = []
        temp_y = []
        for pt in iter_stat.trajectory:
            temp_x.append(pt.position.x)
            temp_y.append(pt.position.y)
        one_frame_pos['data'] = {'x': temp_x, 'y': temp_y}
        data_frame_pos.append(one_frame_pos)

        count = count + 1

    # omega fig
    omega_figure_viz = FigureViz('Omega', 'Index', 'Val / degree/s', y_range=[-15, 15], 
                                         width=800 ,height = 150, x_range=[-1, 31])
    PlotUtils.add_layer_to_figure_with_dataframe(omega_figure_viz, data_frame_omega, 
                                                    args=dict(plot_type='index_line_circle',
                                                                color='red',
                                                                label='omega'))
    omega_figure = omega_figure_viz.plot()
    callback_omega_figure = omega_figure_viz.get_callback_list()

    # acc fig
    acc_figure_viz = FigureViz('Acc', 'Index', 'Val', y_range=None, 
                                         width=800 ,height = 150, x_range=[-1, 31])
    PlotUtils.add_layer_to_figure_with_dataframe(acc_figure_viz, data_frame_acc, 
                                                    args=dict(plot_type='index_line_circle',
                                                                color='red',
                                                                label='acc'))
    acc_figure = acc_figure_viz.plot()
    callback_acc_figure = acc_figure_viz.get_callback_list()

    # pos fig
    pos_figure_viz = FigureViz('Pos', 'X / m', 'Y / m', y_range=None, 
                                         width=900 ,height = 400, match_aspect=True)
    PlotUtils.add_layer_to_figure_with_dataframe(pos_figure_viz, data_frame_pos, 
                                                    args=dict(plot_type='scatter',
                                                                label='pos',
                                                                line_width=2,
                                                                size=8,
                                                                color=Magma[6][3]))
    pos_figure = pos_figure_viz.plot()
    callback_pos_figure = pos_figure_viz.get_callback_list()

    # lanes
    env = motion.get_env()
    lanes = env.get_all_center_points()
    a = 0
    for lane_pts in lanes:
        pts_tmp_x = []
        pts_tmp_y = []
        for pt in lane_pts:
            pts_tmp_x.append(pt.x)
            pts_tmp_y.append(pt.y)

        pos_figure.scatter(pts_tmp_x, pts_tmp_y, line_width=2, color=Magma[6][a], legend_label="lane")
        a = a +1

    # 汇合所有的callback
    all_callback = [*callback_omega_figure, *callback_pos_figure, *callback_acc_figure]
    figs = [omega_figure, pos_figure, acc_figure]

    # 创建滑块
    time_slider = Slider(start=0.0, end=max(count_list), 
                                value=0.0, step=1.0, title="Frame")
    time_slider.js_on_change('value', *all_callback)
    
    PlotUtils.activate_figure_option(figs)
    output_notebook()
    show(column(time_slider,
                pos_figure,
                omega_figure,
                acc_figure
                )
        )