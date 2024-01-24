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
import rsclpy
from bokeh.models import CustomJS, CheckboxButtonGroup
from old_framework_baseplot import PlotBase

class PlotXpilotViz(PlotBase):
    def __init__(self, bag_path,output):
        super().__init__(bag_path,output)

    def creat_figure(self):
        # main hmi window
        # self.figs_['fig1'] = figure(width=800 ,height = 800,x_range = [-5,100],y_range = [-7.5,7.5] ,match_aspect=True, title = 'Xviz')
        self.figs_['fig1'] = figure(width=800, height = 800, match_aspect=True, title = 'Xviz')
        self.figs_['lane_center_dist_to_boundary'] = figure(width=600 ,height = 300)
        self.figs_['svlimit'] = figure(width=600 ,height = 300,y_range = [0,40])
        self.figs_['traj_theta'] = figure(width=600 ,height = 300,y_range = [-1.5,1.5])
        self.figs_['svlimit_vt'] = figure(width=600 ,height = 300,y_range = [0,40])
        self.figs_['osqp_s_t'] = figure(width=600 ,height = 300,x_axis_label='t/s', y_axis_label='s/m')
        self.figs_['osqp_v_t'] = figure(width=600 ,height = 300,x_axis_label='t/s', y_axis_label='m/s',y_range = [0,40])
        self.figs_['osqp_a_t'] = figure(width=600 ,height = 300,x_axis_label='t/s', y_axis_label='m/s^2')
        self.figs_['osqp_jerk_t'] = figure(width=600 ,height = 300,x_axis_label='t/s', y_axis_label='m/s^3')
        self.figs_['nop_counter'] = figure(width=600 ,height = 300)

        for fig_name in ['fig1', 'lane_center_dist_to_boundary','svlimit','svlimit_vt','traj_theta','osqp_s_t','osqp_v_t','osqp_a_t','osqp_jerk_t','nop_counter']:
            fig = self.figs_[fig_name]
            checkbox_group = CheckboxButtonGroup(labels=[fig_name], active=[0], width=600)
            callback = CustomJS(args=dict(fig=fig, checkbox_group=checkbox_group), code="""
                fig.visible = checkbox_group.active.includes(0);
            """)
            checkbox_group.js_on_change('active', callback)
            self.checkbox_groups_[fig_name] = checkbox_group  # Add the checkbox group to the dictionary
    
    # plot real-time dmpp refline points
    def add_layer_processed_map_scatter(self, fig, rt_source_name_, rt_name_, time_list_, 
                                       source_x_, source_y_, legend_, size_ = 3, line_alpha_ = 0.4, color_ = 'blue'):
        self.plot_data_set_[rt_source_name_] = dict(
            time = time_list_,
            source_x = source_x_,
            source_y = source_y_,
        )
        self.plot_data_set_[rt_name_] = ColumnDataSource(data=dict(
            x = [],
            y = [],
        ))

        fig.scatter('x','y',source=self.plot_data_set_[rt_name_], 
                          color = color_ ,line_alpha=line_alpha_,legend_label = legend_, size=size_)
        return
    
    def add_layer_processed_map_line(self, fig, rt_source_name_, rt_name_, time_list_, 
                                    source_x_, source_y_, legend_, size_ = 3, line_alpha_ = 0.4, color_ = 'blue'):
        self.plot_data_set_[rt_source_name_] = dict(
            time = time_list_,
            source_x = source_x_,
            source_y = source_y_,
        )
        self.plot_data_set_[rt_name_] = ColumnDataSource(data=dict(
            x = [],
            y = [],
        ))

        # print(rt_source_name_)
        # print(rt_name_)

        fig.line('x','y',source=self.plot_data_set_[rt_name_], 
                       color = color_ ,line_alpha=line_alpha_,legend_label = legend_, line_width=size_)
        return
    
    def add_layer_predict_traj_scatter(self, time_list_, source_x_,source_y_,line_width_ = 3,line_alpha_ = 0.4, color_ = 'black'):
        self.plot_data_set_['predict_trajectory_rt_source'] = dict(
            time = time_list_,
            source_x = source_x_,
            source_y = source_y_,
        )
        self.plot_data_set_['predict_trajectory_rt'] = ColumnDataSource(data=dict(
            x = [],
            y = [],
        ))

        self.figs_['fig1'].scatter('x','y',source=self.plot_data_set_['predict_trajectory_rt'], 
                                   color = color_ ,line_alpha=line_alpha_,legend_label = 'Predict trajectory Point')
        return 

    def add_layer_planning_traj_scatter(self, time_list_, source_x_,source_y_,line_width_ = 3,line_alpha_ = 0.4, color_ = 'green'):
        self.plot_data_set_['planning_trajectory_rt_source'] = dict(
            time = time_list_,
            source_x = source_x_,
            source_y = source_y_,
        )
        self.plot_data_set_['planning_trajectory_rt'] = ColumnDataSource(data=dict(
            x = [],
            y = [],
        ))

        self.figs_['fig1'].scatter('x','y',source=self.plot_data_set_['planning_trajectory_rt'], 
                                   color = color_ ,line_alpha=line_alpha_,legend_label = 'Planning trajectory Point')
        return 
    
    def add_layer_ego_obj_point(self,time_list_,x_list_,y_list_):
        self.plot_data_set_['ego_obj_rt_source'] = dict(
            time = time_list_,
            source_x = x_list_,
            source_y = y_list_,
        )
        self.plot_data_set_['ego_obj_rt'] = ColumnDataSource(data=dict(
            x = [],
            y = [],
        ))
        self.figs_['fig1'].scatter('x','y',source=self.plot_data_set_['ego_obj_rt'], line_width = 10,
                                   color = 'red', line_alpha=0.4, legend_label = 'Ego Geometry Center')
        return
    
    def add_layer_polygons(self, fig, rt_source_name_, rt_name_, time_list_, 
                                    source_polygon_xs_, source_polygon_ys_, legend_, size_ = 3, line_alpha_ = 0.4, color_ = 'blue'):
        self.plot_data_set_[rt_source_name_] = dict(
            time = time_list_,
            source_polygon_xs = source_polygon_xs_,
            source_polygon_ys = source_polygon_ys_,
        )
        self.plot_data_set_[rt_name_] = ColumnDataSource(data=dict(
            xs = [[]],
            ys = [[]],
        ))
        
        # the points to be plotted
        # xs = [[1, 2, 3], [3, 5, 3, 5]]
        # ys = [[1, 2, 1], [2, 2, 4, 5]]
            
        fig.patches('xs','ys',source=self.plot_data_set_[rt_name_], 
                       fill_color = color_, fill_alpha=0, line_alpha=line_alpha_,legend_label = legend_, line_width=size_)

        return
    
    def add_layer_scatter(self, time_list_, x_values_,y_values_, y_key, source_key, figure_key, 
                               legend_label, xaxis_lable,yaxis_lable,color_= 'green', line_width_=3, line_alpha_=0.4):
        self.plot_data_set_[source_key] = dict(
            time=time_list_,
            source_x=x_values_,
            source_y=y_values_,
        )
        self.plot_data_set_[y_key] = ColumnDataSource(data=dict(
            x=[],
            y=[],
        ))
        self.figs_[figure_key].scatter('x', 'y', source=self.plot_data_set_[y_key],
                                       color=color_, line_alpha=line_alpha_, legend_label=legend_label,)
        self.figs_[figure_key].xaxis.axis_label = xaxis_lable
        self.figs_[figure_key].yaxis.axis_label = yaxis_lable
        return

    def add_layer_vtlimit_scatter(self, time_list_, x_values_,y_values_, y_key, source_key, figure_key, 
                               legend_label, color_= 'green', line_width_=3, line_alpha_=0.4):
        self.plot_data_set_[source_key] = dict(
            time=time_list_,
            source_x=x_values_,
            source_y=y_values_,
        )
        self.plot_data_set_[y_key] = ColumnDataSource(data=dict(
            x=[],
            y=[],
        ))
        self.figs_[figure_key].scatter('x', 'y', source=self.plot_data_set_[y_key],
                                       color=color_, line_alpha=line_alpha_, legend_label=legend_label,)
        self.figs_[figure_key].xaxis.axis_label = "t/s"
        self.figs_[figure_key].yaxis.axis_label = "v/s"
        return
    # 横坐标是index,纵坐标是value
    def add_traj_layer_scatter(self, time_list_, y_values_, y_key, source_key, figure_key, 
                               legend_label, line_width_=3, line_alpha_=0.4, color_='black'):
        self.plot_data_set_[source_key] = dict(
            time=time_list_,
            y=y_values_,
        )
        self.plot_data_set_[y_key] = ColumnDataSource(data=dict(
            index=[],
            y=[],
        ))
        self.figs_[figure_key].scatter('index', 'y', source=self.plot_data_set_[y_key], 
                                       color=color_, line_alpha=line_alpha_, legend_label=legend_label)
        self.figs_[figure_key].xaxis.axis_label = "index"
        return
    
    def add_time_scatter(self, time_list_, y_values_, y_key, source_key, figure_key, 
                         legend_label, line_width_=3, line_alpha_=0.1, color_='grey'):
        self.plot_data_set_[source_key] = ColumnDataSource(data=dict(
            time=time_list_,
            y=y_values_
        ))
        self.plot_data_set_[y_key] = ColumnDataSource(data=dict(
            time=[],
            y=[],
        ))
        self.figs_[figure_key].scatter('time', 'y', source=self.plot_data_set_[source_key],
                                        color=color_,size=5,
                                        line_alpha=line_alpha_, legend_label=legend_label)
        self.figs_[figure_key].scatter('time', 'y', source=self.plot_data_set_[y_key],
                                        color='red', size=10)
        self.figs_[figure_key].xaxis.axis_label = "time"      
        return

    def add_rt_callback(self):
        self.callback_ =CustomJS(args=dict(time = self.time_slider_,

                                           cur_lane_center_line_rt = self.plot_data_set_['cur_lane_center_line_rt'],
                                           source_cur_lane_center_line_rt = self.plot_data_set_['cur_lane_center_line_rt_source'],
                                           cur_lane_left_line_rt = self.plot_data_set_['cur_lane_left_line_rt'],
                                           source_cur_lane_left_line_rt = self.plot_data_set_['cur_lane_left_line_rt_source'],
                                           cur_lane_right_line_rt = self.plot_data_set_['cur_lane_right_line_rt'],
                                           source_cur_lane_right_line_rt = self.plot_data_set_['cur_lane_right_line_rt_source'],
                                           left_lane_center_line_rt = self.plot_data_set_['left_lane_center_line_rt'],
                                           source_left_lane_center_line_rt = self.plot_data_set_['left_lane_center_line_rt_source'],
                                           right_lane_center_line_rt = self.plot_data_set_['right_lane_center_line_rt'],
                                           source_right_lane_center_line_rt = self.plot_data_set_['right_lane_center_line_rt_source'],
                                           cur_lane_center_dist_to_left_bound_rt = self.plot_data_set_['cur_lane_center_dist_to_left_bound_rt'],
                                           source_cur_lane_center_dist_to_left_bound_rt = self.plot_data_set_['cur_lane_center_dist_to_left_bound_rt_source'],
                                           cur_lane_center_dist_to_right_bound_rt = self.plot_data_set_['cur_lane_center_dist_to_right_bound_rt'],
                                           source_cur_lane_center_dist_to_right_bound_rt = self.plot_data_set_['cur_lane_center_dist_to_right_bound_rt_source'],
                                           source_cur_raw_lane_center_rt = self.plot_data_set_['cur_raw_lane_center_rt_source'],
                                           cur_raw_lane_center_rt = self.plot_data_set_['cur_raw_lane_center_rt'],


                                        #    cur_lane_center_line_rt_enu = self.plot_data_set_['cur_lane_center_line_rt_enu'],
                                        #    source_cur_lane_center_line_rt_enu = self.plot_data_set_['cur_lane_center_line_rt_source_enu'],
                                           
                                           predict_traj_rt = self.plot_data_set_['predict_trajectory_rt'],
                                           source_predict_traj = self.plot_data_set_['predict_trajectory_rt_source'],

                                           planning_traj_rt = self.plot_data_set_['planning_trajectory_rt'],
                                           source_planning_traj = self.plot_data_set_['planning_trajectory_rt_source'],
                                           
                                           ego_obj_rt = self.plot_data_set_['ego_obj_rt'],
                                           source_ego_obj = self.plot_data_set_['ego_obj_rt_source'],

                                           obj_polygons_rt = self.plot_data_set_['obj_polygons_rt'],
                                           source_predict_obj_polygons = self.plot_data_set_['obj_polygons_rt_source'],
                                           sum_vel_upper = self.plot_data_set_['sum_vel_upper'],
                                           sum_vel_upper_source = self.plot_data_set_['sum_vel_upper_rt_source'],
                                           sum_vel_lower = self.plot_data_set_['sum_vel_lower'],
                                           sum_vel_lower_source = self.plot_data_set_['sum_vel_lower_rt_source'],
                                           traj_theta = self.plot_data_set_['traj_theta'],
                                           traj_theta_source = self.plot_data_set_['traj_theta_rt_source'],
                                           sum_vel_ref = self.plot_data_set_['sum_vel_ref'],
                                           sum_vel_ref_source = self.plot_data_set_['sum_vel_ref_rt_source'],
                                        #    svlimit_ref_st_s = self.plot_data_set_['svlimit_ref_st_s'],
                                        #    svlimit_ref_st_s_source = self.plot_data_set_['svlimit_ref_st_s_rt_source'],
                                           svlimit_ref_vt_v = self.plot_data_set_['svlimit_ref_vt_v'],
                                           svlimit_ref_vt_v_source = self.plot_data_set_['svlimit_ref_vt_v_rt_source'],
                                           svlimit_ref_at_a = self.plot_data_set_['svlimit_ref_at_a'],
                                           svlimit_ref_at_a_source = self.plot_data_set_['svlimit_ref_at_a_rt_source'],
                                           oprimizer_vel_rt = self.plot_data_set_['optimizer_vel_rt'],
                                           oprimizer_vel_source = self.plot_data_set_['optimizer_vel_rt_source'],
                                           oprimizer_vel_ref_rt = self.plot_data_set_['optimizer_vel_ref_rt'],
                                           oprimizer_vel_ref_source = self.plot_data_set_['optimizer_vel_ref_rt_source'], 
                                           oprimizer_vel_upper_rt = self.plot_data_set_['optimizer_vel_upper_rt'],
                                           oprimizer_vel_upper_source = self.plot_data_set_['optimizer_vel_upper_rt_source'],
                                           oprimizer_acc_rt = self.plot_data_set_['optimizer_acc_rt'],
                                           oprimizer_acc_source = self.plot_data_set_['optimizer_acc_rt_source'],
                                           oprimizer_acc_ref_rt = self.plot_data_set_['optimizer_acc_ref_rt'],
                                           oprimizer_acc_ref_source = self.plot_data_set_['optimizer_acc_ref_rt_source'],
                                           oprimizer_acc_upper_rt = self.plot_data_set_['optimizer_acc_upper_rt'],
                                           oprimizer_acc_upper_source = self.plot_data_set_['optimizer_acc_upper_rt_source'],
                                           oprimizer_acc_lower_rt = self.plot_data_set_['optimizer_acc_lower_rt'],
                                           oprimizer_acc_lower_source = self.plot_data_set_['optimizer_acc_lower_rt_source'],
                                           oprimizer_jerk_rt = self.plot_data_set_['optimizer_jerk_rt'],
                                           oprimizer_jerk_source = self.plot_data_set_['optimizer_jerk_rt_source'],
                                           oprimizer_jerk_upper_rt = self.plot_data_set_['optimizer_jerk_upper_rt'],
                                           oprimizer_jerk_upper_source = self.plot_data_set_['optimizer_jerk_upper_rt_source'],
                                           oprimizer_jerk_lower_rt = self.plot_data_set_['optimizer_jerk_lower_rt'],
                                           oprimizer_jerk_lower_source = self.plot_data_set_['optimizer_jerk_lower_rt_source'],  
                                           
                                           nop_counter_source=self.plot_data_set_['nop_counter_source'],
                                           nop_counter_current_value_source=self.plot_data_set_['nop_counter_time'],
                                           ),
                                 code ="""

    const cur_time = time.value
    // console.log("cur_time:", cur_time);

    cur_lane_center_line_rt.data.x.length = 0
    cur_lane_center_line_rt.data.y.length = 0
    cur_lane_left_line_rt.data.x.length = 0
    cur_lane_left_line_rt.data.y.length = 0
    cur_lane_right_line_rt.data.x.length = 0
    cur_lane_right_line_rt.data.y.length = 0
    left_lane_center_line_rt.data.x.length = 0
    left_lane_center_line_rt.data.y.length = 0
    cur_raw_lane_center_rt.data.x.length = 0
    cur_raw_lane_center_rt.data.y.length = 0
    right_lane_center_line_rt.data.x.length = 0
    right_lane_center_line_rt.data.y.length = 0
    const cur_lane_center_line_rt_time_list = source_cur_lane_center_line_rt['time']
    for (let i = 0; i < cur_lane_center_line_rt_time_list.length; i++) {
        if ((cur_time - cur_lane_center_line_rt_time_list[i] < 0.1) && (cur_time - cur_lane_center_line_rt_time_list[i] > 0)){
            cur_lane_center_line_rt.data.x = Array.from(source_cur_lane_center_line_rt.source_x[i])
            cur_lane_center_line_rt.data.y = Array.from(source_cur_lane_center_line_rt.source_y[i])

            cur_lane_left_line_rt.data.x = Array.from(source_cur_lane_left_line_rt.source_x[i])
            cur_lane_left_line_rt.data.y = Array.from(source_cur_lane_left_line_rt.source_y[i])

            cur_lane_right_line_rt.data.x = Array.from(source_cur_lane_right_line_rt.source_x[i])
            cur_lane_right_line_rt.data.y = Array.from(source_cur_lane_right_line_rt.source_y[i])

            cur_raw_lane_center_rt.data.x = Array.from(source_cur_raw_lane_center_rt.source_x[i])
            cur_raw_lane_center_rt.data.y = Array.from(source_cur_raw_lane_center_rt.source_y[i])
            break
       }
    }
    const left_lane_center_line_rt_time_list = source_left_lane_center_line_rt['time']
    for (let i = 0; i < left_lane_center_line_rt_time_list.length; i++) {
        if ((cur_time - left_lane_center_line_rt_time_list[i] < 0.1) && (cur_time - left_lane_center_line_rt_time_list[i] > 0)){
            left_lane_center_line_rt.data.x = Array.from(source_left_lane_center_line_rt.source_x[i])
            left_lane_center_line_rt.data.y = Array.from(source_left_lane_center_line_rt.source_y[i])
            break
       }
    }
    const right_lane_center_line_rt_time_list = source_right_lane_center_line_rt['time']
    for (let i = 0; i < right_lane_center_line_rt_time_list.length; i++) {
        if ((cur_time - right_lane_center_line_rt_time_list[i] < 0.1) && (cur_time - right_lane_center_line_rt_time_list[i] > 0)){
            right_lane_center_line_rt.data.x = Array.from(source_right_lane_center_line_rt.source_x[i])
            right_lane_center_line_rt.data.y = Array.from(source_right_lane_center_line_rt.source_y[i])
            break
       }
    }
    cur_lane_center_line_rt.change.emit()
    cur_lane_left_line_rt.change.emit()
    cur_lane_right_line_rt.change.emit()
    left_lane_center_line_rt.change.emit()
    right_lane_center_line_rt.change.emit()
    cur_raw_lane_center_rt.change.emit()

    cur_lane_center_dist_to_left_bound_rt.data.x.length = 0
    cur_lane_center_dist_to_left_bound_rt.data.y.length = 0
    cur_lane_center_dist_to_right_bound_rt.data.x.length = 0
    cur_lane_center_dist_to_right_bound_rt.data.y.length = 0
    const cur_lane_center_dist_to_bound_rt_time_list = source_cur_lane_center_dist_to_left_bound_rt['time']
    for (let i = 0; i < cur_lane_center_dist_to_bound_rt_time_list.length; i++) {
        if ((cur_time - cur_lane_center_dist_to_bound_rt_time_list[i] < 0.1) && (cur_time - cur_lane_center_dist_to_bound_rt_time_list[i] > 0)){
            cur_lane_center_dist_to_left_bound_rt.data.x = Array.from(source_cur_lane_center_dist_to_left_bound_rt.source_x[i])
            cur_lane_center_dist_to_left_bound_rt.data.y = Array.from(source_cur_lane_center_dist_to_left_bound_rt.source_y[i])
            
            cur_lane_center_dist_to_right_bound_rt.data.x = Array.from(source_cur_lane_center_dist_to_right_bound_rt.source_x[i])
            cur_lane_center_dist_to_right_bound_rt.data.y = Array.from(source_cur_lane_center_dist_to_right_bound_rt.source_y[i])
            
            break
       }
    }
    cur_lane_center_dist_to_left_bound_rt.change.emit()
    cur_lane_center_dist_to_right_bound_rt.change.emit()

    // cur_lane_center_line_rt_enu.data.x.length = 0
    // cur_lane_center_line_rt_enu.data.y.length = 0
    // const cur_lane_center_line_rt_time_list_enu = source_cur_lane_center_line_rt_enu['time']
    // for (let i = 0; i < cur_lane_center_line_rt_time_list_enu.length; i++) {
    //     if ((cur_time - cur_lane_center_line_rt_time_list_enu[i] < 0.1) && (cur_time - cur_lane_center_line_rt_time_list_enu[i] > 0)){
    //         cur_lane_center_line_rt_enu.data.x = Array.from(source_cur_lane_center_line_rt_enu.source_x[i])
    //         cur_lane_center_line_rt_enu.data.y = Array.from(source_cur_lane_center_line_rt_enu.source_y[i])
    // 
    //         break
    //    }
    // }
    // cur_lane_center_line_rt_enu.change.emit()

    predict_traj_rt.data.x.length = 0
    predict_traj_rt.data.y.length = 0
    obj_polygons_rt.data.xs.length = 0
    obj_polygons_rt.data.ys.length = 0
    const prediction_time_list = source_predict_traj['time']
    for (let i = 0; i < prediction_time_list.length; i++) {
        if ((cur_time - prediction_time_list[i] < 0.1) && 
            (cur_time - prediction_time_list[i] > 0)){
            predict_traj_rt.data.x = Array.from(source_predict_traj.source_x[i])
            predict_traj_rt.data.y = Array.from(source_predict_traj.source_y[i])

            const polygon_xs = source_predict_obj_polygons.source_polygon_xs[i]
            for (let i = 0; i < polygon_xs.length; i++) {
                obj_polygons_rt.data.xs.push(Array.from(polygon_xs[i]))
            }
            const polygon_ys = source_predict_obj_polygons.source_polygon_ys[i]
            for (let i = 0; i < polygon_ys.length; i++) {
                obj_polygons_rt.data.ys.push(Array.from(polygon_ys[i]))
            }
            
            break
        }
    }
    predict_traj_rt.change.emit()
    obj_polygons_rt.change.emit()

    const planning_time_list = source_planning_traj['time']
    for (let i = 0; i < planning_time_list.length; i++) {
        if ((cur_time - planning_time_list[i] < 0.1) && 
            (cur_time - planning_time_list[i] > 0)){
            planning_traj_rt.data.x = Array.from(source_planning_traj.source_x[i])
            planning_traj_rt.data.y = Array.from(source_planning_traj.source_y[i])           
            break
        }
    }
    planning_traj_rt.change.emit()

    ego_obj_rt.data.x.length = 0
    ego_obj_rt.data.y.length = 0
    const ego_obj_time_list = source_ego_obj['time']
    for (let i = 0; i < ego_obj_time_list.length; i++) {
        if ((cur_time - ego_obj_time_list[i] < 0.1) && 
            (cur_time - ego_obj_time_list[i] > 0)){
            ego_obj_rt.data.x.push(source_ego_obj.source_x[i])
            ego_obj_rt.data.y.push(source_ego_obj.source_y[i])

            console.log("source_x:", source_ego_obj.source_x[i]);
            console.log("data_x:", ego_obj_rt.data.x);
            
            break
        }
    }
    ego_obj_rt.change.emit()

    const time_list = sum_vel_upper_source['time']
    for (let i = 0; i < time_list.length; i++) {
        if ((cur_time - time_list[i] < 0.1) && (cur_time - time_list[i] > 0)){
            sum_vel_upper.data.x = Array.from(sum_vel_upper_source.source_x[i])
            sum_vel_upper.data.y = Array.from(sum_vel_upper_source.source_y[i])
            sum_vel_lower.data.x = Array.from(sum_vel_lower_source.source_x[i])
            sum_vel_lower.data.y = Array.from(sum_vel_lower_source.source_y[i])
            sum_vel_ref.data.x = Array.from(sum_vel_ref_source.source_x[i])
            sum_vel_ref.data.y = Array.from(sum_vel_ref_source.source_y[i])

            //svlimit_ref_st_s.data.x = Array.from(svlimit_ref_st_s_source.source_x[i])
            //svlimit_ref_st_s.data.y = Array.from(svlimit_ref_st_s_source.source_y[i])
            svlimit_ref_vt_v.data.x = Array.from(svlimit_ref_vt_v_source.source_x[i])
            svlimit_ref_vt_v.data.y = Array.from(svlimit_ref_vt_v_source.source_y[i])
            svlimit_ref_at_a.data.x = Array.from(svlimit_ref_at_a_source.source_x[i])
            svlimit_ref_at_a.data.y = Array.from(svlimit_ref_at_a_source.source_y[i])

            const vel = Array.from(oprimizer_vel_source.y[i]);
            oprimizer_vel_rt.data.index = vel.map((_, index) => index);
            oprimizer_vel_rt.data.y = vel;  

            const vel_ref = Array.from(oprimizer_vel_ref_source.y[i]);
            oprimizer_vel_ref_rt.data.index = vel_ref.map((_, index) => index);
            oprimizer_vel_ref_rt.data.y = vel_ref; 

            const vel_upper = Array.from(oprimizer_vel_upper_source.y[i]);
            oprimizer_vel_upper_rt.data.index = vel_upper.map((_, index) => index);
            oprimizer_vel_upper_rt.data.y = vel_upper;

            const acc = Array.from(oprimizer_acc_source.y[i]);
            oprimizer_acc_rt.data.index = acc.map((_, index) => index);
            oprimizer_acc_rt.data.y = acc;

            const acc_ref = Array.from(oprimizer_acc_ref_source.y[i]);
            oprimizer_acc_ref_rt.data.index = acc_ref.map((_, index) => index);
            oprimizer_acc_ref_rt.data.y = acc_ref;

            const acc_upper = Array.from(oprimizer_acc_upper_source.y[i]);
            oprimizer_acc_upper_rt.data.index = acc_upper.map((_, index) => index);
            oprimizer_acc_upper_rt.data.y = acc_upper;

            const acc_lower = Array.from(oprimizer_acc_lower_source.y[i]);
            oprimizer_acc_lower_rt.data.index = acc_lower.map((_, index) => index);
            oprimizer_acc_lower_rt.data.y = acc_lower;

            const jerk = Array.from(oprimizer_jerk_source.y[i]);
            oprimizer_jerk_rt.data.index = jerk.map((_, index) => index);
            oprimizer_jerk_rt.data.y = jerk;

            const jerk_upper = Array.from(oprimizer_jerk_upper_source.y[i]);
            oprimizer_jerk_upper_rt.data.index = jerk_upper.map((_, index) => index);
            oprimizer_jerk_upper_rt.data.y = jerk_upper;

            const jerk_lower = Array.from(oprimizer_jerk_lower_source.y[i]);
            oprimizer_jerk_lower_rt.data.index = jerk_lower.map((_, index) => index);
            oprimizer_jerk_lower_rt.data.y = jerk_lower;
        }
    } 
    sum_vel_upper.change.emit();
    sum_vel_lower.change.emit();
    sum_vel_ref.change.emit();
    //svlimit_ref_st_s.change.emit();
    svlimit_ref_vt_v.change.emit();
    svlimit_ref_at_a.change.emit();
    oprimizer_vel_rt.change.emit();
    oprimizer_vel_ref_rt.change.emit(); 
    oprimizer_vel_upper_rt.change.emit();
    oprimizer_acc_rt.change.emit();
    oprimizer_acc_ref_rt.change.emit(); 
    oprimizer_acc_upper_rt.change.emit();
    oprimizer_acc_lower_rt.change.emit();  

    oprimizer_jerk_rt.change.emit();
    oprimizer_jerk_upper_rt.change.emit();
    oprimizer_jerk_lower_rt.change.emit();

    const traj_theta_time_list = traj_theta_source['time']
    for (let i = 0; i < traj_theta_time_list.length; i++) {
        if ((cur_time - traj_theta_time_list[i] < 0.1) && (cur_time - traj_theta_time_list[i] > 0)){
            traj_theta.data.x = Array.from(traj_theta_source.source_x[i])
            traj_theta.data.y = Array.from(traj_theta_source.source_y[i])
        }
    } 
    traj_theta.change.emit();

    const time_list_nop_counter = nop_counter_source.data['time'];
    const y_values_nop_counter = nop_counter_source.data['y'];
    nop_counter_current_value_source.data['time'] = [];
    nop_counter_current_value_source.data['y'] = [];
    for (let i = 0; i < time_list_nop_counter.length; i++) {
        if ((cur_time - time_list_nop_counter[i] < 0.1) && 
            (cur_time - time_list_nop_counter[i] > 0)) {  
            nop_counter_current_value_source.data['time'].push(time_list_nop_counter[i]);
            nop_counter_current_value_source.data['y'].push(y_values_nop_counter[i]);
        }
    }
    nop_counter_current_value_source.change.emit();
    
                        """)
        self.time_slider_.js_on_change('value', self.callback_)
        pass
    
    def add_slider(self):
        self.time_slider_ = Slider(start=0.0, end=self.bag_time_length_, value=0.0, step=.1, title="Time")
    
    def figure_plot(self):
        if self.read_bag_flag_  == False:
            self.read_bag()
            self.match_timestamp()

        self.creat_figure()

         # plot processed_map_info
        processed_map_data_map  = self.get_processed_map_debug_info('lane_debug')
         
        
        # plot predict trajectory
        predict_obj_time_rt = []
        predict_obj_x_rt = []
        predict_obj_y_rt = []
        obj_polygon_xs_rt = [] 
        obj_polygon_ys_rt = []
        nop_counter_rt = []
        for channel in self.data_set_.keys():
            if channel == XDEBUG_CHANNEL:
                for data_item in self.data_set_[channel]:
                    for msg_item in data_item.items():
                        if msg_item[0] == None:
                            continue 
                        if msg_item[1] == None or len(msg_item[1]['obstacle_debug']) == 0:
                            continue

                        predict_obj_time_rt.append(msg_item[0])
                        tmp_pre_x = []
                        tmp_pre_y = []
                        tmp_predict_obj_polygon_xs_rt = [] 
                        tmp_predict_obj_polygon_ys_rt = [] 
                        for object in msg_item[1]['obstacle_debug']:
                            # sum_of_squares = object['pred_position']['x']**2 + object['pred_position']['y']**2
                            # if math.sqrt(sum_of_squares) > 100:
                            #     continue

                            for traj_point in object['pred_traj']:
                                tmp_pre_x.append(traj_point['x'])
                                tmp_pre_y.append(traj_point['y'])
                            
                            tmp_predict_obj_polygon_x_rt = [] 
                            tmp_predict_obj_polygon_y_rt = [] 
                            for tmp_point in object['pred_first_polygon']:
                                tmp_predict_obj_polygon_x_rt.append(tmp_point['x'])
                                tmp_predict_obj_polygon_y_rt.append(tmp_point['y'])
                            tmp_predict_obj_polygon_xs_rt.append(tmp_predict_obj_polygon_x_rt)
                            tmp_predict_obj_polygon_ys_rt.append(tmp_predict_obj_polygon_y_rt)
                        predict_obj_x_rt.append(tmp_pre_x)
                        predict_obj_y_rt.append(tmp_pre_y)

                        ego_obj_polygon_x_rt = [] 
                        ego_obj_polygon_y_rt = []
                        if len(msg_item[1]['obstacle_debug']) != 0:
                            ego_obj_polygon_x_rt, ego_obj_polygon_y_rt =  \
                                self.get_ego_obj_polygon(msg_item[1]['ego_info_debug']['veh_position']['x'], \
                                                        msg_item[1]['ego_info_debug']['veh_position']['y'], \
                                                        msg_item[1]['ego_info_debug']['veh_yaw'])
                        tmp_predict_obj_polygon_xs_rt.append(ego_obj_polygon_x_rt)
                        tmp_predict_obj_polygon_ys_rt.append(ego_obj_polygon_y_rt)
                        obj_polygon_xs_rt.append(tmp_predict_obj_polygon_xs_rt)
                        obj_polygon_ys_rt.append(tmp_predict_obj_polygon_ys_rt)

                        tmp_nop_counter = []
                        tmp_nop_counter.append(msg_item[1]['nop_counter'])
                        nop_counter_rt.append(tmp_nop_counter)

        # plot real-time planning trajectory
        planning_traj_time_rt = []
        planning_traj_x_rt = []
        planning_traj_y_rt = []
        planning_traj_theta_rt = []
        planning_traj_s_rt = []
        for channel in self.data_set_.keys():
            if channel == XDEBUG_CHANNEL:
                for data_item in self.data_set_[XDEBUG_CHANNEL]:
                    for msg_item in data_item.items():
                        planning_traj_time_rt.append(msg_item[0])
                        tmp_traj_x = []
                        tmp_traj_y = []                      
                        tmp_traj_theta = []                      
                        tmp_traj_s = []                      
                        for traj_point in msg_item[1]['traj_x']:
                            tmp_traj_x.append(traj_point)
                        for traj_point in msg_item[1]['traj_y']:
                            tmp_traj_y.append(traj_point)
                        for traj_point in msg_item[1]['traj_theta']:
                            tmp_traj_theta.append(traj_point)
                        for traj_point in msg_item[1]['traj_s']:
                            tmp_traj_s.append(traj_point)
                        planning_traj_x_rt.append(tmp_traj_x)
                        planning_traj_y_rt.append(tmp_traj_y)
                        planning_traj_theta_rt.append(tmp_traj_theta)
                        planning_traj_s_rt.append(tmp_traj_s)

        # plot ego  
        ego_obj_time_rt = []
        ego_obj_x_rt = []
        ego_obj_y_rt = []
        for channel in self.data_set_.keys():
            if channel == XDEBUG_CHANNEL:
                for data_item in self.data_set_[channel]:
                    for msg_item in data_item.items():
                        if msg_item[0] == None:
                            continue 
                        if msg_item[1] == None or len(msg_item[1]['ego_info_debug']) == 0:
                            continue

                        ego_obj_time_rt.append(msg_item[0])
                        ego_obj_x_rt.append(msg_item[1]['ego_info_debug']['veh_position']['x'])
                        ego_obj_y_rt.append(msg_item[1]['ego_info_debug']['veh_position']['y'])

         # plot svlimit
        svlimit_time_rt = []
        svlimit_s_rt = []
        svlimit_v_upper_rt = []
        svlimit_v_lower_rt = []
        svlimit_ref_sv_v_rt = []
        svlimit_ref_st_t_rt = []
        # svlimit_ref_st_s_rt = []
        svlimit_ref_vt_v_rt = []
        svlimit_ref_at_a_rt = []
        optimize_s_out_rt = []
        optimize_v_out_rt = []
        optimizer_v_ref_rt = []
        optimizer_v_upper_rt = []
        optimizer_a_out_rt = []
        optimizer_a_ref_rt = []
        optimizer_a_upper_rt = []
        optimizer_a_lower_rt = []
        optimizer_jerk_out_rt = []
        optimizer_jerk_upper_rt = []
        optimizer_jerk_lower_rt = []
        for channel in self.data_set_.keys():
            if channel == XDEBUG_CHANNEL:
                for data_item in self.data_set_[channel]:
                    for msg_item in data_item.items():
                        if msg_item[0] == None:
                            continue 
                        if msg_item[1] == None or len(msg_item[1]) == 0:
                            continue

                        svlimit_time_rt.append(msg_item[0])
                        svlimit_s_rt.append(msg_item[1]['svlimit_s'])
                        svlimit_v_upper_rt.append(msg_item[1]['svlimit_v_upper']) 
                        svlimit_v_lower_rt.append(msg_item[1]['svlimit_v_lower'])
                        svlimit_ref_sv_v_rt.append(msg_item[1]['svlimit_ref_sv_v'])
                        svlimit_ref_st_t_rt.append(msg_item[1]['svlimit_ref_st_t'])
                        # svlimit_ref_st_s_rt.append(msg_item[1]['svlimit_ref_st_s'])
                        svlimit_ref_vt_v_rt.append(msg_item[1]['svlimit_ref_vt_v'])
                        svlimit_ref_at_a_rt.append(msg_item[1]['svlimit_ref_at_a'])
                        optimize_s_out_rt.append(msg_item[1]['optimizer_s_out'])
                        optimize_v_out_rt.append(msg_item[1]['optimizer_v_out'])
                        optimizer_v_ref_rt.append(msg_item[1]['optimizer_v_ref'])
                        optimizer_v_upper_rt.append(msg_item[1]['optimizer_v_upper'])
                        optimizer_a_out_rt.append(msg_item[1]['optimizer_a_out'])
                        optimizer_a_ref_rt.append(msg_item[1]['optimizer_a_ref'])
                        optimizer_a_upper_rt.append(msg_item[1]['optimizer_a_upper'])
                        optimizer_a_lower_rt.append(msg_item[1]['optimizer_a_lower'])
                        optimizer_jerk_out_rt.append(msg_item[1]['optimizer_jerk_out'])
                        optimizer_jerk_upper_rt.append(msg_item[1]['optimizer_jerk_upper'])
                        optimizer_jerk_lower_rt.append(msg_item[1]['optimizer_jerk_lower'])

        print("*" * 30 + " start adding bag data to source " + "*" * 30)

        cur_lane_center_line_time_rt = processed_map_data_map['cur_lane_center_line_time_rt']
        cur_lane_center_line_x_rt = processed_map_data_map['cur_lane_center_line_x_rt']
        cur_lane_center_line_y_rt = processed_map_data_map['cur_lane_center_line_y_rt']
        cur_lane_left_line_x_rt = processed_map_data_map['cur_lane_left_line_x_rt']
        cur_lane_left_line_y_rt = processed_map_data_map['cur_lane_left_line_y_rt']
        cur_lane_right_line_x_rt = processed_map_data_map['cur_lane_right_line_x_rt']
        cur_lane_right_line_y_rt = processed_map_data_map['cur_lane_right_line_y_rt']
        left_lane_center_line_time_rt = processed_map_data_map['left_lane_center_line_time_rt']
        left_lane_center_line_x_rt = processed_map_data_map['left_lane_center_line_x_rt']
        left_lane_center_line_y_rt = processed_map_data_map['left_lane_center_line_y_rt']
        right_lane_center_line_time_rt = processed_map_data_map['right_lane_center_line_time_rt']
        right_lane_center_line_x_rt = processed_map_data_map['right_lane_center_line_x_rt']
        right_lane_center_line_y_rt = processed_map_data_map['right_lane_center_line_y_rt']
        cur_lane_center_dist_to_bound_time_rt = processed_map_data_map['cur_lane_center_dist_to_bound_time_rt']
        cur_lane_center_dist_to_left_bound_rt = processed_map_data_map['cur_lane_center_dist_to_left_bound_rt']
        cur_lane_center_dist_to_bound_index_rt  = processed_map_data_map['cur_lane_center_dist_to_bound_index_rt']
        cur_lane_center_dist_to_right_bound_rt = processed_map_data_map['cur_lane_center_dist_to_right_bound_rt']
        cur_raw_lane_center_line_x_rt = processed_map_data_map['cur_raw_lane_center_line_x_rt']
        cur_raw_lane_center_line_y_rt =  processed_map_data_map['cur_raw_lane_center_line_y_rt']

        self.add_layer_processed_map_scatter(self.figs_['fig1'], 'cur_lane_center_line_rt_source', 'cur_lane_center_line_rt', cur_lane_center_line_time_rt, cur_lane_center_line_x_rt, cur_lane_center_line_y_rt, 'Cur Lane Center', 4, 0.2, 'blue')
        self.add_layer_processed_map_scatter(self.figs_['fig1'], 'cur_lane_left_line_rt_source', 'cur_lane_left_line_rt', cur_lane_center_line_time_rt, cur_lane_left_line_x_rt, cur_lane_left_line_y_rt, 'Cur Lane Left', 4, 0.2, 'Green')
        self.add_layer_processed_map_scatter(self.figs_['fig1'], 'cur_lane_right_line_rt_source', 'cur_lane_right_line_rt', cur_lane_center_line_time_rt, cur_lane_right_line_x_rt, cur_lane_right_line_y_rt, 'Cur Lane Right', 4, 0.2, 'coral')
        # self.add_layer_processed_map_line(self.figs_['fig1'], 'cur_lane_left_line_rt_source', 'cur_lane_left_line_rt', cur_lane_center_line_time_rt, cur_lane_left_line_x_rt, cur_lane_left_line_y_rt, 'Cur Lane Left', 4, 0.2, 'Green')
        # self.add_layer_processed_map_line(self.figs_['fig1'], 'cur_lane_right_line_rt_source', 'cur_lane_right_line_rt', cur_lane_center_line_time_rt, cur_lane_right_line_x_rt, cur_lane_right_line_y_rt, 'Cur Lane Right', 4, 0.2, 'coral')
        self.add_layer_processed_map_scatter(self.figs_['fig1'], 'left_lane_center_line_rt_source', 'left_lane_center_line_rt', left_lane_center_line_time_rt, left_lane_center_line_x_rt, left_lane_center_line_y_rt, 'Left Lane Center', 4, 0.2, 'hotpink')
        self.add_layer_processed_map_scatter(self.figs_['fig1'], 'right_lane_center_line_rt_source', 'right_lane_center_line_rt', right_lane_center_line_time_rt, right_lane_center_line_x_rt, right_lane_center_line_y_rt, 'Right Lane Center', 4, 0.2, 'indianred')
        self.add_layer_processed_map_scatter(self.figs_['fig1'], 'cur_raw_lane_center_rt_source', 'cur_raw_lane_center_rt', cur_lane_center_line_time_rt, cur_raw_lane_center_line_x_rt, cur_raw_lane_center_line_y_rt, 'Raw Cur Lane Center', 6, 0.6, 'red')

        self.add_layer_processed_map_line(self.figs_['lane_center_dist_to_boundary'], 'cur_lane_center_dist_to_left_bound_rt_source', 'cur_lane_center_dist_to_left_bound_rt', cur_lane_center_dist_to_bound_time_rt, cur_lane_center_dist_to_left_bound_rt, cur_lane_center_dist_to_bound_index_rt, 'Dist to Left Bound', 4, 0.2, 'Green')
        self.add_layer_processed_map_line(self.figs_['lane_center_dist_to_boundary'], 'cur_lane_center_dist_to_right_bound_rt_source', 'cur_lane_center_dist_to_right_bound_rt', cur_lane_center_dist_to_bound_time_rt, cur_lane_center_dist_to_right_bound_rt, cur_lane_center_dist_to_bound_index_rt, 'Dist to Right Bound', 4, 0.2, 'coral')

        # self.add_layer_processed_map_scatter(self.figs_['fig1'], 'cur_lane_center_line_rt_source_enu', 'cur_lane_center_line_rt_enu', cur_lane_center_line_time_rt_enu, cur_lane_center_line_x_rt_enu, cur_lane_center_line_y_rt_enu, 'Lane Center ENU', 4, 0.2, 'red')

        self.add_layer_predict_traj_scatter(predict_obj_time_rt, predict_obj_x_rt, predict_obj_y_rt)
        self.add_layer_planning_traj_scatter(planning_traj_time_rt,planning_traj_x_rt,planning_traj_y_rt)
        self.add_layer_ego_obj_point(ego_obj_time_rt, ego_obj_x_rt, ego_obj_y_rt)

        self.add_layer_polygons(self.figs_['fig1'], 'obj_polygons_rt_source', 'obj_polygons_rt', predict_obj_time_rt, obj_polygon_xs_rt, obj_polygon_ys_rt, 'obj_polygons', 4, 0.4, 'white')
        self.add_layer_scatter(time_list_=svlimit_time_rt, x_values_=svlimit_s_rt, y_values_=svlimit_v_upper_rt,y_key='sum_vel_upper', 
                                         source_key='sum_vel_upper_rt_source', figure_key='svlimit', legend_label='v_upper',xaxis_lable="x/m",yaxis_lable="m/s",
                                         color_='black')
        self.add_layer_scatter(time_list_=svlimit_time_rt, x_values_=svlimit_s_rt, y_values_=svlimit_v_lower_rt,y_key='sum_vel_lower', 
                                         source_key='sum_vel_lower_rt_source', figure_key='svlimit', legend_label='v_lower',xaxis_lable="x/m",yaxis_lable="m/s",
                                         color_='green')
        self.add_layer_scatter(time_list_=svlimit_time_rt, x_values_=svlimit_s_rt, y_values_=svlimit_ref_sv_v_rt,y_key='sum_vel_ref', 
                                    source_key='sum_vel_ref_rt_source', figure_key='svlimit', legend_label='v_ref',xaxis_lable="x/m",yaxis_lable="m/s",
                                    color_='red')
        self.add_layer_scatter(time_list_=planning_traj_time_rt,x_values_=planning_traj_s_rt,y_values_=planning_traj_theta_rt,y_key='traj_theta', 
                                         source_key='traj_theta_rt_source', figure_key='traj_theta', legend_label='traj_theta',xaxis_lable="s/m",yaxis_lable="heading/rad",
                                         color_='green')


        # self.add_layer_vtlimit_scatter(time_list_=svlimit_time_rt, x_values_=svlimit_ref_st_t_rt, y_values_=svlimit_ref_st_s_rt,y_key='svlimit_ref_st_s', 
        #                                  source_key='svlimit_ref_st_s_rt_source', figure_key='svlimit_vt', legend_label='s_ref',
        #                                  color_='green')
        self.add_layer_scatter(time_list_=svlimit_time_rt, x_values_=svlimit_ref_st_t_rt, y_values_=svlimit_ref_vt_v_rt,y_key='svlimit_ref_vt_v', 
                                         source_key='svlimit_ref_vt_v_rt_source', figure_key='svlimit_vt', legend_label='v_ref',xaxis_lable="t/s",yaxis_lable="m/s",
                                         color_='red')
        self.add_layer_scatter(time_list_=svlimit_time_rt, x_values_=svlimit_ref_st_t_rt, y_values_=svlimit_ref_at_a_rt,y_key='svlimit_ref_at_a', 
                                         source_key='svlimit_ref_at_a_rt_source', figure_key='svlimit_vt', legend_label='a_ref',xaxis_lable="t/s",yaxis_lable="m/s",
                                         color_='blue')
         # 纵向优化v-out
        self.add_traj_layer_scatter(time_list_=svlimit_time_rt, y_values_=optimize_v_out_rt, y_key='optimizer_vel_rt', 
                                    source_key='optimizer_vel_rt_source', figure_key='osqp_v_t', legend_label='Vel-Time')
        # 纵向优化v-ref
        self.add_traj_layer_scatter(time_list_=svlimit_time_rt, y_values_=optimizer_v_ref_rt, y_key='optimizer_vel_ref_rt', 
                                    source_key='optimizer_vel_ref_rt_source', figure_key='osqp_v_t', legend_label='Vel-REF-Time',
                                    color_ = 'red')
        # 纵向优化v-upper
        self.add_traj_layer_scatter(time_list_=svlimit_time_rt, y_values_=optimizer_v_upper_rt, y_key='optimizer_vel_upper_rt', 
                                    source_key='optimizer_vel_upper_rt_source', figure_key='osqp_v_t', legend_label='Vel-UPPER-Time',
                                    color_ = 'blue')
        # 纵向优化a-out
        self.add_traj_layer_scatter(time_list_=svlimit_time_rt, y_values_=optimizer_a_out_rt, y_key='optimizer_acc_rt', 
                                    source_key='optimizer_acc_rt_source', figure_key='osqp_a_t', legend_label='Acc-Time')
        # 纵向优化a-ref
        self.add_traj_layer_scatter(time_list_=svlimit_time_rt, y_values_=optimizer_a_ref_rt, y_key='optimizer_acc_ref_rt', 
                                    source_key='optimizer_acc_ref_rt_source', figure_key='osqp_a_t', legend_label='Acc-Ref-Time',
                                    color_ = 'red')
        # 纵向优化a-upper
        self.add_traj_layer_scatter(time_list_=svlimit_time_rt, y_values_=optimizer_a_upper_rt, y_key='optimizer_acc_upper_rt', 
                                    source_key='optimizer_acc_upper_rt_source', figure_key='osqp_a_t', legend_label='Acc-Upper-Time',
                                    color_ = 'blue')
        # 纵向优化a-lower
        self.add_traj_layer_scatter(time_list_=svlimit_time_rt, y_values_=optimizer_a_lower_rt, y_key='optimizer_acc_lower_rt', 
                                    source_key='optimizer_acc_lower_rt_source', figure_key='osqp_a_t', legend_label='Acc-Lower-Time',
                                    color_ = 'blue')
        # 纵向优化jerk-out
        self.add_traj_layer_scatter(time_list_=svlimit_time_rt, y_values_=optimizer_jerk_out_rt, y_key='optimizer_jerk_rt', 
                                    source_key='optimizer_jerk_rt_source', figure_key='osqp_jerk_t', legend_label='Jerk-Time',
                                    color_ = 'black')
        # 纵向优化jerk-upper
        self.add_traj_layer_scatter(time_list_=svlimit_time_rt, y_values_=optimizer_jerk_upper_rt, y_key='optimizer_jerk_upper_rt', 
                                    source_key='optimizer_jerk_upper_rt_source', figure_key='osqp_jerk_t', legend_label='Jerk-Upper-Time',
                                    color_ = 'blue')
        # 纵向优化jerk-lower
        self.add_traj_layer_scatter(time_list_=svlimit_time_rt, y_values_=optimizer_jerk_lower_rt, y_key='optimizer_jerk_lower_rt', 
                                    source_key='optimizer_jerk_lower_rt_source', figure_key='osqp_jerk_t', legend_label='Jerk-Lower-Time',
                                    color_ = 'blue')

        self.add_time_scatter(time_list_=planning_traj_time_rt, y_values_=nop_counter_rt, y_key='nop_counter_time', 
                                    source_key='nop_counter_source', figure_key='nop_counter', legend_label='nop_counter-Time')

        self.add_slider()
        self.add_rt_callback()

        self.activate_figure_option()

        self.show2html()
        # self.show2notebook()

    def show2html(self):
        output_file(self.output_)
        row1 = row(self.figs_['fig1'], 
                   column(self.checkbox_groups_['lane_center_dist_to_boundary'],self.figs_['lane_center_dist_to_boundary'],
                          self.checkbox_groups_['svlimit'],self.figs_['svlimit'],
                          self.checkbox_groups_['svlimit_vt'],self.figs_['svlimit_vt'],
                          self.checkbox_groups_['traj_theta'],self.figs_['traj_theta']),
                #    column(self.checkbox_groups_['osqp_s_t'],self.figs_['osqp_s_t'],
                        #   self.checkbox_groups_['osqp_v_t'],self.figs_['osqp_v_t']),
                #    column(self.checkbox_groups_['osqp_a_t'],self.figs_['osqp_a_t'],
                        #   self.checkbox_groups_['osqp_jerk_t'],self.figs_['osqp_jerk_t'])
                   column(self.checkbox_groups_['nop_counter'],self.figs_['nop_counter'],)
                          )
        layout = column(self.time_slider_, row1)
        show(layout)
        return

    def show2notebook(self):
        output_notebook()
        show(column(self.time_slider_,self.fig_))
        return
    
    def get_processed_map_debug_info(self, json_key):
        # lane center line and boundary line
        cur_lane_center_line_time_rt = []
        cur_lane_center_line_x_rt = []
        cur_lane_center_line_y_rt = []
        cur_raw_lane_center_line_x_rt = []
        cur_raw_lane_center_line_y_rt = []
        cur_lane_left_line_x_rt = []
        cur_lane_left_line_y_rt = []
        cur_lane_right_line_x_rt = []
        cur_lane_right_line_y_rt = []
        left_lane_center_line_time_rt = []
        left_lane_center_line_x_rt = []
        left_lane_center_line_y_rt = []
        left_lane_left_line_x_rt = []
        left_lane_left_line_y_rt = []
        left_lane_right_line_x_rt = []
        left_lane_right_line_y_rt = []
        right_lane_center_line_time_rt = []
        right_lane_center_line_x_rt = []
        right_lane_center_line_y_rt = []
        right_lane_left_line_x_rt = []
        right_lane_left_line_y_rt = []
        right_lane_right_line_x_rt = []
        right_lane_right_line_y_rt = []

        for channel in self.data_set_.keys():
            if channel == XDEBUG_CHANNEL:
                for data_item in self.data_set_[channel]:
                    for msg_item in data_item.items():
                        if msg_item[0] == None:
                            continue 
                        if msg_item[1] == None or len(msg_item[1][json_key]) == 0:
                            continue

                        for lane_debug in msg_item[1][json_key]:
                            if lane_debug['id'] == 0:
                                cur_lane_center_line_time_rt.append(msg_item[0])
                                center_line_x_rt = cur_lane_center_line_x_rt
                                center_line_y_rt = cur_lane_center_line_y_rt
                                left_line_x_rt = cur_lane_left_line_x_rt
                                left_line_y_rt = cur_lane_left_line_y_rt
                                right_line_x_rt = cur_lane_right_line_x_rt
                                right_line_y_rt = cur_lane_right_line_y_rt
                            elif lane_debug['id'] == -1:
                                left_lane_center_line_time_rt.append(msg_item[0])
                                center_line_x_rt = left_lane_center_line_x_rt
                                center_line_y_rt = left_lane_center_line_y_rt
                                left_line_x_rt = left_lane_left_line_x_rt
                                left_line_y_rt = left_lane_left_line_y_rt
                                right_line_x_rt = left_lane_right_line_x_rt
                                right_line_y_rt = left_lane_right_line_y_rt
                            elif lane_debug['id'] == 1:
                                right_lane_center_line_time_rt.append(msg_item[0])
                                center_line_x_rt = right_lane_center_line_x_rt
                                center_line_y_rt = right_lane_center_line_y_rt
                                left_line_x_rt = right_lane_left_line_x_rt
                                left_line_y_rt = right_lane_left_line_y_rt
                                right_line_x_rt = right_lane_right_line_x_rt
                                right_line_y_rt = right_lane_right_line_y_rt
                            else:
                                continue

                            tmp_cur_raw_x = []
                            tmp_cur_raw_y = []
                            if lane_debug['id'] == 0:
                                for tmp_point2d in lane_debug['raw_points']:
                                    tmp_cur_raw_x.append(tmp_point2d['x'])
                                    tmp_cur_raw_y.append(tmp_point2d['y'])
                                cur_raw_lane_center_line_x_rt.append(tmp_cur_raw_x)
                                cur_raw_lane_center_line_y_rt.append(tmp_cur_raw_y)

                            tmp_x = []
                            tmp_y = []

                            for tmp_point in lane_debug['lane_points']:
                                tmp_x.append(tmp_point['center_point']['x'])
                                tmp_y.append(tmp_point['center_point']['y'])
                            # if len(center_line_x_rt) > 0:
                            #     tmp_x += center_line_x_rt[-1]
                            # if len(center_line_y_rt) > 0:
                            #     tmp_y += center_line_y_rt[-1]
                            center_line_x_rt.append(tmp_x)
                            center_line_y_rt.append(tmp_y)

                            tmp_left_x_list = []
                            tmp_left_y_list = []
                            tmp_right_x_list = []
                            tmp_right_y_list = []
                            for lane_segment in lane_debug['lane_segments']:
                                tmp_x = []
                                tmp_y = []
                                for tmp_point in lane_segment['left_line_points']:
                                    tmp_x.append(tmp_point['pos']['x'])
                                    tmp_y.append(tmp_point['pos']['y'])
                                tmp_left_x_list += tmp_x
                                tmp_left_y_list += tmp_y

                                tmp_x = []
                                tmp_y = []
                                for tmp_point in lane_segment['right_line_points']:
                                    tmp_x.append(tmp_point['pos']['x'])
                                    tmp_y.append(tmp_point['pos']['y'])
                                tmp_right_x_list += tmp_x
                                tmp_right_y_list += tmp_y
                            left_line_x_rt.append(tmp_left_x_list)
                            left_line_y_rt.append(tmp_left_y_list)
                            right_line_x_rt.append(tmp_right_x_list)
                            right_line_y_rt.append(tmp_right_y_list)


        # cur lane center dist to boundary
        cur_lane_center_dist_to_bound_time_rt = []
        cur_lane_center_dist_to_left_bound_rt = []
        cur_lane_center_dist_to_right_bound_rt = []
        cur_lane_center_dist_to_bound_index_rt = []

        for channel in self.data_set_.keys():
            if channel == XDEBUG_CHANNEL:
                for data_item in self.data_set_[channel]:
                    for msg_item in data_item.items():
                        if msg_item[0] == None:
                            continue 
                        if msg_item[1] == None or len(msg_item[1][json_key]) == 0:
                            continue
                        
                        cur_lane_center_dist_to_bound_time_rt.append(msg_item[0])
                        for lane_debug in msg_item[1]['lane_debug']:
                            if lane_debug['id'] != 0:
                                continue
                            
                            tmp_left_boundry = []
                            tmp_right_boundry = []
                            for tmp_point in lane_debug['lane_points']:
                                tmp_left_boundry.append(tmp_point['left_boundry']['dist_to_boundry'])
                                tmp_right_boundry.append(tmp_point['right_boundry']['dist_to_boundry'])
                            cur_lane_center_dist_to_left_bound_rt.append(tmp_left_boundry)
                            cur_lane_center_dist_to_right_bound_rt.append(tmp_right_boundry)
                            cur_lane_center_dist_to_bound_index_rt.append(list(range(len(tmp_left_boundry))))
        
        data_map = {
            'cur_lane_center_line_time_rt':cur_lane_center_line_time_rt,
            'cur_lane_center_line_x_rt':cur_lane_center_line_x_rt,
            'cur_lane_center_line_y_rt':cur_lane_center_line_y_rt,
            'cur_lane_left_line_x_rt':cur_lane_left_line_x_rt,
            'cur_lane_left_line_y_rt':cur_lane_left_line_y_rt,
            'cur_lane_right_line_x_rt':cur_lane_right_line_x_rt,
            'cur_lane_right_line_y_rt':cur_lane_right_line_y_rt,
            'left_lane_center_line_time_rt':left_lane_center_line_time_rt,
            'left_lane_center_line_x_rt':left_lane_center_line_x_rt,
            'left_lane_center_line_y_rt':left_lane_center_line_y_rt,
            'left_lane_left_line_x_rt':left_lane_left_line_x_rt,
            'left_lane_left_line_y_rt':left_lane_left_line_y_rt,
            'left_lane_right_line_x_rt':left_lane_right_line_x_rt,
            'left_lane_right_line_y_rt':left_lane_right_line_y_rt,
            'right_lane_center_line_time_rt':right_lane_center_line_time_rt,
            'right_lane_center_line_x_rt':right_lane_center_line_x_rt,
            'right_lane_center_line_y_rt':right_lane_center_line_y_rt,
            'right_lane_left_line_x_rt':right_lane_left_line_x_rt,
            'right_lane_left_line_y_rt':right_lane_left_line_y_rt,
            'right_lane_right_line_x_rt':right_lane_right_line_x_rt,
            'right_lane_right_line_y_rt':right_lane_right_line_y_rt,
            'cur_lane_center_dist_to_bound_time_rt':cur_lane_center_dist_to_bound_time_rt,
            'cur_lane_center_dist_to_left_bound_rt':cur_lane_center_dist_to_left_bound_rt,
            'cur_lane_center_dist_to_right_bound_rt':cur_lane_center_dist_to_right_bound_rt,
            'cur_lane_center_dist_to_bound_index_rt':cur_lane_center_dist_to_bound_index_rt,
            'cur_raw_lane_center_line_x_rt':cur_raw_lane_center_line_x_rt,
            'cur_raw_lane_center_line_y_rt':cur_raw_lane_center_line_y_rt
            }

        return  data_map
    
    def get_processed_map_enu_debug_info(self, json_key):
        cur_lane_center_line_time_rt = []
        cur_lane_center_line_x_rt = []
        cur_lane_center_line_y_rt = []
        cur_lane_left_line_x_rt = []
        cur_lane_left_line_y_rt = []
        cur_lane_right_line_x_rt = []
        cur_lane_right_line_y_rt = []
        left_lane_center_line_time_rt = []
        left_lane_center_line_x_rt = []
        left_lane_center_line_y_rt = []
        left_lane_left_line_x_rt = []
        left_lane_left_line_y_rt = []
        left_lane_right_line_x_rt = []
        left_lane_right_line_y_rt = []
        right_lane_center_line_time_rt = []
        right_lane_center_line_x_rt = []
        right_lane_center_line_y_rt = []
        right_lane_left_line_x_rt = []
        right_lane_left_line_y_rt = []
        right_lane_right_line_x_rt = []
        right_lane_right_line_y_rt = []
        cur_lane_center_dist_to_bound_time_rt = []
        cur_lane_center_dist_to_left_bound_rt = []
        cur_lane_center_dist_to_right_bound_rt = []
        cur_lane_center_dist_to_bound_index_rt = []

        for channel in self.data_set_.keys():
            if channel == XDEBUG_CHANNEL:
                for data_item in self.data_set_[channel]:
                    for msg_item in data_item.items():
                        if msg_item[0] == None:
                            continue 
                        if msg_item[1] == None or len(msg_item[1][json_key]) == 0:
                            continue

                        cur_lane_center_line_time_rt.append(msg_item[0])
                        tmp_x = []
                        tmp_y = []
                        for lane_debug in msg_item[1][json_key]:    
                            for tmp_point in lane_debug['lane_points']:
                                tmp_x.append(tmp_point['center_point']['x'])
                                tmp_y.append(tmp_point['center_point']['y'])
                        # if len(cur_lane_center_line_x_rt) > 0:
                        #     tmp_x += cur_lane_center_line_x_rt[-1]
                        # if len(cur_lane_center_line_y_rt) > 0:
                        #     tmp_y += cur_lane_center_line_y_rt[-1]
                        cur_lane_center_line_x_rt.append(tmp_x)
                        cur_lane_center_line_y_rt.append(tmp_y)

        return cur_lane_center_line_time_rt, cur_lane_center_line_x_rt, cur_lane_center_line_y_rt, \
               cur_lane_left_line_x_rt, cur_lane_left_line_y_rt, cur_lane_right_line_x_rt, \
               cur_lane_right_line_y_rt, left_lane_center_line_time_rt, \
               left_lane_center_line_x_rt, left_lane_center_line_y_rt, \
               left_lane_left_line_x_rt, left_lane_left_line_y_rt, \
               left_lane_right_line_x_rt, left_lane_right_line_y_rt, \
               right_lane_center_line_time_rt, right_lane_center_line_x_rt, \
               right_lane_center_line_y_rt, right_lane_left_line_x_rt, right_lane_left_line_y_rt, \
               right_lane_right_line_x_rt, right_lane_right_line_y_rt, \
               cur_lane_center_dist_to_bound_time_rt, cur_lane_center_dist_to_left_bound_rt, \
               cur_lane_center_dist_to_right_bound_rt, cur_lane_center_dist_to_bound_index_rt
    
    def get_ego_obj_polygon(self, geometry_center_x, geometry_center_y, yaw):
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
        
        # print('******')
        # print(dist_center_to_corner)
        # print(math.sqrt((left_front_corner_x - geometry_center_x)**2 + \
        #                 (left_front_corner_y - geometry_center_y)**2))
        # print(math.sqrt((right_front_corner_x - geometry_center_x)**2 + \
        #                 (right_front_corner_y - geometry_center_y)**2))
        # print(math.sqrt((left_rear_corner_x - geometry_center_x)**2 + \
        #                 (left_rear_corner_y - geometry_center_y)**2))
        # print(math.sqrt((right_rear_corner_x - geometry_center_x)**2 + \
        #                 (right_rear_corner_y - geometry_center_y)**2))
        
        # print(geometry_center_x, geometry_center_y)
        # print(left_front_corner_x, left_front_corner_y)
        # print(right_front_corner_x, right_front_corner_y)
        # print(left_rear_corner_x, left_rear_corner_y)
        # print(right_rear_corner_x, right_rear_corner_y)
        # print((left_front_corner_x + right_front_corner_x + left_rear_corner_x + right_rear_corner_x) / 4, \
        #       (left_front_corner_y + right_front_corner_y + left_rear_corner_y + right_rear_corner_y) / 4)
        
        
        x.append(left_front_corner_x)
        x.append(right_front_corner_x)
        x.append(right_rear_corner_x)
        x.append(left_rear_corner_x)
        y.append(left_front_corner_y)
        y.append(right_front_corner_y)
        y.append(right_rear_corner_y)
        y.append(left_rear_corner_y)
        return x, y

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

    xpilot_plot = PlotXpilotViz(args.source,args.output)

    xpilot_plot.add_required_channel(XDEBUG_CHANNEL)

    xpilot_plot.figure_plot()
