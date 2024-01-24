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
from old_framework_baseplot import PlotBase

class PlotXpilotViz(PlotBase):
    def __init__(self, bag_path,output):
        super().__init__(bag_path,output)

    def creat_figure(self):
        # main hmi window
        # self.figs_['fig1'] = figure(width=800 ,height = 800,x_range = [-5,100],y_range = [-7.5,7.5] ,match_aspect=True, title = 'Xviz')
        self.figs_['fig1'] = figure(width=800, height = 1300, match_aspect=True, title = 'Xviz')
        self.figs_['lane_center_dist_to_boundary'] = figure(width=600 ,height = 300)
        self.figs_['svlimit'] = figure(width=600 ,height = 300,y_range = [0,40])
        self.figs_['traj_theta'] = figure(width=600 ,height = 300)
        self.figs_['traj_theta_error'] = figure(width=600 ,height = 300,y_range = [-0.1,0.1])
        self.figs_['traj_d'] = figure(width=600 ,height = 300,y_range = [-0.6,0.6])
        self.figs_['traj_delta'] = figure(width=600 ,height = 300,y_range = [-0.1,0.1])
        self.figs_['traj_curve'] = figure(width=600 ,height = 300,y_range = [-0.008,0.008])
        self.figs_['nop_counter'] = figure(width=600 ,height = 300)
        self.figs_['svlimit_vt'] = figure(width=600 ,height = 300,y_range = [0,40])
        self.figs_['osqp_s_t'] = figure(width=600 ,height = 300,x_axis_label='t/s', y_axis_label='s/m')
        self.figs_['osqp_v_t'] = figure(width=600 ,height = 300,x_axis_label='t/s', y_axis_label='m/s',y_range = [0,40])
        self.figs_['osqp_a_t'] = figure(width=600 ,height = 300,x_axis_label='t/s', y_axis_label='m/s^2')
        self.figs_['osqp_jerk_t'] = figure(width=600 ,height = 300,x_axis_label='t/s', y_axis_label='m/s^3')
        self.figs_['traj_vel'] = figure(width=600 ,height = 300, y_range = [0, 30])
        self.figs_['traj_s_v'] = figure(width=600 ,height = 300)
        self.figs_['traj_s_a'] = figure(width=600 ,height = 300, y_range = [-4,5])
        self.figs_['traj_update'] = figure(width=600 ,height = 300, y_range = [-1,2],title = 'traj_update')
        self.figs_['lc_lon_search_st'] = figure(width=600 ,height = 300, x_axis_label='t/s', y_axis_label='s/m')
        self.figs_['lc_lat_search_sd'] = figure(width=600 ,height = 300, x_range = [10,-10], x_axis_label='d/m', y_axis_label='s/m')

        for fig_name in ['fig1', 'lane_center_dist_to_boundary','svlimit','svlimit_vt','traj_theta','osqp_s_t','osqp_v_t','osqp_a_t','osqp_jerk_t'
                         ,'traj_vel','traj_s_v','traj_s_a','traj_update','traj_d','traj_theta_error','traj_delta','traj_curve','nop_counter',
                         'lc_lon_search_st', 'lc_lat_search_sd']:
            fig = self.figs_[fig_name]
            checkbox_group = CheckboxButtonGroup(labels=[fig_name], active=[0], width=600)
            callback = CustomJS(args=dict(fig=fig, checkbox_group=checkbox_group), code="""
                fig.visible = checkbox_group.active.includes(0);
            """)
            checkbox_group.js_on_change('active', callback)
            self.checkbox_groups_[fig_name] = checkbox_group  # Add the checkbox group to the dictionary

    # 根据滑块显示对应时间点数据
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

    def add_layer_planning_traj_scatter(self, time_list_, source_x_,source_y_,line_width_ = 3,line_alpha_ = 0.4, color_ = 'red'):
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
    
    def add_layer_longi_search_sample_point(self, fig, rt_source_name_, rt_name_, time_list_, 
                                       source_x_, source_y_, source_v_, source_a_, source_cost_, source_color_, legend_, size_ = 3, line_alpha_ = 0.4):
        self.plot_data_set_[rt_source_name_] = dict(
            time = time_list_,
            source_x = source_x_,
            source_y = source_y_,
            source_v = source_v_,
            source_a = source_a_,
            source_color = source_color_,
            source_cost = source_cost_,
        )
        self.plot_data_set_[rt_name_] = ColumnDataSource(data=dict(
            x = [],
            y = [],
            v = [],
            a = [],
            color = [],
            cost = [],
        ))

        scatter_point = fig.scatter('y','x',source=self.plot_data_set_[rt_name_], 
                          color='color' ,line_alpha=line_alpha_,legend_label = legend_, size=size_)

        # fig.scatter('y','x',source=self.plot_data_set_[rt_name_], 
        #                     color=self.color_mapper_,
        #                     line_alpha=line_alpha_,legend_label = legend_, 
        #                     size=size_)

        tooltips = [
            ("s", "@x"),
            ("v","@v"),
            ("a","@a"),
            ("t", "@y"),
            ("cost","@cost"),
        ]

        scatter_point_hover = HoverTool(renderers=[scatter_point],
                      tooltips=tooltips,
                      mode='mouse')
        fig.add_tools(scatter_point_hover)
        return
    
    def add_layer_dmpp_polygons(self, fig, rt_source_name_, rt_name_, time_list_, 
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
                       fill_color = color_ ,line_alpha=line_alpha_,legend_label = legend_, line_width=size_)

        return

    def add_lon_search_st_graph(self, fig_name, source_suffix, lon_search_debug_time_rt, st_path_t_rt, st_path_s_rt, \
                st_sample_points_t_rt, st_sample_points_s_rt, st_sample_points_a_rt, \
                st_sample_points_v_rt, st_sample_points_cost_rt, st_obj_polygon_xs_rt, \
                st_obj_polygon_ys_rt, vt_path_t_rt, vt_path_v_rt, vt_limit_t_rt, vt_limit_v_rt, \
                st_sample_points_color_rt):
        self.add_layer_processed_map_line(self.figs_[fig_name], 'st_path_rt_source' + source_suffix, 'st_path_rt' + source_suffix, lon_search_debug_time_rt, st_path_t_rt, st_path_s_rt, 'st_path', 4, 0.2, 'coral')
        self.add_layer_longi_search_sample_point(self.figs_[fig_name], 'st_sample_points_rt_source' + source_suffix, 'st_sample_points_rt' + source_suffix, lon_search_debug_time_rt, 
                                       st_sample_points_s_rt, st_sample_points_t_rt, st_sample_points_v_rt, st_sample_points_a_rt, st_sample_points_cost_rt, st_sample_points_color_rt, 'st_sample_points', 10, 0.2)
        self.add_layer_dmpp_polygons(self.figs_[fig_name], 'st_obj_polygons_rt_source' + source_suffix, 'st_obj_polygons_rt' + source_suffix, lon_search_debug_time_rt, st_obj_polygon_xs_rt, st_obj_polygon_ys_rt, 'st_obj_polygons', 4, 0.2, 'hotpink')

        return
    
        
    def planning_origin_source(self,time_list_,x_list_,y_list_):
        self.plot_data_set_['planning_origin_source'] = dict(
            time = time_list_,
            source_x = x_list_,
            source_y = y_list_,
        )
        self.plot_data_set_['planning_origin_rt'] = ColumnDataSource(data=dict(
            x = [],
            y = [],
        ))
        self.figs_['fig1'].scatter('x','y',source=self.plot_data_set_['planning_origin_rt'], line_width = 10,
                                   color = 'black', line_alpha=0.6, legend_label = 'Planning Origin')
        return

    def planning_origin_curve_source(self,time_list_,x_list_,y_list_):
        self.plot_data_set_['planning_origin_curve_source'] = dict(
            time = time_list_,
            source_x = x_list_,
            source_y = y_list_,
        )
        self.plot_data_set_['planning_origin_curve_rt'] = ColumnDataSource(data=dict(
            x = [],
            y = [],
        ))
        self.figs_['traj_curve'].scatter('x','y',source=self.plot_data_set_['planning_origin_curve_rt'], line_width = 5,
                                   color = 'black', line_alpha=0.6, legend_label = 'Planning Origin Curve')
        return
    
    def add_layer_polygons(self, fig, rt_source_name_, rt_name_, time_list_, 
                                    source_polygon_xs_, source_polygon_ys_, fill_alpha_,legend_, size_ = 3, line_alpha_ = 0.4, color_ = 'blue'):
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
                       fill_color = color_, fill_alpha=fill_alpha_, line_alpha=line_alpha_,legend_label = legend_, line_width=size_)

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
                                           predict_traj_rt = self.plot_data_set_['predict_trajectory_rt'],
                                           source_predict_traj = self.plot_data_set_['predict_trajectory_rt_source'],
                                           planning_traj_rt = self.plot_data_set_['planning_trajectory_rt'],
                                           source_planning_traj = self.plot_data_set_['planning_trajectory_rt_source'],
                                           traj_r_upper_rt = self.plot_data_set_['traj_r_upper'],
                                           source_traj_r_upper = self.plot_data_set_['traj_r_upper_source'],
                                           path_bound_high_rt = self.plot_data_set_['path_bound_high'],
                                           source_path_bound_high= self.plot_data_set_['path_bound_high_source'],
                                           path_bound_lower_rt = self.plot_data_set_['path_bound_lower'],
                                           source_path_bound_lower = self.plot_data_set_['path_bound_lower_source'],
                                           traj_r_lower_rt = self.plot_data_set_['traj_r_lower'],
                                           source_traj_r_lower = self.plot_data_set_['traj_r_lower_source'],
                                           traj_r_ref_rt = self.plot_data_set_['traj_r_ref'],
                                           source_traj_r_ref = self.plot_data_set_['traj_r_ref_source'],
                                           ego_obj_rt = self.plot_data_set_['ego_obj_rt'],
                                           source_ego_obj = self.plot_data_set_['ego_obj_rt_source'],
                                           planning_orgin_point_source = self.plot_data_set_['planning_origin_source'],
                                           planning_orgin_point_rt = self.plot_data_set_['planning_origin_rt'],
                                           planning_orgin_curve_source = self.plot_data_set_['planning_origin_curve_source'],
                                           planning_orgin_curve_rt = self.plot_data_set_['planning_origin_curve_rt'],
                                           obj_polygons_rt = self.plot_data_set_['obj_polygons_rt'],
                                           source_predict_obj_polygons = self.plot_data_set_['obj_polygons_rt_source'],
                                           nudge_obj_polygons_rt = self.plot_data_set_['nudge_obj_polygons_rt'],
                                           nudge_source_predict_obj_polygons = self.plot_data_set_['nudge_obj_polygons_rt_source'],
                                           sum_vel_curvature = self.plot_data_set_['sum_vel_curvature'],
                                           sum_vel_curvature_source = self.plot_data_set_['sum_vel_curvature_rt_source'],
                                           sum_vel_upper = self.plot_data_set_['sum_vel_upper'],
                                           sum_vel_upper_source = self.plot_data_set_['sum_vel_upper_rt_source'],
                                           sum_vel_lower = self.plot_data_set_['sum_vel_lower'],
                                           sum_vel_lower_source = self.plot_data_set_['sum_vel_lower_rt_source'],
                                           leader_s_v_rt = self.plot_data_set_['leader_s_v_rt'],
                                           leader_s_v_source = self.plot_data_set_['leader_s_v_rt_source'],
                                           traj_theta_rt = self.plot_data_set_['traj_theta'],
                                           traj_theta_source = self.plot_data_set_['traj_theta_rt_source'],
                                           traj_theta_error_rt = self.plot_data_set_['traj_theta_error_ref'],
                                           traj_theta_error_source = self.plot_data_set_['traj_theta_error_ref_rt_source'],
                                           traj_d_rt = self.plot_data_set_['traj_d'],
                                           traj_d_source = self.plot_data_set_['traj_d_rt_source'],
                                           traj_ref_d_rt = self.plot_data_set_['traj_ref_d'],
                                           traj_ref_d_source = self.plot_data_set_['traj_ref_d_rt_source'],

                                           traj_delta_rt = self.plot_data_set_['traj_delta'],
                                           traj_delta_source = self.plot_data_set_['traj_delta_rt_source'],
                                           traj_ref_delta_rt = self.plot_data_set_['traj_ref_delta'],
                                           traj_ref_delta_source = self.plot_data_set_['traj_ref_delta_rt_source'],

                                           traj_curve_rt = self.plot_data_set_['traj_curve'],
                                           traj_curve_source = self.plot_data_set_['traj_curve_rt_source'],

                                           nop_counter_source=self.plot_data_set_['nop_counter_source'],
                                           nop_counter_current_value_source=self.plot_data_set_['nop_counter_time'],

                                           sum_vel_ref = self.plot_data_set_['sum_vel_ref'],
                                           sum_vel_ref_source = self.plot_data_set_['sum_vel_ref_rt_source'],

                                           svlimit_ref_st_s = self.plot_data_set_['svlimit_ref_st_s'],
                                           svlimit_ref_st_s_source = self.plot_data_set_['svlimit_ref_st_s_rt_source'],
                                           svlimit_ref_vt_v = self.plot_data_set_['svlimit_ref_vt_v'],
                                           svlimit_ref_vt_v_source = self.plot_data_set_['svlimit_ref_vt_v_rt_source'],
                                           svlimit_ref_at_a = self.plot_data_set_['svlimit_ref_at_a'],
                                           svlimit_ref_at_a_source = self.plot_data_set_['svlimit_ref_at_a_rt_source'],

                                           idm_ref_st_s = self.plot_data_set_['idm_ref_st_s'],
                                           idm_ref_st_s_source = self.plot_data_set_['idm_ref_st_s_rt_source'],
                                           idm_ref_vt_v = self.plot_data_set_['idm_ref_vt_v'],
                                           idm_ref_vt_v_source = self.plot_data_set_['idm_ref_vt_v_rt_source'],
                                           idm_ref_at_a = self.plot_data_set_['idm_ref_at_a'],
                                           idm_ref_at_a_source = self.plot_data_set_['idm_ref_at_a_rt_source'],

                                           optimizer_s_v_rt = self.plot_data_set_['optimizer_s_v_rt'],
                                           optimizer_s_v_source = self.plot_data_set_['optimizer_s_v_rt_source'],

                                           oprimizer_s_rt = self.plot_data_set_['optimizer_s_rt'],
                                           oprimizer_s_source = self.plot_data_set_['optimizer_s_rt_source'],
                                           oprimizer_s_ref_rt = self.plot_data_set_['optimizer_s_ref_rt'],
                                           oprimizer_s_ref_source = self.plot_data_set_['optimizer_s_ref_rt_source'], 
                                           oprimizer_s_upper_rt = self.plot_data_set_['optimizer_s_upper_rt'],
                                           oprimizer_s_upper_source = self.plot_data_set_['optimizer_s_upper_rt_source'],

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
                                           vel_rt=self.plot_data_set_['vel_time'],
                                           vel_source=self.plot_data_set_['vel_time_source'],
                                           sum_vel_rt = self.plot_data_set_['sum_vel_rt'],
                                           sum_vel_source = self.plot_data_set_['sum_vel_rt_source'],
                                           sum_acc_rt = self.plot_data_set_['sum_acc_rt'],
                                           sum_acc_source = self.plot_data_set_['sum_acc_rt_source'],

                                           st_path_rt_lc_decider = self.plot_data_set_['st_path_rt_lc_decider'],
                                           source_st_path_rt_lc_decider = self.plot_data_set_['st_path_rt_source_lc_decider'],
                                           st_sample_points_rt_lc_decider = self.plot_data_set_['st_sample_points_rt_lc_decider'],
                                           source_st_sample_points_rt_lc_decider = self.plot_data_set_['st_sample_points_rt_source_lc_decider'],
                                           st_obj_polygons_rt_lc_decider = self.plot_data_set_['st_obj_polygons_rt_lc_decider'],
                                           source_st_obj_polygons_rt_lc_decider = self.plot_data_set_['st_obj_polygons_rt_source_lc_decider'],

                                           ds_bound_rt_lc_decider = self.plot_data_set_['ds_bound_rt_lc_decider'],
                                           source_ds_bound_rt_lc_decider = self.plot_data_set_['ds_bound_rt_source_lc_decider'],
                                           ds_range_rt_lc_decider = self.plot_data_set_['ds_range_rt_lc_decider'],
                                           source_ds_range_rt_lc_decider = self.plot_data_set_['ds_range_rt_source_lc_decider'],
                                           ds_goal_rt_lc_decider = self.plot_data_set_['ds_goal_rt_lc_decider'],
                                           source_ds_goal_rt_lc_decider = self.plot_data_set_['ds_goal_rt_source_lc_decider'],
                                           ds_ego_rt_lc_decider = self.plot_data_set_['ds_ego_rt_lc_decider'],
                                           source_ds_ego_rt_lc_decider = self.plot_data_set_['ds_ego_rt_source_lc_decider'],
                                           ds_traj_rt_lc_decider = self.plot_data_set_['ds_traj_rt_lc_decider'],
                                           source_ds_traj_rt_lc_decider = self.plot_data_set_['ds_traj_rt_source_lc_decider'],
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

    nudge_obj_polygons_rt.data.xs.length = 0
    nudge_obj_polygons_rt.data.ys.length = 0
    const nudge_time_list = nudge_source_predict_obj_polygons['time']
    for (let i = 0; i < nudge_time_list.length; i++) {
        if ((cur_time - nudge_time_list[i] < 0.1) && 
            (cur_time - nudge_time_list[i] > 0)){
            const nudge_polygon_xs = nudge_source_predict_obj_polygons.source_polygon_xs[i]
            for (let i = 0; i < nudge_polygon_xs.length; i++) {
                nudge_obj_polygons_rt.data.xs.push(Array.from(nudge_polygon_xs[i]))
            }
            const nudge_polygon_ys = nudge_source_predict_obj_polygons.source_polygon_ys[i]
            for (let i = 0; i < nudge_polygon_ys.length; i++) {
                nudge_obj_polygons_rt.data.ys.push(Array.from(nudge_polygon_ys[i]))
            }
            
            break
        }
    }
    nudge_obj_polygons_rt.change.emit()

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

    const traj_r_upper_time_list = source_traj_r_upper['time']
    for (let i = 0; i < traj_r_upper_time_list.length; i++) {
        if ((cur_time - traj_r_upper_time_list[i] < 0.1) && 
            (cur_time - traj_r_upper_time_list[i] > 0)){
            traj_r_upper_rt.data.x = Array.from(source_traj_r_upper.source_x[i])
            traj_r_upper_rt.data.y = Array.from(source_traj_r_upper.source_y[i])           
            break
        }
    }
    traj_r_upper_rt.change.emit()


    const path_bound_high_time_list = source_path_bound_high['time']
    for (let i = 0; i < path_bound_high_time_list.length; i++) {
        if ((cur_time - path_bound_high_time_list[i] < 0.1) && 
            (cur_time - path_bound_high_time_list[i] > 0)){
            path_bound_high_rt.data.x = Array.from(source_path_bound_high.source_x[i])
            path_bound_high_rt.data.y = Array.from(source_path_bound_high.source_y[i])           
            break
        }
    }
    path_bound_high_rt.change.emit()


    const path_bound_low_time_list = source_path_bound_lower['time']
    for (let i = 0; i < path_bound_low_time_list.length; i++) {
        if ((cur_time - path_bound_low_time_list[i] < 0.1) && 
            (cur_time - path_bound_low_time_list[i] > 0)){
            path_bound_lower_rt.data.x = Array.from(source_path_bound_lower.source_x[i])
            path_bound_lower_rt.data.y = Array.from(source_path_bound_lower.source_y[i])           
            break
        }
    }
    path_bound_lower_rt.change.emit()

    const traj_r_lower_time_list = source_traj_r_lower['time']
    for (let i = 0; i < traj_r_lower_time_list.length; i++) {
        if ((cur_time - traj_r_lower_time_list[i] < 0.1) && 
            (cur_time - traj_r_lower_time_list[i] > 0)){
            traj_r_lower_rt.data.x = Array.from(source_traj_r_lower.source_x[i])
            traj_r_lower_rt.data.y = Array.from(source_traj_r_lower.source_y[i])           
            break
        }
    }
    traj_r_lower_rt.change.emit()

    const traj_r_ref_time_list = source_traj_r_ref['time']
    for (let i = 0; i < traj_r_ref_time_list.length; i++) {
        if ((cur_time - traj_r_ref_time_list[i] < 0.1) && 
            (cur_time - traj_r_ref_time_list[i] > 0)){
            traj_r_ref_rt.data.x = Array.from(source_traj_r_ref.source_x[i])
            traj_r_ref_rt.data.y = Array.from(source_traj_r_ref.source_y[i])           
            break
        }
    }
    traj_r_ref_rt.change.emit()

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

    planning_orgin_point_rt.data.x.length = 0
    planning_orgin_point_rt.data.y.length = 0
    const planning_origin_time_list = planning_orgin_point_source['time']
    for (let i = 0; i < planning_origin_time_list.length; i++) {
        if ((cur_time - planning_origin_time_list[i] < 0.1) && 
            (cur_time - planning_origin_time_list[i] > 0)){
            planning_orgin_point_rt.data.x.push(planning_orgin_point_source.source_x[i])
            planning_orgin_point_rt.data.y.push(planning_orgin_point_source.source_y[i])
            break
        }
    }
    planning_orgin_point_rt.change.emit()

    planning_orgin_curve_rt.data.x.length = 0
    planning_orgin_curve_rt.data.y.length = 0
    const planning_origin_curve_time_list = planning_orgin_curve_source['time']
    for (let i = 0; i < planning_origin_curve_time_list.length; i++) {
        if ((cur_time - planning_origin_curve_time_list[i] < 0.1) && 
            (cur_time - planning_origin_curve_time_list[i] > 0)){
            planning_orgin_curve_rt.data.x.push(planning_orgin_curve_source.source_x[i])
            planning_orgin_curve_rt.data.y.push(planning_orgin_curve_source.source_y[i])
            break
        }
    }
    planning_orgin_curve_rt.change.emit()

    const time_list = sum_vel_upper_source['time']
    for (let i = 0; i < time_list.length; i++) {
        if ((cur_time - time_list[i] < 0.1) && (cur_time - time_list[i] > 0)){
            sum_vel_upper.data.x = Array.from(sum_vel_upper_source.source_x[i])
            sum_vel_upper.data.y = Array.from(sum_vel_upper_source.source_y[i])
            sum_vel_lower.data.x = Array.from(sum_vel_lower_source.source_x[i])
            sum_vel_lower.data.y = Array.from(sum_vel_lower_source.source_y[i])
            leader_s_v_rt.data.x = Array.from(leader_s_v_source.source_x[i])
            leader_s_v_rt.data.y = Array.from(leader_s_v_source.source_y[i])
            sum_vel_ref.data.x = Array.from(sum_vel_ref_source.source_x[i])
            sum_vel_ref.data.y = Array.from(sum_vel_ref_source.source_y[i])
            sum_vel_curvature.data.x = Array.from(sum_vel_curvature_source.source_x[i])
            sum_vel_curvature.data.y = Array.from(sum_vel_curvature_source.source_y[i])

            svlimit_ref_st_s.data.x = Array.from(svlimit_ref_st_s_source.source_x[i])
            svlimit_ref_st_s.data.y = Array.from(svlimit_ref_st_s_source.source_y[i])
            svlimit_ref_vt_v.data.x = Array.from(svlimit_ref_vt_v_source.source_x[i])
            svlimit_ref_vt_v.data.y = Array.from(svlimit_ref_vt_v_source.source_y[i])
            svlimit_ref_at_a.data.x = Array.from(svlimit_ref_at_a_source.source_x[i])
            svlimit_ref_at_a.data.y = Array.from(svlimit_ref_at_a_source.source_y[i])

            idm_ref_st_s.data.x = Array.from(idm_ref_st_s_source.source_x[i])
            idm_ref_st_s.data.y = Array.from(idm_ref_st_s_source.source_y[i])
            idm_ref_vt_v.data.x = Array.from(idm_ref_vt_v_source.source_x[i])
            idm_ref_vt_v.data.y = Array.from(idm_ref_vt_v_source.source_y[i])
            idm_ref_at_a.data.x = Array.from(idm_ref_at_a_source.source_x[i])
            idm_ref_at_a.data.y = Array.from(idm_ref_at_a_source.source_y[i])

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

            optimizer_s_v_rt.data.x = Array.from(optimizer_s_v_source.source_x[i])
            optimizer_s_v_rt.data.y = Array.from(optimizer_s_v_source.source_y[i])

            const sum = Array.from(oprimizer_s_source.y[i]);
            oprimizer_s_rt.data.index = sum.map((_, index) => index);
            oprimizer_s_rt.data.y = sum;

            const sum_ref = Array.from(oprimizer_s_ref_source.y[i]);
            oprimizer_s_ref_rt.data.index = sum_ref.map((_, index) => index);
            oprimizer_s_ref_rt.data.y = sum_ref; 

            const sum_upper = Array.from(oprimizer_s_upper_source.y[i]);
            oprimizer_s_upper_rt.data.index = sum_upper.map((_, index) => index);
            oprimizer_s_upper_rt.data.y = sum_upper;
        }
    } 
    sum_vel_upper.change.emit();
    sum_vel_lower.change.emit();
    sum_vel_ref.change.emit();
    svlimit_ref_st_s.change.emit();
    svlimit_ref_vt_v.change.emit();
    svlimit_ref_at_a.change.emit();
    sum_vel_curvature.change.emit();
    idm_ref_st_s.change.emit();
    idm_ref_vt_v.change.emit();
    idm_ref_at_a.change.emit();
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
    optimizer_s_v_rt.change.emit();
    leader_s_v_rt.change.emit();
    oprimizer_s_rt.change.emit();
    oprimizer_s_ref_rt.change.emit(); 
    oprimizer_s_upper_rt.change.emit();

    const time_list_traj = vel_source['time']
    for (let i = 0; i < time_list_traj.length; i++) {
        if ((cur_time - time_list_traj[i] < 0.1) && (cur_time - time_list_traj[i] > 0)){
            const vels = Array.from(vel_source.y[i]);
            vel_rt.data.index = vels.map((_, index) => index);
            vel_rt.data.y = vels;

            sum_vel_rt.data.x = Array.from(sum_vel_source.source_x[i])
            sum_vel_rt.data.y = Array.from(sum_vel_source.source_y[i])

            sum_acc_rt.data.x = Array.from(sum_acc_source.source_x[i])
            sum_acc_rt.data.y = Array.from(sum_acc_source.source_y[i])
        }
    }
    vel_rt.change.emit();
    sum_vel_rt.change.emit();
    sum_acc_rt.change.emit(); 

    const traj_theta_time_list = traj_theta_source['time']
    for (let i = 0; i < traj_theta_time_list.length; i++) {
        if ((cur_time - traj_theta_time_list[i] < 0.1) && (cur_time - traj_theta_time_list[i] > 0)){
            traj_theta_rt.data.x = Array.from(traj_theta_source.source_x[i])
            traj_theta_rt.data.y = Array.from(traj_theta_source.source_y[i])
        }
    } 
    traj_theta_rt.change.emit()

    const traj_theta_error_time_list = traj_theta_error_source['time']
    for (let i = 0; i < traj_theta_error_time_list.length; i++) {
        if ((cur_time - traj_theta_error_time_list[i] < 0.1) && (cur_time - traj_theta_error_time_list[i] > 0)){
            traj_theta_error_rt.data.x = Array.from(traj_theta_error_source.source_x[i])
            traj_theta_error_rt.data.y = Array.from(traj_theta_error_source.source_y[i])
        }
    } 
    traj_theta_error_rt.change.emit()

    const traj_d_time_list = traj_d_source['time']
    for (let i = 0; i < traj_d_time_list.length; i++) {
        if ((cur_time - traj_d_time_list[i] < 0.1) && (cur_time - traj_d_time_list[i] > 0)){
            traj_d_rt.data.x = Array.from(traj_d_source.source_x[i])
            traj_d_rt.data.y = Array.from(traj_d_source.source_y[i])
        }
    } 
    traj_d_rt.change.emit()

    const traj_ref_d_time_list = traj_ref_d_source['time']
    for (let i = 0; i < traj_ref_d_time_list.length; i++) {
        if ((cur_time - traj_ref_d_time_list[i] < 0.1) && (cur_time - traj_ref_d_time_list[i] > 0)){
            traj_ref_d_rt.data.x = Array.from(traj_ref_d_source.source_x[i])
            traj_ref_d_rt.data.y = Array.from(traj_ref_d_source.source_y[i])
        }
    } 
    traj_ref_d_rt.change.emit()

    const traj_ref_delta_time_list = traj_ref_delta_source['time']
    for (let i = 0; i < traj_ref_delta_time_list.length; i++) {
        if ((cur_time - traj_ref_delta_time_list[i] < 0.1) && (cur_time - traj_ref_delta_time_list[i] > 0)){
            traj_ref_delta_rt.data.x = Array.from(traj_ref_delta_source.source_x[i])
            traj_ref_delta_rt.data.y = Array.from(traj_ref_delta_source.source_y[i])
        }
    } 
    traj_ref_delta_rt.change.emit()


    const traj_delta_time_list = traj_delta_source['time']
    for (let i = 0; i < traj_delta_time_list.length; i++) {
        if ((cur_time - traj_delta_time_list[i] < 0.1) && (cur_time - traj_delta_time_list[i] > 0)){
            traj_delta_rt.data.x = Array.from(traj_delta_source.source_x[i])
            traj_delta_rt.data.y = Array.from(traj_delta_source.source_y[i])
        }
    } 
    traj_delta_rt.change.emit()

    const traj_curve_time_list = traj_curve_source['time']
    for (let i = 0; i < traj_curve_time_list.length; i++) {
        if ((cur_time - traj_curve_time_list[i] < 0.1) && (cur_time - traj_curve_time_list[i] > 0)){
            traj_curve_rt.data.x = Array.from(traj_curve_source.source_x[i])
            traj_curve_rt.data.y = Array.from(traj_curve_source.source_y[i])
        }
    } 
    traj_curve_rt.change.emit()

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

    st_path_rt_lc_decider.data.x.length = 0
    st_path_rt_lc_decider.data.y.length = 0
    st_sample_points_rt_lc_decider.data.x.length = 0
    st_sample_points_rt_lc_decider.data.y.length = 0
    st_sample_points_rt_lc_decider.data.v.length = 0
    st_sample_points_rt_lc_decider.data.a.length = 0
    st_sample_points_rt_lc_decider.data.color.length = 0
    st_sample_points_rt_lc_decider.data.cost.length = 0
    st_obj_polygons_rt_lc_decider.data.xs.length = 0
    st_obj_polygons_rt_lc_decider.data.ys.length = 0
    const st_path_rt_time_list_lc_decider = source_st_path_rt_lc_decider['time']
    for (let i = 0; i < st_path_rt_time_list_lc_decider.length; i++) {
        if ((cur_time - st_path_rt_time_list_lc_decider[i] < 0.1) && (cur_time - st_path_rt_time_list_lc_decider[i] > 0)){
            st_path_rt_lc_decider.data.x = Array.from(source_st_path_rt_lc_decider.source_x[i])
            st_path_rt_lc_decider.data.y = Array.from(source_st_path_rt_lc_decider.source_y[i])
            
            st_sample_points_rt_lc_decider.data.x = Array.from(source_st_sample_points_rt_lc_decider.source_x[i])
            st_sample_points_rt_lc_decider.data.y = Array.from(source_st_sample_points_rt_lc_decider.source_y[i])
            st_sample_points_rt_lc_decider.data.v = Array.from(source_st_sample_points_rt_lc_decider.source_v[i])
            st_sample_points_rt_lc_decider.data.a = Array.from(source_st_sample_points_rt_lc_decider.source_a[i])

            st_sample_points_rt_lc_decider.data.cost = Array.from(source_st_sample_points_rt_lc_decider.source_cost[i])
            st_sample_points_rt_lc_decider.data.color = Array.from(source_st_sample_points_rt_lc_decider.source_color[i])

            const polygon_xs = source_st_obj_polygons_rt_lc_decider.source_polygon_xs[i]
            for (let i = 0; i < polygon_xs.length; i++) {
                st_obj_polygons_rt_lc_decider.data.xs.push(Array.from(polygon_xs[i]))
            }
            const polygon_ys = source_st_obj_polygons_rt_lc_decider.source_polygon_ys[i]
            for (let i = 0; i < polygon_ys.length; i++) {
                st_obj_polygons_rt_lc_decider.data.ys.push(Array.from(polygon_ys[i]))
            }

            break
       }
    }
    st_path_rt_lc_decider.change.emit()
    st_sample_points_rt_lc_decider.change.emit()
    st_obj_polygons_rt_lc_decider.change.emit()

    ds_bound_rt_lc_decider.data.x.length = 0
    ds_bound_rt_lc_decider.data.y.length = 0
    ds_range_rt_lc_decider.data.x.length = 0
    ds_range_rt_lc_decider.data.y.length = 0
    ds_goal_rt_lc_decider.data.x.length = 0
    ds_goal_rt_lc_decider.data.y.length = 0
    ds_ego_rt_lc_decider.data.x.length = 0
    ds_ego_rt_lc_decider.data.y.length = 0
    ds_traj_rt_lc_decider.data.x.length = 0
    ds_traj_rt_lc_decider.data.y.length = 0
    const lat_search_sd_rt_time_list_lc_decider = source_ds_bound_rt_lc_decider['time']
    for (let i = 0; i < lat_search_sd_rt_time_list_lc_decider.length; i++) {
        if ((cur_time - lat_search_sd_rt_time_list_lc_decider[i] < 0.1) && (cur_time - lat_search_sd_rt_time_list_lc_decider[i] > 0)){
            ds_bound_rt_lc_decider.data.x = Array.from(source_ds_bound_rt_lc_decider.source_x[i])
            ds_bound_rt_lc_decider.data.y = Array.from(source_ds_bound_rt_lc_decider.source_y[i])

            ds_range_rt_lc_decider.data.x = Array.from(source_ds_range_rt_lc_decider.source_x[i])
            ds_range_rt_lc_decider.data.y = Array.from(source_ds_range_rt_lc_decider.source_y[i])

            ds_goal_rt_lc_decider.data.x = Array.from(source_ds_goal_rt_lc_decider.source_x[i])
            ds_goal_rt_lc_decider.data.y = Array.from(source_ds_goal_rt_lc_decider.source_y[i])

            ds_ego_rt_lc_decider.data.x = Array.from(source_ds_ego_rt_lc_decider.source_x[i])
            ds_ego_rt_lc_decider.data.y = Array.from(source_ds_ego_rt_lc_decider.source_y[i])

            ds_traj_rt_lc_decider.data.x = Array.from(source_ds_traj_rt_lc_decider.source_x[i])
            ds_traj_rt_lc_decider.data.y = Array.from(source_ds_traj_rt_lc_decider.source_y[i])

            break
       }
    }
    ds_bound_rt_lc_decider.change.emit()
    ds_range_rt_lc_decider.change.emit()
    ds_goal_rt_lc_decider.change.emit()
    ds_ego_rt_lc_decider.change.emit()
    ds_traj_rt_lc_decider.change.emit()
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
        cur_lane_center_line_time_rt, cur_lane_center_line_x_rt, cur_lane_center_line_y_rt, \
            cur_lane_left_line_x_rt, cur_lane_left_line_y_rt, cur_lane_right_line_x_rt, \
            cur_lane_right_line_y_rt, left_lane_center_line_time_rt, \
            left_lane_center_line_x_rt, left_lane_center_line_y_rt, \
            left_lane_left_line_x_rt, left_lane_left_line_y_rt, \
            left_lane_right_line_x_rt, left_lane_right_line_y_rt, \
            right_lane_center_line_time_rt, right_lane_center_line_x_rt, \
            right_lane_center_line_y_rt, right_lane_left_line_x_rt, right_lane_left_line_y_rt, \
            right_lane_right_line_x_rt, right_lane_right_line_y_rt, \
            cur_lane_center_dist_to_bound_time_rt, cur_lane_center_dist_to_left_bound_rt, \
            cur_lane_center_dist_to_right_bound_rt, cur_lane_center_dist_to_bound_index_rt \
                = self.get_processed_map_debug_info('lane_debug')
        
        # # plot processed_map_info with enu val
        # cur_lane_center_line_time_rt_enu, cur_lane_center_line_x_rt_enu, cur_lane_center_line_y_rt_enu, \
        #     cur_lane_left_line_x_rt_enu, cur_lane_left_line_y_rt_enu, cur_lane_right_line_x_rt_enu, \
        #     cur_lane_right_line_y_rt_enu, left_lane_center_line_time_rt_enu, \
        #     left_lane_center_line_x_rt_enu, left_lane_center_line_y_rt_enu, \
        #     left_lane_left_line_x_rt_enu, left_lane_left_line_y_rt_enu, \
        #     left_lane_right_line_x_rt_enu, left_lane_right_line_y_rt_enu, \
        #     right_lane_center_line_time_rt_enu, right_lane_center_line_x_rt_enu, \
        #     right_lane_center_line_y_rt_enu, right_lane_left_line_x_rt_enu, right_lane_left_line_y_rt_enu, \
        #     right_lane_right_line_x_rt, right_lane_right_line_y_rt, \
        #     cur_lane_center_dist_to_bound_time_rt_enu, cur_lane_center_dist_to_left_bound_rt_enu, \
        #     cur_lane_center_dist_to_right_bound_rt_enu, cur_lane_center_dist_to_bound_index_rt_enu \
        #         = self.get_processed_map_enu_debug_info('lane_debug_enu')

        # plot predict trajectory
        predict_obj_time_rt = []
        predict_obj_x_rt = []
        predict_obj_y_rt = []
        obj_polygon_xs_rt = [] 
        obj_polygon_ys_rt = []
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

        # plot nudge obs polygon
        nudge_obj_time_rt = []
        nudge_obj_polygon_xs_rt = [] 
        nudge_obj_polygon_ys_rt = []
        for channel in self.data_set_.keys():
            if channel == XDEBUG_CHANNEL:
                for data_item in self.data_set_[channel]:
                    for msg_item in data_item.items():
                        if msg_item[0] == None:
                            continue 
                        if msg_item[1] == None or len(msg_item[1]['nudge_obstacle_debug']) == 0:
                            continue
                        nudge_obj_time_rt.append(msg_item[0])
                        tmp_predict_obj_polygon_xs_rt = [] 
                        tmp_predict_obj_polygon_ys_rt = [] 
                        for object in msg_item[1]['nudge_obstacle_debug']:                     
                            tmp_predict_obj_polygon_x_rt = [] 
                            tmp_predict_obj_polygon_y_rt = [] 
                            for tmp_point in object['pred_first_polygon']:
                                tmp_predict_obj_polygon_x_rt.append(tmp_point['x'])
                                tmp_predict_obj_polygon_y_rt.append(tmp_point['y'])
                            tmp_predict_obj_polygon_xs_rt.append(tmp_predict_obj_polygon_x_rt)
                            tmp_predict_obj_polygon_ys_rt.append(tmp_predict_obj_polygon_y_rt)

                        nudge_obj_polygon_xs_rt.append(tmp_predict_obj_polygon_xs_rt)
                        nudge_obj_polygon_ys_rt.append(tmp_predict_obj_polygon_ys_rt)

        # plot path bound
        path_bound_time_rt = []
        path_bound_low_x_rt = []
        path_bound_low_y_rt = []
        path_bound_high_x_rt = []
        path_bound_high_y_rt = []
        for channel in self.data_set_.keys():
            if channel == XDEBUG_CHANNEL:
                for data_item in self.data_set_[XDEBUG_CHANNEL]:
                    for msg_item in data_item.items():
                        path_bound_time_rt.append(msg_item[0])
                        tmp_path_bound_low_x_rt = []
                        tmp_path_bound_low_y_rt = []                      
                        tmp_path_bound_high_x_rt = []                      
                        tmp_path_bound_high_y_rt = []                      
                 
                        for traj_point in msg_item[1]['path_bound_low_x']:
                            tmp_path_bound_low_x_rt.append(traj_point)
                        for traj_point in msg_item[1]['path_bound_low_y']:
                            tmp_path_bound_low_y_rt.append(traj_point)
                        for traj_point in msg_item[1]['path_bound_high_x']:
                            tmp_path_bound_high_x_rt.append(traj_point)
                        for traj_point in msg_item[1]['path_bound_high_y']:
                            tmp_path_bound_high_y_rt.append(traj_point)


                        path_bound_low_x_rt.append(tmp_path_bound_low_x_rt)
                        path_bound_low_y_rt.append(tmp_path_bound_low_y_rt)
                        path_bound_high_x_rt.append(tmp_path_bound_high_x_rt)
                        path_bound_high_y_rt.append(tmp_path_bound_high_y_rt)


        # plot real-time planning trajectory
        planning_traj_time_rt = []
        planning_traj_x_rt = []
        planning_traj_y_rt = []
        planning_traj_theta_rt = []
        planning_traj_s_rt = []
        planning_traj_d_rt = []
        planning_traj_ref_d_rt = []
        planning_traj_ref_delta_rt = []
        planning_traj_delta_rt = []
        planning_traj_curve_rt = []
        nop_counter_rt = []

        traj_r_upper_x = []
        traj_r_upper_y = []
        traj_r_lower_x = []
        traj_r_lower_y = []
        traj_r_ref_x = []
        traj_r_ref_y = []
        traj_theta_error_ref_index = []
        traj_theta_error_ref = []
        i = 0
        for channel in self.data_set_.keys():
            if channel == XDEBUG_CHANNEL:
                for data_item in self.data_set_[XDEBUG_CHANNEL]:
                    for msg_item in data_item.items():
                        i = 0
                        planning_traj_time_rt.append(msg_item[0])
                        tmp_traj_x = []
                        tmp_traj_y = []                      
                        tmp_traj_theta = []                      
                        tmp_traj_s = []                      
                        tmp_traj_d = []                      
                        tmp_traj_ref_d = []                      
                        tmp_traj_r_upper_x = []                      
                        tmp_traj_r_upper_y = []                      
                        tmp_traj_r_lower_x = []                      
                        tmp_traj_r_lower_y = []                      
                        tmp_traj_r_ref_x = []                      
                        tmp_traj_r_ref_y = []   
                        tmp_index = []                   
                        tmp_traj_delta = []                   
                        tmp_traj_ref_delta = []                   
                        tmp_traj_theta_error_ref = []                      
                        tmp_traj_curve_ref = []                      
                        tmp_nop_counter = []                      
                        for traj_point in msg_item[1]['traj_x']:
                            tmp_traj_x.append(traj_point)
                        for traj_point in msg_item[1]['traj_y']:
                            tmp_traj_y.append(traj_point)
                        for traj_point in msg_item[1]['traj_theta']:
                            tmp_traj_theta.append(traj_point*180/3.1415926)
                        for traj_point in msg_item[1]['traj_s']:
                            tmp_traj_s.append(traj_point)
                        for traj_point in msg_item[1]['traj_d']:
                            tmp_traj_d.append(traj_point)
                        for traj_point in msg_item[1]['traj_ref_d']:
                            tmp_traj_ref_d.append(traj_point)
                        for traj_point in msg_item[1]['traj_r_upper_x']:
                            tmp_traj_r_upper_x.append(traj_point)
                        for traj_point in msg_item[1]['traj_r_upper_y']:
                            tmp_traj_r_upper_y.append(traj_point)
                        for traj_point in msg_item[1]['traj_r_lower_x']:
                            tmp_traj_r_lower_x.append(traj_point)
                        for traj_point in msg_item[1]['traj_r_lower_y']:
                            tmp_traj_r_lower_y.append(traj_point)
                        for traj_point in msg_item[1]['traj_r_ref_x']:
                            tmp_traj_r_ref_x.append(traj_point)
                        for traj_point in msg_item[1]['traj_r_ref_y']:
                            tmp_traj_r_ref_y.append(traj_point)
                        for traj_point in msg_item[1]['traj_theta_error_ref']:
                            tmp_traj_theta_error_ref.append(traj_point)
                            tmp_index.append(i)
                            i = i + 1
                        for traj_point in msg_item[1]['traj_delta_ref']:
                            tmp_traj_ref_delta.append(traj_point)
                        for traj_point in msg_item[1]['traj_delta']:
                            tmp_traj_delta.append(traj_point)
                        for traj_point in msg_item[1]['traj_curve_spline_ref']:
                            tmp_traj_curve_ref.append(traj_point)
                        tmp_nop_counter.append(msg_item[1]['nop_counter'])
                        planning_traj_x_rt.append(tmp_traj_x)
                        planning_traj_y_rt.append(tmp_traj_y)
                        planning_traj_theta_rt.append(tmp_traj_theta)
                        planning_traj_s_rt.append(tmp_traj_s)
                        planning_traj_d_rt.append(tmp_traj_d)
                        planning_traj_ref_d_rt.append(tmp_traj_ref_d)
                        traj_r_upper_x.append(tmp_traj_r_upper_x)
                        traj_r_upper_y.append(tmp_traj_r_upper_y)
                        traj_r_lower_x.append(tmp_traj_r_lower_x)
                        traj_r_lower_y.append(tmp_traj_r_lower_y)
                        traj_r_ref_x.append(tmp_traj_r_ref_x)
                        traj_r_ref_y.append(tmp_traj_r_ref_y)
                        traj_theta_error_ref.append(tmp_traj_theta_error_ref)
                        traj_theta_error_ref_index.append(tmp_index)
                        planning_traj_ref_delta_rt.append(tmp_traj_ref_delta)
                        planning_traj_delta_rt.append(tmp_traj_delta)
                        planning_traj_curve_rt.append(tmp_traj_curve_ref)
                        nop_counter_rt.append(tmp_nop_counter)

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
        s_leader_vector_rt = []
        v_leader_vector_rt = []
        svlimit_v_upper_rt = []
        svlimit_v_lower_rt = []
        svlimit_ref_sv_v_rt = []
        svlimit_ref_st_t_rt = []

        svlimit_ref_st_s_rt = []
        svlimit_ref_vt_v_rt = []
        svlimit_ref_at_a_rt = []

        s_ref_follow_vector_rt = []
        v_ref_follow_vector_rt = []
        a_ref_follow_vector_rt = []

        optimize_s_out_rt = []
        optimize_s_ref_rt = []
        optimize_s_upper_rt = []
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
        svlimit_v_by_curvature_rt = []
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
                        s_leader_vector_rt.append(msg_item[1]['s_leader'])
                        v_leader_vector_rt.append(msg_item[1]['v_leader'])
                        svlimit_v_upper_rt.append(msg_item[1]['svlimit_v_upper']) 
                        svlimit_v_lower_rt.append(msg_item[1]['svlimit_v_lower'])
                        svlimit_ref_sv_v_rt.append(msg_item[1]['svlimit_ref_sv_v'])
                        svlimit_ref_st_t_rt.append(msg_item[1]['svlimit_ref_st_t'])
                        svlimit_ref_st_s_rt.append(msg_item[1]['svlimit_ref_st_s'])
                        svlimit_ref_vt_v_rt.append(msg_item[1]['svlimit_ref_vt_v'])
                        svlimit_ref_at_a_rt.append(msg_item[1]['svlimit_ref_at_a'])
                        s_ref_follow_vector_rt.append(msg_item[1]['s_ref_follow'])
                        v_ref_follow_vector_rt.append(msg_item[1]['v_ref_follow'])
                        a_ref_follow_vector_rt.append(msg_item[1]['a_ref_follow'])
                        optimize_s_out_rt.append(msg_item[1]['optimizer_s_out'])
                        optimize_s_ref_rt.append(msg_item[1]['optimizer_s_ref'])
                        optimize_s_upper_rt.append(msg_item[1]['optimizer_s_upper'])
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
                        svlimit_v_by_curvature_rt.append(msg_item[1]['svlimit_v_by_curvature'])
        # plot traj info
        time_rt = []
        gear_rt = []
        traj_state_rt = []
        traj_mode_rt = []
        turn_sign_rt = []
        update_by_vehsta_rt = []
        timestamp_ns_rt = []
        traj_x_rt = []
        traj_y_rt = []
        traj_curv_rt = [] 
        traj_theta_rt = []
        traj_yawrate_rt = []
        traj_acc_rt = []
        traj_vel_rt = []
        traj_sum_rt = []
        for channel in self.data_set_.keys():
            if channel == PLANNING_CHANNEL:
                for data_item in self.data_set_[channel]:
                    for msg_item in data_item.items():
                        time_rt.append(msg_item[0])
                        gear_rt.append(msg_item[1]['gearEnum'])
                        traj_state_rt.append(msg_item[1]['trajState'])
                        traj_mode_rt.append(msg_item[1]['trajectoryMode'])
                        turn_sign_rt.append(msg_item[1]['turnSignalEnum'])
                        update_by_vehsta_rt.append(msg_item[1]['updatedByVehicleStatus'])
                        timestamp_ns_rt.append(msg_item[1]['trajTimestampNs'])
                        tmp_x = []
                        tmp_y = []
                        tmp_curv = []
                        tmp_theta = []
                        tmp_yawrate = []
                        tmp_acc = []
                        tmp_vel = []
                        tmp_sum = []
                        for traj_point in msg_item[1]['traj']:
                            tmp_x.append(traj_point['x'])
                            tmp_y.append(traj_point['y'])
                            tmp_curv.append(traj_point['curv'])  
                            tmp_theta.append(traj_point['theta'])
                            tmp_yawrate.append(traj_point['yawRate'])
                            tmp_acc.append(traj_point['acceleration'])
                            tmp_vel.append(traj_point['velocity'])
                            tmp_sum.append(traj_point['sumDistance'])
                        traj_x_rt.append(tmp_x)
                        traj_y_rt.append(tmp_y)
                        traj_curv_rt.append(tmp_curv)
                        traj_theta_rt.append(tmp_theta)
                        traj_yawrate_rt.append(tmp_yawrate)
                        traj_acc_rt.append(tmp_acc)
                        traj_vel_rt.append(tmp_vel)
                        traj_sum_rt.append(tmp_sum)
        print("*" * 30 + " start adding bag data to source " + "*" * 30)

        planning_origin_time_rt = []
        planning_origin_x = []
        planning_origin_y = []
        planning_origin_theta = []
        planning_origin_curve = []
        planning_origin_curve_x_axis = []
        for channel in self.data_set_.keys():
                    if channel == XDEBUG_CHANNEL:
                        for data_item in self.data_set_[channel]:
                            for msg_item in data_item.items():
                                if msg_item[0] == None:
                                    continue 
                                if msg_item[1] == None or len(msg_item[1]['planning_origin']) == 0:
                                    continue
                                planning_origin_time_rt.append(msg_item[0])
                                planning_origin_x.append(msg_item[1]['planning_origin']['planning_origin']['x'])
                                planning_origin_y.append(msg_item[1]['planning_origin']['planning_origin']['y'])
                                planning_origin_theta.append(msg_item[1]['planning_origin']['planning_origin']['heading'])
                                planning_origin_curve.append(msg_item[1]['planning_origin']['planning_origin']['curvature'])
                                planning_origin_curve_x_axis.append(0)



        self.planning_origin_source( planning_origin_time_rt, planning_origin_x, planning_origin_y)
        self.planning_origin_curve_source( planning_origin_time_rt, planning_origin_curve_x_axis, planning_origin_curve)

        self.add_layer_processed_map_scatter(self.figs_['fig1'], 'cur_lane_center_line_rt_source', 'cur_lane_center_line_rt', cur_lane_center_line_time_rt, cur_lane_center_line_x_rt, cur_lane_center_line_y_rt, 'Cur Lane Center', 4, 0.2, 'blue')
        self.add_layer_processed_map_scatter(self.figs_['fig1'], 'cur_lane_left_line_rt_source', 'cur_lane_left_line_rt', cur_lane_center_line_time_rt, cur_lane_left_line_x_rt, cur_lane_left_line_y_rt, 'Cur Lane Left', 4, 0.2, 'Green')
        self.add_layer_processed_map_scatter(self.figs_['fig1'], 'cur_lane_right_line_rt_source', 'cur_lane_right_line_rt', cur_lane_center_line_time_rt, cur_lane_right_line_x_rt, cur_lane_right_line_y_rt, 'Cur Lane Right', 4, 0.2, 'coral')
        self.add_layer_processed_map_line(self.figs_['fig1'], 'cur_lane_left_line_rt_source', 'cur_lane_left_line_rt', cur_lane_center_line_time_rt, cur_lane_left_line_x_rt, cur_lane_left_line_y_rt, 'Cur Lane Left', 4, 0.2, 'Green')
        self.add_layer_processed_map_line(self.figs_['fig1'], 'cur_lane_right_line_rt_source', 'cur_lane_right_line_rt', cur_lane_center_line_time_rt, cur_lane_right_line_x_rt, cur_lane_right_line_y_rt, 'Cur Lane Right', 4, 0.2, 'coral')
        self.add_layer_processed_map_scatter(self.figs_['fig1'], 'left_lane_center_line_rt_source', 'left_lane_center_line_rt', left_lane_center_line_time_rt, left_lane_center_line_x_rt, left_lane_center_line_y_rt, 'Left Lane Center', 4, 0.2, 'hotpink')
        self.add_layer_processed_map_scatter(self.figs_['fig1'], 'right_lane_center_line_rt_source', 'right_lane_center_line_rt', right_lane_center_line_time_rt, right_lane_center_line_x_rt, right_lane_center_line_y_rt, 'Right Lane Center', 4, 0.2, 'indianred')

        self.add_layer_processed_map_line(self.figs_['lane_center_dist_to_boundary'], 'cur_lane_center_dist_to_left_bound_rt_source', 'cur_lane_center_dist_to_left_bound_rt', cur_lane_center_dist_to_bound_time_rt, cur_lane_center_dist_to_left_bound_rt, cur_lane_center_dist_to_bound_index_rt, 'Dist to Left Bound', 4, 0.2, 'Green')
        self.add_layer_processed_map_line(self.figs_['lane_center_dist_to_boundary'], 'cur_lane_center_dist_to_right_bound_rt_source', 'cur_lane_center_dist_to_right_bound_rt', cur_lane_center_dist_to_bound_time_rt, cur_lane_center_dist_to_right_bound_rt, cur_lane_center_dist_to_bound_index_rt, 'Dist to Right Bound', 4, 0.2, 'coral')

        # self.add_layer_processed_map_scatter(self.figs_['fig1'], 'cur_lane_center_line_rt_source_enu', 'cur_lane_center_line_rt_enu', cur_lane_center_line_time_rt_enu, cur_lane_center_line_x_rt_enu, cur_lane_center_line_y_rt_enu, 'Lane Center ENU', 4, 0.2, 'red')

        self.add_layer_predict_traj_scatter(predict_obj_time_rt, predict_obj_x_rt, predict_obj_y_rt)
        self.add_layer_planning_traj_scatter(planning_traj_time_rt,planning_traj_x_rt,planning_traj_y_rt)

        # s-traj_r_upper
        self.add_layer_scatter(time_list_=planning_traj_time_rt,x_values_=traj_r_upper_x,y_values_=traj_r_upper_y,y_key='traj_r_upper', 
                                         source_key='traj_r_upper_source', figure_key='fig1', legend_label='traj_r_upper',xaxis_lable="",yaxis_lable="",
                                         color_='brown')
        # s-traj_r_lower
        self.add_layer_scatter(time_list_=planning_traj_time_rt,x_values_=traj_r_lower_x,y_values_=traj_r_lower_y,y_key='traj_r_lower', 
                                         source_key='traj_r_lower_source', figure_key='fig1', legend_label='traj_r_lower',xaxis_lable="",yaxis_lable="",
                                         color_='black')
        # s-traj_r_ref
        self.add_layer_scatter(time_list_=planning_traj_time_rt,x_values_=traj_r_ref_x,y_values_=traj_r_ref_y,y_key='traj_r_ref', 
                                         source_key='traj_r_ref_source', figure_key='fig1', legend_label='traj_r_ref',xaxis_lable="",yaxis_lable="",
                                         color_='green')
        # path bound high
        self.add_layer_scatter(time_list_=path_bound_time_rt,x_values_=path_bound_high_x_rt,y_values_=path_bound_high_y_rt,y_key='path_bound_high', 
                                         source_key='path_bound_high_source', figure_key='fig1', legend_label='path_bound_high',xaxis_lable="",yaxis_lable="",
                                         color_='Green')

        # path bound lower
        self.add_layer_scatter(time_list_=path_bound_time_rt,x_values_=path_bound_low_x_rt,y_values_=path_bound_low_y_rt,y_key='path_bound_lower', 
                                         source_key='path_bound_lower_source', figure_key='fig1', legend_label='path_bound_lower',xaxis_lable="",yaxis_lable="",
                                         color_='orange')

        
        self.add_layer_ego_obj_point(ego_obj_time_rt, ego_obj_x_rt, ego_obj_y_rt)

        self.add_layer_polygons(self.figs_['fig1'], 'obj_polygons_rt_source', 'obj_polygons_rt', predict_obj_time_rt, obj_polygon_xs_rt, obj_polygon_ys_rt, 0,'obj_polygons', 4, 0.4, 'red')
        # plot nudge obj
        self.add_layer_polygons(self.figs_['fig1'], 'nudge_obj_polygons_rt_source', 'nudge_obj_polygons_rt', nudge_obj_time_rt, nudge_obj_polygon_xs_rt, nudge_obj_polygon_ys_rt, 0.5,'nudge_obj_polygons', 4, 0.4, 'red')

        # s-v-limit(lane points,v_upper)
        self.add_layer_scatter(time_list_=svlimit_time_rt, x_values_=svlimit_s_rt, y_values_=svlimit_v_upper_rt,y_key='sum_vel_upper', 
                                         source_key='sum_vel_upper_rt_source', figure_key='svlimit', legend_label='v_upper',xaxis_lable="x/m",yaxis_lable="m/s",
                                         color_='black')
        # s-v-limit(lane points,v_lower)
        self.add_layer_scatter(time_list_=svlimit_time_rt, x_values_=svlimit_s_rt, y_values_=svlimit_v_lower_rt,y_key='sum_vel_lower', 
                                         source_key='sum_vel_lower_rt_source', figure_key='svlimit', legend_label='v_lower',xaxis_lable="x/m",yaxis_lable="m/s",
                                         color_='green')
        # s-v-limit(cacluate,v_ref)
        self.add_layer_scatter(time_list_=svlimit_time_rt, x_values_=svlimit_s_rt, y_values_=svlimit_ref_sv_v_rt,y_key='sum_vel_ref', 
                                    source_key='sum_vel_ref_rt_source', figure_key='svlimit', legend_label='v_ref',xaxis_lable="x/m",yaxis_lable="m/s",
                                    color_='red')

        # s-v-limit(cacluate,v_cuvature)
        self.add_layer_scatter(time_list_=svlimit_time_rt, x_values_=svlimit_s_rt, y_values_=svlimit_v_by_curvature_rt,y_key='sum_vel_curvature', 
                                    source_key='sum_vel_curvature_rt_source', figure_key='svlimit', legend_label='v_cuvature',xaxis_lable="x/m",yaxis_lable="m/s",
                                    color_='orange')
        # 纵向优化s-v out
        self.add_layer_scatter(time_list_=svlimit_time_rt, x_values_=optimize_s_out_rt, y_values_=optimize_v_out_rt,y_key='optimizer_s_v_rt', 
                                         source_key='optimizer_s_v_rt_source', figure_key='svlimit', legend_label='V-S-OUT',xaxis_lable="x/m",yaxis_lable="m/s",
                                         color_='black')
        # leader s-v
        self.add_layer_scatter(time_list_=svlimit_time_rt, x_values_=s_leader_vector_rt, y_values_=v_leader_vector_rt,y_key='leader_s_v_rt', 
                                         source_key='leader_s_v_rt_source', figure_key='svlimit', legend_label='leader_v-s',xaxis_lable="x/m",yaxis_lable="m/s",
                                         color_='blue')
        # trajectory s-theta
        max_val = max(max(planning_traj_theta_rt))
        min_val = min(min(planning_traj_theta_rt))
        self.figs_['traj_theta'].y_range.start = min_val - 1
        self.figs_['traj_theta'].y_range.end = max_val + 1
        self.add_layer_scatter(time_list_=planning_traj_time_rt,x_values_=planning_traj_s_rt,y_values_=planning_traj_theta_rt,y_key='traj_theta', 
                                         source_key='traj_theta_rt_source', figure_key='traj_theta', legend_label='traj_theta',xaxis_lable="s/m",yaxis_lable="heading/deg",
                                         color_='green')
        # trajectory index-traj_theta_error_ref
        self.add_layer_scatter(time_list_=planning_traj_time_rt,x_values_=traj_theta_error_ref_index,y_values_=traj_theta_error_ref,y_key='traj_theta_error_ref', 
                                         source_key='traj_theta_error_ref_rt_source', figure_key='traj_theta_error', legend_label='traj_theta_error_ref',xaxis_lable="index",yaxis_lable="heading/rad",
                                         color_='green')
        # trajectory index-d
        self.add_layer_scatter(time_list_=planning_traj_time_rt,x_values_=traj_theta_error_ref_index,y_values_=planning_traj_d_rt,y_key='traj_d', 
                                         source_key='traj_d_rt_source', figure_key='traj_d', legend_label='traj_d',xaxis_lable="index",yaxis_lable="d/m",
                                         color_='red')    
        # trajectory index-ref_d
        self.add_layer_scatter(time_list_=planning_traj_time_rt,x_values_=traj_theta_error_ref_index,y_values_=planning_traj_ref_d_rt,y_key='traj_ref_d', 
                                         source_key='traj_ref_d_rt_source', figure_key='traj_d', legend_label='traj_ref_d',xaxis_lable="index",yaxis_lable="d/m",
                                         color_='green')
        # trajectory index-ref_delta
        self.add_layer_scatter(time_list_=planning_traj_time_rt,x_values_=traj_theta_error_ref_index,y_values_=planning_traj_ref_delta_rt,y_key='traj_ref_delta', 
                                         source_key='traj_ref_delta_rt_source', figure_key='traj_delta', legend_label='traj_ref_delta',xaxis_lable="index",yaxis_lable="angle/rad",
                                         color_='green')
        
        # trajectory index-delta
        self.add_layer_scatter(time_list_=planning_traj_time_rt,x_values_=traj_theta_error_ref_index,y_values_=planning_traj_delta_rt,y_key='traj_delta', 
                                         source_key='traj_delta_rt_source', figure_key='traj_delta', legend_label='traj_delta',xaxis_lable="index",yaxis_lable="angle/rad",
                                         color_='red')
        # trajectory index-curve
        self.add_layer_scatter(time_list_=planning_traj_time_rt,x_values_=traj_theta_error_ref_index,y_values_=planning_traj_curve_rt,y_key='traj_curve', 
                                         source_key='traj_curve_rt_source', figure_key='traj_curve', legend_label='traj_curve',xaxis_lable="index",yaxis_lable="1/m/rad",
                                         color_='red')
        # t-nop_counter                               
        self.add_time_scatter(time_list_=planning_traj_time_rt, y_values_=nop_counter_rt, y_key='nop_counter_time', 
                                    source_key='nop_counter_source', figure_key='nop_counter', legend_label='nop_counter-Time')

        # s-refs
        self.add_layer_scatter(time_list_=svlimit_time_rt, x_values_=svlimit_ref_st_t_rt, y_values_=svlimit_ref_st_s_rt,y_key='svlimit_ref_st_s', 
                                         source_key='svlimit_ref_st_s_rt_source', figure_key='svlimit_vt', legend_label='s_ref',xaxis_lable="t/s",yaxis_lable="m/s",
                                         color_='green')
        # v-refs
        self.add_layer_scatter(time_list_=svlimit_time_rt, x_values_=svlimit_ref_st_t_rt, y_values_=svlimit_ref_vt_v_rt,y_key='svlimit_ref_vt_v', 
                                         source_key='svlimit_ref_vt_v_rt_source', figure_key='svlimit_vt', legend_label='v_ref',xaxis_lable="t/s",yaxis_lable="m/s",
                                         color_='red')
        # a-refs
        self.add_layer_scatter(time_list_=svlimit_time_rt, x_values_=svlimit_ref_st_t_rt, y_values_=svlimit_ref_at_a_rt,y_key='svlimit_ref_at_a', 
                                         source_key='svlimit_ref_at_a_rt_source', figure_key='svlimit_vt', legend_label='a_ref',xaxis_lable="t/s",yaxis_lable="m/s",
                                         color_='blue')
        # s-refs-follow(idm)
        self.add_layer_scatter(time_list_=svlimit_time_rt, x_values_=svlimit_ref_st_t_rt, y_values_=s_ref_follow_vector_rt,y_key='idm_ref_st_s', 
                                         source_key='idm_ref_st_s_rt_source', figure_key='svlimit_vt', legend_label='s_ref_idm',xaxis_lable="t/s",yaxis_lable="m/s",
                                         color_='lime')
        # v-refs-follow(idm)
        self.add_layer_scatter(time_list_=svlimit_time_rt, x_values_=svlimit_ref_st_t_rt, y_values_=v_ref_follow_vector_rt,y_key='idm_ref_vt_v', 
                                         source_key='idm_ref_vt_v_rt_source', figure_key='svlimit_vt', legend_label='v_ref_idm',xaxis_lable="t/s",yaxis_lable="m/s",
                                         color_='coral')
        # a-refs-follow(idm)
        self.add_layer_scatter(time_list_=svlimit_time_rt, x_values_=svlimit_ref_st_t_rt, y_values_=a_ref_follow_vector_rt,y_key='idm_ref_at_a', 
                                         source_key='idm_ref_at_a_rt_source', figure_key='svlimit_vt', legend_label='a_ref_idm',xaxis_lable="t/s",yaxis_lable="m/s",
                                         color_='deeppink')

        # 纵向优化s-out
        self.add_traj_layer_scatter(time_list_=svlimit_time_rt, y_values_=optimize_s_out_rt, y_key='optimizer_s_rt', 
                                    source_key='optimizer_s_rt_source', figure_key='osqp_s_t', legend_label='S-Time')
        # 纵向优化s-upper
        self.add_traj_layer_scatter(time_list_=svlimit_time_rt, y_values_=optimize_s_upper_rt, y_key='optimizer_s_upper_rt', 
                                    source_key='optimizer_s_upper_rt_source', figure_key='osqp_s_t', legend_label='S-UPPER',color_='blue')
        # 纵向优化s-ref
        self.add_traj_layer_scatter(time_list_=svlimit_time_rt, y_values_=optimize_s_ref_rt, y_key='optimizer_s_ref_rt', 
                                    source_key='optimizer_s_ref_rt_source', figure_key='osqp_s_t', legend_label='S-REF',color_='red')

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
        # 轨迹速度随时间变化
        self.add_traj_layer_scatter(time_list_=time_rt, y_values_=traj_vel_rt, y_key='vel_time', 
                                    source_key='vel_time_source', figure_key='traj_vel', legend_label='Vel-Time')
        # 轨迹s-v的关系
        self.add_layer_scatter(time_list_=time_rt, x_values_=traj_sum_rt, y_values_=traj_vel_rt,y_key='sum_vel_rt', 
                                         source_key='sum_vel_rt_source', figure_key='traj_s_v', legend_label='V-S',xaxis_lable="x/m",yaxis_lable="m/s",
                                         color_='black')
        # 轨迹s-a的关系
        self.add_layer_scatter(time_list_=time_rt, x_values_=traj_sum_rt, y_values_=traj_acc_rt,y_key='sum_acc_rt', 
                                         source_key='sum_acc_rt_source', figure_key='traj_s_a', legend_label='A-S',xaxis_lable="x/m",yaxis_lable="m/^s",
                                         color_='black')
        
        # plot lc longi search debug
        lon_search_debug_time_rt, st_path_t_rt, st_path_s_rt, \
            st_sample_points_t_rt, st_sample_points_s_rt, st_sample_points_a_rt, \
            st_sample_points_v_rt, st_sample_points_cost_rt, st_obj_polygon_xs_rt, \
            st_obj_polygon_ys_rt, vt_path_t_rt, vt_path_v_rt, vt_limit_t_rt, vt_limit_v_rt, \
            st_sample_points_color_rt = self.get_lon_search_debug_info('lc_lon_search_debug')

        self.add_lon_search_st_graph('lc_lon_search_st', '_lc_decider', lon_search_debug_time_rt, st_path_t_rt, st_path_s_rt, \
            st_sample_points_t_rt, st_sample_points_s_rt, st_sample_points_a_rt, \
            st_sample_points_v_rt, st_sample_points_cost_rt, st_obj_polygon_xs_rt, \
            st_obj_polygon_ys_rt, vt_path_t_rt, vt_path_v_rt, vt_limit_t_rt, vt_limit_v_rt, \
            st_sample_points_color_rt)
        
        # plot lc lat search debug
        lat_search_debug_time_rt, \
            ds_bound_s_rt, ds_bound_d_rt, \
            ds_range_d_rt, ds_goal_d_rt, ds_ego_s_rt, \
            ds_ego_d_rt, ds_traj_s_rt, ds_traj_d_rt = \
                self.get_lat_search_debug_info('lc_lat_search_debug')
        
        self.add_lat_search_sd_graph('lc_lat_search_sd', '_lc_decider', lat_search_debug_time_rt, \
            ds_bound_s_rt, ds_bound_d_rt, ds_range_d_rt, ds_goal_d_rt, ds_ego_s_rt, \
            ds_ego_d_rt, ds_traj_s_rt, ds_traj_d_rt)
        
        self.add_slider()
        self.add_rt_callback()

        self.activate_figure_option()

        self.show2html()
        # self.show2notebook()

    def show2html(self):
        output_file(self.output_)
        row1 = row(column(self.figs_['fig1'], 
                          self.checkbox_groups_['lane_center_dist_to_boundary'],
                          self.figs_['lane_center_dist_to_boundary']),
                   column(self.checkbox_groups_['svlimit'],self.figs_['svlimit'],
                          self.checkbox_groups_['svlimit_vt'],self.figs_['svlimit_vt'],
                          self.checkbox_groups_['traj_theta'],self.figs_['traj_theta'],
                          self.checkbox_groups_['traj_d'],self.figs_['traj_d'],
                          self.checkbox_groups_['traj_theta_error'],self.figs_['traj_theta_error'],
                          self.checkbox_groups_['traj_delta'],self.figs_['traj_delta'],
                          self.checkbox_groups_['lc_lon_search_st'], 
                          self.figs_['lc_lon_search_st'],),
                   column(self.checkbox_groups_['osqp_s_t'],self.figs_['osqp_s_t'],
                          self.checkbox_groups_['osqp_v_t'],self.figs_['osqp_v_t'],
                          self.checkbox_groups_['traj_s_v'],self.figs_['traj_s_v'],
                          self.checkbox_groups_['traj_s_a'],self.figs_['traj_s_a'],
                          self.checkbox_groups_['traj_curve'],self.figs_['traj_curve'],
                          self.checkbox_groups_['nop_counter'],self.figs_['nop_counter'],
                          self.checkbox_groups_['lc_lat_search_sd'], 
                          self.figs_['lc_lat_search_sd'],),
                   column(self.checkbox_groups_['osqp_a_t'],self.figs_['osqp_a_t'],
                          self.checkbox_groups_['osqp_jerk_t'],self.figs_['osqp_jerk_t'],
                          self.checkbox_groups_['traj_vel'],self.figs_['traj_vel'],
                          self.checkbox_groups_['traj_update'],self.figs_['traj_update']))
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

    def get_lon_search_debug_info(self, json_key):
        lon_search_debug_time_rt = []
        st_path_t_rt = []
        st_path_s_rt = []
        st_sample_points_t_rt = []
        st_sample_points_s_rt = []
        st_sample_points_a_rt = []
        st_sample_points_v_rt = []
        st_sample_points_cost_rt = []
        st_obj_polygon_xs_rt = [] 
        st_obj_polygon_ys_rt = []  
        vt_path_t_rt = []
        vt_path_v_rt = []
        vt_limit_t_rt = []
        vt_limit_v_rt = []
        for channel in self.data_set_.keys():
            if channel == XDEBUG_CHANNEL:
                print_once = False
                for data_item in self.data_set_[channel]:
                    for msg_item in data_item.items():
                        if msg_item[0] == None:
                            continue 
                        if msg_item[1] == None or (json_key not in msg_item[1]) \
                            or len(msg_item[1][json_key]) == 0:
                            if not print_once:
                                print_once = True
                                print("[Warnning]:{} not exist".format(json_key))
                            continue
                        
                        lon_search_debug_time_rt.append(msg_item[0])

                        tmp_st_path_t_rt = []
                        tmp_st_path_s_rt = []
                        for tmp_point in msg_item[1][json_key]['st_path']:
                            tmp_st_path_t_rt.append(tmp_point['t'])
                            tmp_st_path_s_rt.append(tmp_point['s'])
                        st_path_t_rt.append(tmp_st_path_t_rt)
                        st_path_s_rt.append(tmp_st_path_s_rt)

                        tmp_st_sample_points_t_rt = []
                        tmp_st_sample_points_s_rt = []
                        tmp_st_sample_points_a_rt = []
                        tmp_st_sample_points_v_rt = []
                        tmp_st_sample_points_cost_rt = []
                        for tmp_point in msg_item[1][json_key]['sample_points']:
                            tmp_st_sample_points_t_rt.append(tmp_point['point']['t'])
                            tmp_st_sample_points_s_rt.append(tmp_point['point']['s'])
                            tmp_st_sample_points_v_rt.append(tmp_point['point']['v'])
                            tmp_st_sample_points_a_rt.append(tmp_point['point']['a'])
                            tmp_st_sample_points_cost_rt.append(tmp_point['cost'])
                        st_sample_points_t_rt.append(tmp_st_sample_points_t_rt)
                        st_sample_points_s_rt.append(tmp_st_sample_points_s_rt)
                        st_sample_points_a_rt.append(tmp_st_sample_points_a_rt)
                        st_sample_points_v_rt.append(tmp_st_sample_points_v_rt)
                        st_sample_points_cost_rt.append(tmp_st_sample_points_cost_rt)

                        tmp_st_obj_polygon_xs_rt = [] 
                        tmp_st_obj_polygon_ys_rt = []
                        for tmp_obj in msg_item[1][json_key]['st_obstacle']:
                            tmp_st_obj_polygon_x_rt = [] 
                            tmp_st_obj_polygon_y_rt = [] 
                            for tmp_point in tmp_obj['polygon']:
                                tmp_st_obj_polygon_x_rt.append(tmp_point['x'])
                                tmp_st_obj_polygon_y_rt.append(tmp_point['y'])
                            tmp_st_obj_polygon_xs_rt.append(tmp_st_obj_polygon_x_rt)
                            tmp_st_obj_polygon_ys_rt.append(tmp_st_obj_polygon_y_rt)
                        st_obj_polygon_xs_rt.append(tmp_st_obj_polygon_xs_rt)
                        st_obj_polygon_ys_rt.append(tmp_st_obj_polygon_ys_rt)

                        tmp_vt_path_t_rt = []
                        tmp_vt_path_v_rt = []
                        for tmp_point in msg_item[1][json_key]['st_path']:
                            tmp_vt_path_t_rt.append(tmp_point['t'])
                            tmp_vt_path_v_rt.append(tmp_point['v'])
                        vt_path_t_rt.append(tmp_vt_path_t_rt)
                        vt_path_v_rt.append(tmp_vt_path_v_rt)

                        tmp_vt_limit_t_rt = []
                        tmp_vt_limit_v_rt = []
                        for tmp_point in msg_item[1][json_key]['vt_limit']:
                            tmp_vt_limit_t_rt.append(tmp_point['x'])
                            tmp_vt_limit_v_rt.append(tmp_point['y'])
                        vt_limit_t_rt.append(tmp_vt_limit_t_rt)
                        vt_limit_v_rt.append(tmp_vt_limit_v_rt)
        st_sample_points_color_rt = []
        for st_sample_points_cost_single_frame in st_sample_points_cost_rt:
            tmp_st_sample_points_color_rt = []
            if len(st_sample_points_cost_single_frame) > 0:
                try:
                    tmp_pts = [i for i in st_sample_points_cost_single_frame if not i == None]
                    if len(tmp_pts) > 0:
                        minima = min(tmp_pts)
                        maxima = max(tmp_pts)
                    else:
                        minima = 0.0
                        maxima = 1e5
                except TypeError:
                    print(TypeError)
                    print("st_sample_points_cost_single_frame = ", st_sample_points_cost_single_frame)
                    minima = 0.0
                    maxima = 1e5
                
                norm = matplotlib.colors.Normalize(vmin=minima, vmax=maxima, clip=True)
                mapper = cm.ScalarMappable(norm=norm, cmap=cm.get_cmap("Wistia", 256))
                for v in st_sample_points_cost_single_frame:
                    if v == None:
                        color = matplotlib.colors.rgb2hex(mapper.to_rgba(minima))
                    else:
                        color = matplotlib.colors.rgb2hex(mapper.to_rgba(v))
                    tmp_st_sample_points_color_rt.append(color)
            st_sample_points_color_rt.append(tmp_st_sample_points_color_rt)

        return lon_search_debug_time_rt, st_path_t_rt, st_path_s_rt, \
                st_sample_points_t_rt, st_sample_points_s_rt, st_sample_points_a_rt, \
                st_sample_points_v_rt, st_sample_points_cost_rt, st_obj_polygon_xs_rt, \
                st_obj_polygon_ys_rt, vt_path_t_rt, vt_path_v_rt, vt_limit_t_rt, vt_limit_v_rt, \
                st_sample_points_color_rt
    
    def get_lat_search_debug_info(self, json_key):
        lat_search_debug_time_rt = []
        ds_bound_s_rt = [] 
        ds_bound_d_rt = []
        ds_range_d_rt = []
        ds_goal_d_rt = []
        ds_ego_s_rt = [] 
        ds_ego_d_rt = []
        ds_traj_s_rt = [] 
        ds_traj_d_rt = []
        for channel in self.data_set_.keys():
            if channel == XDEBUG_CHANNEL:
                print_once = False
                for data_item in self.data_set_[channel]:
                    for msg_item in data_item.items():
                        if msg_item[0] == None:
                            continue 
                        if msg_item[1] == None or (json_key not in msg_item[1]) \
                            or len(msg_item[1][json_key]) == 0:
                            if not print_once:
                                print_once = True
                                print("[Warnning]:{} not exist".format(json_key))
                            continue
                        
                        lat_search_debug_time_rt.append(msg_item[0])

                        tmp_ds_bound_s_rt = []
                        tmp_ds_bound_d_rt = []
                        bound = msg_item[1][json_key]['ds_bound']
                        if len(bound['s_seq']) != 0 and len(bound['s_seq']) == len(bound['d_low_seq']) and len(bound['s_seq']) == len(bound['d_high_seq']):
                            for i in range(len(bound['s_seq'])):
                                tmp_ds_bound_s_rt.append(bound['s_seq'][i])
                                tmp_ds_bound_d_rt.append(bound['d_low_seq'][i])
                            for i in range(len(bound['s_seq'])):
                                tmp_ds_bound_s_rt.append(bound['s_seq'][i])
                                tmp_ds_bound_d_rt.append(bound['d_high_seq'][i])
                        ds_bound_s_rt.append(tmp_ds_bound_s_rt)
                        ds_bound_d_rt.append(tmp_ds_bound_d_rt)

                        tmp_ds_range_d_rt = []
                        for i in range(len(bound['s_seq'])):
                            tmp_ds_range_d_rt.append(msg_item[1][json_key]['ds_range']['d_low'])
                        for i in range(len(bound['s_seq'])):
                            tmp_ds_range_d_rt.append(msg_item[1][json_key]['ds_range']['d_high'])
                        ds_range_d_rt.append(tmp_ds_range_d_rt)

                        tmp_ds_goal_d_rt = []
                        for pt in msg_item[1][json_key]['lateral_goal']:
                            tmp_ds_goal_d_rt.append(pt['y'])
                        ds_goal_d_rt.append(tmp_ds_goal_d_rt)

                        tmp_ds_ego_s_rt = []
                        tmp_ds_ego_d_rt = []
                        tmp_ds_ego_s_rt.append(msg_item[1][json_key]['origin_pos']['x'])
                        tmp_ds_ego_d_rt.append(msg_item[1][json_key]['origin_pos']['y'])
                        ds_ego_s_rt.append(tmp_ds_ego_s_rt)
                        ds_ego_d_rt.append(tmp_ds_ego_d_rt)

                        tmp_ds_traj_s_rt = []
                        tmp_ds_traj_d_rt = []
                        for pt in msg_item[1][json_key]['ref_points']:
                            tmp_ds_traj_s_rt.append(pt['x'])
                            tmp_ds_traj_d_rt.append(pt['y'])
                        ds_traj_s_rt.append(tmp_ds_traj_s_rt)
                        ds_traj_d_rt.append(tmp_ds_traj_d_rt)
        
        return lat_search_debug_time_rt, \
                ds_bound_s_rt, ds_bound_d_rt, \
                ds_range_d_rt, ds_goal_d_rt, ds_ego_s_rt, \
                ds_ego_d_rt, ds_traj_s_rt, ds_traj_d_rt

    def add_lat_search_sd_graph(self, fig_name, source_suffix, lat_search_debug_time_rt, \
                ds_bound_s_rt, ds_bound_d_rt, ds_range_d_rt, ds_goal_d_rt, ds_ego_s_rt, \
                ds_ego_d_rt, ds_traj_s_rt, ds_traj_d_rt):
        self.add_layer_processed_map_scatter(self.figs_[fig_name], 'ds_bound_rt_source' + source_suffix, 'ds_bound_rt' + source_suffix, lat_search_debug_time_rt, ds_bound_d_rt, ds_bound_s_rt, 'ds_bound', 4, 0.2, 'red')
        self.add_layer_processed_map_scatter(self.figs_[fig_name], 'ds_range_rt_source' + source_suffix, 'ds_range_rt' + source_suffix, lat_search_debug_time_rt, ds_range_d_rt, ds_bound_s_rt, 'ds_range', 4, 0.2, 'blueviolet')
        self.add_layer_processed_map_scatter(self.figs_[fig_name], 'ds_goal_rt_source' + source_suffix, 'ds_goal_rt' + source_suffix, lat_search_debug_time_rt, ds_goal_d_rt, ds_bound_s_rt, 'ds_goal', 4, 0.2, 'burlywood')
        self.add_layer_processed_map_scatter(self.figs_[fig_name], 'ds_ego_rt_source' + source_suffix, 'ds_ego_rt' + source_suffix, lat_search_debug_time_rt, ds_ego_d_rt, ds_ego_s_rt, 'ds_ego', 10, 4, 'cadetblue')
        self.add_layer_processed_map_scatter(self.figs_[fig_name], 'ds_traj_rt_source' + source_suffix, 'ds_traj_rt' + source_suffix, lat_search_debug_time_rt, ds_traj_d_rt, ds_traj_s_rt, 'ds_traj', 4, 0.2, 'cyan')

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

    xpilot_plot = PlotXpilotViz(args.source,args.output)

    xpilot_plot.add_required_channel(XDEBUG_CHANNEL)
    xpilot_plot.add_required_channel(PLANNING_CHANNEL)

    xpilot_plot.figure_plot()
