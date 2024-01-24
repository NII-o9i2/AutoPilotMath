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


class PlotBase:
    def __init__(self,bag_path,output) -> None:
        self.bag_path_ = bag_path
        self.output_ = output
        self.data_set_ = dict()
        self.figs_ = dict()
        self.plot_data_set_ = dict()
        # self.color_mapper_ = linear_cmap(field_name='color', 
        #                                       palette=Magma256, 
        #                                       low=0.0, 
        #                                       high=1e7)
        self.required_channel_list_ = []
        self.read_bag_flag_ = False
        self.checkbox_groups_ = {}
        pass

    def add_required_channel(self,channel):
        self.required_channel_list_.append(channel)

    def read_bag(self):
        print("*" * 30 + " begin py bag reader " + "*" * 30)
        attr = rsclpy.BagReaderAttribute()
        attr.included_channels = set(self.required_channel_list_)
        bag_reader = rsclpy.BagReader(self.bag_path_,attr)
        if not bag_reader.is_valid():
            print("Error: bag is invalid! Exit")
            return
        bag_header = bag_reader.get_bag_header()

        self.bag_begin_time_sec_ = bag_header.begin_time * 1e-9
        self.end_time_sec_ = bag_header.end_time * 1e-9

        print("bag time size: ", - self.bag_begin_time_sec_ + self.end_time_sec_)
        
        channel_list = bag_reader.get_channel_list()
        for required_channel in self.required_channel_list_:
            if required_channel not in channel_list:
                print("Error: required topic ",required_channel, " is missing!")
            self.data_set_[required_channel] = []

        while True:
            tmp_msg = bag_reader.read_next_message()
            if tmp_msg is None:
                break
            if tmp_msg.channel_name not in self.required_channel_list_:
                continue

            tmp_data = msg_callback_map[tmp_msg.channel_name](tmp_msg.message_json)
   
            self.data_set_[tmp_msg.channel_name].append({tmp_msg.timestamp * 1e-9:tmp_data})

        self.read_bag_flag_ = True
        # x_debug_begin = list(self.data_set_['/xpilot/xdebug'][0].keys())
        # x_debug_end = list(self.data_set_['/xpilot/xdebug'][-1].keys())
        # total = x_debug_end[0] - x_debug_begin[0]
        return
    
    def match_timestamp(self):
        begin_time_rt = -1
        end_time_rt = -1
        for chanel in self.required_channel_list_:
            if len(self.data_set_[chanel]) > 0:
                for time_key in  self.data_set_[chanel][0]:
                    if begin_time_rt < 0 or time_key < begin_time_rt:
                        begin_time_rt = time_key
                        break
                    break
                for time_key in  self.data_set_[chanel][-1]:
                    if end_time_rt < 0 or time_key > end_time_rt:
                        end_time_rt = time_key
                        break
                    break
        self.bag_time_length_ = end_time_rt  - begin_time_rt
        # change time to relative time
        for chanel in self.required_channel_list_:
            for msg_item in self.data_set_[chanel]:
                for time_key in  msg_item:
                    old_time = time_key
                    new_time = old_time - begin_time_rt
                    value = msg_item.pop(old_time)
                    msg_item[new_time] = value
                    break
        print("*" * 30 + " finish relative time process " + "*" * 30)
        return

    def creat_figure(self, wid = 1200,hei = 800, title_ = 'Default title'):
        self.figs_['fig'] = figure(width=wid, height=hei,title = title_)

    def add_layer_point(self,name_,time_list_,x_list_,y_list_,size_ = 10, color_ = 'red', alpha_ = 0.8):
        self.plot_data_set_[name_] = ColumnDataSource(data=dict(
            time = time_list_,
            x = x_list_,
            y = y_list_,
        ))
        self.figs_['fig'].circle('x','y',source=self.plot_data_set_[name_], size=size_, 
                       color = color_ ,legend_label = name_,alpha = alpha_)
        return

    # plot line which include points (source_x_,source_y_)
    def add_layer_line(self,name_ , time_list_, source_x_,source_y_,line_width_ = 3,line_alpha_ = 0.4, color_ = 'black'):
        self.plot_data_set_[name_] = ColumnDataSource(data=dict(
            time = time_list_,
            x = source_x_,
            y = source_y_,
        ))
        self.figs_['fig'].line('x','y',source=self.plot_data_set_[name_], line_width=line_width_, 
                       color = color_ ,line_alpha=line_alpha_,legend_label = name_)

        return

    # plot points which include points (source_x_,source_y_)
    def add_layer_scatter(self,name_ , time_list_, source_x_,source_y_,line_width_ = 3,line_alpha_ = 0.6, color_ = 'blue'):
        self.plot_data_set_[name_] = ColumnDataSource(data=dict(
            x = source_x_,
            y = source_y_,
            time = time_list_,
        ))
        self.figs_['fig'].scatter('x','y',source=self.plot_data_set_[name_], line_width=line_width_, 
                          color = color_, alpha=line_alpha_,legend_label = name_)
        return

    def figure_plot(self):
        if self.read_bag_flag_  == False:
            self.read_bag()
            self.match_timestamp()
        
        self.creat_figure()

        # for test plot first cycle
        time = [0,0,0]
        test_x = [1,3,5]
        test_y = [2,4,6]

        self.add_layer_line('test_plot',time,test_x,test_y)
        self.add_layer_scatter('test_plot point',time,test_x,test_y)

        self.activate_figure_option()
        self.show2notebook()
        # self.show2html()
        return

    def activate_figure_option(self):
        for fig_name, fig in self.figs_.items():
            if fig is None:
                continue
            fig.toolbar.active_scroll = fig.select_one(WheelZoomTool)
            fig.legend.click_policy = 'hide'
        return

    def show2html(self):
        output_file(self.output_)
        show(self.figs_['fig'])
        return
    
    def show2notebook(self):
        output_notebook()
        show(self.figs_['fig'])
        return


class PlotHopViz(PlotBase):
    def __init__(self, bag_path,output):
        super().__init__(bag_path,output)

    def creat_figure(self):
        # main hmi window
        self.figs_['fig1'] = figure(width=750 ,height = 1500,x_range = [7.5,-7.5],y_range = [-5,55] ,match_aspect=True, title = 'Xviz')

        # final planning trajectory figure
        self.figs_['traj_curv'] = figure(width=600 ,height = 300, y_range = [-0.01, 0.05])
        self.figs_['traj_vel'] = figure(width=600 ,height = 300, y_range = [0, 30])
        self.figs_['traj_yawrate'] = figure(width=600 ,height = 300, y_range = [-0.06,0.06],title = 'traj_yawrate')
        self.figs_['traj_theta'] = figure(width=600 ,height = 300, y_range = [-0.25, 0.25],title = 'traj_theta')
        self.figs_['traj_s_v'] = figure(width=600 ,height = 300)
        self.figs_['traj_s_a'] = figure(width=600 ,height = 300, y_range = [-4,5])
        self.figs_['traj_update'] = figure(width=400 ,height = 250, y_range = [-1,2],title = 'traj_update')
        self.figs_['traj_gear'] = figure(width=400 ,height = 150, y_range = [-1,5])
        self.figs_['traj_sum_distance'] = figure(width=600 ,height = 300)
        self.figs_['leader_s_t'] = figure(width=800 ,height = 400)
        self.figs_['leader_v_t'] = figure(width=800 ,height = 400)
        
        # optimizer output trajectory
        self.figs_['optimizer_s_v'] = figure(width=600 ,height = 300)
        self.figs_['leader_s_v'] = figure(width=600 ,height = 300, title = 'leader_s_v')
        self.figs_['optimizer_vel'] = figure(width=600 ,height = 300)
        self.figs_['optimizer_acc'] = figure(width=600 ,height = 300)
        self.figs_['optimizer_jerk'] = figure(width=600 ,height = 300)
        self.figs_['curva_speed_limit'] = figure(width=600 ,height = 300, title = 'curva_speed')

        # dp search
        self.figs_['lon_search_st'] = figure(width=600 ,height = 300, x_axis_label='t/s', y_axis_label='s/m')
        self.figs_['lon_search_vt'] = figure(width=600 ,height = 300, x_axis_label='t/s', y_axis_label='v/m/s')
        self.figs_['lat_search_sd'] = figure(width=600 ,height = 300, x_range = [10,-10], x_axis_label='d/m', y_axis_label='s/m')

        self.figs_['lc_decider_lon_search_st'] = figure(width=600 ,height = 300, x_axis_label='t/s', y_axis_label='s/m')
        self.figs_['lc_decider_lat_search_sd'] = figure(width=600 ,height = 300, x_range = [10,-10], x_axis_label='d/m', y_axis_label='s/m')

        # vehicle state
        self.figs_['ego_vel'] = figure(width=400 ,height = 250, title = 'ego_vel')
        self.figs_['ego_accX'] = figure(width=400 ,height = 250, title = 'ego_accX')
        self.figs_['enable_auto'] = figure(width=400 ,height = 250, y_range = [-1,2],title = 'ego_auto_eanble')

        # Add the checkbox group to the dictionary
        for fig_name in ['fig1', 'traj_curv', 'traj_gear', 'traj_sum_distance', 'traj_vel',
                         'traj_update', 'ego_vel', 'traj_s_v', 'ego_accX', 'enable_auto',
                         'leader_s_t', 'leader_v_t', 'traj_s_a', 'optimizer_s_v', 'leader_s_v',
                         'optimizer_vel', 'optimizer_acc', 'optimizer_jerk',
                         'lon_search_st', 'lon_search_vt', 'lat_search_sd', 'lc_decider_lon_search_st', 'lc_decider_lat_search_sd',
                         'curva_speed_limit', 'traj_yawrate', 'traj_theta']:
            fig = self.figs_[fig_name]
            checkbox_group = CheckboxButtonGroup(labels=[fig_name], active=[0], width=600)
            callback = CustomJS(args=dict(fig=fig, checkbox_group=checkbox_group), code="""
                fig.visible = checkbox_group.active.includes(0);
            """)
            checkbox_group.js_on_change('active', callback)
            self.checkbox_groups_[fig_name] = checkbox_group

    # plot real-time planning trajectory line(for control)
    def add_layer_traj_line(self, time_list_, source_x_,source_y_,line_width_ = 3,line_alpha_ = 0.4, color_ = 'blue'):
        self.plot_data_set_['trajectory_rt_source'] = dict(
            time = time_list_,
            source_x = source_x_,
            source_y = source_y_,
        )
        self.plot_data_set_['trajectory_rt_line'] = ColumnDataSource(data=dict(
            x = [],
            y = [],
        ))
        self.figs_['fig1'].line('x','y',source=self.plot_data_set_['trajectory_rt_line'], 
                                color = color_ ,line_alpha=line_alpha_,legend_label = 'Trajectory Line Ctrl')
        points = {'x':[-0.9, 0.9, 0.9, -0.9], 'y':[-1.13, -1.13, 3.87, 3.87]}
        self.figs_['fig1'].patch(points['x'], points['y'], fill_alpha=0.5, line_color='red', legend_label='ego_car')
        return
    
    # 横纵坐标是一组数据点,通过滑块控制time来显示数据
    def add_layer_traj_time_scatter(self, time_list_, x_values_,y_values_, y_key, source_key, figure_key, 
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
        self.figs_[figure_key].xaxis.axis_label = "x/m"
        return
    
    # 横坐标是时间,纵坐标是value
    def add_layer_time_scatter(self, time_list_, y_values_, y_key, source_key, figure_key, 
                                    legend_label, line_width_=3, line_alpha_=0.4, color_='black'):
        if not isinstance(time_list_, list) or not isinstance(y_values_, list):
            raise ValueError("time_list_ and y_values_ must be lists.")
        if len(time_list_) != len(y_values_):
            raise ValueError("time_list_ and y_values_ must have the same length.")
        self.plot_data_set_[source_key] = dict(
            time=time_list_,
            y=y_values_,
        )
        self.plot_data_set_[y_key] = ColumnDataSource(data=dict(
            time=[],
            y=[],
        ))
        self.figs_[figure_key].scatter('time', 'y', source=self.plot_data_set_[source_key], 
                                       color=color_, line_alpha=line_alpha_, legend_label=legend_label)
        self.figs_[figure_key].xaxis.axis_label = "time"
        return
    
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
    
    # plot real-time object predict point which include points (source_x_,source_y_)
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

        self.figs_['fig1'].scatter('y','x',source=self.plot_data_set_['predict_trajectory_rt'], 
                                   color = color_ ,line_alpha=line_alpha_,legend_label = 'Predict trajectory Point')
        return 

    #  plot real-time obstacle center
    def add_layer_obs_point(self,time_list_,x_list_,y_list_,id_list_):
        self.plot_data_set_['obstacle_center_rt_source'] = dict(
            time = time_list_,
            source_x = x_list_,
            source_y = y_list_,
            source_id = id_list_
        )
        self.plot_data_set_['obstacle_center_rt'] = ColumnDataSource(data=dict(
            x = [],
            y = [],
            id = []
        ))
        self.obstacle_point_render_ = self.figs_['fig1'].circle('x','y',source=self.plot_data_set_['obstacle_center_rt'], size=10,
                                                       color = 'red' ,legend_label = 'Obstacle Point',alpha = 0.8)
        return
    
    # plot real-time dmpp refline points
    def add_layer_dmpp_scatter(self, fig, rt_source_name_, rt_name_, time_list_, 
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

        fig.scatter('y','x',source=self.plot_data_set_[rt_name_], 
                          color = color_ ,line_alpha=line_alpha_,legend_label = legend_, size=size_)
        return
    
    def add_layer_dmpp_line(self, fig, rt_source_name_, rt_name_, time_list_, 
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

        fig.line('y','x',source=self.plot_data_set_[rt_name_], 
                       color = color_ ,line_alpha=line_alpha_,legend_label = legend_, line_width=size_)
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
        ds_obs_polygon_xs_rt = [] 
        ds_obs_polygon_ys_rt = []
        ds_bound_s_rt = [] 
        ds_bound_d_rt = []
        ds_range_d_rt = []
        ds_goal_d_rt = []
        ds_ego_s_rt = [] 
        ds_ego_d_rt = []
        ds_longi_sample_s_rt = [] 
        ds_longi_sample_d_rt = []
        ds_lat_sample_s_rt = [] 
        ds_lat_sample_d_rt = []
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

                        tmp_ds_obs_polygon_xs_rt = [] 
                        tmp_ds_obs_polygon_ys_rt = []
                        for obs in msg_item[1][json_key]['ds_obstacle']:
                            if len(obs['s_seq']) == 0 or len(obs['s_seq']) != len(obs['d_start_seq']) or len(obs['s_seq']) != len(obs['d_end_seq']):
                                continue
                            tmp_ds_obs_polygon_x_rt = [] 
                            tmp_ds_obs_polygon_y_rt = [] 
                            for i in range(len(obs['s_seq'])):
                                tmp_ds_obs_polygon_x_rt.append(obs['s_seq'][i])
                                tmp_ds_obs_polygon_y_rt.append(obs['d_start_seq'][i])
                            for i in reversed(range(len(obs['s_seq']))):
                                tmp_ds_obs_polygon_x_rt.append(obs['s_seq'][i])
                                tmp_ds_obs_polygon_y_rt.append(obs['d_end_seq'][i])
                            tmp_ds_obs_polygon_xs_rt.append(tmp_ds_obs_polygon_x_rt)
                            tmp_ds_obs_polygon_ys_rt.append(tmp_ds_obs_polygon_y_rt)

                            # print(tmp_ds_obs_polygon_x_rt)
                            # print(tmp_ds_obs_polygon_y_rt)
                        ds_obs_polygon_xs_rt.append(tmp_ds_obs_polygon_xs_rt)
                        ds_obs_polygon_ys_rt.append(tmp_ds_obs_polygon_ys_rt)

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

                        tmp_ds_longi_sample_s_rt = []
                        tmp_ds_longi_sample_d_rt = []
                        for pt in msg_item[1][json_key]['longi_sample_points']:
                            tmp_ds_longi_sample_s_rt.append(pt['x'])
                            tmp_ds_longi_sample_d_rt.append(pt['y'])
                        ds_longi_sample_s_rt.append(tmp_ds_longi_sample_s_rt)
                        ds_longi_sample_d_rt.append(tmp_ds_longi_sample_d_rt)

                        tmp_ds_lat_sample_s_rt = []
                        tmp_ds_lat_sample_d_rt = []
                        for pt in msg_item[1][json_key]['lat_sample_points']:
                            tmp_ds_lat_sample_s_rt.append(pt['x'])
                            tmp_ds_lat_sample_d_rt.append(pt['y'])
                        ds_lat_sample_s_rt.append(tmp_ds_lat_sample_s_rt)
                        ds_lat_sample_d_rt.append(tmp_ds_lat_sample_d_rt)

                        tmp_ds_traj_s_rt = []
                        tmp_ds_traj_d_rt = []
                        for pt in msg_item[1][json_key]['picked_polynomial_points']:
                            tmp_ds_traj_s_rt.append(pt['x'])
                            tmp_ds_traj_d_rt.append(pt['y'])
                        ds_traj_s_rt.append(tmp_ds_traj_s_rt)
                        ds_traj_d_rt.append(tmp_ds_traj_d_rt)
        
        return lat_search_debug_time_rt, ds_obs_polygon_xs_rt, ds_obs_polygon_ys_rt, \
                ds_bound_s_rt, ds_bound_d_rt, ds_range_d_rt, ds_goal_d_rt, ds_ego_s_rt, \
                ds_ego_d_rt, ds_longi_sample_s_rt, ds_longi_sample_d_rt, ds_lat_sample_s_rt, \
                ds_lat_sample_d_rt, ds_traj_s_rt, ds_traj_d_rt

    def add_lon_search_st_graph(self, fig_name, source_suffix, lon_search_debug_time_rt, st_path_t_rt, st_path_s_rt, \
                st_sample_points_t_rt, st_sample_points_s_rt, st_sample_points_a_rt, \
                st_sample_points_v_rt, st_sample_points_cost_rt, st_obj_polygon_xs_rt, \
                st_obj_polygon_ys_rt, vt_path_t_rt, vt_path_v_rt, vt_limit_t_rt, vt_limit_v_rt, \
                st_sample_points_color_rt):
        self.add_layer_dmpp_line(self.figs_[fig_name], 'st_path_rt_source' + source_suffix, 'st_path_rt' + source_suffix, lon_search_debug_time_rt, st_path_s_rt, st_path_t_rt, 'st_path', 4, 0.2, 'coral')
        # self.add_layer_longi_search_sample_point(self.figs_[fig_name], 'st_sample_points_rt_source', 'st_sample_points_rt', lon_search_debug_time_rt, 
        #                                st_sample_points_s_rt, st_sample_points_t_rt, st_sample_points_cost_rt, 'st_sample_points', 4, 0.2)
        self.add_layer_longi_search_sample_point(self.figs_[fig_name], 'st_sample_points_rt_source' + source_suffix, 'st_sample_points_rt' + source_suffix, lon_search_debug_time_rt, 
                                       st_sample_points_s_rt, st_sample_points_t_rt, st_sample_points_v_rt, st_sample_points_a_rt, st_sample_points_cost_rt, st_sample_points_color_rt, 'st_sample_points', 10, 0.2)
        self.add_layer_dmpp_polygons(self.figs_[fig_name], 'st_obj_polygons_rt_source' + source_suffix, 'st_obj_polygons_rt' + source_suffix, lon_search_debug_time_rt, st_obj_polygon_xs_rt, st_obj_polygon_ys_rt, 'st_obj_polygons', 4, 0.2, 'hotpink')

        return

    def add_lon_search_vt_graph(self, fig_name, source_suffix, lon_search_debug_time_rt, st_path_t_rt, st_path_s_rt, \
                st_sample_points_t_rt, st_sample_points_s_rt, st_sample_points_a_rt, \
                st_sample_points_v_rt, st_sample_points_cost_rt, st_obj_polygon_xs_rt, \
                st_obj_polygon_ys_rt, vt_path_t_rt, vt_path_v_rt, vt_limit_t_rt, vt_limit_v_rt, \
                st_sample_points_color_rt):
        self.add_layer_dmpp_scatter(self.figs_[fig_name], 'vt_path_rt_source' + source_suffix, 'vt_path_rt' + source_suffix, lon_search_debug_time_rt, vt_path_v_rt, vt_path_t_rt, 'vt_path', 4, 0.2, 'coral')
        self.add_layer_dmpp_scatter(self.figs_[fig_name], 'vt_limit_rt_source' + source_suffix, 'vt_limit_rt' + source_suffix, lon_search_debug_time_rt, vt_limit_v_rt, vt_limit_t_rt, 'vt_limit', 4, 0.2, 'Green')
        
        return
    
    def add_lat_search_sd_graph(self, fig_name, source_suffix, lat_search_debug_time_rt, ds_obs_polygon_xs_rt, ds_obs_polygon_ys_rt, \
                ds_bound_s_rt, ds_bound_d_rt, ds_range_d_rt, ds_goal_d_rt, ds_ego_s_rt, \
                ds_ego_d_rt, ds_longi_sample_s_rt, ds_longi_sample_d_rt, ds_lat_sample_s_rt, \
                ds_lat_sample_d_rt, ds_traj_s_rt, ds_traj_d_rt):
        self.add_layer_dmpp_polygons(self.figs_[fig_name], 'ds_obj_polygons_rt_source' + source_suffix, 'ds_obj_polygons_rt' + source_suffix, lat_search_debug_time_rt, ds_obs_polygon_ys_rt, ds_obs_polygon_xs_rt, 'ds_obs', 4, 0.2, 'blue')
        self.add_layer_dmpp_scatter(self.figs_[fig_name], 'ds_bound_rt_source' + source_suffix, 'ds_bound_rt' + source_suffix, lat_search_debug_time_rt, ds_bound_s_rt, ds_bound_d_rt, 'ds_bound', 4, 0.2, 'red')
        self.add_layer_dmpp_scatter(self.figs_[fig_name], 'ds_range_rt_source' + source_suffix, 'ds_range_rt' + source_suffix, lat_search_debug_time_rt, ds_bound_s_rt, ds_range_d_rt, 'ds_range', 4, 0.2, 'blueviolet')
        self.add_layer_dmpp_scatter(self.figs_[fig_name], 'ds_goal_rt_source' + source_suffix, 'ds_goal_rt' + source_suffix, lat_search_debug_time_rt, ds_bound_s_rt, ds_goal_d_rt, 'ds_goal', 4, 0.2, 'burlywood')
        self.add_layer_dmpp_scatter(self.figs_[fig_name], 'ds_ego_rt_source' + source_suffix, 'ds_ego_rt' + source_suffix, lat_search_debug_time_rt, ds_ego_s_rt, ds_ego_d_rt, 'ds_ego', 10, 4, 'cadetblue')
        self.add_layer_dmpp_scatter(self.figs_[fig_name], 'ds_longi_sample_rt_source' + source_suffix, 'ds_longi_sample_rt' + source_suffix, lat_search_debug_time_rt, ds_longi_sample_s_rt, ds_longi_sample_d_rt, 'ds_longi_sample', 4, 0.2, 'hotpink')
        self.add_layer_dmpp_scatter(self.figs_[fig_name], 'ds_lat_sample_rt_source' + source_suffix, 'ds_lat_sample_rt' + source_suffix, lat_search_debug_time_rt, ds_lat_sample_s_rt, ds_lat_sample_d_rt, 'ds_lat_sample', 4, 0.2, 'indianred')
        self.add_layer_dmpp_scatter(self.figs_[fig_name], 'ds_traj_rt_source' + source_suffix, 'ds_traj_rt' + source_suffix, lat_search_debug_time_rt, ds_traj_s_rt, ds_traj_d_rt, 'ds_traj', 4, 0.2, 'cyan')

        return

    def add_rt_callback(self):
        self.callback_ =CustomJS(args=dict(time = self.time_slider_,
                                           traj_rt = self.plot_data_set_['trajectory_rt'],
                                           traj_rt_line = self.plot_data_set_['trajectory_rt_line'],
                                           source_traj = self.plot_data_set_['trajectory_rt_source'],

                                           traj_pt_rt = self.plot_data_set_['traj_point_rt'],
                                           source_pt = self.plot_data_set_['traj_point_rt_source'],

                                           predict_traj_rt = self.plot_data_set_['predict_trajectory_rt'],
                                           source_predict_traj = self.plot_data_set_['predict_trajectory_rt_source'],
                                           obstacle_center_rt = self.plot_data_set_['obstacle_center_rt'],
                                           source_obstacle_center_rt = self.plot_data_set_['obstacle_center_rt_source'],
                                           dm_cur_refline_rt = self.plot_data_set_['dm_cur_refline_rt'],
                                           source_dm_cur_refline_rt = self.plot_data_set_['dm_cur_refline_rt_source'],
                                           dm_target_refline_rt = self.plot_data_set_['dm_target_refline_rt'],
                                           source_dm_target_refline_rt = self.plot_data_set_['dm_target_refline_rt_source'],
                                           pp_refline_rt = self.plot_data_set_['pp_refline_rt'],
                                           source_pp_refline_rt = self.plot_data_set_['pp_refline_rt_source'],
                                           lateral_r_ref_rt = self.plot_data_set_['lateral_r_ref_rt'],
                                           source_lateral_r_ref_rt = self.plot_data_set_['lateral_r_ref_rt_source'],
                                           lateral_lower_bound_rt = self.plot_data_set_['lateral_lower_bound_rt'],
                                           source_lateral_lower_bound_rt = self.plot_data_set_['lateral_lower_bound_rt_source'],
                                           lateral_upper_bound_rt = self.plot_data_set_['lateral_upper_bound_rt'],
                                           source_lateral_upper_bound_rt = self.plot_data_set_['lateral_upper_bound_rt_source'],
                                           
                                           curv_source=self.plot_data_set_['curvature_time_source'],
                                           curv_rt=self.plot_data_set_['curvature_time'],
                                           sum_rt=self.plot_data_set_['sum_time'],
                                           sum_source=self.plot_data_set_['sum_time_source'],

                                           vel_rt=self.plot_data_set_['vel_time'],
                                           vel_source=self.plot_data_set_['vel_time_source'],

                                           yawrate_rt=self.plot_data_set_['yawrate_time'],
                                           yawrate_source=self.plot_data_set_['yawrate_time_source'],

                                           theta_rt=self.plot_data_set_['theta_time'],
                                           theta_source=self.plot_data_set_['theta_time_source'],

                                           curva_speed_rt=self.plot_data_set_['curva_speed_rt'],
                                           curva_speed_source=self.plot_data_set_['curva_speed_rt_source'],

                                           sum_vel_rt = self.plot_data_set_['sum_vel_rt'],
                                           sum_vel_source = self.plot_data_set_['sum_vel_rt_source'],
                                           enabel_auto_source=self.plot_data_set_['enabel_auto_source_data'],
                                           enabel_auto_current_value_source=self.plot_data_set_['enabel_auto_current_value'],
                                           traj_update_source=self.plot_data_set_['traj_update_source_data'],
                                           traj_update_current_value_source=self.plot_data_set_['traj_update_current_value'],
                                           ego_vel_source=self.plot_data_set_['ego_vel_time_source'],
                                           ego_vel_current_value_source=self.plot_data_set_['ego_vel_time'],
                                           ego_acc_x_source=self.plot_data_set_['ego_acc_x_time_source'],
                                           ego_acc_x_current_value_source=self.plot_data_set_['ego_acc_x_time'],
                                           sum_acc_rt = self.plot_data_set_['sum_acc_rt'],
                                           sum_acc_source = self.plot_data_set_['sum_acc_rt_source'],
                                           optimizer_s_v_rt = self.plot_data_set_['optimizer_s_v_rt'],
                                           optimizer_s_v_source = self.plot_data_set_['optimizer_s_v_rt_source'],
                                           leader_s_v_rt = self.plot_data_set_['leader_s_v_rt'],
                                           leader_s_v_source = self.plot_data_set_['leader_s_v_rt_source'],
                                           ref_follow_s_v_rt = self.plot_data_set_['ref_follow_s_v_rt'],
                                           ref_follow_s_v_source = self.plot_data_set_['ref_follow_s_v_rt_source'],                                           
                                        #    oprimizer_s_ref_rt = self.plot_data_set_['optimizer_s_v_ref_rt'],
                                        #    oprimizer_s_v_ref_source = self.plot_data_set_['optimizer_s_v_ref_rt_source'],
                                        #    oprimizer_s_v_upper_rt = self.plot_data_set_['optimizer_s_v_upper_rt'],
                                        #    oprimizer_s_v_upper_source = self.plot_data_set_['optimizer_s_v_upper_rt_source'],
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
                                           
                                           st_path_rt = self.plot_data_set_['st_path_rt'],
                                           source_st_path_rt = self.plot_data_set_['st_path_rt_source'],
                                           st_sample_points_rt = self.plot_data_set_['st_sample_points_rt'],
                                           source_st_sample_points_rt = self.plot_data_set_['st_sample_points_rt_source'],
                                        #    color_mapper_rt = self.color_mapper_,
                                           st_obj_polygons_rt = self.plot_data_set_['st_obj_polygons_rt'],
                                           source_st_obj_polygons_rt = self.plot_data_set_['st_obj_polygons_rt_source'],
                                           
                                           vt_path_rt = self.plot_data_set_['vt_path_rt'],
                                           source_vt_path_rt = self.plot_data_set_['vt_path_rt_source'],
                                           vt_limit_rt = self.plot_data_set_['vt_limit_rt'],
                                           source_vt_limit_rt = self.plot_data_set_['vt_limit_rt_source'],
                                           
                                           ds_obj_polygons_rt = self.plot_data_set_['ds_obj_polygons_rt'],
                                           source_ds_obj_polygons_rt = self.plot_data_set_['ds_obj_polygons_rt_source'],
                                           ds_bound_rt = self.plot_data_set_['ds_bound_rt'],
                                           source_ds_bound_rt = self.plot_data_set_['ds_bound_rt_source'],
                                           ds_range_rt = self.plot_data_set_['ds_range_rt'],
                                           source_ds_range_rt = self.plot_data_set_['ds_range_rt_source'],
                                           ds_goal_rt = self.plot_data_set_['ds_goal_rt'],
                                           source_ds_goal_rt = self.plot_data_set_['ds_goal_rt_source'],
                                           ds_ego_rt = self.plot_data_set_['ds_ego_rt'],
                                           source_ds_ego_rt = self.plot_data_set_['ds_ego_rt_source'],
                                           ds_longi_sample_rt = self.plot_data_set_['ds_longi_sample_rt'],
                                           source_ds_longi_sample_rt = self.plot_data_set_['ds_longi_sample_rt_source'],
                                           ds_lat_sample_rt = self.plot_data_set_['ds_lat_sample_rt'],
                                           source_ds_lat_sample_rt = self.plot_data_set_['ds_lat_sample_rt_source'],
                                           ds_traj_rt = self.plot_data_set_['ds_traj_rt'],
                                           source_ds_traj_rt = self.plot_data_set_['ds_traj_rt_source'],

                                           st_path_rt_lc_decider = self.plot_data_set_['st_path_rt_lc_decider'],
                                           source_st_path_rt_lc_decider = self.plot_data_set_['st_path_rt_source_lc_decider'],
                                           st_sample_points_rt_lc_decider = self.plot_data_set_['st_sample_points_rt_lc_decider'],
                                           source_st_sample_points_rt_lc_decider = self.plot_data_set_['st_sample_points_rt_source_lc_decider'],
                                        #    color_mapper_rt = self.color_mapper_,
                                           st_obj_polygons_rt_lc_decider = self.plot_data_set_['st_obj_polygons_rt_lc_decider'],
                                           source_st_obj_polygons_rt_lc_decider = self.plot_data_set_['st_obj_polygons_rt_source_lc_decider'],
                                           
                                           ds_obj_polygons_rt_lc_decider = self.plot_data_set_['ds_obj_polygons_rt_lc_decider'],
                                           source_ds_obj_polygons_rt_lc_decider = self.plot_data_set_['ds_obj_polygons_rt_source_lc_decider'],
                                           ds_bound_rt_lc_decider = self.plot_data_set_['ds_bound_rt_lc_decider'],
                                           source_ds_bound_rt_lc_decider = self.plot_data_set_['ds_bound_rt_source_lc_decider'],
                                           ds_range_rt_lc_decider = self.plot_data_set_['ds_range_rt_lc_decider'],
                                           source_ds_range_rt_lc_decider = self.plot_data_set_['ds_range_rt_source_lc_decider'],
                                           ds_goal_rt_lc_decider = self.plot_data_set_['ds_goal_rt_lc_decider'],
                                           source_ds_goal_rt_lc_decider = self.plot_data_set_['ds_goal_rt_source_lc_decider'],
                                           ds_ego_rt_lc_decider = self.plot_data_set_['ds_ego_rt_lc_decider'],
                                           source_ds_ego_rt_lc_decider = self.plot_data_set_['ds_ego_rt_source_lc_decider'],
                                           ds_longi_sample_rt_lc_decider = self.plot_data_set_['ds_longi_sample_rt_lc_decider'],
                                           source_ds_longi_sample_rt_lc_decider = self.plot_data_set_['ds_longi_sample_rt_source_lc_decider'],
                                           ds_lat_sample_rt_lc_decider = self.plot_data_set_['ds_lat_sample_rt_lc_decider'],
                                           source_ds_lat_sample_rt_lc_decider = self.plot_data_set_['ds_lat_sample_rt_source_lc_decider'],
                                           ds_traj_rt_lc_decider = self.plot_data_set_['ds_traj_rt_lc_decider'],
                                           source_ds_traj_rt_lc_decider = self.plot_data_set_['ds_traj_rt_source_lc_decider'],
                                           ),
                                 code ="""

    const cur_time = time.value
    // console.log("cur_time:", cur_time);
    // console.log("source_traj:", source_traj);
    const time_list = source_traj['time']
    // console.log("time_list:", time_list);
    for (let i = 0; i < time_list.length; i++) {
        if ((cur_time - time_list[i] < 0.1) && (cur_time - time_list[i] > 0)){
            traj_rt.data.x = Array.from(source_traj.source_x[i])
            traj_rt.data.y = Array.from(source_traj.source_y[i])
            traj_rt_line.data.x = Array.from(source_traj.source_x[i])
            traj_rt_line.data.y = Array.from(source_traj.source_y[i])

            sum_vel_rt.data.x = Array.from(sum_vel_source.source_x[i])
            sum_vel_rt.data.y = Array.from(sum_vel_source.source_y[i])

            sum_acc_rt.data.x = Array.from(sum_acc_source.source_x[i])
            sum_acc_rt.data.y = Array.from(sum_acc_source.source_y[i])

            const curvatures = Array.from(curv_source.y[i]);
            curv_rt.data.index = curvatures.map((_, index) => index);
            curv_rt.data.y = curvatures;
            const sums = Array.from(sum_source.y[i]);
            sum_rt.data.index = sums.map((_, index) => index);
            sum_rt.data.y = sums;

            //trajectory point vel(x-axis is index)    
            const vels = Array.from(vel_source.y[i]);
            vel_rt.data.index = vels.map((_, index) => index);
            vel_rt.data.y = vels; 

            //trajectory point yawrate(x-axis is index)
            const yawrate = Array.from(yawrate_source.y[i]);
            yawrate_rt.data.index = yawrate.map((_, index) => index);
            yawrate_rt.data.y = yawrate;

            //trajectory point theta(x-axis is index) 
            const theta = Array.from(theta_source.y[i]);
            theta_rt.data.index = theta.map((_, index) => index);
            theta_rt.data.y = theta;             
        }
    } 
    traj_rt.change.emit()
    traj_rt_line.change.emit()
    curv_rt.change.emit();
    sum_rt.change.emit();
    vel_rt.change.emit();
    yawrate_rt.change.emit();
    theta_rt.change.emit();
    sum_vel_rt.change.emit();
    sum_acc_rt.change.emit();

    // longi optimize info from xpilot debug
    const time_list_xdebug = optimizer_s_v_source['time']
    for (let i = 0; i < time_list_xdebug.length; i++) {
        if ((cur_time - time_list_xdebug[i] < 0.1) && (cur_time - time_list_xdebug[i] > 0)){
        
            traj_pt_rt.data.x = Array.from(source_pt.source_x[i])
            traj_pt_rt.data.y = Array.from(source_pt.source_y[i])

            optimizer_s_v_rt.data.x = Array.from(optimizer_s_v_source.source_x[i])
            optimizer_s_v_rt.data.y = Array.from(optimizer_s_v_source.source_y[i])

            curva_speed_rt.data.x = Array.from(curva_speed_source.source_x[i])
            curva_speed_rt.data.y = Array.from(curva_speed_source.source_y[i])

            leader_s_v_rt.data.x = Array.from(leader_s_v_source.source_x[i])
            leader_s_v_rt.data.y = Array.from(leader_s_v_source.source_y[i])

            ref_follow_s_v_rt.data.x = Array.from(ref_follow_s_v_source.source_x[i])
            ref_follow_s_v_rt.data.y = Array.from(ref_follow_s_v_source.source_y[i])

            // oprimizer_s_v_ref_rt.data.x = Array.from(oprimizer_s_v_ref_source.source_x[i])
            // oprimizer_s_v_ref_rt.data.y = Array.from(oprimizer_s_v_ref_source.source_y[i])

            // oprimizer_s_v_upper_rt.data.x = Array.from(oprimizer_s_v_upper_source.source_x[i])
            // oprimizer_s_v_upper_rt.data.y = Array.from(oprimizer_s_v_upper_source.source_y[i])

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
    traj_pt_rt.change.emit();
    optimizer_s_v_rt.change.emit();
    leader_s_v_rt.change.emit();
    ref_follow_s_v_rt.change.emit();
    curva_speed_rt.change.emit();
    // oprimizer_s_v_ref_rt.change.emit();
    // oprimizer_s_v_upper_rt.change.emit();
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

    // enable_auto from control debug
    const time_list_control_debug = enabel_auto_source.data['time'];
    const y_values_control_debug = enabel_auto_source.data['y'];
    enabel_auto_current_value_source.data['time'] = [];
    enabel_auto_current_value_source.data['y'] = [];
    for (let i = 0; i < time_list_control_debug.length; i++) {
        if ((cur_time - time_list_control_debug[i] < 0.1) && 
            (cur_time - time_list_control_debug[i] > 0)) {  
            enabel_auto_current_value_source.data['time'].push(time_list_control_debug[i]);
            enabel_auto_current_value_source.data['y'].push(y_values_control_debug[i]);
        }
    }
    enabel_auto_current_value_source.change.emit();

    // traj update state from traj debug
    const y_values_traj_update = traj_update_source.data['y'];
    traj_update_current_value_source.data['time'] = [];
    traj_update_current_value_source.data['y'] = [];
    for (let i = 0; i < time_list.length; i++) {
        if ((cur_time - time_list[i] < 0.1) && (cur_time - time_list[i] > 0)) {  
            traj_update_current_value_source.data['time'].push(time_list[i]);
            traj_update_current_value_source.data['y'].push(y_values_traj_update[i]);
        }
    }
    traj_update_current_value_source.change.emit();

    // ego vel/acc from vehicle inport debug
    const time_list_vehicle_report = ego_vel_source.data['time'];
    const y_values_vehivle_vel = ego_vel_source.data['y'];
    ego_vel_current_value_source.data['time'] = [];
    ego_vel_current_value_source.data['y'] = [];
    for (let i = 0; i < time_list_vehicle_report.length; i++) {
        if ((cur_time - time_list_vehicle_report[i] < 0.1) && 
            (cur_time - time_list_vehicle_report[i] > 0)) {  
            ego_vel_current_value_source.data['time'].push(time_list_vehicle_report[i]);
            ego_vel_current_value_source.data['y'].push(y_values_vehivle_vel[i]);
        }
    }
    ego_vel_current_value_source.change.emit();

    const y_values_vehivle_acc_x = ego_acc_x_source.data['y'];
    ego_acc_x_current_value_source.data['time'] = [];
    ego_acc_x_current_value_source.data['y'] = [];
    for (let i = 0; i < time_list_vehicle_report.length; i++) {
        if ((cur_time - time_list_vehicle_report[i] < 0.1) && 
            (cur_time - time_list_vehicle_report[i] > 0)) {  
            ego_acc_x_current_value_source.data['time'].push(time_list_vehicle_report[i]);
            ego_acc_x_current_value_source.data['y'].push(y_values_vehivle_acc_x[i]);
        }
    }
    ego_acc_x_current_value_source.change.emit();


    const prediction_time_list = source_predict_traj['time']
    for (let i = 0; i < prediction_time_list.length; i++) {
        if ((cur_time - prediction_time_list[i] < 0.1) && 
            (cur_time - prediction_time_list[i] > 0)){
            // console.log("predict traj found time_list[i]:", i,time_list[i]);
            predict_traj_rt.data.x = Array.from(source_predict_traj.source_x[i])
            predict_traj_rt.data.y = Array.from(source_predict_traj.source_y[i])
        }
    }
    // console.log("predict_traj_rt.x",predict_traj_rt.data.x);
    predict_traj_rt.change.emit()
 
    const perception_time_list = source_obstacle_center_rt['time']
    for (let i = 0; i < perception_time_list.length; i++) {
        if ((cur_time - perception_time_list[i] < 0.1) && (cur_time - perception_time_list[i] > 0)){
            //console.log("found perception_time_list[i]:", i,perception_time_list[i]);
            //console.log("perception object found time_list[i]:", i,perception_time_list[i]);
            obstacle_center_rt.data.x = Array.from(source_obstacle_center_rt.source_x[i])
            obstacle_center_rt.data.y = Array.from(source_obstacle_center_rt.source_y[i])
            obstacle_center_rt.data.id = Array.from(source_obstacle_center_rt.source_id[i])
        }
    }
    obstacle_center_rt.change.emit()

    const dm_cur_refline_rt_time_list = source_dm_cur_refline_rt['time']
    for (let i = 0; i < dm_cur_refline_rt_time_list.length; i++) {
        if ((cur_time - dm_cur_refline_rt_time_list[i] < 0.1) && (cur_time - dm_cur_refline_rt_time_list[i] > 0)){
            dm_cur_refline_rt.data.x = Array.from(source_dm_cur_refline_rt.source_x[i])
            dm_cur_refline_rt.data.y = Array.from(source_dm_cur_refline_rt.source_y[i])

            dm_target_refline_rt.data.x = Array.from(source_dm_target_refline_rt.source_x[i])
            dm_target_refline_rt.data.y = Array.from(source_dm_target_refline_rt.source_y[i])
        }
    }
    dm_cur_refline_rt.change.emit()
    dm_target_refline_rt.change.emit()
   

    const pp_refline_rt_time_list = source_pp_refline_rt['time']
    for (let i = 0; i < pp_refline_rt_time_list.length; i++) {
        if ((cur_time - pp_refline_rt_time_list[i] < 0.1) && (cur_time - pp_refline_rt_time_list[i] > 0)){
            pp_refline_rt.data.x = Array.from(source_pp_refline_rt.source_x[i])
            pp_refline_rt.data.y = Array.from(source_pp_refline_rt.source_y[i])

            lateral_r_ref_rt.data.x = Array.from(source_lateral_r_ref_rt.source_x[i])
            lateral_r_ref_rt.data.y = Array.from(source_lateral_r_ref_rt.source_y[i])

            lateral_lower_bound_rt.data.x = Array.from(source_lateral_lower_bound_rt.source_x[i])
            lateral_lower_bound_rt.data.y = Array.from(source_lateral_lower_bound_rt.source_y[i])

            lateral_upper_bound_rt.data.x = Array.from(source_lateral_upper_bound_rt.source_x[i])
            lateral_upper_bound_rt.data.y = Array.from(source_lateral_upper_bound_rt.source_y[i])
        }
    }
    pp_refline_rt.change.emit()
    lateral_r_ref_rt.change.emit()
    lateral_lower_bound_rt.change.emit()
    lateral_upper_bound_rt.change.emit()

    st_path_rt.data.x.length = 0
    st_path_rt.data.y.length = 0
    st_sample_points_rt.data.x.length = 0
    st_sample_points_rt.data.y.length = 0
    st_sample_points_rt.data.v.length = 0
    st_sample_points_rt.data.a.length = 0
    st_sample_points_rt.data.color.length = 0
    st_sample_points_rt.data.cost.length = 0
    st_obj_polygons_rt.data.xs.length = 0
    st_obj_polygons_rt.data.ys.length = 0
    vt_path_rt.data.x.length = 0
    vt_path_rt.data.y.length = 0
    vt_limit_rt.data.x.length = 0
    vt_limit_rt.data.y.length = 0
    const st_path_rt_time_list = source_st_path_rt['time']
    for (let i = 0; i < st_path_rt_time_list.length; i++) {
        if ((cur_time - st_path_rt_time_list[i] < 0.1) && (cur_time - st_path_rt_time_list[i] > 0)){
            st_path_rt.data.x = Array.from(source_st_path_rt.source_x[i])
            st_path_rt.data.y = Array.from(source_st_path_rt.source_y[i])
            
            st_sample_points_rt.data.x = Array.from(source_st_sample_points_rt.source_x[i])
            st_sample_points_rt.data.y = Array.from(source_st_sample_points_rt.source_y[i])
            st_sample_points_rt.data.v = Array.from(source_st_sample_points_rt.source_v[i])
            st_sample_points_rt.data.a = Array.from(source_st_sample_points_rt.source_a[i])

            //console.log(source_st_sample_points_rt.source_color.length)
            //console.log(source_st_sample_points_rt.source_color[i])

            st_sample_points_rt.data.cost = Array.from(source_st_sample_points_rt.source_cost[i])
            st_sample_points_rt.data.color = Array.from(source_st_sample_points_rt.source_color[i])

            //color_mapper_rt.low = Math.min(...st_sample_points_rt.data.color)
            //color_mapper_rt.high = Math.max(...st_sample_points_rt.data.color)
            //
            //console.log(st_sample_points_rt.data.color)
            //console.log(color_mapper_rt.low)
            //console.log(color_mapper_rt.high)

            const polygon_xs = source_st_obj_polygons_rt.source_polygon_xs[i]
            for (let i = 0; i < polygon_xs.length; i++) {
                st_obj_polygons_rt.data.xs.push(Array.from(polygon_xs[i]))
            }
            const polygon_ys = source_st_obj_polygons_rt.source_polygon_ys[i]
            for (let i = 0; i < polygon_ys.length; i++) {
                st_obj_polygons_rt.data.ys.push(Array.from(polygon_ys[i]))
            }

            vt_path_rt.data.x = Array.from(source_vt_path_rt.source_x[i])
            vt_path_rt.data.y = Array.from(source_vt_path_rt.source_y[i])

            vt_limit_rt.data.x = Array.from(source_vt_limit_rt.source_x[i])
            vt_limit_rt.data.y = Array.from(source_vt_limit_rt.source_y[i])

            break
       }
    }
    st_path_rt.change.emit()
    st_sample_points_rt.change.emit()
    st_obj_polygons_rt.change.emit()
    vt_path_rt.change.emit()
    vt_limit_rt.change.emit()

    ds_obj_polygons_rt.data.xs.length = 0
    ds_obj_polygons_rt.data.ys.length = 0
    ds_bound_rt.data.x.length = 0
    ds_bound_rt.data.y.length = 0
    ds_range_rt.data.x.length = 0
    ds_range_rt.data.y.length = 0
    ds_goal_rt.data.x.length = 0
    ds_goal_rt.data.y.length = 0
    ds_ego_rt.data.x.length = 0
    ds_ego_rt.data.y.length = 0
    ds_longi_sample_rt.data.x.length = 0
    ds_longi_sample_rt.data.y.length = 0
    ds_lat_sample_rt.data.x.length = 0
    ds_lat_sample_rt.data.y.length = 0
    ds_traj_rt.data.x.length = 0
    ds_traj_rt.data.y.length = 0
    const lat_search_sd_rt_time_list = source_ds_obj_polygons_rt['time']
    for (let i = 0; i < lat_search_sd_rt_time_list.length; i++) {
        if ((cur_time - lat_search_sd_rt_time_list[i] < 0.1) && (cur_time - lat_search_sd_rt_time_list[i] > 0)){
            const polygon_xs = source_ds_obj_polygons_rt.source_polygon_xs[i]
            for (let i = 0; i < polygon_xs.length; i++) {
                ds_obj_polygons_rt.data.xs.push(Array.from(polygon_xs[i]))

                //console.log(polygon_xs[i])
            }
            const polygon_ys = source_ds_obj_polygons_rt.source_polygon_ys[i]
            for (let i = 0; i < polygon_ys.length; i++) {
                ds_obj_polygons_rt.data.ys.push(Array.from(polygon_ys[i]))

                //console.log(polygon_ys[i])
            }
            
            ds_bound_rt.data.x = Array.from(source_ds_bound_rt.source_x[i])
            ds_bound_rt.data.y = Array.from(source_ds_bound_rt.source_y[i])

            ds_range_rt.data.x = Array.from(source_ds_range_rt.source_x[i])
            ds_range_rt.data.y = Array.from(source_ds_range_rt.source_y[i])

            ds_goal_rt.data.x = Array.from(source_ds_goal_rt.source_x[i])
            ds_goal_rt.data.y = Array.from(source_ds_goal_rt.source_y[i])

            ds_ego_rt.data.x = Array.from(source_ds_ego_rt.source_x[i])
            ds_ego_rt.data.y = Array.from(source_ds_ego_rt.source_y[i])

            ds_longi_sample_rt.data.x = Array.from(source_ds_longi_sample_rt.source_x[i])
            ds_longi_sample_rt.data.y = Array.from(source_ds_longi_sample_rt.source_y[i])

            ds_lat_sample_rt.data.x = Array.from(source_ds_lat_sample_rt.source_x[i])
            ds_lat_sample_rt.data.y = Array.from(source_ds_lat_sample_rt.source_y[i])

            ds_traj_rt.data.x = Array.from(source_ds_traj_rt.source_x[i])
            ds_traj_rt.data.y = Array.from(source_ds_traj_rt.source_y[i])

            break
       }
    }
    ds_obj_polygons_rt.change.emit()
    ds_bound_rt.change.emit()
    ds_range_rt.change.emit()
    ds_goal_rt.change.emit()
    ds_ego_rt.change.emit()
    ds_longi_sample_rt.change.emit()
    ds_lat_sample_rt.change.emit()
    ds_traj_rt.change.emit()

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

    ds_obj_polygons_rt_lc_decider.data.xs.length = 0
    ds_obj_polygons_rt_lc_decider.data.ys.length = 0
    ds_bound_rt_lc_decider.data.x.length = 0
    ds_bound_rt_lc_decider.data.y.length = 0
    ds_range_rt_lc_decider.data.x.length = 0
    ds_range_rt_lc_decider.data.y.length = 0
    ds_goal_rt_lc_decider.data.x.length = 0
    ds_goal_rt_lc_decider.data.y.length = 0
    ds_ego_rt_lc_decider.data.x.length = 0
    ds_ego_rt_lc_decider.data.y.length = 0
    ds_longi_sample_rt_lc_decider.data.x.length = 0
    ds_longi_sample_rt_lc_decider.data.y.length = 0
    ds_lat_sample_rt_lc_decider.data.x.length = 0
    ds_lat_sample_rt_lc_decider.data.y.length = 0
    ds_traj_rt_lc_decider.data.x.length = 0
    ds_traj_rt_lc_decider.data.y.length = 0
    const lat_search_sd_rt_time_list_lc_decider = source_ds_obj_polygons_rt_lc_decider['time']
    for (let i = 0; i < lat_search_sd_rt_time_list_lc_decider.length; i++) {
        if ((cur_time - lat_search_sd_rt_time_list_lc_decider[i] < 0.1) && (cur_time - lat_search_sd_rt_time_list_lc_decider[i] > 0)){
            const polygon_xs = source_ds_obj_polygons_rt_lc_decider.source_polygon_xs[i]
            for (let i = 0; i < polygon_xs.length; i++) {
                ds_obj_polygons_rt_lc_decider.data.xs.push(Array.from(polygon_xs[i]))

                //console.log(polygon_xs[i])
            }
            const polygon_ys = source_ds_obj_polygons_rt_lc_decider.source_polygon_ys[i]
            for (let i = 0; i < polygon_ys.length; i++) {
                ds_obj_polygons_rt_lc_decider.data.ys.push(Array.from(polygon_ys[i]))

                //console.log(polygon_ys[i])
            }
            
            ds_bound_rt_lc_decider.data.x = Array.from(source_ds_bound_rt_lc_decider.source_x[i])
            ds_bound_rt_lc_decider.data.y = Array.from(source_ds_bound_rt_lc_decider.source_y[i])

            ds_range_rt_lc_decider.data.x = Array.from(source_ds_range_rt_lc_decider.source_x[i])
            ds_range_rt_lc_decider.data.y = Array.from(source_ds_range_rt_lc_decider.source_y[i])

            ds_goal_rt_lc_decider.data.x = Array.from(source_ds_goal_rt_lc_decider.source_x[i])
            ds_goal_rt_lc_decider.data.y = Array.from(source_ds_goal_rt_lc_decider.source_y[i])

            ds_ego_rt_lc_decider.data.x = Array.from(source_ds_ego_rt_lc_decider.source_x[i])
            ds_ego_rt_lc_decider.data.y = Array.from(source_ds_ego_rt_lc_decider.source_y[i])

            ds_longi_sample_rt_lc_decider.data.x = Array.from(source_ds_longi_sample_rt_lc_decider.source_x[i])
            ds_longi_sample_rt_lc_decider.data.y = Array.from(source_ds_longi_sample_rt_lc_decider.source_y[i])

            ds_lat_sample_rt_lc_decider.data.x = Array.from(source_ds_lat_sample_rt_lc_decider.source_x[i])
            ds_lat_sample_rt_lc_decider.data.y = Array.from(source_ds_lat_sample_rt_lc_decider.source_y[i])

            ds_traj_rt_lc_decider.data.x = Array.from(source_ds_traj_rt_lc_decider.source_x[i])
            ds_traj_rt_lc_decider.data.y = Array.from(source_ds_traj_rt_lc_decider.source_y[i])

            break
       }
    }
    ds_obj_polygons_rt_lc_decider.change.emit()
    ds_bound_rt_lc_decider.change.emit()
    ds_range_rt_lc_decider.change.emit()
    ds_goal_rt_lc_decider.change.emit()
    ds_ego_rt_lc_decider.change.emit()
    ds_longi_sample_rt_lc_decider.change.emit()
    ds_lat_sample_rt_lc_decider.change.emit()
    ds_traj_rt_lc_decider.change.emit()

                        """)
        self.time_slider_.js_on_change('value', self.callback_)
        pass
    
    def add_slider(self):
        self.time_slider_ = Slider(start=0.0, end=self.bag_time_length_, value=0.0, step=.1, title="Time")
    
    def add_hover_tool(self):
        hover_obstacle_point = HoverTool(renderers=[self.obstacle_point_render_],
                      tooltips=[
                      ("ID", "@id"),
                      ("x", "@x"),
                      ("y", "@y"),
                      ("name","Info")],
                      mode='mouse')
        self.figs_['fig1'].add_tools(hover_obstacle_point)
    
    def figure_plot(self):
        if self.read_bag_flag_  == False:
            self.read_bag()
            self.match_timestamp()

        self.creat_figure()

        # plot real-time trajectory
        time_rt = []
        #some car state
        gear_rt = []
        traj_state_rt = []
        traj_mode_rt = []
        turn_sign_rt = []
        update_by_vehsta_rt = []
        timestamp_ns_rt = []
        #traj planner state
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

        # plot real-time predict trajectory
        predict_obj_time_rt = []
        predict_obj_x_rt = []
        predict_obj_y_rt = []
        for channel in self.data_set_.keys():
            if channel == PREDICTION_CHANNEL:
                for data_item in self.data_set_[PREDICTION_CHANNEL]:
                    for msg_item in data_item.items():
                        predict_obj_time_rt.append(msg_item[0])
                        tmp_pre_x = []
                        tmp_pre_y = []
                        
                        for object in msg_item[1]['prediction_traj']:
                            sum_of_squares = object['x']**2 + object['y']**2
                            if math.sqrt(sum_of_squares) > 50:
                                continue
                            for traj_point in object['traj']:
                                tmp_pre_x.append(traj_point['x'])
                                tmp_pre_y.append(traj_point['y'])
                        predict_obj_x_rt.append(tmp_pre_x)
                        predict_obj_y_rt.append(tmp_pre_y)

        # plot obstacle  
        obj_time_rt = []
        obj_x_rt = []
        obj_y_rt = []  
        obj_id_rt = []
        obj_polygon_x_rt = [] 
        obj_polygon_y_rt = []           
        for channel in self.data_set_.keys():
            if channel == PERCEPTION_CHANNEL:
                for data_item in self.data_set_[channel]:
                    for msg_item in data_item.items():
                        obj_time_rt.append(msg_item[0])
                        obj_x_rt_tmp = []
                        obj_y_rt_tmp  = []  
                        obj_id_rt_tmp  = []
                        obj_polygon_x_rt_tmp  = [] 
                        obj_polygon_y_rt_tmp  = [] 
                        for obj in msg_item[1]['obstacles']:
                            obj_x_rt_tmp .append(obj['center']['x'])
                            obj_y_rt_tmp .append(obj['center']['y'])
                            obj_id_rt_tmp .append(obj['id'])
                            tmp_polygon_x = []
                            tmp_polygon_y = []
                            for point in obj['polygon']:
                                tmp_polygon_x.append(point['x'])
                                tmp_polygon_y.append(point['y'])
                            obj_polygon_x_rt_tmp.append(tmp_polygon_x)
                            obj_polygon_y_rt_tmp.append(tmp_polygon_y)
                        obj_x_rt.append(obj_x_rt_tmp)
                        obj_y_rt.append(obj_y_rt_tmp)
                        obj_id_rt.append(obj_id_rt_tmp)
                        obj_polygon_x_rt.append(obj_polygon_x_rt_tmp)
                        obj_polygon_y_rt.append(obj_polygon_y_rt_tmp)

        # plot old dmpp refline
        dm_cur_refline_time_rt = []
        dm_cur_refline_x_rt = []
        dm_cur_refline_y_rt = []
        dm_target_refline_x_rt = []
        dm_target_refline_y_rt = []
        for channel in self.data_set_.keys():
            if channel == DECISION_TARGET_CHANNEL:
                for data_item in self.data_set_[channel]:
                    for msg_item in data_item.items():
                        if msg_item[1] == None:
                            continue
                        
                        dm_cur_refline_time_rt.append(msg_item[0])
                        tmp_x = []
                        tmp_y = []
                        for tmp_point in msg_item[1]['dm_current_refline']['path_pts']:
                            tmp_x.append(tmp_point['x'])
                            tmp_y.append(tmp_point['y'])
                        dm_cur_refline_x_rt.append(tmp_x)
                        dm_cur_refline_y_rt.append(tmp_y)

                        tmp_x = []
                        tmp_y = []
                        for tmp_point in msg_item[1]['dm_target_refline']['path_pts']:
                            tmp_x.append(tmp_point['x'])
                            tmp_y.append(tmp_point['y'])
                        dm_target_refline_x_rt.append(tmp_x)
                        dm_target_refline_y_rt.append(tmp_y)
        
        pp_refline_time_rt = []
        pp_refline_x_rt = []
        pp_refline_y_rt = []
        lateral_r_ref_x_rt = []
        lateral_r_ref_y_rt = []
        lateral_lower_bound_x_rt = []
        lateral_lower_bound_y_rt = []
        lateral_upper_bound_x_rt = []
        lateral_upper_bound_y_rt = []
        for channel in self.data_set_.keys():
            if channel == PLANNING_DEBUG_CHANNEL:
                for data_item in self.data_set_[channel]:
                    for msg_item in data_item.items():
                        if msg_item[1] == None:
                            continue

                        pp_refline_time_rt.append(msg_item[0])
                        tmp_x = []
                        tmp_y = []
                        if 'path_pts' in msg_item[1]['pp_refline']:
                            for tmp_point in msg_item[1]['pp_refline']['path_pts']:
                                tmp_x.append(tmp_point['x'])
                                tmp_y.append(tmp_point['y'])
                        pp_refline_x_rt.append(tmp_x)
                        pp_refline_y_rt.append(tmp_y)

                        tmp_x = []
                        tmp_y = []
                        if 'path_pts' in msg_item[1]['lateral_r_ref']:
                            for tmp_point in msg_item[1]['lateral_r_ref']['path_pts']:
                                tmp_x.append(tmp_point['x'])
                                tmp_y.append(tmp_point['y'])
                        lateral_r_ref_x_rt.append(tmp_x)
                        lateral_r_ref_y_rt.append(tmp_y)

                        tmp_x = []
                        tmp_y = []
                        if 'path_pts' in msg_item[1]['lateral_lower_bound']:
                            for tmp_point in msg_item[1]['lateral_lower_bound']['path_pts']:
                                tmp_x.append(tmp_point['x'])
                                tmp_y.append(tmp_point['y'])
                        lateral_lower_bound_x_rt.append(tmp_x)
                        lateral_lower_bound_y_rt.append(tmp_y)

                        tmp_x = []
                        tmp_y = []
                        if 'path_pts' in msg_item[1]['lateral_upper_bound']:
                            for tmp_point in msg_item[1]['lateral_upper_bound']['path_pts']:
                                tmp_x.append(tmp_point['x'])
                                tmp_y.append(tmp_point['y'])
                        lateral_upper_bound_x_rt.append(tmp_x)
                        lateral_upper_bound_y_rt.append(tmp_y)

        # plot vehicle report
        ego_vel_rt = []
        ego_acc_x_rt =[]
        vehivle_report_time_rt = []

        for channel in self.data_set_.keys():
            if channel == VEHICLE_REPORT_CHANNEL:
                for data_item in self.data_set_[channel]:
                    for msg_item in data_item.items():
                        vehivle_report_time_rt.append(msg_item[0])
                        ego_vel_rt.append(msg_item[1]['vehicleMps'])
                        ego_acc_x_rt.append(msg_item[1]['accelX'])

        # plot control debug
        control_debug_time_rt = []
        ego_auto_enable_rt =[]
        for channel in self.data_set_.keys():
            if channel == CONTROL_DEBUG_CHANNEL:
                for data_item in self.data_set_[channel]:
                    for msg_item in data_item.items():
                        control_debug_time_rt.append(msg_item[0])
                        ego_auto_enable_rt.append(msg_item[1]['vehicleAutoEnable'])

        # plot longi optimizer info from xpilot debug
        xpilot_debug_time_rt = []
        leader_s_rt = []
        leader_v_rt = []
        optimizer_v_out_rt = []
        optimizer_v_ref_rt = []
        optimizer_v_upper_rt = []
        optimizer_s_out_rt = []
        optimizer_s_ref_rt = []
        optimizer_s_upper_rt = []
        optimizer_a_out_rt = []
        optimizer_a_ref_rt = []
        optimizer_a_upper_rt = []
        optimizer_a_lower_rt = []
        optimizer_jerk_out_rt = []
        optimizer_jerk_upper_rt = []
        optimizer_jerk_lower_rt = []
        s_leader_vector_rt = []
        v_leader_vector_rt = []
        s_ref_follow_vector_rt = []
        v_ref_follow_vector_rt = []
        traj_point_x = []
        traj_point_y = []
        curva_speed_limit = []
        curva_speed_limit_s = []
        for channel in self.data_set_.keys():
            if channel == XDEBUG_CHANNEL:
                for data_item in self.data_set_[channel]:
                    for msg_item in data_item.items():
                        if 'leader_s' not in msg_item[1] or 'leader_v' not in msg_item[1]:
                            continue
                        xpilot_debug_time_rt.append(msg_item[0])
                        leader_s_rt.append(msg_item[1]['leader_s'])
                        leader_v_rt.append(msg_item[1]['leader_v'])
                        optimizer_v_out_rt.append(msg_item[1]['optimizer_v_out'])
                        optimizer_v_ref_rt.append(msg_item[1]['optimizer_v_ref'])
                        optimizer_v_upper_rt.append(msg_item[1]['optimizer_v_upper'])
                        optimizer_s_out_rt.append(msg_item[1]['optimizer_s_out'])
                        optimizer_s_ref_rt.append(msg_item[1]['optimizer_s_ref'])
                        optimizer_s_upper_rt.append(msg_item[1]['optimizer_s_upper'])
                        optimizer_a_out_rt.append(msg_item[1]['optimizer_a_out'])
                        optimizer_a_ref_rt.append(msg_item[1]['optimizer_a_ref'])
                        optimizer_a_upper_rt.append(msg_item[1]['optimizer_a_upper'])
                        optimizer_a_lower_rt.append(msg_item[1]['optimizer_a_lower'])
                        optimizer_jerk_out_rt.append(msg_item[1]['optimizer_jerk_out'])
                        optimizer_jerk_upper_rt.append(msg_item[1]['optimizer_jerk_upper'])
                        optimizer_jerk_lower_rt.append(msg_item[1]['optimizer_jerk_lower'])
                        s_leader_vector_rt.append(msg_item[1]['s_leader'])
                        v_leader_vector_rt.append(msg_item[1]['v_leader'])
                        traj_point_x.append(msg_item[1]['traj_point_x'])
                        traj_point_y.append(msg_item[1]['traj_point_y'])
                        s_ref_follow_vector_rt.append(msg_item[1]['s_ref_follow'])
                        v_ref_follow_vector_rt.append(msg_item[1]['v_ref_follow'])
                        curva_speed_limit.append(msg_item[1]['curve_speed_limit'])
                        curva_speed_limit_s.append(msg_item[1]['curve_speed_limit_s'])
                        # print("leaders:",type(s_leader_vector_rt))
                        # print("leaders:",type(optimizer_v_out_rt))                 

        # 规划自车位置随时间变化(下发控制)
                        # print("leaders:",type(leader_s_rt))
                        # print("leaders:",type(optimizer_v_out_rt))

        # plot longi search debug
        lon_search_debug_time_rt, st_path_t_rt, st_path_s_rt, \
            st_sample_points_t_rt, st_sample_points_s_rt, st_sample_points_a_rt, \
            st_sample_points_v_rt, st_sample_points_cost_rt, st_obj_polygon_xs_rt, \
            st_obj_polygon_ys_rt, vt_path_t_rt, vt_path_v_rt, vt_limit_t_rt, vt_limit_v_rt, \
            st_sample_points_color_rt = self.get_lon_search_debug_info('lon_search_debug')

        # plot lat search debug
        lat_search_debug_time_rt, ds_obs_polygon_xs_rt, ds_obs_polygon_ys_rt, \
            ds_bound_s_rt, ds_bound_d_rt, ds_range_d_rt, ds_goal_d_rt, ds_ego_s_rt, \
            ds_ego_d_rt, ds_longi_sample_s_rt, ds_longi_sample_d_rt, ds_lat_sample_s_rt, \
            ds_lat_sample_d_rt, ds_traj_s_rt, ds_traj_d_rt = \
                self.get_lat_search_debug_info('lat_search_debug')
        
        # plot lc_decider longi search debug
        lon_search_debug_time_rt_lc_decider, st_path_t_rt_lc_decider, st_path_s_rt_lc_decider, \
            st_sample_points_t_rt_lc_decider, st_sample_points_s_rt_lc_decider, st_sample_points_a_rt_lc_decider, \
            st_sample_points_v_rt_lc_decider, st_sample_points_cost_rt_lc_decider, st_obj_polygon_xs_rt_lc_decider, \
            st_obj_polygon_ys_rt_lc_decider, vt_path_t_rt_lc_decider, vt_path_v_rt_lc_decider, vt_limit_t_rt_lc_decider, vt_limit_v_rt_lc_decider, \
            st_sample_points_color_rt_lc_decider = self.get_lon_search_debug_info('lc_decider_lon_search_debug')

        # plot lc_decider lat search debug
        lat_search_debug_time_rt_lc_decider, ds_obs_polygon_xs_rt_lc_decider, ds_obs_polygon_ys_rt_lc_decider, \
            ds_bound_s_rt_lc_decider, ds_bound_d_rt_lc_decider, ds_range_d_rt_lc_decider, ds_goal_d_rt_lc_decider, ds_ego_s_rt_lc_decider, \
            ds_ego_d_rt_lc_decider, ds_longi_sample_s_rt_lc_decider, ds_longi_sample_d_rt_lc_decider, ds_lat_sample_s_rt_lc_decider, \
            ds_lat_sample_d_rt_lc_decider, ds_traj_s_rt_lc_decider, ds_traj_d_rt_lc_decider = \
                self.get_lat_search_debug_info('lc_decider_lat_search_debug')
        
        # print(lat_search_debug_time_rt_lc_decider)
        # print(ds_traj_d_rt_lc_decider)
        
        print("*" * 30 + " start adding bag data to source " + "*" * 30)

        #规划自车位置随时间变化
        self.add_layer_traj_line(time_rt,traj_y_rt,traj_x_rt)

        self.add_layer_traj_time_scatter(time_list_=time_rt, x_values_=traj_y_rt,y_values_=traj_x_rt,y_key='trajectory_rt', 
                                         source_key='trajectory_rt_source', figure_key='fig1', legend_label='Trajectory Point Ctrl',
                                         color_='red')
        # 规划自车位置随时间变化
        self.add_layer_traj_time_scatter(time_list_=xpilot_debug_time_rt, x_values_=traj_point_y,y_values_=traj_point_x,y_key='traj_point_rt', 
                                         source_key='traj_point_rt_source', figure_key='fig1', legend_label='Trajectory Point',
                                         color_='purple')
        # 规划曲率随时间变化
        abs_traj_curv_rt = [[abs(x) for x in sublist] for sublist in traj_curv_rt]
        self.add_traj_layer_scatter(time_list_=time_rt, y_values_=abs_traj_curv_rt, y_key='curvature_time', 
                                    source_key='curvature_time_source', figure_key='traj_curv', legend_label='Curv-Time')
        # 规划sum随时间变化
        self.add_traj_layer_scatter(time_list_=time_rt, y_values_=traj_sum_rt, y_key='sum_time', 
                                    source_key='sum_time_source', figure_key='traj_sum_distance', legend_label='Sum-Time')
        # 规划速度随时间变化
        self.add_traj_layer_scatter(time_list_=time_rt, y_values_=traj_vel_rt, y_key='vel_time', 
                                    source_key='vel_time_source', figure_key='traj_vel', legend_label='Vel-Time')

        # planning trajectory yawrate（x-axis is index）
        self.add_traj_layer_scatter(time_list_=time_rt, y_values_=traj_yawrate_rt, y_key='yawrate_time', 
                                    source_key='yawrate_time_source', figure_key='traj_yawrate', legend_label='yawrate-index')

        # planning trajectory theta (x-axis is index)
        self.add_traj_layer_scatter(time_list_=time_rt, y_values_=traj_theta_rt, y_key='theta_time', 
                                    source_key='theta_time_source', figure_key='traj_theta', legend_label='theta-index')    
                        
        # 规划档位随时间变化
        self.add_layer_time_scatter(time_list_=time_rt, y_values_=gear_rt, y_key='gear_time', 
                                    source_key='gear_time_source', figure_key='traj_gear', legend_label='Gear-Time')
        # 规划更新状态
        self.add_time_scatter(time_list_=time_rt, y_values_=update_by_vehsta_rt, y_key='traj_update_current_value', 
                              source_key='traj_update_source_data', figure_key='traj_update', legend_label='Traj-Update-Time')
        # 自车速度随时间变化
        self.add_time_scatter(time_list_=vehivle_report_time_rt, y_values_=ego_vel_rt, y_key='ego_vel_time', 
                              source_key='ego_vel_time_source', figure_key='ego_vel', legend_label='Ego-Vel-Time')
        # 自车加速度随时间变化
        self.add_time_scatter(time_list_=vehivle_report_time_rt, y_values_=ego_acc_x_rt, y_key='ego_acc_x_time', 
                                    source_key='ego_acc_x_time_source', figure_key='ego_accX', legend_label='Ego-accX-Time')
        # 规划s-v的关系
        self.add_layer_traj_time_scatter(time_list_=time_rt, x_values_=traj_sum_rt, y_values_=traj_vel_rt,y_key='sum_vel_rt', 
                                         source_key='sum_vel_rt_source', figure_key='traj_s_v', legend_label='V-S',
                                         color_='black')
        # 规划s-a的关系
        self.add_layer_traj_time_scatter(time_list_=time_rt, x_values_=traj_sum_rt, y_values_=traj_acc_rt,y_key='sum_acc_rt', 
                                         source_key='sum_acc_rt_source', figure_key='traj_s_a', legend_label='A-S',
                                         color_='black')
        # leader_car s-t的关系
        self.add_layer_time_scatter(time_list_=xpilot_debug_time_rt, y_values_=leader_s_rt, y_key='leader_st_time', 
                                         source_key='leader_st_time_source', figure_key='leader_s_t', legend_label='Leader-S-Time')
        # leader_car v-t的关系
        self.add_layer_time_scatter(time_list_=xpilot_debug_time_rt, y_values_=leader_v_rt, y_key='leader_sv_time', 
                                    source_key='leader_sv_time_source', figure_key='leader_v_t', legend_label='Leader-SV-Time')        
        # 是否自驾模式
        self.add_time_scatter(time_list_=control_debug_time_rt, y_values_=ego_auto_enable_rt, y_key='enabel_auto_current_value', 
                              source_key='enabel_auto_source_data', figure_key='enable_auto', legend_label='Ego-Auto-Time')
        # 纵向优化s-v out
        self.add_layer_traj_time_scatter(time_list_=xpilot_debug_time_rt, x_values_=optimizer_s_out_rt, 
                                         y_values_=optimizer_v_out_rt,y_key='optimizer_s_v_rt', 
                                         source_key='optimizer_s_v_rt_source', figure_key='optimizer_s_v', legend_label='V-S-OUT',
                                         color_='black')
        # leader s-v
        self.add_layer_traj_time_scatter(time_list_=xpilot_debug_time_rt, x_values_=s_leader_vector_rt, 
                                         y_values_=v_leader_vector_rt,y_key='leader_s_v_rt', 
                                         source_key='leader_s_v_rt_source', figure_key='optimizer_s_v', legend_label='leader_v-s',
                                         color_='blue')
        # follow s-v
        self.add_layer_traj_time_scatter(time_list_=xpilot_debug_time_rt, x_values_=s_ref_follow_vector_rt, 
                                         y_values_=v_ref_follow_vector_rt,y_key='ref_follow_s_v_rt', 
                                         source_key='ref_follow_s_v_rt_source', figure_key='optimizer_s_v', legend_label='ref_v-s',
                                         color_='red')
        # 纵向优化v-out
        self.add_traj_layer_scatter(time_list_=xpilot_debug_time_rt, y_values_=optimizer_v_out_rt, y_key='optimizer_vel_rt', 
                                    source_key='optimizer_vel_rt_source', figure_key='optimizer_vel', legend_label='Vel-Time')
        # 纵向优化v-ref
        self.add_traj_layer_scatter(time_list_=xpilot_debug_time_rt, y_values_=optimizer_v_ref_rt, y_key='optimizer_vel_ref_rt', 
                                    source_key='optimizer_vel_ref_rt_source', figure_key='optimizer_vel', legend_label='Vel-REF-Time',
                                    color_ = 'red')
        # 纵向优化v-upper
        self.add_traj_layer_scatter(time_list_=xpilot_debug_time_rt, y_values_=optimizer_v_upper_rt, y_key='optimizer_vel_upper_rt', 
                                    source_key='optimizer_vel_upper_rt_source', figure_key='optimizer_vel', legend_label='Vel-UPPER-Time',
                                    color_ = 'blue')
        # 纵向优化a-out
        self.add_traj_layer_scatter(time_list_=xpilot_debug_time_rt, y_values_=optimizer_a_out_rt, y_key='optimizer_acc_rt', 
                                    source_key='optimizer_acc_rt_source', figure_key='optimizer_acc', legend_label='Acc-Time')
        # 纵向优化a-ref
        self.add_traj_layer_scatter(time_list_=xpilot_debug_time_rt, y_values_=optimizer_a_ref_rt, y_key='optimizer_acc_ref_rt', 
                                    source_key='optimizer_acc_ref_rt_source', figure_key='optimizer_acc', legend_label='Acc-Ref-Time',
                                    color_ = 'red')
        # 纵向优化a-upper
        self.add_traj_layer_scatter(time_list_=xpilot_debug_time_rt, y_values_=optimizer_a_upper_rt, y_key='optimizer_acc_upper_rt', 
                                    source_key='optimizer_acc_upper_rt_source', figure_key='optimizer_acc', legend_label='Acc-Upper-Time',
                                    color_ = 'blue')
        # 纵向优化a-lower
        self.add_traj_layer_scatter(time_list_=xpilot_debug_time_rt, y_values_=optimizer_a_lower_rt, y_key='optimizer_acc_lower_rt', 
                                    source_key='optimizer_acc_lower_rt_source', figure_key='optimizer_acc', legend_label='Acc-Lower-Time',
                                    color_ = 'blue')
        # 纵向优化jerk-out
        self.add_traj_layer_scatter(time_list_=xpilot_debug_time_rt, y_values_=optimizer_jerk_out_rt, y_key='optimizer_jerk_rt', 
                                    source_key='optimizer_jerk_rt_source', figure_key='optimizer_jerk', legend_label='Jerk-Time',
                                    color_ = 'black')
        # 纵向优化jerk-upper
        self.add_traj_layer_scatter(time_list_=xpilot_debug_time_rt, y_values_=optimizer_jerk_upper_rt, y_key='optimizer_jerk_upper_rt', 
                                    source_key='optimizer_jerk_upper_rt_source', figure_key='optimizer_jerk', legend_label='Jerk-Upper-Time',
                                    color_ = 'blue')
        # 纵向优化jerk-lower
        self.add_traj_layer_scatter(time_list_=xpilot_debug_time_rt, y_values_=optimizer_jerk_lower_rt, y_key='optimizer_jerk_lower_rt', 
                                    source_key='optimizer_jerk_lower_rt_source', figure_key='optimizer_jerk', legend_label='Jerk-Lower-Time',
                                    color_ = 'blue')
        # 曲率限速s-v
        self.add_layer_traj_time_scatter(time_list_=xpilot_debug_time_rt, x_values_=curva_speed_limit_s, y_values_=curva_speed_limit, y_key='curva_speed_rt', 
                                         source_key='curva_speed_rt_source', figure_key='curva_speed_limit', legend_label='speed-s',
                                         color_='black')
        self.add_layer_predict_traj_scatter(predict_obj_time_rt,predict_obj_x_rt,predict_obj_y_rt)
        self.add_layer_obs_point(obj_time_rt,obj_y_rt,obj_x_rt,obj_id_rt)

        self.add_layer_dmpp_scatter(self.figs_['fig1'], 'dm_cur_refline_rt_source', 'dm_cur_refline_rt', dm_cur_refline_time_rt, dm_cur_refline_x_rt, dm_cur_refline_y_rt, 'DM Cur Refline', 0.3, 0.2, 'blue')
        self.add_layer_dmpp_scatter(self.figs_['fig1'], 'dm_target_refline_rt_source', 'dm_target_refline_rt', dm_cur_refline_time_rt, dm_target_refline_x_rt, dm_target_refline_y_rt, 'DM Target Refline', 0.3, 0.2, 'Green')
        self.add_layer_dmpp_scatter(self.figs_['fig1'], 'pp_refline_rt_source', 'pp_refline_rt', pp_refline_time_rt, pp_refline_x_rt, pp_refline_y_rt, 'PP Refline', 0.3, 0.2, 'coral')
        self.add_layer_dmpp_scatter(self.figs_['fig1'], 'lateral_r_ref_rt_source', 'lateral_r_ref_rt', pp_refline_time_rt, lateral_r_ref_x_rt, lateral_r_ref_y_rt, 'Lateral R Ref', 6, 0.2, 'blueviolet')
        self.add_layer_dmpp_scatter(self.figs_['fig1'], 'lateral_lower_bound_rt_source', 'lateral_lower_bound_rt', pp_refline_time_rt, lateral_lower_bound_x_rt, lateral_lower_bound_y_rt, 'Lateral Lower Bound', 6, 0.2, 'burlywood')
        self.add_layer_dmpp_scatter(self.figs_['fig1'], 'lateral_upper_bound_rt_source', 'lateral_upper_bound_rt', pp_refline_time_rt, lateral_upper_bound_x_rt, lateral_upper_bound_y_rt, 'Lateral Upper Bound', 6, 0.2, 'cadetblue')
        self.add_layer_dmpp_line(self.figs_['fig1'], 'lateral_r_ref_rt_source', 'lateral_r_ref_rt', pp_refline_time_rt, lateral_r_ref_x_rt, lateral_r_ref_y_rt, 'Lateral R Ref', 4, 0.2, 'blueviolet')
        self.add_layer_dmpp_line(self.figs_['fig1'], 'lateral_lower_bound_rt_source', 'lateral_lower_bound_rt', pp_refline_time_rt, lateral_lower_bound_x_rt, lateral_lower_bound_y_rt, 'Lateral Lower Bound', 4, 0.2, 'burlywood')
        self.add_layer_dmpp_line(self.figs_['fig1'], 'lateral_upper_bound_rt_source', 'lateral_upper_bound_rt', pp_refline_time_rt, lateral_upper_bound_x_rt, lateral_upper_bound_y_rt, 'Lateral Upper Bound', 4, 0.2, 'cadetblue')

        self.add_lon_search_st_graph('lon_search_st', '', lon_search_debug_time_rt, st_path_t_rt, st_path_s_rt, \
            st_sample_points_t_rt, st_sample_points_s_rt, st_sample_points_a_rt, \
            st_sample_points_v_rt, st_sample_points_cost_rt, st_obj_polygon_xs_rt, \
            st_obj_polygon_ys_rt, vt_path_t_rt, vt_path_v_rt, vt_limit_t_rt, vt_limit_v_rt, \
            st_sample_points_color_rt)
        
        self.add_lon_search_vt_graph('lon_search_vt', '', lon_search_debug_time_rt, st_path_t_rt, st_path_s_rt, \
            st_sample_points_t_rt, st_sample_points_s_rt, st_sample_points_a_rt, \
            st_sample_points_v_rt, st_sample_points_cost_rt, st_obj_polygon_xs_rt, \
            st_obj_polygon_ys_rt, vt_path_t_rt, vt_path_v_rt, vt_limit_t_rt, vt_limit_v_rt, \
            st_sample_points_color_rt)
        
        self.add_lat_search_sd_graph('lat_search_sd', '', lat_search_debug_time_rt, ds_obs_polygon_xs_rt, ds_obs_polygon_ys_rt, \
            ds_bound_s_rt, ds_bound_d_rt, ds_range_d_rt, ds_goal_d_rt, ds_ego_s_rt, \
            ds_ego_d_rt, ds_longi_sample_s_rt, ds_longi_sample_d_rt, ds_lat_sample_s_rt, \
            ds_lat_sample_d_rt, ds_traj_s_rt, ds_traj_d_rt)
        
        self.add_lon_search_st_graph('lc_decider_lon_search_st', '_lc_decider', lon_search_debug_time_rt_lc_decider, st_path_t_rt_lc_decider, st_path_s_rt_lc_decider, \
            st_sample_points_t_rt_lc_decider, st_sample_points_s_rt_lc_decider, st_sample_points_a_rt_lc_decider, \
            st_sample_points_v_rt_lc_decider, st_sample_points_cost_rt_lc_decider, st_obj_polygon_xs_rt_lc_decider, \
            st_obj_polygon_ys_rt_lc_decider, vt_path_t_rt_lc_decider, vt_path_v_rt_lc_decider, vt_limit_t_rt_lc_decider, vt_limit_v_rt_lc_decider, \
            st_sample_points_color_rt_lc_decider)
        
        self.add_lat_search_sd_graph('lc_decider_lat_search_sd', '_lc_decider', lat_search_debug_time_rt_lc_decider, ds_obs_polygon_xs_rt_lc_decider, ds_obs_polygon_ys_rt_lc_decider, \
            ds_bound_s_rt_lc_decider, ds_bound_d_rt_lc_decider, ds_range_d_rt_lc_decider, ds_goal_d_rt_lc_decider, ds_ego_s_rt_lc_decider, \
            ds_ego_d_rt_lc_decider, ds_longi_sample_s_rt_lc_decider, ds_longi_sample_d_rt_lc_decider, ds_lat_sample_s_rt_lc_decider, \
            ds_lat_sample_d_rt_lc_decider, ds_traj_s_rt_lc_decider, ds_traj_d_rt_lc_decider)


        self.add_slider()
        self.add_rt_callback()

        self.activate_figure_option()
        self.add_hover_tool()

        self.show2html()
        # self.show2notebook()

    def show2html(self):
        output_file(self.output_)
        col1 = column(
            row(self.figs_['enable_auto'], self.figs_['ego_vel'],self.figs_['traj_update'], self.figs_['ego_accX']),
            row(self.figs_['traj_yawrate'], self.figs_['traj_theta'], self.figs_['curva_speed_limit']),
            row(
                column(self.checkbox_groups_['traj_s_v'], self.figs_['traj_s_v']),
                column(self.checkbox_groups_['traj_s_a'], self.figs_['traj_s_a']),
                column(self.checkbox_groups_['traj_vel'], self.figs_['traj_vel']),
            ),
            row(
                column(self.checkbox_groups_['traj_curv'], self.figs_['traj_curv']),
                column(self.checkbox_groups_['optimizer_s_v'], self.figs_['optimizer_s_v']),
                column(self.checkbox_groups_['optimizer_vel'], self.figs_['optimizer_vel']),
                
            ),
            row(
                column(self.checkbox_groups_['optimizer_acc'], self.figs_['optimizer_acc']),
                column(self.checkbox_groups_['optimizer_jerk'], self.figs_['optimizer_jerk']),
                column(self.checkbox_groups_['lat_search_sd'], self.figs_['lat_search_sd']),
            ),
            row(               
                column(self.checkbox_groups_['lon_search_st'], self.figs_['lon_search_st']),
                column(self.checkbox_groups_['lon_search_vt'], self.figs_['lon_search_vt']),
                column(self.checkbox_groups_['lc_decider_lat_search_sd'], self.figs_['lc_decider_lat_search_sd']),
            ),
            row(
                column(self.checkbox_groups_['lc_decider_lon_search_st'], self.figs_['lc_decider_lon_search_st']),
            ),
        )
        row1 = row(self.figs_['fig1'], col1)
        layout = column(self.time_slider_, row1)
        show(layout)
        return

    def show2notebook(self):
        output_notebook()
        show(column(self.time_slider_,self.fig_))
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

    hviz_plot = PlotHopViz(args.source,args.output)

    #decision channel
    hviz_plot.add_required_channel(DECISION_DEBUG_CHANNEL)
    hviz_plot.add_required_channel(DECISION_TARGET_CHANNEL)
    #planning channel
    hviz_plot.add_required_channel(PLANNING_CHANNEL)
    hviz_plot.add_required_channel(XDEBUG_CHANNEL)
    hviz_plot.add_required_channel(PLANNING_DEBUG_CHANNEL)
    #perception channel
    hviz_plot.add_required_channel(PERCEPTION_CHANNEL)
    #prediction channel
    hviz_plot.add_required_channel(PREDICTION_CHANNEL)
    #vehicle state channel
    hviz_plot.add_required_channel(VEHICLE_REPORT_CHANNEL)
    #control channel
    hviz_plot.add_required_channel(CONTROL_DEBUG_CHANNEL)

    hviz_plot.figure_plot()

