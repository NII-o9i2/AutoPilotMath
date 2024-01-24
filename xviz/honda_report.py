import bokeh
import math
from bokeh.io import curdoc
from bokeh.plotting import figure, show, output_file, output_notebook
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
from bokeh.models import CustomJS, CheckboxButtonGroup
from old_framework_baseplot import PlotBase


class PlotXpilotViz(PlotBase):
    def __init__(self, bag_path, output):
        super().__init__(bag_path, output)

    def creat_figure(self):
        # main hmi window
        self.figs_['fig1'] = figure(
            width=800, height=800, match_aspect=True, title='Xviz')
        self.figs_['ego_target_vel'] = figure(
            width=400, height=400, match_aspect=True, title='ego_vel', y_range=[0, 25])
        self.figs_['ego_acc'] = figure(
            width=400, height=400, match_aspect=True, title='ego_acc', y_range=[-2, 2])
        self.figs_['follow_car'] = figure(
            width=400, height=400, match_aspect=True, title='follow_car',y_range=[0, 100])
        self.figs_['road_curva'] = figure(
            width=400, height=400, match_aspect=True, title='road_curva',x_range=[0,40],y_range=[0, 15000])
        self.figs_['ego_steering'] = figure(
            width=400, height=400, match_aspect=True, title='ego_steering', y_range=[-10, 10])
        self.figs_['ego_heading'] = figure(
            width=400, height=400, match_aspect=True, title='ego_heading', y_range=[-3.14, 3.14])
        self.figs_['ego_yaw_rate'] = figure(
            width=400, height=400, match_aspect=True, title='ego_yaw_rate', y_range=[-0.1, 0.1])
        self.figs_['lateral_offset_error'] = figure(
            width=400, height=400, match_aspect=True, title='lateral_offset_error', y_range=[-4, 4])
        self.figs_['lateral_heading_error'] = figure(
            width=400, height=400, match_aspect=True, title='lateral_heading_error', y_range=[-0.15, 0.15])
        self.figs_['nop_state'] = figure(
            width=400, height=400, match_aspect=True, title='nop_state', y_range=[0, 2])
        self.figs_['steering_torque'] = figure(
            width=400, height=400, match_aspect=True, title='steering_torque', y_range=[-2, 2])
        for fig_name in []:
            fig = self.figs_[fig_name]
            checkbox_group = CheckboxButtonGroup(
                labels=[fig_name], active=[0], width=800)
            callback = CustomJS(args=dict(fig=fig, checkbox_group=checkbox_group), code="""
                fig.visible = checkbox_group.active.includes(0);
            """)
            checkbox_group.js_on_change('active', callback)
            # Add the checkbox group to the dictionary
            self.checkbox_groups_[fig_name] = checkbox_group

    # 根据滑块显示对应时间点数据
    def add_time_line(self, time_list_, y_values_, y_key, source_key, figure_key,
                      legend_label, color_, line_width_=1, line_alpha_=0.7):
        self.plot_data_set_[source_key] = ColumnDataSource(data=dict(
            time=time_list_,
            y=y_values_
        ))
        self.plot_data_set_[y_key] = ColumnDataSource(data=dict(
            time=[],
            y=[],
        ))
        self.figs_[figure_key].line('time', 'y', source=self.plot_data_set_[source_key],
                                    color=color_, line_width=line_width_,
                                    line_alpha=line_alpha_, legend_label=legend_label)
        # self.figs_[figure_key].scatter('time', 'y', source=self.plot_data_set_[y_key],
        #                             color='black', size=10)
        self.figs_[figure_key].circle('time', 'y', source=self.plot_data_set_[y_key],
                                      color='red', size=5, alpha=1)
        self.figs_[figure_key].xaxis.axis_label = "time/s"
        self.figs_[figure_key].yaxis.axis_label = "m/s"
        return

    def add_time_line_2(self, time_list_, y_values_, source_key, figure_key,
                        legend_label, color_, xaxis, yaxis, line_width_=1, line_alpha_=0.7):
        self.plot_data_set_[source_key] = ColumnDataSource(data=dict(
            time=time_list_,
            y=y_values_
        ))
        self.figs_[figure_key].line('time', 'y', source=self.plot_data_set_[source_key],
                                    color=color_, line_width=line_width_,
                                    line_alpha=line_alpha_, legend_label=legend_label)
        self.figs_[figure_key].xaxis.axis_label = xaxis
        self.figs_[figure_key].yaxis.axis_label = yaxis
        return

    def add_rt_callback(self):
        self.callback_ = CustomJS(args=dict(time=self.time_slider_,
                                            ego_vel_source=self.plot_data_set_[
                                                'ego_vel_time_source'],
                                            ego_vel_current_value_source=self.plot_data_set_[
                                                'ego_vel_time'],

                                            target_vel_source=self.plot_data_set_[
                                                'target_vel_time_source'],
                                            target_vel_current_value_source=self.plot_data_set_[
                                                'target_vel_time'],
                                            ),
                                  code="""
    const cur_time = time.value
    const time_list = ego_vel_source.data['time'];
    const y_values_vehivle_vel = ego_vel_source.data['y'];
    ego_vel_current_value_source.data['time'] = [];
    ego_vel_current_value_source.data['y'] = [];
    for (let i = 0; i < time_list.length; i++) {
        if ((cur_time - time_list[i] < 0.1) &&
            (cur_time - time_list[i] > 0)) {
            ego_vel_current_value_source.data['time'].push(time_list[i]);
            ego_vel_current_value_source.data['y'].push(y_values_vehivle_vel[i]);
        }
    }
    ego_vel_current_value_source.change.emit();
    const y_values_vehivle_set_vel = target_vel_source.data['y'];
    target_vel_current_value_source.data['time'] = [];
    target_vel_current_value_source.data['y'] = [];
    for (let i = 0; i < time_list.length; i++) {
        if ((cur_time - time_list[i] < 0.1) &&
            (cur_time - time_list[i] > 0)) {
            target_vel_current_value_source.data['time'].push(time_list[i]);
            target_vel_current_value_source.data['y'].push(y_values_vehivle_set_vel[i]);
        }
    }
    target_vel_current_value_source.change.emit();
                        """)
        self.time_slider_.js_on_change('value', self.callback_)
        pass

    def add_slider(self):
        self.time_slider_ = Slider(
            start=0.0, end=self.bag_time_length_, value=0.0, step=.1, title="Time")

    def figure_plot(self):
        if self.read_bag_flag_ == False:
            self.read_bag()
            self.match_timestamp()
        self.creat_figure()
        # plot hodna report
        # 巡航速度与自车车速
        time_rt = []
        time_relative_speed_rt = []
        time_target_dis_rt = []
        time_real_dis_rt = []
        ego_vel_rt = []
        set_speed_rt = []
        # 实际道路和自车行驶位置
        ego_position_x = []
        ego_position_y = []
        left_lane_x = []
        left_lane_y = []
        right_lane_x = []
        right_lane_y = []
        ego_position_x = []
        ego_position_y = []
        # 自车加速度与planning加速度随时间变化
        planning_acc = []
        ego_acc = []
        # 实际跟车距离与目标跟车距离随时间变化
        follow_dis_desire = []
        follow_dis_real = []
        # 道路曲率随时间变化
        road_curva = []
        # 自车方向盘转角随时间变化
        ego_steer_angle = []
        # 自车航向角随时间变化
        ego_heading = []
        # 横向位置跟踪误差随时间变化
        lateral_offset_error = []
        # 横向曲率跟踪误差随时间变化
        lateral_heading_error = []

        has_leader = []
        relative_speed = []
        ego_yaw_rate = []
        plot_relative_speed_flag = False

        for channel in self.data_set_.keys():
            if channel == XDEBUG_CHANNEL:
                for data_item in self.data_set_[channel]:
                    for msg_item in data_item.items():
                        time_rt.append(msg_item[0])
                        ego_vel_rt.append(msg_item[1]['ego_vel'])
                        if msg_item[1]['set_speed'] < 0.01:
                            set_speed_rt.append(set_speed_rt[-1])
                        else:
                            set_speed_rt.append(msg_item[1]['set_speed'])
                        left_lane_x.append(msg_item[1]['left_lane_x'])
                        left_lane_y.append(msg_item[1]['left_lane_y'])
                        right_lane_x.append(msg_item[1]['right_lane_x'])
                        right_lane_y.append(msg_item[1]['right_lane_y'])
                        ego_position_x.append(msg_item[1]['ego_position_x'])
                        ego_position_y.append(msg_item[1]['ego_position_y'])
                        lateral_heading_error.append(
                            msg_item[1]['lateral_heading_error'])
                        lateral_offset_error.append(
                            msg_item[1]['lateral_offset_error']+0.2)
                        ego_heading.append(msg_item[1]['ego_heading'])
                        ego_steer_angle.append(
                            msg_item[1]['ego_steer_angle']*180/math.pi)
                        if (abs(msg_item[1]['road_curva']) < 0.0001):
                            road_curva.append(10000)
                        else:
                            road_curva.append(1 / abs(msg_item[1]['road_curva']))
                        # road_curva.append(1 / msg_item[1]['road_curva'])
                        if msg_item[1]['follow_dis_real'] < 0.1:
                            follow_dis_real.append(float('nan'))
                            time_real_dis_rt.append(float('nan'))
                        else:
                            follow_dis_real.append(msg_item[1]['follow_dis_real'])
                            time_real_dis_rt.append(msg_item[0])

                        if msg_item[1]['follow_dis_desire'] < 0.1:
                            follow_dis_desire.append(float('nan'))
                            time_target_dis_rt.append(float('nan'))
                        else:
                            follow_dis_desire.append(msg_item[1]['follow_dis_desire'])
                            time_target_dis_rt.append(msg_item[0])
                            
                        # ego_acc.append(msg_item[1]['ego_acc'])
                        planning_acc.append(msg_item[1]['planning_acc'])
                        has_leader.append(msg_item[1]['has_leader'])
                        if msg_item[1]['relative_speed'] == 0:
                            relative_speed.append(float('nan'))
                            time_relative_speed_rt.append(float('nan'))
                        else:
                            relative_speed.append(msg_item[1]['relative_speed'])
                            time_relative_speed_rt.append(msg_item[0])
                        ego_yaw_rate.append(msg_item[1]['ego_yaw_rate'])
        for lead in has_leader:
            if lead == True:
                plot_relative_speed_flag = True
                self.figs_['relative_speed'] = figure(
                    width=400, height=400, match_aspect=True, title='relative_speed', y_range=[-10, 10])
                break

        control_time = []
        accCmd = []
        velCmd = []
        for channel in self.data_set_.keys():
            if channel == CONTROL_DEBUG_CHANNEL:
                for data_item in self.data_set_[channel]:
                    for msg_item in data_item.items():
                        control_time.append(msg_item[0])
                        accCmd.append(msg_item[1]['accCmd'])
                        velCmd.append(msg_item[1]['velCmd'])

        a_channel_time = []
        steering_torque_act = []
        for channel in self.data_set_.keys():
            if channel == CHASSIS_A_CHANNEL:
                for data_item in self.data_set_[channel]:
                    for msg_item in data_item.items():
                        a_channel_time.append(msg_item[0])
                        steering_torque_act.append(msg_item[1]['steering_torque_act'])
                        ego_acc.append(msg_item[1]['ego_acc'])

        # 自车实际速度与目标速度与设定速度随时间变化
        self.add_time_line(time_list_=time_rt, y_values_=ego_vel_rt, y_key='ego_vel_time',
                           source_key='ego_vel_time_source', figure_key='ego_target_vel',
                           legend_label='EgoSpeed', color_='blue')
        self.add_time_line(time_list_=time_rt, y_values_=set_speed_rt, y_key='target_vel_time',
                           source_key='target_vel_time_source', figure_key='ego_target_vel',
                           legend_label='SetSpeed', color_='red')
        self.add_time_line(time_list_=control_time, y_values_=velCmd, y_key='velCmd_time',
                           source_key='velCmd_time_source', figure_key='ego_target_vel',
                           legend_label='PlanSpeed', color_='green')
        # 自车加速度与planning加速度随时间变化
        self.add_time_line_2(time_list_=control_time, y_values_=accCmd,
                             source_key='ego_acc_source', figure_key='ego_acc',
                             legend_label='PlanAcceleration', color_='red', xaxis='time', yaxis='m/s^2')
        self.add_time_line_2(time_list_=a_channel_time, y_values_=ego_acc,
                             source_key='ego_acc_source', figure_key='ego_acc',
                             legend_label='EgoAcceleration', color_='blue', xaxis='time/s', yaxis='m/s^2')

        # 实际跟车距离与目标跟车距离随时间变化
        self.add_time_line_2(time_list_=time_target_dis_rt, y_values_=follow_dis_desire,
                             source_key='follow_dis_desire', figure_key='follow_car',
                             legend_label='TargetVehicleFollowDistance', color_='red', xaxis='time/s', yaxis='m')
        self.add_time_line_2(time_list_=time_real_dis_rt, y_values_=follow_dis_real,
                             source_key='follow_dis_real', figure_key='follow_car',
                             legend_label='ActualVehicleFollowDistance', color_='blue', xaxis='time/s', yaxis='m')

        # 跟车相对速度随时间变化
        if plot_relative_speed_flag == True:
            self.add_time_line_2(time_list_=time_relative_speed_rt, y_values_=relative_speed,
                             source_key='relative_speed', figure_key='relative_speed',
                             legend_label='VehicleFollowRelativeSpeed', color_='blue', xaxis='time/s', yaxis='m/s')

        # 道路曲率随时间变化
        self.add_time_line_2(time_list_=time_rt, y_values_=road_curva,
                             source_key='road_curva', figure_key='road_curva',
                             legend_label='RoadCurvatureRadius', color_='red', xaxis='time/s', yaxis='m')

        # 自车方向盘转角随时间变化
        self.add_time_line_2(time_list_=time_rt, y_values_=ego_steer_angle,
                             source_key='ego_steer_angle', figure_key='ego_steering',
                             legend_label='EgoActualSteeringAngle', color_='red', xaxis='time/s', yaxis='deg')
        # 自车yawrate随时间变化
        self.add_time_line_2(time_list_=time_rt, y_values_=ego_yaw_rate,
                             source_key='ego_yaw_rate', figure_key='ego_yaw_rate',
                             legend_label='EgoActualYawRate', color_='red', xaxis='time/s', yaxis='rad')

        # 自车航向角随时间变化
        self.add_time_line_2(time_list_=time_rt, y_values_=ego_heading,
                             source_key='ego_heading', figure_key='ego_heading',
                             legend_label='EgoActualHeadingAngle', color_='red', xaxis='time/s', yaxis='rad')

        # 横向位置跟踪误差随时间变化
        self.add_time_line_2(time_list_=time_rt, y_values_=lateral_offset_error,
                             source_key='lateral_offset_error', figure_key='lateral_offset_error',
                             legend_label='LateralOffsetError', color_='red', xaxis='time/s', yaxis='m')

        # 横向曲率跟踪误差随时间变化
        self.add_time_line_2(time_list_=time_rt, y_values_=lateral_heading_error,
                             source_key='lateral_heading_error', figure_key='lateral_heading_error',
                             legend_label='LateralHeadingError', color_='red', xaxis='time/s', yaxis='rad')
        # nop状态
        nop_state=[]
        for counter in time_rt:
            nop_state.append(1)
        self.add_time_line_2(time_list_=time_rt, y_values_=nop_state,
                             source_key='nop_state', figure_key='nop_state',
                             legend_label='NopState', color_='red', xaxis='time/s', yaxis='')

        vehicle_command_time = []
        steering_torque_command = []
        for channel in self.data_set_.keys():
            if channel == VEHICLE_COMMAND_CHANNEL:
                for data_item in self.data_set_[channel]:
                    for msg_item in data_item.items():
                        vehicle_command_time.append(msg_item[0])
                        steering_torque_command.append(msg_item[1]['steering_torque_command'])
        # 实际扭矩与目标扭矩
        self.add_time_line_2(time_list_=vehicle_command_time, y_values_=steering_torque_command,
                             source_key='steering_torque', figure_key='steering_torque',
                             legend_label='TargetSteeringTorque', color_='red', xaxis='time/s', yaxis='N*m')
        self.add_time_line_2(time_list_=a_channel_time, y_values_=steering_torque_act,
                             source_key='steering_torque', figure_key='steering_torque',
                             legend_label='ActualSteeringTorque', color_='blue', xaxis='time/s', yaxis='N*m')

        # 实际道路和自车行驶位置
        self.plot_data_set_['left_lane'] = ColumnDataSource(data=dict(
            x=left_lane_x,
            y=left_lane_y,
        ))
        self.figs_['fig1'].scatter('x', 'y', source=self.plot_data_set_['left_lane'],
                                   color='orange', line_alpha=0.4, legend_label='LeftLane', size=3)

        self.plot_data_set_['right_lane'] = ColumnDataSource(data=dict(
            x=right_lane_x,
            y=right_lane_y,
        ))
        self.figs_['fig1'].scatter('x', 'y', source=self.plot_data_set_['right_lane'],
                                   color='green', line_alpha=0.4, legend_label='RightLane', size=3)

        self.plot_data_set_['ego actual path'] = ColumnDataSource(data=dict(
            x=ego_position_x,
            y=ego_position_y,
        ))
        self.figs_['fig1'].line('x', 'y', source=self.plot_data_set_['ego actual path'],
                                color='blue', line_alpha=0.4, legend_label='EgoActualpath', line_width=3)

        self.add_slider()
        self.add_rt_callback()
        self.activate_figure_option()
        self.show2html(plot_relative_speed_flag)
        # self.show2notebook()

    def show2html(self, plot_relative_speed_flag):
        output_file(self.output_)
        if plot_relative_speed_flag:
            row1 = row(column(self.figs_['fig1']),
                       column(self.figs_['ego_target_vel'],
                              self.figs_['ego_acc'],
                              self.figs_['follow_car'],
                              self.figs_['relative_speed']),
                       column(self.figs_['road_curva'],
                              self.figs_['ego_steering'],
                              self.figs_['ego_heading'],
                              self.figs_['ego_yaw_rate']),
                       column(self.figs_['lateral_offset_error'],
                              self.figs_['lateral_heading_error'],
                              self.figs_['nop_state'],
                              self.figs_['steering_torque']))
        else:
            row1 = row(column(self.figs_['fig1']),
                       column(self.figs_['ego_target_vel'],
                              self.figs_['ego_acc'],
                              self.figs_['follow_car']),
                       column(self.figs_['road_curva'],
                              self.figs_['ego_steering'],
                              self.figs_['ego_heading'],
                              self.figs_['ego_yaw_rate']),
                       column(self.figs_['lateral_offset_error'],
                              self.figs_['lateral_heading_error'],
                              self.figs_['nop_state'],
                              self.figs_['steering_torque']))

        layout = column(self.time_slider_, row1)
        # layout = column(row1)
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

    xpilot_plot = PlotXpilotViz(args.source, args.output)

    xpilot_plot.add_required_channel(XDEBUG_CHANNEL)
    xpilot_plot.add_required_channel(CONTROL_DEBUG_CHANNEL)
    xpilot_plot.add_required_channel(CHASSIS_A_CHANNEL)
    xpilot_plot.add_required_channel(VEHICLE_COMMAND_CHANNEL)

    xpilot_plot.figure_plot()
