from bokeh.plotting import figure, show, output_file, output_notebook
import argparse
from scipy.interpolate import interp1d
import numpy as np
import os
import json

import bokeh
from bokeh.plotting import figure
from bokeh.layouts import column, row
from bokeh.io import output_file
from bokeh.models import WheelZoomTool

import sys
sys.path.append('/home/sensetime/ws/common_math/algorithm/build/env_simulator')
sys.path.append('/home/sensetime/ws/common_math/algorithm/build/cilqr')
sys.path.append('/home/sensetime/ws/common_math/algorithm/cilqr')
import pybind_ilqr

def vehicle_model_test():
    param=pybind_ilqr.ILQRParam()
    param.delta_t = 0.2
    param.horizon =  30
    param.max_iter_num = 100
    param.tol = 1e-3

    vehicle_model = pybind_ilqr.VehicleModelBicycle()
    vehicle_model.update_parameter(param)

    state = [3,4,20,1.0]
    action = [0.5, 0.02]

    next_state = vehicle_model.step_std(state,action)
    print( "output: " , next_state)
    
def vehicle_model_kappa_test():
    param=pybind_ilqr.ILQRParam()
    param.delta_t = 0.2
    param.horizon =  30
    param.max_iter_num = 100
    param.tol = 1e-3
    param.kappa_thr = 1e-5

    vehicle_model = pybind_ilqr.VehicleModelBicycle()
    vehicle_model.update_parameter(param)

    state = [3,4,20,1.0]
    action = [0.5, 0.02]

    next_state = vehicle_model.step_kappa_std(state,action)
    print( "output: " , next_state)

def vehicle_model_kappa_fuzzy_test():
    param=pybind_ilqr.ILQRParam()
    param.delta_t = 0.2
    param.horizon =  30
    param.max_iter_num = 100
    param.tol = 1e-3
    param.kappa_thr = 1e-5

    vehicle_model = pybind_ilqr.VehicleModelBicycle()
    vehicle_model.update_parameter(param)

    state = [3,4,20,1.0]
    action = [0.5, 0.02]

    next_state = vehicle_model.step_kappa_fuzzy_std(state,action)
    print( "output: " , next_state)

def vehicle_model_delta_test():
    param=pybind_ilqr.ILQRParam()
    param.delta_t = 0.2
    param.horizon =  30
    param.max_iter_num = 100
    param.tol = 1e-3
    param.kappa_thr = 1e-5

    vehicle_model = pybind_ilqr.VehicleModelBicycle()
    vehicle_model.update_parameter(param)
    delta= 0.2
    wheel_base = 2.8
    kappa = tan(delta) / wheel_base
    state = [3,4,20,1.0]
    action = [0.5, kappa]

    next_state = vehicle_model.step_kappa_std(state,action)
    print( "output: " , next_state)

def check_kappa_thr_diff(kappa_thr):
    diff_point_list = []
    theta_init_list = []

    v_init = 20.0
    # first use raw cal 
    param=pybind_ilqr.ILQRParam()
    param.delta_t = 0.2
    param.horizon =  30
    param.max_iter_num = 100
    param.tol = 1e-3
    param.kappa_thr = kappa_thr * 0.5

    vehicle_model = pybind_ilqr.VehicleModelBicycle()
    vehicle_model.update_parameter(param)

    theta_init_array = np.arange(0, np.pi + 0.1, 0.1)
    l = param.delta_t * v_init
    kappa_input = kappa_thr / l

    for theta_init in theta_init_array:

        param.kappa_thr = kappa_thr * 0.5
        vehicle_model.update_parameter(param)

        state = [3,4,v_init,theta_init]
        action = [0.0, kappa_input]

        next_state_1 = vehicle_model.step_kappa_std(state,action)
    #     print("kappa", kappa_input, "output: " , next_state_1)

        # first use fuzzy cal 
        param.kappa_thr = kappa_thr * 2
        vehicle_model.update_parameter(param)

        next_state_2 = vehicle_model.step_kappa_std(state,action)
    #     print("kappa", kappa_input, "output: " , next_state_2)
        diff = (next_state_2[0] - next_state_1[0]) **2 +(next_state_2[1] - next_state_1[1]) **2
        theta_init_list.append(theta_init)
        diff_point_list.append(diff)

    print("theta_thr: ",kappa_thr)
    p = figure(width=1200, height=400)

    p.scatter(theta_init_list, diff_point_list)
    output_notebook()
    show(p)
    
def vehicle_model_diff_test():
    param = pybind_ilqr.ILQRParam()
    param.delta_t = 0.2
    param.horizon = 30
    param.max_iter_num = 100
    param.tol = 1e-3

    vehicle_model = pybind_ilqr.VehicleModelBicycle()
    vehicle_model.update_parameter(param)
    #state: x, y, v, theta
    state_list = []
    #action:a,w
    action_list = []

    path = '/home/sensetime/ws/common_math/algorithm/cilqr/data'
    for filename in os.listdir(path):
        if filename.endswith('.json'):  # 确保文件是以 .json 结尾的 JSON 文件
            file_path = os.path.join(path, filename)
            # 读取 JSON 文件
            with open(file_path, 'r') as file:
                data = json.load(file)
            # state : x, y, v, theta
            for data_frame in data:
                for key in data_frame:
                    temp_state = []
                    temp_state.append(data_frame[key]['pos_x'])
                    temp_state.append(data_frame[key]['pos_y'])
                    temp_state.append(data_frame[key]['v'])
                    temp_state.append(data_frame[key]['theta'])
                    state_list.append(temp_state)

                    temp_action = []
                    temp_action.append(data_frame[key]['a'])
                    temp_action.append(data_frame[key]['w'])
                    action_list.append(temp_action)
    state_input=state_list[0]
    next_state_tmp = []
    next_state_list=[]
    for i in range(len(state_list)):
        next_state_tmp=vehicle_model.step_std(state_input, action_list[i])
        next_state_list.append(next_state_tmp)
        state_input=next_state_tmp
    # output
    next_x_list = []
    next_y_list = []
    for next_state in next_state_list:
        next_x_list.append(next_state[0])
        next_y_list.append(next_state[1])

    # input
    state_x = []
    state_y = []
    state_v = []
    state_theta = []

    for state in state_list:
        state_x.append(state[0])
        state_y.append(state[1])
        state_v.append(state[2])
        state_theta.append(state[3])

    fig_x_y = figure(width=800, height=400, match_aspect=True)
    fig_x_y.scatter(next_x_list, next_y_list, color='red',
                    line_alpha=0.4, legend_label="x_out-y_out")
    fig_x_y.scatter(state_x, state_y, color='green',
                    line_alpha=0.4, legend_label="x_real-y_real")
    fig_x_y.xaxis.axis_label = "x"
    fig_x_y.yaxis.axis_label = "y"

    fig_x_y.toolbar.active_scroll = fig_x_y.select_one(WheelZoomTool)
    fig_x_y.legend.click_policy = 'hide'
    output_notebook()
    show(fig_x_y)
    # print("output: ", next_state_list)

# kappa vehicle model
def vehicle_model_kappa_diff_test():
    param=pybind_ilqr.ILQRParam()
    param.delta_t = 0.2
    param.horizon =  30
    param.max_iter_num = 100
    param.tol = 1e-3
    param.kappa_thr = 1e-5

    vehicle_model = pybind_ilqr.VehicleModelBicycle()
    vehicle_model.update_parameter(param)

    #state: x, y, v, theta
    state_list = []
    #action:a,delta
    action_list = []
    action_list_w = []

    current_path = os.getcwd()
    path = current_path + "/data"
    for filename in os.listdir(path):
        if filename.endswith('.json'):  # 确保文件是以 .json 结尾的 JSON 文件
            file_path = os.path.join(path, filename)

            # 读取 JSON 文件
            with open(file_path, 'r') as file:
                data = json.load(file)
                   
            # state : x, y, v, theta
            for data_frame in data:
                for key in data_frame:
                    temp_state = []
                    temp_state.append(data_frame[key]['pos_x'])
                    temp_state.append(data_frame[key]['pos_y'])
                    temp_state.append(data_frame[key]['v'])
                    temp_state.append(data_frame[key]['theta'])
                    state_list.append(temp_state)

                    temp_action = []
                    temp_action.append(data_frame[key]['a'])
                    temp_action.append(data_frame[key]['w']/data_frame[key]['v'])
                    action_list.append(temp_action)
                    temp_action = []
                    temp_action.append(data_frame[key]['a'])
                    temp_action.append(data_frame[key]['w'])
                    action_list_w.append(temp_action)

    state_input=state_list[0]
    next_state_tmp = []
    next_state_list=[]
    
    for i in range(len(state_list)):
        next_state_tmp = vehicle_model.step_kappa_std(state_input,action_list[i])
        next_state_list.append(next_state_tmp)
        state_input=next_state_tmp
        
    
    state_input=state_list[0]
    next_state_tmp = []
    next_state_list_w=[]
    for i in range(len(state_list)):
        next_state_tmp=vehicle_model.step_std(state_input, action_list_w[i])
        next_state_list_w.append(next_state_tmp)
        state_input=next_state_tmp

    state_input=state_list[0]
    next_state_tmp = []
    next_state_list_kappa_fuzzy =[]
    for i in range(len(state_list)):
        next_state_tmp=vehicle_model.step_kappa_fuzzy_std(state_input, action_list[i])
        next_state_list_kappa_fuzzy.append(next_state_tmp)
        state_input=next_state_tmp

    # output plot data
    next_x_list = []
    next_y_list = []
    for next_state in next_state_list:
        next_x_list.append(next_state[0])
        next_y_list.append(next_state[1])
        
    next_x_list_2 = []
    next_y_list_2 = []
    for next_state in next_state_list_w:
        next_x_list_2.append(next_state[0])
        next_y_list_2.append(next_state[1])

    next_x_list_kappa_fuzzy = []
    next_y_list_kappa_fuzzy = []
    for next_state in next_state_list_kappa_fuzzy:
        next_x_list_kappa_fuzzy.append(next_state[0])
        next_y_list_kappa_fuzzy.append(next_state[1])

    # input plot data
    state_x = []
    state_y = []
    state_v = []
    state_theta = []

    for state in state_list:
        state_x.append(state[0])
        state_y.append(state[1])
        state_v.append(state[2])
        state_theta.append(state[3])

    fig_x_y = figure(width=800, height=400, match_aspect=True)
    
    fig_x_y.scatter(next_x_list, next_y_list, color='red',
                    line_alpha=0.4, legend_label="model_kappa")
    
    fig_x_y.scatter(state_x, state_y, color='green',
                    line_alpha=0.4, legend_label="x_real-y_real")
    
    fig_x_y.scatter(next_x_list_2, next_y_list_2, color='blue',
                    line_alpha=0.4, legend_label="model_angle_speed_fuzzy")
    
    fig_x_y.scatter(next_x_list_kappa_fuzzy, next_y_list_kappa_fuzzy,
                    line_alpha=0.4, legend_label="model_kappa_fuzzy")

    fig_x_y.xaxis.axis_label = "x"
    fig_x_y.yaxis.axis_label = "y"

    fig_x_y.toolbar.active_scroll = fig_x_y.select_one(WheelZoomTool)
    fig_x_y.legend.click_policy = 'hide'
    output_notebook()
    show(fig_x_y)
    # print("output: ", next_state_list)

if __name__ == '__main__':
    vehicle_model_test()
    vehicle_model_kappa_test()
