
import sys
import numpy as np
import os
from bokeh.plotting import figure, show
from bokeh.io import output_notebook
from bokeh.models import WheelZoomTool
import json
from pprint import pprint
from collections import defaultdict
import itertools
from bokeh.plotting import figure, show, output_notebook
from bokeh.models import LabelSet, ColumnDataSource
from bokeh.palettes import Magma, Viridis, Category20
from bokeh.models import ColumnDataSource, LabelSet, Legend
from bokeh.models import CustomJS, Toggle
from bokeh.models import Button, CustomJS
from bokeh.layouts import column
from bokeh.palettes import Magma, Viridis, Category20
from bokeh.palettes import Magma256
current_directory = os.getcwd()
relative_path = '../../build/dcp_tree'
file_path4 = os.path.join(current_directory, relative_path)
sys.path.append(relative_path)
relative_path2 = '../../build/cilqr'
file_path2 = os.path.join(current_directory, relative_path2)
sys.path.append(file_path2)
import pybind_ilqr
import pybind_dcp_tree


def converted_sequences(all_paths):
    sequences = []
    enum_to_str = {
        -1: "LLC",
        0: "LK",
        1: "RLC", 
    }
    # print("111111111111")
    # print(all_paths)
    for path in all_paths:
        new_path = []
        for step in path:
            # print(step)
            cost, time, action_dir, debug_info = step  
            new_step = (cost, time, enum_to_str[action_dir], debug_info)
            new_path.append(new_step)
        sequences.append(new_path)
    # print("all_paths22222")
    # pprint(len(sequences))
    all_paths_coordinates = []
    for path in sequences:
        coordinates = [(10, 10)]
        current_x, current_y = 10, 10
        for i, (cost, time, action, debug_info) in enumerate(path[1:], start=1):
            if action == "LK":
                current_y -= 1
            elif action == "LLC":
                current_x -= 5 / i 
                current_y -= 1
            elif action == "RLC":
                current_x += 5 / i
                current_y -= 1
            coordinates.append((current_x, current_y))
        all_paths_coordinates.append(coordinates)
    # print("333333333333")    
    # print(len(all_paths_coordinates))
    new_structure = []
    # 遍历sequences和all_paths_coordinates
    for coord, seq in zip(all_paths_coordinates, sequences):
        # 确保序列和坐标长度匹配
        # print("seq size is: {}".format(len(seq)))
        # print("coord size is: {}".format(len(coord)))    
        assert len(seq) == len(coord), "Sequences and Coordinates lengths do not match"
        # 组合坐标点和序列的步骤
        combined_path = [(x, y, step[0], step[1], step[2]) for (x, y), step in zip(coord, seq)]
        # 将组合后的路径添加到新的结构中
        new_structure.append(combined_path)
    # print("444444444444")
    # pprint(new_structure)
    return new_structure


def plot_dcp_tree_node(sequences):
    p = figure(width=1000, height=800, 
            x_axis_label='X', y_axis_label='Y',
            x_range =[6, 15], title="DCP_TREE with Action",match_aspect=True)
    for i, data in enumerate(sequences):
        # pprint(sequences)
        colors = []
        for item in data:
            if item[4] == 'LK':
                colors.append('red')  # LK为红色
            elif item[4] == 'LLC':
                colors.append('blue') # LLC为蓝色
            elif item[4] == 'RLC':
                colors.append('green') # RLC为绿色
            else:
                colors.append('black') # 默认颜色  
        labels = ["{:.1f}_{:.1f}".format(point[2], point[3]) for point in data]
        total_cost = data[-1][2]
        source = ColumnDataSource(data=dict(
            x=[item[0] for item in data],
            y=[item[1] for item in data],
            colors=colors,
            labels=labels
        ))
        labels = LabelSet(x='x', y='y', text='labels', level='glyph',
            x_offset=5, y_offset=5, source=source)
        # legend_label = f"Path_{i+1} Total Cost: {total_cost:.1f}"
        legend_label = "Path_{0} Total Cost: {1:.1f}".format(i+1, total_cost)
        p.line('x', 'y', source=source, line_width=2, color='black', legend_label=legend_label)
        p.circle('x', 'y', size=20, source=source, color='colors', legend_label=legend_label)
        p.add_layout(labels)
    p.legend.location = 'top_right'
    p.legend.click_policy = "hide"
    return p

# 计算四个角点的函数
def calculate_corners(center):
    car_length = 5  # 车长
    car_width = 3  # 车宽
    half_length = car_length / 2
    half_width = car_width / 2
    x, y = center
    return [(x - half_length, y + half_width), (x + half_length, y + half_width),
            (x + half_length, y - half_width), (x - half_length, y - half_width)]

def plot_lane_and_objs(obs, ego):
    # plot lane
    p = figure(title="Objs and Ego", width=1000, height=400, x_range=(-100, 100), y_range=(-7, 7),match_aspect=True)
    y_start = [3.75, 0.0, -3.75]
    x_start = [-90, -90, -90]
    y_end = [3.75, 0.0, -3.75]
    x_end =  [90, 90, 90]
    labels = ["LLane", "CLane", "RLane"]
    colors = ["blue", "green", "red"]
    x_coords = {
        -1: y_start[0],
        0: y_start[1],
        1: y_start[2]
    }
    for i in range(len(x_start)):
        p.segment(x_start[i], y_start[i], x_end[i], y_end[i], line_dash="dashed", 
                  line_width=2, line_color=colors[i], legend_label=labels[i])
        p.legend.click_policy = "hide"
    # plot cars
    obs_xy = []
    ego_pos = [ego.car_s, y_start[1]]
    obs_labels = []
    obs_xy.append(ego_pos)
    obs_labels.append((ego.car_s, ego.car_v, ego.car_a)) 
    for lane_dir, cars_info in obs.items():
        y = x_coords.get(lane_dir, 0)
        for car_info in cars_info:
            x = car_info.car_s
            obs_xy.append((x, y))
            obs_labels.append((car_info.car_s, car_info.car_v, car_info.car_a))
    cars_corners = [calculate_corners(center) for center in obs_xy]
    color_step = len(Magma256) // len(cars_corners)
    for i, corners in enumerate(cars_corners): 
        if i == 0:
            s, v, a = obs_labels[i]
            legend_label = "Ego_sva({},{},{})".format(s, v, a)
        else:
            s, v, a = obs_labels[i]
            legend_label = "Obs{}_sva({},{},{})".format(i, s, v, a)
        color = Magma256[i * color_step]
        x_coords = [corner[0] for corner in corners] + [corners[0][0]]
        y_coords = [corner[1] for corner in corners] + [corners[0][1]]
        p.patch(x_coords, y_coords, alpha=0.5, color=color, legend_label=legend_label)

    p.legend.click_policy = "hide"
    return p

def print_idm_car_info(car_info):
    print("IDMCarInfo: s={}, v={}, a={}, length={}".format(
        car_info.car_s, car_info.car_v, car_info.car_a, car_info.car_length))
    
def read_json(file):
    try:
        with open(file, 'r') as file:
            j = json.load(file)
    except FileNotFoundError:
        print("Could not open the file")
        sys.exit(1)
    scenario_name = j["name"]
    # print("test case name is: {}".format(scenario_name))
    input_test = {}
    for lane in j["lanes_objs"]:
        # print("lane id is: {}".format(lane['lane_id']))
        lane_dir = pybind_dcp_tree.DCPLaneDir(lane["lane_id"])
        lane_cars = []
        for car in lane["cars"]:
            car_info = pybind_ilqr.IDMCarInfo(car["car_s"], car["car_v"], car["car_a"], car["car_length"])
            lane_cars.append(car_info)
            car_info.print_car_info()
        input_test[lane_dir] = lane_cars
    # print("input size: {}".format(len(input_test)))
    # for lane_id, cars in input_test.items():
    #     print("lane id is: {}, obj size is: {}".format(lane_id, len(cars)))
    #     for car in cars:
    #         car.print_car_info()
    # lc_type = j["change_direction"]
    # print("lc_type is: {}".format(lc_type))
    # def print_idm_car_info(car_info):
    #     print("IDMCarInfo: s={}, v={}, a={}, length={}".format(
    #         car_info.car_s, car_info.car_v, car_info.car_a, car_info.car_length))

    # for lane_dir, cars in input_test.items():
    #     print("lane id is: {}, obj size is: {}".format(lane_dir, len(cars)))
    #     for car in cars:
    #         print_idm_car_info(car)
    return input_test