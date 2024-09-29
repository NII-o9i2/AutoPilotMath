import json
import argparse
from bokeh.plotting import figure, show, output_file
from bokeh.models import HoverTool, ColumnDataSource, CrosshairTool, CustomJS, Div, TapTool
from bokeh.layouts import column
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
from bokeh.palettes import Viridis256

def construct_source_data1(x, y, directions, lane_indices):
    return ColumnDataSource(data=dict(
        x=x,
        y=y,
        directions=directions,
        lane_indices=lane_indices,
        label=[f"Dir: {d}, Lane: {l}" for d, l in zip(directions, lane_indices)],
        x_info=[f"{x:.5f}" for x in x],
        y_info=[f"{y:.5f}" for y in y]
    ))

def construct_source_data2(x, y):
    return ColumnDataSource(data=dict(
        x=x,
        y=y,
        x_info=[f"{x:.5f}" for x in x],
        y_info=[f"{y:.5f}" for y in y]
    ))
    
def construct_source_data3(x, y, pro):
    return ColumnDataSource(data=dict(
        x=x,
        y=y,
        pro=pro,
        x_info=[f"{xi:.5f}" for xi in x],
        y_info=[f"{yi:.5f}" for yi in y],
        pro_info=[f"{pi:.5f}" for pi in pro]
    ))

class XvizPlotLatLonMotion(XvizPlotBase):
    def __init__(self, bag_path, output):
        super().__init__(bag_path, output)
        
    def get_data_frame(self):
        # pprint(self.data_frame_)
        # x_coords_path, y_coords_path = [], []
        # for item in self.data_frame_["/maplesslm/scene_navi_map"]["lla"]:
        #     x_coords_path.append(item["data"]["x"])
        #     y_coords_path.append(item["data"]["y"])
        # source2 = construct_source_data2(x_coords_path, y_coords_path)
        return self.data_frame_
        

def read_json_and_plot(source_path, lla_json, output_path):
    with open(source_path, "r") as file:
        data = json.load(file)

    # 提取 intersection_list 中的 x 和 y 坐标
    x_coords, y_coords, directions, lane_indices = [], [], [], []
    for intersection in data["sd_map"]["intersection_list"]:
        position = intersection["position"]
        x_coords.append(position["x"])
        y_coords.append(position["y"])
        directions.append(intersection['direction'])
        lane_indices.append(intersection['lane_index'])
    source1 = construct_source_data1(x_coords, y_coords, directions, lane_indices)
    
    # 提取 path_points 中的 x 和 y 坐标
    x_coords_path, y_coords_path = [], []
    property = []
    for path_points in data["sd_map"]["path_points"]:
        position = path_points["position"]
        x_coords_path.append(position["x"])
        y_coords_path.append(position["y"])
        property.append(path_points["property"])
    source2 = construct_source_data3(x_coords_path, y_coords_path, property)
    
    # 提取json文件的lla字段
    x_coords_lla, y_coords_lla = [], []
    for item in lla_json["/maplesslm/scene_navi_map"]["lla"]:
        x_coords_lla.append(item["data"]["x"])
        y_coords_lla.append(item["data"]["y"])
    source3 = construct_source_data2(x_coords_lla, y_coords_lla) 

    plot = figure(
        title="Fake_SD_MapP",
        x_axis_label="X Coordinate",
        y_axis_label="Y Coordinate",
        width=750,
        height=900,
        tools="pan,wheel_zoom,box_zoom,reset,save,box_select,lasso_select"
    )

    hover1 = HoverTool(renderers=[], tooltips=[("Index", "$index"), ("Direction", "@directions"), ("Lane Index", "@lane_indices"), ("(X,Y)", "(@x_info, @y_info)")])
    point_glyph = plot.scatter("x", "y", size=10, color="red", alpha=0.5, legend_label="Intersection_Points", source=source1)
    hover1.renderers.append(point_glyph)

    mapper = linear_cmap(field_name='pro', palette=Viridis256, low=min(property), high=max(property))
    hover2 = HoverTool(renderers=[], tooltips=[("Index", "$index"), ("(X,Y)", "(@x_info, @y_info)"), ("Property", "@pro")])
    path_glyph = plot.scatter("x", "y", size=5, color=mapper, alpha=0.3, legend_label="Path_Points", source=source2)
    hover2.renderers.append(path_glyph)
    
    hover3 = HoverTool(renderers=[], tooltips=[("Index", "$index"), ("(X,Y)", "(@x_info, @y_info)")])
    path_glyph = plot.scatter("x", "y", size=5, color="black", alpha=0.3, legend_label="LLA_Points", source=source3)
    hover3.renderers.append(path_glyph)

    # 添加 CrosshairTool
    crosshair = CrosshairTool()
    plot.add_tools(crosshair)

    # 创建一个 Div 元素来显示坐标
    div = Div(width=200, height=50)

    # 创建 CustomJS 回调来更新 Div 内容
    callback = CustomJS(args=dict(div=div), code="""
        const {x, y} = cb_obj;
        div.text = `x: ${x.toFixed(7)}, y: ${y.toFixed(7)}`;
    """)

    # # 创建 CustomJS 回调来处理点击事件
    # callback2 = CustomJS(args=dict(source=source), code="""
    #     const data = source.data;
    #     const x = data['x'];
    #     const y = data['y'];
    #     const geometry = cb_data['geometry'];
    #     const new_x = geometry['x'];
    #     const new_y = geometry['y'];
    #     x.push(new_x);
    #     y.push(new_y);
    #     source.change.emit();
    # """)

    # taptool = plot.select(type=TapTool)
    # taptool.callback = callback2

    # 将回调添加到图表的 mousemove 事件
    plot.js_on_event('mousemove', callback)

    plot.add_tools(hover1, hover2, crosshair)
    plot.legend.click_policy = "hide"

    # 显示图表和 Div
    layout = column(plot, div)

    output_file(output_path)
    show(layout)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Plot intersections from a JSON file.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument("-s", "--source", required=True, help="Specify the source JSON file containing the sd_map data.")
    parser.add_argument("-r", "--rsclbag", required=True, help="rsclbag to lla json.")    
    parser.add_argument("-o", "--output", required=True, help="Specify the output HTML file for the plot.")
    args = parser.parse_args()
    xpilot_plot = XvizPlotLatLonMotion(args.rsclbag, args.output)
    xpilot_plot.add_required_channel(MAPLESS_CHANNEL)
    xpilot_plot.read_msg()
    lla_json = xpilot_plot.get_data_frame()
    read_json_and_plot(args.source, lla_json, args.output)


