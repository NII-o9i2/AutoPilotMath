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

def construct_source_data2(x, y):
    return ColumnDataSource(data=dict(
        x=x,
        y=y,
        x_info=[f"{x:.2f}" for x in x],
        y_info=[f"{y:.2f}" for y in y]
    ))

class XvizPlotLatLonMotion(XvizPlotBase):
    def __init__(self, bag_path, output):
        super().__init__(bag_path, output)
        
    def figure_plot(self, output):
        # pprint(self.data_frame_)
        x_coords_path, y_coords_path = [], []
        for item in self.data_frame_["/maplesslm/scene_navi_map"]["lla"]:
            x_coords_path.append(item["data"]["x"])
            y_coords_path.append(item["data"]["y"])
        source2 = construct_source_data2(x_coords_path, y_coords_path)

        plot = figure(
            title="SDMAP_LLA_Points",
            x_axis_label="X Coordinate",
            y_axis_label="Y Coordinate",
            width=750,
            height=900,
            tools="pan,wheel_zoom,box_zoom,reset,save,box_select,lasso_select"
        )
        hover2 = HoverTool(renderers=[], tooltips=[("Index", "$index"), ("(X,Y)", "(@x_info, @y_info)")])
        path_glyph = plot.scatter("x", "y", size=5, color="blue", alpha=0.3, legend_label="LLA_Points", source=source2)
        hover2.renderers.append(path_glyph)

        plot.add_tools(hover2)
        plot.legend.click_policy = "hide"

        output_file(output)
        show(plot)
        

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

    xpilot_plot = XvizPlotLatLonMotion(args.source, args.output)

    # xpilot_plot.add_required_channel(XDEBUG_CHANNEL)
    # xpilot_plot.add_required_channel(PLANNING_CHANNEL)
    xpilot_plot.add_required_channel(MAPLESS_CHANNEL)
    xpilot_plot.read_msg()

    xpilot_plot.figure_plot(args.output)