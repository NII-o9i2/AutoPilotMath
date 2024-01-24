from bokeh.models import CustomJS, Slider
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
from bokeh.models import CustomJS, CheckboxButtonGroup
import pandas as pd

class FigureViz:
    def __init__(self, title, x_axis_label, y_axis_label, x_range=None, 
                 y_range=None, width=600, height =600, match_aspect = True) -> None:
        self.fig_core_ = figure(title=title, x_axis_label=x_axis_label, \
                                y_axis_label=y_axis_label, x_range=x_range, \
                                y_range=y_range, width=width, height =height, \
                                    match_aspect = match_aspect)
        self.layer_list = []

    def add_layer(self, layer_class, args):
        args['fig_core'] = self.fig_core_
        layer = layer_class(args)
        self.layer_list.append(layer)

    def plot(self):
        for layer in self.layer_list:
            layer.plot()
        return self.fig_core_

    def get_callback_list(self):
        callback_list = []
        for layer in self.layer_list:
            callback_list.append(layer.get_callback())
        return callback_list