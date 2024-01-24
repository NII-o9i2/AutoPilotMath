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
from common.layer_viz import *

class PlotUtils:
    @staticmethod
    def add_layer_to_figure_with_dataframe(fig, data_frame, args):
            plot_type = args['plot_type']
            args['data_frame'] = data_frame
            
            if plot_type == 'index_line':
                fig.add_layer(LayerIndexLine, args)
            if plot_type == 'index_circle':
                fig.add_layer(LayerIndexCircle, args)
            if plot_type == 'index_line_circle':
                fig.add_layer(LayerIndexLine, args)
                fig.add_layer(LayerIndexCircle, args)
            if plot_type == 'scatter':
                fig.add_layer(LayerScatter, args)
            if plot_type == 'line':
                fig.add_layer(LayerLine, args)
            if plot_type == 'line_scatter':
                fig.add_layer(LayerLine, args)
                fig.add_layer(LayerScatter, args)
            if plot_type == 'single_point':
                fig.add_layer(LayerSinglePoint, args)
            if plot_type == 'multi_polygon':
                fig.add_layer(LayerMultiPolygon, args)
            if plot_type == 'multi_ellipse':
                fig.add_layer(LayerMultiEllipse, args)
    
    @staticmethod
    def activate_figure_option(figs):
        if isinstance(figs, list):
            for fig in figs:
                if fig is None:
                    continue
                fig.toolbar.active_scroll = fig.select_one(WheelZoomTool)
                fig.legend.click_policy = 'hide'
        elif isinstance(figs, dict):
            for fig_name, fig in figs.items():
                if fig is None:
                    continue
                fig.toolbar.active_scroll = fig.select_one(WheelZoomTool)
                fig.legend.click_policy = 'hide'
        return
