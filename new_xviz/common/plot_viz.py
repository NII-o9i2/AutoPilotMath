import bokeh
import math
from bokeh.io import curdoc
from bokeh.io import show, output_file , output_notebook
from bokeh.models import WheelZoomTool
from bokeh.layouts import column, row
from bokeh.models import ColumnDataSource, CustomJS, Slider
from bokeh.models import HoverTool
from bokeh.models import LinearColorMapper
from bokeh.colors import HSL
from bokeh.palettes import Magma256
from bokeh.transform import linear_cmap
import argparse
import numpy as np
from common.bag_reader import * 
from common.json_reader import *
from common.layer_viz import *
from common.plot_util import * 
import rsclpy
from bokeh.models import CustomJS, CheckboxButtonGroup
from pprint import pprint
from decimal import Decimal
import pandas as pd
from enum import Enum
import os

class InputFileType(Enum):
    RSCLBAG = 1
    JSON = 2
    OTHER = 3

class XvizPlotBase:
    def __init__(self,file_input_path,output) -> None:
        self.file_input_path_ = file_input_path
        self.input_file_type = InputFileType.OTHER
        self.output_ = output
        self.figs_ = {}
        self.data_set_ = {}
        self.required_channel_list_ = []
        self.read_input_flag_ = False
        self.data_frame_ = {}
        self.checkbox_groups_ = {}
        pass

    def add_required_channel(self,channel):
        self.required_channel_list_.append(channel)

    def read_bag(self,read_time_length=80.0):
        print("*" * 30 + " begin py bag reader " + "*" * 30)
        attr = rsclpy.BagReaderAttribute()
        attr.included_channels = set(self.required_channel_list_)
        bag_reader = rsclpy.BagReader(self.file_input_path_,attr)
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
            if len(self.data_set_[tmp_msg.channel_name])>0 and tmp_msg.timestamp* 1e-9-next(iter(self.data_set_[tmp_msg.channel_name][0])) > read_time_length:
                continue 
            self.data_set_[tmp_msg.channel_name].append({tmp_msg.timestamp * 1e-9:tmp_data})
        self.read_input_flag_ = True
        # pprint(self.data_set_)
        return
    
    def construct_msg(self):
        """
        construct_msg会构造data_frame
        data_frame是一个list, list的每个元素都是一个dict
        dict中包含data, index, t这几项信息
        """
        for channel_name, channel_entries in self.data_set_.items():
            self.data_frame_[channel_name] = {}
            for channel_msg_index, channel_entry in enumerate(channel_entries):
                for time_key, sub_dict in channel_entry.items():
                    for data_key, value in sub_dict.items():
                        if data_key == 'obstacle_debug':
                            for data_key, value in sub_dict.items():
                                if data_key == 'obstacle_debug':
                                    if value:  # 检查value是否非空
                                        nested_keys = value[0].keys()
                                        for nested_key in nested_keys:
                                            new_key = "{}_{}".format(data_key, nested_key)
                                            self.data_frame_[channel_name].setdefault(new_key, [])
                                            nested_data = []
                                            for obs in value:
                                                nested_value = obs[nested_key]
                                                nested_data.append(nested_value)
                                            self.data_frame_[channel_name][new_key].append({'data': nested_data, 
                                                                                            'index': channel_msg_index, 
                                                                                            't': time_key})
                                    else:
                                        # print("Warning: 'obstacle_debug' data is empty.")
                                        pass
                        else:
                            self.data_frame_[channel_name].setdefault(data_key, [])
                            self.data_frame_[channel_name][data_key].append({'data': value, 
                                                                            'index': channel_msg_index, 
                                                                                't': time_key})
        # pprint(self.data_frame_) 
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
    
    def get_file_type(self):
        file_extension = os.path.splitext(self.file_input_path_)[1][1:].lower()
        if file_extension == 'rsclbag':
            self.input_file_type = InputFileType.RSCLBAG
        elif file_extension == 'json':
            self.input_file_type = InputFileType.JSON
        else:
            self.input_file_type = InputFileType.OTHER

    def read_json(self):
        try:
            with open(self.file_input_path_, 'r', encoding='utf-8') as file:
                print("*" * 30 + " one frame json reader and plot" + "*" * 30)
                data = json.load(file)
                for channel, func in json_callback_map.items():
                    if channel in data:
                        processed_data = func(data[channel])
                        self.data_set_[channel] = [{str(1): processed_data}]
                self.read_input_flag_ = True
        except FileNotFoundError:
            return "文件未找到。"
        except json.JSONDecodeError:
            return "文件不是有效的JSON格式。"
            
    def read_msg(self,read_time_length=80.0):
        if self.read_input_flag_  == False:
            self.get_file_type()
            if self.input_file_type == InputFileType.RSCLBAG:
                self.read_bag(read_time_length)
                self.match_timestamp()
                self.construct_msg()
            elif self.input_file_type == InputFileType.JSON:
                self.read_json()
                self.bag_time_length_ = 0.1
                self.construct_msg()
            else: 
                pass

    def get_data_frame_at_datakey(self, _key_name, _channel_name = None):
        result = {}
        for channel_name, channel_dict in self.data_frame_.items():
            if _channel_name != None:
                    if _channel_name != channel_name:
                        continue
            for data_key, data_dict in channel_dict.items():
                if data_key == _key_name:
                    result = data_dict
                    return result
        return result
    
    def set_data_frame_at_datakey(self, channel_name, key_name, data_frame):
        if channel_name not in self.data_frame_:
            self.data_frame_[channel_name] = {}
        self.data_frame_[channel_name][key_name] = data_frame

    def add_layer_to_figure(self, fig, args):
        if 'data_key' not in args:
            print('add_layer_to_figure: can not find data_key in args')
            return
        data_key = args['data_key']
        data_frame = self.get_data_frame_at_datakey(data_key)
        if not data_frame:
            print('add_layer_to_figure: data_frame not valid')
            return
        PlotUtils.add_layer_to_figure_with_dataframe(fig, data_frame, args)

    def activate_figure_option(self):
        PlotUtils.activate_figure_option(self.figs_)
    
    def get_common_time_list(self):
        times = pd.DataFrame(self.get_data_frame_at_datakey('nop_counter'))['t'].tolist()
        return times

    def add_slider(self):
        # times = self.get_common_time_list()
        # _end = 39.9
        self.time_slider_ = Slider(start=0.0, end=self.bag_time_length_, 
                                   value=0.0, step=0.1, title="Time")
        return