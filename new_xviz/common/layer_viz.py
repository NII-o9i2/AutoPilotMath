from bokeh.plotting import figure
from bokeh.models import ColumnDataSource, CustomJS
import pandas as pd
from bokeh.models import HoverTool

class LayerBase:
    """
    Layer的基类
    子类需要继承LayerBase并实现函数: plot, prepare_layer_data, create_callback
    """

    def __init__(self, args) -> None:
        # read args
        fig_core = None
        data_frame = None
        color = None
        label = 'None'
        line_alpha = None
        line_width = None
        fill_alpha = None
        size = 2
        if 'fig_core' in args:
            fig_core = args['fig_core']
        if 'data_frame' in args:
            data_frame = args['data_frame']
        if 'color' in args:
            color = args['color']
        if 'label' in args:
            label = args['label']
        if 'line_alpha' in args:
            line_alpha = args['line_alpha']
        if 'line_width' in args:
            line_width = args['line_width']
        if 'fill_alpha' in args:
            fill_alpha = args['fill_alpha']
        if 'size' in args:
            size = args['size']
        self.fig_core_ = fig_core
        self.data_frame_ = data_frame
        self.color_ = color
        self.label_ = label
        self.line_alpha_ = line_alpha
        self.line_width_ = line_width
        self.fill_alpha_ = fill_alpha
        self.size_ = size

        # 初始化layer相关的data和callback
        self.times_ = []
        self.layer_data_source_ = ColumnDataSource()
        self.layer_data_source_js_ = []
        self.js_callback_ = CustomJS()

        # 构造layer相关的data和callback
        self.prepare_layer_data()
        self.create_callback()
    
    def get_callback(self):
        return self.js_callback_

    def plot(self) -> None:
        pass

    def prepare_layer_data(self):
        pass

    def create_callback(self):
        pass

class LayerIndexLine(LayerBase):
    """
    传入的data_frame中的每个data, 必须是一个list, 且list的元素是纯数字, 如 [2, 4, 6, ..., 10]
    将会绘制对应时刻, list中每个元素连接而成的line
    绘制的line的每个点的y取list中的元素
    绘制的line的每个点的x取list中的元素的下标
    """

    def __init__(self, args):
        super().__init__(args)

    def plot(self):
        if self.fig_core_ == None:
            return

        plot_line = self.fig_core_.line('x', 'y', source=self.layer_data_source_, \
                            line_color=self.color_, legend_label=self.label_)

        hover_tool = HoverTool(renderers=[plot_line], tooltips=[('index', '@x'), \
                                                                ('value', '@y'), \
                                                                ('t', '@t')], mode='mouse')
        self.fig_core_.add_tools(hover_tool)

    def prepare_layer_data(self):
        if self.data_frame_ == None:
            return

        try:
            df = pd.DataFrame(self.data_frame_)
            if df.empty or 'data' not in df.columns or len(df['data']) == 0:
                return
            
            self.times_ = df['t'].tolist()
            self.layer_data_source_ = ColumnDataSource({
                    'x': list(range(len(df.iloc[0]['data']))), 
                    'y': df.iloc[0]['data'],
                    't': [df.iloc[0]['t']] * len(df.iloc[0]['data'])
                })
            self.layer_data_source_js_ = df['data'].tolist()

            if self.layer_data_source_ == None:
                raise ValueError(
                    "data_frame empty")
        except Exception as e:
            print("An error occurred while processing data_frame: {}".format(e))
    
    def create_callback(self):
        callback_code = """
            const t = cb_obj.value;
            let closest_index = 0;
            let minDiff = Math.abs(times[0] - t);
            for (let i = 1; i < times.length; i++) {
                let diff = Math.abs(times[i] - t);
                if (diff < minDiff) {
                    minDiff = diff;
                    closest_index = i;
                }
            }
            const data = source.data;
            data.y = data_js_list[closest_index];
            data.x = Array.from(Array(data.y.length).keys()); // 更新x以匹配y的长度
            data.t = Array(data.y.length).fill(times[closest_index])
            source.change.emit();
        """
        self.js_callback_ = CustomJS(args=dict(source=self.layer_data_source_, \
                                    data_js_list=self.layer_data_source_js_, 
                                    times=self.times_), code=callback_code)

class LayerIndexCircle(LayerBase):
    """
    传入的data_frame中的每个data, 必须是一个list, 且list的元素是纯数字, 如 [2, 4, 6, ..., 10]
    将会绘制对应时刻, list中每个元素对应的circle
    绘制的每个circle的y取list中的元素
    绘制的每个circle的x取list中的元素的下标
    """

    def __init__(self, args):
        super().__init__(args)

    def plot(self):
        if self.fig_core_ == None:
            return

        plot_circle = self.fig_core_.circle('x', 'y', source=self.layer_data_source_, \
                                fill_color=self.color_, size=8, legend_label=self.label_)

        hover_tool = HoverTool(renderers=[plot_circle], tooltips=[('index', '@x'), \
                                                                ('value', '@y'), \
                                                                ('t', '@t')], mode='mouse')
        self.fig_core_.add_tools(hover_tool)

    def prepare_layer_data(self):
        if self.data_frame_ == None:
            return

        try:
            df = pd.DataFrame(self.data_frame_)
            if df.empty or 'data' not in df.columns or len(df['data']) == 0:
                return
            
            self.times_ = df['t'].tolist()
            self.layer_data_source_ = ColumnDataSource({
                    'x': list(range(len(df.iloc[0]['data']))), 
                    'y': df.iloc[0]['data'],
                    't': [df.iloc[0]['t']] * len(df.iloc[0]['data'])
                })
            self.layer_data_source_js_ = df['data'].tolist()

            if self.layer_data_source_ == None:
                raise ValueError(
                    "data_frame empty")
        except Exception as e:
            print("An error occurred while processing data_frame: {}".format(e))
    
    def create_callback(self):
        callback_code = """
            const t = cb_obj.value;
            let closest_index = 0;
            let minDiff = Math.abs(times[0] - t);
            for (let i = 1; i < times.length; i++) {
                let diff = Math.abs(times[i] - t);
                if (diff < minDiff) {
                    minDiff = diff;
                    closest_index = i;
                }
            }
            const data = source.data;
            data.y = data_js_list[closest_index];
            data.x = Array.from(Array(data.y.length).keys()); // 更新x以匹配y的长度
            data.t = Array(data.y.length).fill(times[closest_index])
            source.change.emit();
        """
        self.js_callback_ = CustomJS(args=dict(source=self.layer_data_source_, \
                                    data_js_list=self.layer_data_source_js_, 
                                    times=self.times_), code=callback_code)

class LayerScatter(LayerBase):
    """
    传入的data_frame中的每个data, 必须是一个dict, 且dict包含两个一维list-x和y, 如
    {
        "x": [3, 5, 3, 5]
        "y": [2, 2, 4, 5]
    }
    x和y的list的长度, 表示有几个散点
    将会绘制对应时刻, data中包含的每个散点
    """

    def __init__(self, args):
        super().__init__(args)

    def plot(self):
        if self.fig_core_ == None:
            return

        scatter_point = self.fig_core_.scatter('x', 'y', source=self.layer_data_source_, \
                                color=self.color_, line_alpha=self.line_alpha_, \
                                line_width=self.line_width_, size=self.size_, \
                                legend_label=self.label_)

        hover_tool = HoverTool(renderers=[scatter_point], tooltips=[('x', '@x'), \
                                                                    ('y', '@y'), \
                                                                    ('t', '@t')], mode='mouse')
        self.fig_core_.add_tools(hover_tool)

    def prepare_layer_data(self):
        if self.data_frame_ == None:
            return

        try:
            df = pd.DataFrame(self.data_frame_)
            if df.empty or 'data' not in df.columns or len(df['data']) == 0:
                return
            
            self.times_ = df['t'].tolist()
            self.layer_data_source_ = ColumnDataSource({
                    'x': df.iloc[0]['data']['x'], 
                    'y': df.iloc[0]['data']['y'],
                    't': [df.iloc[0]['t']] * len(df.iloc[0]['data']['x'])
                })
            self.layer_data_source_js_ = df['data'].tolist()

            if self.layer_data_source_ == None:
                raise ValueError(
                    "data_frame empty")
        except Exception as e:
            print("An error occurred while processing data_frame: {}".format(e))
    
    def create_callback(self):
        callback_code = """
            const t = cb_obj.value;
            let closest_index = 0;
            let minDiff = Math.abs(times[0] - t);
            for (let i = 1; i < times.length; i++) {
                let diff = Math.abs(times[i] - t);
                if (diff < minDiff) {
                    minDiff = diff;
                    closest_index = i;
                }
            }
            const data = source.data;
            data.y = data_js_list[closest_index].y;
            data.x = data_js_list[closest_index].x;
            data.t = Array(data.y.length).fill(times[closest_index])
            source.change.emit();
        """
        self.js_callback_ = CustomJS(args=dict(source=self.layer_data_source_, \
                                    data_js_list=self.layer_data_source_js_, 
                                    times=self.times_), code=callback_code)

class LayerLine(LayerBase):
    """
    传入的data_frame中的每个data, 必须是一个dict, 且dict包含两个一维list-x和y, 如
    {
        "x": [3, 5, 3, 5]
        "y": [2, 2, 4, 5]
    }
    x和y的list的长度, 表示有几个散点
    将会绘制对应时刻, data中包含的所有散点连接成的线
    """

    def __init__(self, args):
        super().__init__(args)

    def plot(self):
        if self.fig_core_ == None:
            return

        plot_line = self.fig_core_.line('x', 'y', source=self.layer_data_source_, \
                                color=self.color_, line_alpha=self.line_alpha_, \
                                legend_label=self.label_, line_width=self.line_width_)

        hover_tool = HoverTool(renderers=[plot_line], tooltips=[('x', '@x'), \
                                                                ('y', '@y'), \
                                                                ('t', '@t')], mode='mouse')
        self.fig_core_.add_tools(hover_tool)

    def prepare_layer_data(self):
        if self.data_frame_ == None:
            return

        try:
            df = pd.DataFrame(self.data_frame_)
            if df.empty or 'data' not in df.columns or len(df['data']) == 0:
                return
            
            self.times_ = df['t'].tolist()
            self.layer_data_source_ = ColumnDataSource({
                    'x': df.iloc[0]['data']['x'], 
                    'y': df.iloc[0]['data']['y'],
                    't': [df.iloc[0]['t']] * len(df.iloc[0]['data']['x'])
                })
            self.layer_data_source_js_ = df['data'].tolist()

            if self.layer_data_source_ == None:
                raise ValueError(
                    "data_frame empty")
        except Exception as e:
            print("An error occurred while processing data_frame: {}".format(e))
    
    def create_callback(self):
        callback_code = """
            const t = cb_obj.value;
            let closest_index = 0;
            let minDiff = Math.abs(times[0] - t);
            for (let i = 1; i < times.length; i++) {
                let diff = Math.abs(times[i] - t);
                if (diff < minDiff) {
                    minDiff = diff;
                    closest_index = i;
                }
            }
            const data = source.data;
            data.y = data_js_list[closest_index].y;
            data.x = data_js_list[closest_index].x;
            data.t = Array(data.y.length).fill(times[closest_index])
            source.change.emit();
        """
        self.js_callback_ = CustomJS(args=dict(source=self.layer_data_source_, \
                                    data_js_list=self.layer_data_source_js_, 
                                    times=self.times_), code=callback_code)

class LayerSinglePoint(LayerBase):
    """
    传入的data_frame中的每个data, 必须是一个纯数字, 如 3
    将会一次性绘制所有时刻, 所有data对应的circle, 但是当前时刻的circle会标红显示
    绘制的每个circle的y取data的值
    绘制的每个circle的x取data_frame的index的值
    """

    def __init__(self, args):
        super().__init__(args)

    def plot(self):
        if self.fig_core_ == None:
            return

        self.fig_core_.line('index', 'data', source=self.layer_data_source_)
        plot_circle = self.fig_core_.circle('index', 'data', source=self.layer_data_source_, 
                              color='color', size='size')
        
        hover_tool = HoverTool(renderers=[plot_circle], tooltips=[('index', '@index'), \
                                                                ('value', '@data'), \
                                                                ('t', '@t')], mode='mouse')
        self.fig_core_.add_tools(hover_tool)

    def prepare_layer_data(self):
        if self.data_frame_ == None:
            return

        try:
            df = pd.DataFrame(self.data_frame_)
            if df.empty or 'index' not in df.columns or 'data' not in df.columns:
                return

            self.times_ = df['t'].tolist()
            self.layer_data_source_ = ColumnDataSource(data=dict(
                index=df['index'], 
                data=df['data'],
                color=['blue'] * len(df), 
                size=[5] * len(df),
                t=df['t']
            ))

            if self.layer_data_source_ == None:
                raise ValueError(
                    "data_frame empty")
        except Exception as e:
            print("An error occurred while processing data_frame: {}".format(e))
    
    def create_callback(self):
        callback_code = """
            const t = cb_obj.value;
            let closest_index = 0;
            let minDiff = Math.abs(times[0] - t);
            for (let i = 1; i < times.length; i++) {
                let diff = Math.abs(times[i] - t);
                if (diff < minDiff) {
                    minDiff = diff;
                    closest_index = i;
                }
            }
            console.log(times[closest_index])
            // 更新颜色和大小
            const data = source.data;
            for (let i = 0; i < data.index.length; i++) {
                if (i == closest_index) {
                    data.color[i] = 'red';
                    data.size[i] = 10;
                } else {
                    data.color[i] = 'blue';
                    data.size[i] = 5;
                }
            }
            source.change.emit();
        """
        self.js_callback_ = CustomJS(args=dict(source=self.layer_data_source_, \
                                    times=self.times_), code=callback_code)

class LayerMultiPolygon(LayerBase):
    """
    传入的data_frame中的每个data, 必须是一个dict, 且dict包含两个二维list-xs和ys, 如
    {
        "xs": [[1, 2, 3], [3, 5, 3, 5]]
        "ys": [[1, 2, 1], [2, 2, 4, 5]]
    }
    xs和ys的外层list的长度, 表示有几个polygon; 内层list的长度, 表示polygon有几个角点
    将会绘制对应时刻, data中包含的每个polygon
    """

    def __init__(self, args):
        super().__init__(args)

    def plot(self):
        if self.fig_core_ == None:
            return
        
        plot_polygons = self.fig_core_.patches('xs','ys',source=self.layer_data_source_, \
                                line_color=self.color_ ,line_alpha=self.line_alpha_, \
                                legend_label=self.label_, line_width=self.line_width_, \
                                fill_alpha=self.fill_alpha_)

        hover_tool = HoverTool(renderers=[plot_polygons], tooltips=[('xs', '@xs'), \
                                                                    ('ys', '@ys'), \
                                                                    ('t', '@t')], mode='mouse')
        self.fig_core_.add_tools(hover_tool)

    def prepare_layer_data(self):
        if self.data_frame_ == None:
            return

        try:
            df = pd.DataFrame(self.data_frame_)
            if df.empty or 'data' not in df.columns or len(df['data']) == 0:
                return

            self.times_ = df['t'].tolist()
            self.layer_data_source_ = ColumnDataSource({
                    'xs': df.iloc[0]['data']['xs'], 
                    'ys': df.iloc[0]['data']['ys'],
                    't': [df.iloc[0]['t']] * len(df.iloc[0]['data']['xs'])
                })
            self.layer_data_source_js_ = df['data'].tolist()

            if self.layer_data_source_ == None:
                raise ValueError(
                    "data_frame empty")
        except Exception as e:
            print("An error occurred while processing data_frame: {}".format(e))
    
    def create_callback(self):
        callback_code = """
            const t = cb_obj.value;
            let closest_index = 0;
            let minDiff = Math.abs(times[0] - t);
            for (let i = 1; i < times.length; i++) {
                let diff = Math.abs(times[i] - t);
                if (diff < minDiff) {
                    minDiff = diff;
                    closest_index = i;
                }
            }
            const data = source.data;
            data.xs.length = 0;
            data.ys.length = 0;
            polygon_xs = data_js_list[closest_index].xs
            polygon_ys = data_js_list[closest_index].ys
            for (let i = 0; i < polygon_xs.length; i++) {
                data.xs.push(Array.from(polygon_xs[i]))
            }
            for (let i = 0; i < polygon_ys.length; i++) {
                data.ys.push(Array.from(polygon_ys[i]))
            }
            data.t = Array(data.ys.length).fill(times[closest_index])
            source.change.emit();
        """
        self.js_callback_ = CustomJS(args=dict(source=self.layer_data_source_, \
                                    data_js_list=self.layer_data_source_js_, 
                                    times=self.times_), code=callback_code)

class LayerMultiEllipse(LayerBase):
    """
    传入的data_frame中的每个data, 必须是一个dict, 且dict包含多个一维list-x/y/width/height/angle, 如
    {
        "x": [1, 2, 3]
        "y": [1, 2, 1]
        "width": [2, 2, 4]
        "height": [2, 2, 4]
        "angle": [1, 2, 1]
    }
    list的长度, 表示有几个ellipse; 内层list的长度
    将会绘制对应时刻, data中包含的每个ellipse
    """

    def __init__(self, args):
        super().__init__(args)

    def plot(self):
        if self.fig_core_ == None:
            return
        
        plot_ellipse = self.fig_core_.ellipse('x','y','width','height','angle',
                                            source=self.layer_data_source_, \
                                            line_color=self.color_ ,line_alpha=self.line_alpha_, \
                                            legend_label=self.label_, line_width=self.line_width_, \
                                            fill_alpha=self.fill_alpha_)

        hover_tool = HoverTool(renderers=[plot_ellipse], tooltips=[('x', '@x'), \
                                                                    ('y', '@y'), \
                                                                    ('t', '@t')], mode='mouse')
        self.fig_core_.add_tools(hover_tool)

    def prepare_layer_data(self):
        if self.data_frame_ == None:
            return

        try:
            df = pd.DataFrame(self.data_frame_)
            if df.empty or 'data' not in df.columns or len(df['data']) == 0:
                return

            self.times_ = df['t'].tolist()
            self.layer_data_source_ = ColumnDataSource({
                    'x': df.iloc[0]['data']['x'], 
                    'y': df.iloc[0]['data']['y'],
                    'width': df.iloc[0]['data']['width'], 
                    'height': df.iloc[0]['data']['height'],
                    'angle': df.iloc[0]['data']['angle'], 
                    't': [df.iloc[0]['t']] * len(df.iloc[0]['data']['x'])
                })
            self.layer_data_source_js_ = df['data'].tolist()

            if self.layer_data_source_ == None:
                raise ValueError(
                    "data_frame empty")
        except Exception as e:
            print("An error occurred while processing data_frame: {}".format(e))
    
    def create_callback(self):
        callback_code = """
            const t = cb_obj.value;
            let closest_index = 0;
            let minDiff = Math.abs(times[0] - t);
            for (let i = 1; i < times.length; i++) {
                let diff = Math.abs(times[i] - t);
                if (diff < minDiff) {
                    minDiff = diff;
                    closest_index = i;
                }
            }

            const data = source.data;
            data.y = data_js_list[closest_index].y;
            data.x = data_js_list[closest_index].x;
            data.width = data_js_list[closest_index].width;
            data.height = data_js_list[closest_index].height;
            data.angle = data_js_list[closest_index].angle;
            data.t = Array(data.y.length).fill(times[closest_index])
            source.change.emit();
        """
        self.js_callback_ = CustomJS(args=dict(source=self.layer_data_source_, \
                                    data_js_list=self.layer_data_source_js_, 
                                    times=self.times_), code=callback_code)