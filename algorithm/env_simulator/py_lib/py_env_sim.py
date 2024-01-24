from bokeh.plotting import figure, show
from bokeh.io import output_notebook
from bokeh.palettes import Magma, Viridis
from bokeh.models import Arrow, VeeHead
import math
import numpy as np
def plot_lanes_env(data):
    p = figure(width=1200, height=400, match_aspect=True)
    lane_list = []
    a = 0
    for lane_pts in data:
        pts_tmp_x = []
        pts_tmp_y = []
        for pt in lane_pts:
            pts_tmp_x.append(pt.x)
            pts_tmp_y.append(pt.y)

        p.line(pts_tmp_x, pts_tmp_y, line_width=2, color = Magma[6][a])
        a = a +1
    output_notebook()
    show(p)
    
def plot_freespace(lane,objs,road_edge,freespace_roi_pts,right_point_by_ttc,ego_prediction_position,right_border_pts,ego_car=None,ellipse_param=None):
    p = figure(width=950, height=400, match_aspect=True)
    a = 0
    road_edge_index=1
    if len(road_edge) == 0:
        print("road_edge为空")
        return
    else:
        print("road_edge不为空")
    for road_edge_pts in road_edge:
        pts_tmp_x = []
        pts_tmp_y = []
        for pt in road_edge_pts:
            # print("x:",pt.x,'   ',"y:",pt.y)
            pts_tmp_x.append(pt.x)
            pts_tmp_y.append(pt.y)
        p.scatter(pts_tmp_x, pts_tmp_y, line_width=2, color = Magma[4][a],legend_label="road_edge_"+str(road_edge_index))   
        a = a +2
        road_edge_index+=1
        
    ttc_pt_tmp_x=[]
    ttc_pt_tmp_y=[]    
    for pt in right_point_by_ttc:
        ttc_pt_tmp_x.append(pt.x)
        ttc_pt_tmp_y.append(pt.y)
        print('x: ',pt.x,'  y:  ',pt.y)
    p.line(ttc_pt_tmp_x, ttc_pt_tmp_y, line_width=2, color = 'purple',legend_label="right_ttc_point")
 
    # 自车位置
    # car_length = ego_car.length
    # car_width = ego_car.width
    # car_theta= ego_car.theta
    ego_car_x=[]
    ego_car_y=[]
    ego_car_x.append(ego_car.position.x)
    ego_car_x.append(ego_prediction_position.x)
    ego_car_y.append(ego_car.position.y)
    ego_car_y.append(ego_prediction_position.y)
    p.line(ego_car_x,ego_car_y,line_width=2, color='blue',legend_label="ego_car&prediction")
    
    right_border_pts_tmp_x=[]
    right_border_pts_tmp_y=[]
    for pt in right_border_pts:
        right_border_pts_tmp_x.append(pt.x)
        right_border_pts_tmp_y.append(pt.y)
        # print('x: ',pt.x,'  y:  ',pt.y)
    print("right_border_pts size: ",len(right_border_pts_tmp_x))
    p.scatter(right_border_pts_tmp_x, right_border_pts_tmp_y, line_width=2, color = 'green',legend_label="right_border_pts")
    
    
    # 椭圆参数
    center_x = ellipse_param.center_point.x  # 椭圆中心 x 坐标
    center_y = ellipse_param.center_point.y  # 椭圆中心 y 坐标
    radius_x =  ellipse_param.a  # x 轴半径
    radius_y = ellipse_param.b  # y 轴半径
    # 绘制椭圆
    # p.ellipse(center_x, center_y, radius_x*2, radius_y*2, fill_alpha=0.5,angle=car_theta)
    for freespace_roi_pt in freespace_roi_pts:
        print(freespace_roi_pt.x,freespace_roi_pt.y)
        
    output_notebook()
    show(p)
    
def plot_env(lane,objs,road_edge=[]):
    p = figure(width=950, height=400, match_aspect=True,y_range=[-8,8])
    lane_list = []
    a = 0
    for lane_pts in lane:
        pts_tmp_x = []
        pts_tmp_y = []
        for pt in lane_pts:
            pts_tmp_x.append(pt.x)
            pts_tmp_y.append(pt.y)

        p.scatter(pts_tmp_x, pts_tmp_y, line_width=2, color = Magma[6][a],line_dash='dashed')
        a = a + 1
        
    step_vh = 2.0    
    vh = VeeHead(size=5, fill_color=Viridis[3][0])
    for obj in objs:
        pts_tmp_x = []
        pts_tmp_y = []
        for pt in obj.trajectory_points:
            pts_tmp_x.append(pt.position.x)
            pts_tmp_y.append(pt.position.y)
            next_pt_x = pt.position.x + math.cos(pt.theta) * step_vh
            next_pt_y = pt.position.y + math.sin(pt.theta) * step_vh
            p.add_layout(Arrow(end=vh, x_start=pt.position.x, y_start=pt.position.y, x_end=next_pt_x, y_end=next_pt_y))    
        p.scatter(pts_tmp_x, pts_tmp_y, line_width=2, color = Viridis[6][a])
        a = a +1
    output_notebook()
    show(p)