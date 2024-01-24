from bokeh.plotting import figure, show, ColumnDataSource
from bokeh.layouts import column, row
from bokeh.io import output_notebook
from bokeh.palettes import Magma, Viridis, Category20
from bokeh.models import Arrow, VeeHead
from bokeh.models import WheelZoomTool
from bokeh.models import HoverTool
import matplotlib
import matplotlib.cm as cm
import math
import os
import sys
sys.path.append('../../build/cilqr')
import pybind_ilqr
sys.path.append(os.getcwd() + '/py_lib')
from cilqr_plot_xviz import *


def run_cilqr_case(path,point_init):
    motion = pybind_ilqr.LateralLongitudinalMotion()
    motion.init(path,point_init)
    motion.execute()

    env = motion.get_env()
    pts = env.get_all_center_points()
    obs = env.get_all_obstacle()

    planning_origin = motion.get_planning_origin()
    planning_init = {}
    planning_init['x'] = planning_origin.position.x
    planning_init['y'] = planning_origin.position.y
    planning_init['theta'] = planning_origin.theta

    trajs = motion.get_init_trajectory()
    traj_list = []
    for traj in trajs:
        pt = {}
        pt['x'] = traj.position.x
        pt['y'] = traj.position.y
        pt['theta'] = traj.theta
        pt['v'] = traj.velocity
        pt['acc'] = traj.acceleration
        pt['omega'] = traj.omega
        pt['curva'] = traj.curva
        pt['acc_lat'] = traj.velocity * traj.omega
        traj_list.append(pt)

    trajs_new = motion.get_new_trajectory()
    traj_new_list = []
    for traj in trajs_new:
        pt = {}
        pt['x'] = traj.position.x
        pt['y'] = traj.position.y
        pt['theta'] = traj.theta
        pt['v'] = traj.velocity
        pt['acc'] = traj.acceleration
        pt['omega'] = traj.omega
        pt['curva'] = traj.curva
        pt['acc_lat'] = traj.velocity * traj.omega
        traj_new_list.append(pt)

    math_points = motion.get_match_point()
    ref_omega = motion.get_ref_omega()
    for i, val in enumerate(ref_omega):
        traj_new_list[i]['ref_omega'] = val
    ref_curva = motion.get_ref_kappa()
    for i, val in enumerate(ref_curva):
        traj_new_list[i]['ref_curva'] = val
    ref_v = motion.get_ref_v()
    for i, val in enumerate(ref_v):
        traj_new_list[i]['ref_vel'] = val
    ref_a = motion.get_ref_a()
    for i, val in enumerate(ref_a):
        traj_new_list[i]['ref_acc'] = val
    
    plot_cilqr_env(pts,obs,planning_init,traj_list,traj_new_list,math_points)

    # plot_iter_stat_list(motion)

def plot_cilqr_env(lane,objs,planning_origin, traj,traj_new,match_points):
    p = figure(width=900, height=400, match_aspect=True)
    lane_list = []
    a = 0
    step_vh = 2.0

    # init traj
    # p.scatter(planning_origin['x'], planning_origin['y'], line_width=2, size = 8, color = Magma[6][3])
    vh = VeeHead(size=5, fill_color=Viridis[3][0])
    next_planning_origin_x = planning_origin['x'] + math.cos(planning_origin['theta']) * step_vh
    next_planning_origin_y = planning_origin['y'] + math.sin(planning_origin['theta']) * step_vh
    # p.add_layout(Arrow(end=vh, x_start=planning_origin['x'], y_start=planning_origin['y'], x_end=next_planning_origin_x, y_end=next_planning_origin_y))
    for pt in traj:
      p.scatter(pt['x'], pt['y'], line_width=2, size = 8 ,color = Magma[6][4], legend_label="traj_init")
      next_pt_x = pt['x'] + math.cos(pt['theta']) * step_vh
      next_pt_y = pt['y'] + math.sin(pt['theta']) * step_vh
      p.add_layout(Arrow(end=vh, x_start=pt['x'], y_start=pt['y'], x_end=next_pt_x, y_end=next_pt_y))
    
    # result traj
    x_new = []
    y_new = []
    theta_new = []
    for pt in traj_new:
        x_new.append(pt['x'])
        y_new.append(pt['y'])
        theta_new.append(pt['theta'])

        # draw arrow
        next_pt_x = pt['x'] + math.cos(pt['theta']) * step_vh
        next_pt_y = pt['y'] + math.sin(pt['theta']) * step_vh
        p.add_layout(Arrow(end=vh, x_start=pt['x'], y_start=pt['y'], x_end=next_pt_x, y_end=next_pt_y))
    source = ColumnDataSource(data=dict(
        x=x_new,
        y=y_new,
        theta=theta_new,
    ))
    scatter_new = p.scatter('x', 'y', source=source, line_width=2, size = 8 ,color = Magma[6][3], legend_label="traj_new")
    hover_tool = HoverTool(renderers=[scatter_new], tooltips=[
                                                            ("x", "@x"),
                                                            ("y", "@y"),
                                                            ("theta", "@theta"),
                                                            ], mode='mouse')
    p.add_tools(hover_tool)

    # lane match ref point
    for pt in match_points:
        p.scatter(pt.x, pt.y, line_width=2, size = 8 ,color = 'yellow', legend_label="match_point")
    # lanes
    for lane_pts in lane:
        pts_tmp_x = []
        pts_tmp_y = []
        for pt in lane_pts:
            pts_tmp_x.append(pt.x)
            pts_tmp_y.append(pt.y)

        p.scatter(pts_tmp_x, pts_tmp_y, line_width=2, color = Magma[6][a], legend_label="lane")
        a = a +1
    # objs
    for obj in objs:
        pts_tmp_x = []
        pts_tmp_y = []
        for pt in obj.trajectory_points:
            pts_tmp_x.append(pt.position.x)
            pts_tmp_y.append(pt.position.y)
            
        p.scatter(pts_tmp_x, pts_tmp_y, line_width=2, color = Viridis[6][a], size = 10,legend_label="obj")
        a = a +1
    
    p.toolbar.active_scroll = p.select_one(WheelZoomTool)
    p.legend.click_policy = 'hide'
    output_notebook()
    show(p)

    plot_traj_acc_lat(traj_new)
    plot_traj_omega(traj_new)
    plot_traj_theta(traj_new)
    # plot_traj_curva(traj_new)
    plot_traj_v(traj_new)
    plot_traj_acc(traj_new)

def get_use_exp_list(l_conditions):
    use_exp_list = []
    for condition in l_conditions:
        use_exp = False
#         print(condition.use_exp_model_map)
        for item in condition.use_exp_model_map:
            if condition.use_exp_model_map[item] == True:
                use_exp = True
                break
        use_exp_list.append(use_exp)
    return use_exp_list

def run_motion(path, point_init):
    motion = pybind_ilqr.LateralLongitudinalMotion()
    motion.init(path,point_init)
    motion.execute()
    return motion

def plot_init_trah_gen(motion):
    env = motion.get_env()
    lanes = env.get_all_center_points()
    objs = env.get_all_obstacle()

    init_traj_gen_success = motion.get_init_traj_gen_success()
    if init_traj_gen_success:
        print("init_traj_gen success")
    else:
        print("init_traj_gen failed")

    motion_init_traj = motion.get_init_trajectory()
    init_traj = []
    for traj in motion_init_traj:
        pt = {}
        pt['x'] = traj.position.x
        pt['y'] = traj.position.y
        pt['theta'] = traj.theta
        pt['v'] = traj.velocity
        pt['acc'] = traj.acceleration
        pt['omega'] = traj.omega
        pt['curva'] = traj.curva
        pt['acc_lat'] = traj.velocity * traj.omega
        init_traj.append(pt)

    motion_traj_new = motion.get_new_trajectory()
    traj_new = []
    for traj in motion_traj_new:
        pt = {}
        pt['x'] = traj.position.x
        pt['y'] = traj.position.y
        pt['theta'] = traj.theta
        pt['v'] = traj.velocity
        pt['acc'] = traj.acceleration
        pt['omega'] = traj.omega
        pt['curva'] = traj.curva
        pt['acc_lat'] = traj.velocity * traj.omega
        traj_new.append(pt)
    
    math_points = motion.get_match_point()
    ref_omega = motion.get_ref_omega()
    for i, val in enumerate(ref_omega):
        traj_new[i]['ref_omega'] = val
    ref_curva = motion.get_ref_kappa()
    for i, val in enumerate(ref_curva):
        traj_new[i]['ref_curva'] = val
    ref_v = motion.get_ref_v()
    for i, val in enumerate(ref_v):
        traj_new[i]['ref_vel'] = val
    ref_a = motion.get_ref_a()
    for i, val in enumerate(ref_a):
        traj_new[i]['ref_acc'] = val
    
    l_conditions = motion.get_l_condition()
    use_exp_list = get_use_exp_list(l_conditions)

    # main fig
    p = figure(width=900, height=400, match_aspect=True)
    step_vh = 2.0
    vehicle_length = 4.9
    vehicle_width = 1.9

    # init traj
    vh = VeeHead(size=5, fill_color=Viridis[3][0])
    for pt in init_traj:
      p.scatter(pt['x'], pt['y'], line_width=2, size = 8 ,color = Magma[6][4], legend_label="traj_init")
      next_pt_x = pt['x'] + math.cos(pt['theta']) * step_vh
      next_pt_y = pt['y'] + math.sin(pt['theta']) * step_vh
      p.add_layout(Arrow(end=vh, x_start=pt['x'], y_start=pt['y'], x_end=next_pt_x, y_end=next_pt_y))
    
    # result traj
    # x_new = []
    # y_new = []
    # x_new_use_exp = []
    # y_new_use_exp = []
    # theta_new = []
    # theta_new_use_exp = []
    # x_text = []
    # y_text = []
    # text_l = []
    # index = 0
    # for pt in traj_new:
    #     if use_exp_list[index]:
    #         x_new_use_exp.append(pt['x'])
    #         y_new_use_exp.append(pt['y'])
    #         theta_new_use_exp.append(pt['theta'])
    #     else:
    #         x_new.append(pt['x'])
    #         y_new.append(pt['y'])
    #         theta_new.append(pt['theta'])
    #     # draw arrow
    #     next_pt_x = pt['x'] + math.cos(pt['theta']) * step_vh
    #     next_pt_y = pt['y'] + math.sin(pt['theta']) * step_vh
    #     veh_vec_len_front_x = 0.5 * vehicle_length * math.cos(pt['theta']) 
    #     veh_vec_len_front_y = 0.5 * vehicle_length * math.sin(pt['theta']) 
    #     veh_vec_wid_left_x = 0.5 * vehicle_width * math.cos(pt['theta'] + 0.5 * math.pi)
    #     veh_vec_wid_left_y = 0.5 * vehicle_width * math.sin(pt['theta'] + 0.5 * math.pi) 
    #     r_l = [[1,1],[1,-1],[-1,-1],[-1,1]]
    #     ploy_gon_x = []
    #     ploy_gon_y = []
    #     for r in r_l:
    #         ploy_gon_x.append(pt['x'] + r[0] * veh_vec_len_front_x + r[1] * veh_vec_wid_left_x)
    #         ploy_gon_y.append(pt['y'] + r[0] * veh_vec_len_front_y + r[1] * veh_vec_wid_left_y)
    #     x_text.append(pt['x'])
    #     y_text.append(pt['y'])
    #     text_l.append(str(index))
    #     p.patch(ploy_gon_x, ploy_gon_y, alpha=0.2, line_width=2, color = Magma[6][3],legend_label="traj_new")
    #     p.add_layout(Arrow(end=vh, x_start=pt['x'], y_start=pt['y'], x_end=next_pt_x, y_end=next_pt_y))
    #     index = index + 1
    # source = ColumnDataSource(data=dict(
    #     x=x_new,
    #     y=y_new,
    #     theta=theta_new,
    # ))
    # source_exp = ColumnDataSource(data=dict(
    #     x=x_new_use_exp,
    #     y=y_new_use_exp,
    #     theta=theta_new_use_exp,
    # ))
    # scatter_new = p.scatter('x', 'y', source=source, line_width=2, size = 8 ,color = Magma[6][3], legend_label="traj_new")
    # scatter_new_exp = p.diamond('x', 'y', source=source_exp, line_width=2, size = 10 ,color = Magma[6][4], legend_label="traj_new_exp")
    # p.text(x_text,y_text, text=text_l,text_font_size = "7pt")
    # hover_tool = HoverTool(renderers=[scatter_new,scatter_new_exp], tooltips=[
    #                                                         ("x", "@x"),
    #                                                         ("y", "@y"),
    #                                                         ("theta", "@theta"),
    #                                                         ], mode='mouse')
    # p.add_tools(hover_tool)

    # lanes
    
    for lane_pts in lanes:
        pts_tmp_x = []
        pts_tmp_y = []
        for pt in lane_pts:
            pts_tmp_x.append(pt.x)
            pts_tmp_y.append(pt.y)

        p.scatter(pts_tmp_x, pts_tmp_y, line_width=2, legend_label="lane",alpha = 0.1)

    # objs
    a = 0
    for obj in objs:
        text_l = []
        obj_length = obj.get_length()
        obj_width = obj.get_width()
        pts_tmp_x = []
        pts_tmp_y = []
        index = 0
        for pt in obj.trajectory_points:
            pts_tmp_x.append(pt.position.x)
            pts_tmp_y.append(pt.position.y)
            vec_len_front_x = 0.5 * obj_length * math.cos(pt.theta) 
            vec_len_front_y = 0.5 * obj_length * math.sin(pt.theta) 
            vec_wid_left_x = 0.5 * obj_width * math.cos(pt.theta + 0.5 * math.pi)
            vec_wid_left_y = 0.5 * obj_width * math.sin(pt.theta + 0.5 * math.pi) 
            r_l = [[1,1],[1,-1],[-1,-1],[-1,1]]
            ploy_gon_x = []
            ploy_gon_y = []
            for r in r_l:
                ploy_gon_x.append(pt.position.x + r[0] * vec_len_front_x + r[1] * vec_wid_left_x)
                ploy_gon_y.append(pt.position.y + r[0] * vec_len_front_y + r[1] * vec_wid_left_y)
            p.patch(ploy_gon_x, ploy_gon_y, alpha=0.2, line_width=2, color = Category20[20][a],legend_label="obj")
            text_l.append(str(index))
            if index > 30:
                break
            index = index + 1
        p.scatter(pts_tmp_x, pts_tmp_y, line_width=2, color = Category20[20][a], size = 5,legend_label="obj")
        p.text(pts_tmp_x, pts_tmp_y, text=text_l,text_font_size = "7pt")
        a = a +1
    
    # freespace
    road_edge_list = env.get_all_road_edge_points()
    for road_edge in road_edge_list:
        pts_tmp_x = []
        pts_tmp_y = []
        for pt in road_edge:
            pts_tmp_x.append(pt.x)
            pts_tmp_y.append(pt.y)
        p.scatter(pts_tmp_x, pts_tmp_y, line_width=2, color='red', legend_label="road_edge",alpha = 0.6)
    
    p.toolbar.active_scroll = p.select_one(WheelZoomTool)
    p.legend.click_policy = 'hide'
    output_notebook()

    # plot_traj_acc_lat(traj_new)
    # plot_traj_omega(traj_new)
    # plot_traj_theta(traj_new)
    # # plot_traj_curva(traj_new)
    # plot_traj_v(traj_new)
    # plot_traj_acc(traj_new)
    show(p)

def plot_init_traj_gen_failed_path(motion):
    failed_path_set = motion.get_init_traj_gen_failed_path_set()
    failed_path_size = len(failed_path_set)
    if failed_path_size == 0:
        print("failed_path_set empty")
        return
    else :
        print("failed_path_set size ", failed_path_size)

    norm = matplotlib.colors.Normalize(vmin=0, vmax=failed_path_size, clip=True)
    mapper = cm.ScalarMappable(norm=norm, cmap=cm.get_cmap("Wistia", 1024))

    path_set = []
    for i, path in enumerate(failed_path_set):
        color = matplotlib.colors.rgb2hex(mapper.to_rgba(i))
        acc = []
        w = []
        t = []
        for j, pt in enumerate(path):
            acc.append(pt.acceleration)
            w.append(pt.omega)
            t.append(j*0.2)
        traj_dict = {
            'acc_list': acc,
            'w_list': w,
            't_list': t,
            'color': color,
            'index': i
        }
        path_set.append(traj_dict)
    
    init_traj_gen_success = motion.get_init_traj_gen_success()
    if init_traj_gen_success:
        motion_init_traj = motion.get_init_trajectory()
        init_traj_acc = []
        init_traj_w = []
        init_traj_t = []
        for i, traj in enumerate(motion_init_traj):
            init_traj_acc.append(traj.acceleration)
            init_traj_w.append(traj.omega)
            init_traj_t.append(i*0.2)
        init_traj_dict = {
                'acc_list': init_traj_acc,
                'w_list': init_traj_w,
                't_list': init_traj_t,
                'color': 'green',
                'index': -1
            }
        path_set.append(init_traj_dict)

    # acc_t
    fig_acc_t = figure(width=900, height=400, match_aspect=True, 
                        x_axis_label='t/s', y_axis_label='acc/m/s^2', 
                        x_range = [0,6], y_range = [-5, 5])
    for path in path_set:
        acc_list = path['acc_list']
        w_list = path['w_list']
        t_list = path['t_list']
        color = path['color']
        index = path['index']
        
        fig_acc_t.scatter(t_list, acc_list, line_width=2, size = 2 ,color = color)
        fig_acc_t.line(t_list, acc_list)

        # 只在终点标出path序列
        acc_end = acc_list[-1]
        t_end = t_list[-1]
        scatter_last_pt = fig_acc_t.scatter(t_end, acc_end, line_width=2, size = 8 ,color = color, marker = "x")
        temp_str = ""
        for i, acc in enumerate(acc_list):
            w = w_list[i]
            temp_str += "[" + str(acc) + "," + str(w) + "] "
        hover_tool = HoverTool(renderers=[scatter_last_pt], tooltips=[
                                                                ("index", str(index)),
                                                                ("path", temp_str),
                                                                ], mode='mouse')
        fig_acc_t.add_tools(hover_tool)

    fig_acc_t.toolbar.active_scroll = fig_acc_t.select_one(WheelZoomTool)
    output_notebook()
    show(fig_acc_t)

    # w_t

def plot_cilqr_constraint(lane,objs,planning_origin,traj,traj_new,use_exp_list):
    p = figure(width=900, height=400, match_aspect=True)
    lane_list = []
    a = 0
    step_vh = 2.0

    vehicle_length = 4.9
    vehicle_width = 1.9
    # init traj
    # p.scatter(planning_origin['x'], planning_origin['y'], line_width=2, size = 8, color = Magma[6][3])
    vh = VeeHead(size=5, fill_color=Viridis[3][0])
    for pt in traj:
      p.scatter(pt['x'], pt['y'], line_width=2, size = 8 ,color = Magma[6][4], legend_label="traj_init")
      next_pt_x = pt['x'] + math.cos(pt['theta']) * step_vh
      next_pt_y = pt['y'] + math.sin(pt['theta']) * step_vh
      p.add_layout(Arrow(end=vh, x_start=pt['x'], y_start=pt['y'], x_end=next_pt_x, y_end=next_pt_y))
    # result traj
    x_new = []
    y_new = []
    x_new_use_exp = []
    y_new_use_exp = []
    theta_new = []
    theta_new_use_exp = []
    x_text = []
    y_text = []
    text_l = []
    index = 0
    for pt in traj_new:
        if use_exp_list[index]:
            x_new_use_exp.append(pt['x'])
            y_new_use_exp.append(pt['y'])
            theta_new_use_exp.append(pt['theta'])
        else:
            x_new.append(pt['x'])
            y_new.append(pt['y'])
            theta_new.append(pt['theta'])
        # draw arrow
        next_pt_x = pt['x'] + math.cos(pt['theta']) * step_vh
        next_pt_y = pt['y'] + math.sin(pt['theta']) * step_vh
        veh_vec_len_front_x = 0.5 * vehicle_length * math.cos(pt['theta']) 
        veh_vec_len_front_y = 0.5 * vehicle_length * math.sin(pt['theta']) 
        veh_vec_wid_left_x = 0.5 * vehicle_width * math.cos(pt['theta'] + 0.5 * math.pi)
        veh_vec_wid_left_y = 0.5 * vehicle_width * math.sin(pt['theta'] + 0.5 * math.pi) 
        r_l = [[1,1],[1,-1],[-1,-1],[-1,1]]
        ploy_gon_x = []
        ploy_gon_y = []
        for r in r_l:
            ploy_gon_x.append(pt['x'] + r[0] * veh_vec_len_front_x + r[1] * veh_vec_wid_left_x)
            ploy_gon_y.append(pt['y'] + r[0] * veh_vec_len_front_y + r[1] * veh_vec_wid_left_y)
        x_text.append(pt['x'])
        y_text.append(pt['y'])
        text_l.append(str(index))
        p.patch(ploy_gon_x, ploy_gon_y, alpha=0.2, line_width=2, color = Magma[6][3],legend_label="traj_new")
        p.add_layout(Arrow(end=vh, x_start=pt['x'], y_start=pt['y'], x_end=next_pt_x, y_end=next_pt_y))
        index = index + 1
    source = ColumnDataSource(data=dict(
        x=x_new,
        y=y_new,
        theta=theta_new,
    ))
    source_exp = ColumnDataSource(data=dict(
        x=x_new_use_exp,
        y=y_new_use_exp,
        theta=theta_new_use_exp,
    ))
    scatter_new = p.scatter('x', 'y', source=source, line_width=2, size = 8 ,color = Magma[6][3], legend_label="traj_new")
    scatter_new_exp = p.diamond('x', 'y', source=source_exp, line_width=2, size = 10 ,color = Magma[6][4], legend_label="traj_new_exp")
    p.text(x_text,y_text, text=text_l,text_font_size = "7pt")
    hover_tool = HoverTool(renderers=[scatter_new,scatter_new_exp], tooltips=[
                                                            ("x", "@x"),
                                                            ("y", "@y"),
                                                            ("theta", "@theta"),
                                                            ], mode='mouse')
    p.add_tools(hover_tool)

    # lanes
    for lane_pts in lane:
        pts_tmp_x = []
        pts_tmp_y = []
        for pt in lane_pts:
            pts_tmp_x.append(pt.x)
            pts_tmp_y.append(pt.y)

        p.scatter(pts_tmp_x, pts_tmp_y, line_width=2, color = Magma[6][a], legend_label="lane",alpha = 0.1)
        a = a +1
    # objs

    for obj in objs:
        text_l = []
        obj_length = obj.get_length()
        obj_width = obj.get_width()
        pts_tmp_x = []
        pts_tmp_y = []
        index = 0
        for pt in obj.trajectory_points:
            pts_tmp_x.append(pt.position.x)
            pts_tmp_y.append(pt.position.y)
            vec_len_front_x = 0.5 * obj_length * math.cos(pt.theta) 
            vec_len_front_y = 0.5 * obj_length * math.sin(pt.theta) 
            vec_wid_left_x = 0.5 * obj_width * math.cos(pt.theta + 0.5 * math.pi)
            vec_wid_left_y = 0.5 * obj_width * math.sin(pt.theta + 0.5 * math.pi) 
            r_l = [[1,1],[1,-1],[-1,-1],[-1,1]]
            ploy_gon_x = []
            ploy_gon_y = []
            for r in r_l:
                ploy_gon_x.append(pt.position.x + r[0] * vec_len_front_x + r[1] * vec_wid_left_x)
                ploy_gon_y.append(pt.position.y + r[0] * vec_len_front_y + r[1] * vec_wid_left_y)
            p.patch(ploy_gon_x, ploy_gon_y, alpha=0.2, line_width=2, color = Viridis[6][a],legend_label="obj")
            text_l.append(str(index))
            if index > 30:
                break
            index = index + 1
        p.scatter(pts_tmp_x, pts_tmp_y, line_width=2, color = Viridis[6][a], size = 10,legend_label="obj")
        p.text(pts_tmp_x, pts_tmp_y, text=text_l,text_font_size = "7pt")
        a = a +1
    
    p.toolbar.active_scroll = p.select_one(WheelZoomTool)
    p.legend.click_policy = 'hide'
    output_notebook()


    plot_traj_acc_lat(traj_new)
    plot_traj_omega(traj_new)
    plot_traj_theta(traj_new)
    # plot_traj_curva(traj_new)
    plot_traj_v(traj_new)
    plot_traj_acc(traj_new)
    show(p)

def plot_tree_cilqr_constraint(lane,objs,planning_origin,traj,traj_trees,use_exp_list):
    p = figure(width=900, height=400, match_aspect=True)
    lane_list = []
    a = 0
    step_vh = 2.0

    vehicle_length = 4.9
    vehicle_width = 1.9
    # init traj
    # p.scatter(planning_origin['x'], planning_origin['y'], line_width=2, size = 8, color = Magma[6][3])
    vh = VeeHead(size=5, fill_color=Viridis[3][0])
    for pt in traj:
      p.scatter(pt['x'], pt['y'], line_width=2, size = 8 ,color = Magma[6][4], legend_label="traj_init")
      next_pt_x = pt['x'] + math.cos(pt['theta']) * step_vh
      next_pt_y = pt['y'] + math.sin(pt['theta']) * step_vh
      p.add_layout(Arrow(end=vh, x_start=pt['x'], y_start=pt['y'], x_end=next_pt_x, y_end=next_pt_y))
    # result traj
    x_new = []
    y_new = []
    x_new_use_exp = []
    y_new_use_exp = []
    theta_new = []
    theta_new_use_exp = []
    match_point_x = []
    match_point_y = []
    x_text = []
    y_text = []
    text_l = []
    index = 0
    for traj_new in traj_trees:
        for pt in traj_new:
            if len(use_exp_list) > index and use_exp_list[index]:
                x_new_use_exp.append(pt['x'])
                y_new_use_exp.append(pt['y'])
                theta_new_use_exp.append(pt['theta'])
            else:
                x_new.append(pt['x'])
                y_new.append(pt['y'])
                theta_new.append(pt['theta'])
            # draw arrow
            next_pt_x = pt['x'] + math.cos(pt['theta']) * step_vh
            next_pt_y = pt['y'] + math.sin(pt['theta']) * step_vh
            veh_vec_len_front_x = 0.5 * vehicle_length * math.cos(pt['theta']) 
            veh_vec_len_front_y = 0.5 * vehicle_length * math.sin(pt['theta']) 
            veh_vec_wid_left_x = 0.5 * vehicle_width * math.cos(pt['theta'] + 0.5 * math.pi)
            veh_vec_wid_left_y = 0.5 * vehicle_width * math.sin(pt['theta'] + 0.5 * math.pi) 
            r_l = [[1,1],[1,-1],[-1,-1],[-1,1]]
            ploy_gon_x = []
            ploy_gon_y = []
            for r in r_l:
                ploy_gon_x.append(pt['x'] + r[0] * veh_vec_len_front_x + r[1] * veh_vec_wid_left_x)
                ploy_gon_y.append(pt['y'] + r[0] * veh_vec_len_front_y + r[1] * veh_vec_wid_left_y)
            x_text.append(pt['x'])
            y_text.append(pt['y'])
            match_point_x.append(pt['match_point'].x)
            match_point_y.append(pt['match_point'].y)
            text_l.append(str(index))
            p.patch(ploy_gon_x, ploy_gon_y, alpha=0.2, line_width=2, color = Magma[6][3],legend_label="traj_new")
            p.add_layout(Arrow(end=vh, x_start=pt['x'], y_start=pt['y'], x_end=next_pt_x, y_end=next_pt_y))
            index = index + 1
    source = ColumnDataSource(data=dict(
        x=x_new,
        y=y_new,
        theta=theta_new,
    ))
    source_exp = ColumnDataSource(data=dict(
        x=x_new_use_exp,
        y=y_new_use_exp,
        theta=theta_new_use_exp,
    ))
    source_match_point = ColumnDataSource(data=dict(
        x=match_point_x,
        y=match_point_y,
    ))
    scatter_new = p.scatter('x', 'y', source=source, line_width=2, size = 8 ,color = Magma[6][3], legend_label="traj_new")
    scatter_match_point = p.scatter('x', 'y', source=source_match_point, line_width=2, size = 8 ,color = Magma[6][5], legend_label="match_point")
    scatter_new_exp = p.diamond('x', 'y', source=source_exp, line_width=2, size = 10 ,color = Magma[6][4], legend_label="traj_new_exp")
    p.text(x_text,y_text, text=text_l,text_font_size = "7pt")
    hover_tool = HoverTool(renderers=[scatter_new,scatter_new_exp], tooltips=[
                                                            ("x", "@x"),
                                                            ("y", "@y"),
                                                            ("theta", "@theta"),
                                                            ], mode='mouse')
    p.add_tools(hover_tool)

    # lanes
    for lane_pts in lane:
        pts_tmp_x = []
        pts_tmp_y = []
        for pt in lane_pts:
            pts_tmp_x.append(pt.x)
            pts_tmp_y.append(pt.y)

        p.scatter(pts_tmp_x, pts_tmp_y, line_width=2, color = Magma[6][a], legend_label="lane",alpha = 0.1)
        a = a +1
    # objs

    for obj in objs:
        text_l = []
        obj_length = obj.get_length()
        obj_width = obj.get_width()
        pts_tmp_x = []
        pts_tmp_y = []
        index = 0
        for pt in obj.trajectory_points:
            pts_tmp_x.append(pt.position.x)
            pts_tmp_y.append(pt.position.y)
            vec_len_front_x = 0.5 * obj_length * math.cos(pt.theta) 
            vec_len_front_y = 0.5 * obj_length * math.sin(pt.theta) 
            vec_wid_left_x = 0.5 * obj_width * math.cos(pt.theta + 0.5 * math.pi)
            vec_wid_left_y = 0.5 * obj_width * math.sin(pt.theta + 0.5 * math.pi) 
            r_l = [[1,1],[1,-1],[-1,-1],[-1,1]]
            ploy_gon_x = []
            ploy_gon_y = []
            for r in r_l:
                ploy_gon_x.append(pt.position.x + r[0] * vec_len_front_x + r[1] * vec_wid_left_x)
                ploy_gon_y.append(pt.position.y + r[0] * vec_len_front_y + r[1] * vec_wid_left_y)
            p.patch(ploy_gon_x, ploy_gon_y, alpha=0.2, line_width=2, color = Viridis[6][a],legend_label="obj")
            text_l.append(str(index))
            if index > 30:
                break
            index = index + 1
        p.scatter(pts_tmp_x, pts_tmp_y, line_width=2, color = Viridis[6][a], size = 10,legend_label="obj")
        p.text(pts_tmp_x, pts_tmp_y, text=text_l,text_font_size = "7pt")
        a = a +1
    
    p.toolbar.active_scroll = p.select_one(WheelZoomTool)
    p.legend.click_policy = 'hide'
    output_notebook()


    plot_traj_acc_lat(traj_new)
    plot_traj_omega(traj_new)
    plot_traj_theta(traj_new)
    # plot_traj_curva(traj_new)
    plot_traj_lat_dis(traj_new)
    plot_traj_omega_dot(traj_new)
    plot_traj_v(traj_new)
    plot_traj_acc(traj_new)
    plot_traj_jerk(traj_new)
    show(p)

def plot_traj_v(traj_new):
    res_list = []
    ref_list = []
    for pt in traj_new:
        if 'v' not in pt:
            return
        res_list.append(pt['v'])
        if 'ref_vel' not in pt:
            return
        ref_list.append(pt['ref_vel']) 
        
    fig_source = ColumnDataSource({
                    'x': list(range(len(res_list))), 
                    'y': res_list
                    })
    
    ref_source = ColumnDataSource({
                'x': list(range(len(ref_list))), 
                'y': ref_list
                })

    fig = figure(width=800, height=150, match_aspect=True, x_range=[-1, 31])
    fig.scatter('x', 'y', source=fig_source, line_width=2, size = 8 ,color = 'red', legend_label="res_vel")
    fig.line('x', 'y', source=fig_source)
    
    fig.scatter('x', 'y', source=ref_source, line_width=2, size = 8 ,color = 'blue', legend_label="ref_vel")
    fig.line('x', 'y', source=ref_source)
    
    hover_tool_res_v = HoverTool( tooltips=[("v", "@y"), ("index", "@x"),], mode='mouse')
    fig.add_tools(hover_tool_res_v)
    fig.toolbar.active_scroll = fig.select_one(WheelZoomTool)
    fig.legend.click_policy = 'hide'
    output_notebook()
    show(fig)

def plot_traj_acc(traj_new):
    res_list = []
    ref_list = []
    for pt in traj_new:
        if 'acc' not in pt:
            return
        res_list.append(pt['acc'])
        if 'ref_acc' not in pt:
            return
        ref_list.append(pt['ref_acc']) 
        
    fig_source = ColumnDataSource({
                    'x': list(range(len(res_list))), 
                    'y': res_list
                    })
    
    ref_source = ColumnDataSource({
                'x': list(range(len(ref_list))), 
                'y': ref_list
                })

    fig = figure(width=800, height=150, match_aspect=True, x_range=[-1, 31])
    fig.scatter('x', 'y', source=fig_source, line_width=2, size = 8 ,color = 'red', legend_label="res_acc")
    fig.line('x', 'y', source=fig_source)
    
    fig.scatter('x', 'y', source=ref_source, line_width=2, size = 8 ,color = 'blue', legend_label="ref_acc")
    fig.line('x', 'y', source=ref_source)
    
    hover_tool_res_v = HoverTool( tooltips=[("acc", "@y"),
                                   ("index", "@x"),
                                     ], mode='mouse')
    fig.add_tools(hover_tool_res_v)
    fig.toolbar.active_scroll = fig.select_one(WheelZoomTool)
    fig.legend.click_policy = 'hide'
    output_notebook()
    show(fig)

def plot_traj_jerk(traj_new):
    res_list = []
    ref_list = []
    for pt in traj_new:
        if 'jerk' not in pt:
            return
        res_list.append(pt['jerk'])
    
        
    fig_source = ColumnDataSource({
                    'x': list(range(len(res_list))), 
                    'y': res_list
                    })


    fig = figure(width=800, height=150, match_aspect=True, x_range=[-1, 31])
    fig.scatter('x', 'y', source=fig_source, line_width=2, size = 8 ,color = 'red', legend_label="jerk")
    fig.line('x', 'y', source=fig_source)

    
    hover_tool_res_v = HoverTool( tooltips=[("jerk", "@y"),
                                   ("index", "@x"),
                                     ], mode='mouse')
    fig.add_tools(hover_tool_res_v)
    fig.toolbar.active_scroll = fig.select_one(WheelZoomTool)
    fig.legend.click_policy = 'hide'
    output_notebook()
    show(fig)

def plot_traj_lat_dis(traj_new):
    res_list = []
    for pt in traj_new:
        if 'lat_dis' not in pt:
            return
        res_list.append(pt['lat_dis'])
        
    fig_source = ColumnDataSource({
                    'x': list(range(len(res_list))), 
                    'y': res_list
                    })

    fig = figure(width=800, height=150, match_aspect=True, x_range=[-1, 31], y_range=[-1, 1],
                    y_axis_label="lat dis")
    scatter_res = fig.scatter('x', 'y', source=fig_source, line_width=2, size = 8 ,color = 'red', legend_label="lat dis")
    fig.line('x', 'y', source=fig_source)
    hover_tool_res_v = HoverTool(renderers=[scatter_res], tooltips=[("lat_dis", "@y"),
                                                            ("index", "@x"),
                                                            ], mode='mouse')
    fig.add_tools(hover_tool_res_v)
    fig.toolbar.active_scroll = fig.select_one(WheelZoomTool)
    fig.legend.click_policy = 'hide'
    output_notebook()
    show(fig)

def plot_traj_omega_dot(traj_new):
    res_list = []
    for pt in traj_new:
        if 'omega_dot' not in pt:
            return
        res_list.append(pt['omega_dot'])
        
    fig_source = ColumnDataSource({
                    'x': list(range(len(res_list))), 
                    'y': res_list
                    })

    fig = figure(width=800, height=150, match_aspect=True, x_range=[-1, 31],
                    y_axis_label="omega dot")
    scatter_res = fig.scatter('x', 'y', source=fig_source, line_width=2, size = 8 ,color = 'red', legend_label="omega dot")
    fig.line('x', 'y', source=fig_source)
    hover_tool_res_v = HoverTool(renderers=[scatter_res], tooltips=[("omega_dot", "@y"),
                                                            ("index", "@x"),
                                                            ], mode='mouse')
    fig.add_tools(hover_tool_res_v)
    fig.toolbar.active_scroll = fig.select_one(WheelZoomTool)
    fig.legend.click_policy = 'hide'
    output_notebook()
    show(fig)

def plot_traj_theta(traj_new):
    res_list = []
    for pt in traj_new:
        if 'theta' not in pt:
            return
        res_list.append(pt['theta'] * 180.0 / math.pi)
        
    fig_source = ColumnDataSource({
                    'x': list(range(len(res_list))), 
                    'y': res_list
                    })

    fig = figure(width=800, height=150, match_aspect=True, x_range=[-1, 31],
                    y_axis_label="Theta / degree")
    scatter_res = fig.scatter('x', 'y', source=fig_source, line_width=2, size = 8 ,color = 'red', legend_label="res_theta")
    fig.line('x', 'y', source=fig_source)
    hover_tool_res_v = HoverTool(renderers=[scatter_res], tooltips=[("theta", "@y"),
                                                            ("index", "@x"),
                                                            ], mode='mouse')
    fig.add_tools(hover_tool_res_v)
    fig.toolbar.active_scroll = fig.select_one(WheelZoomTool)
    fig.legend.click_policy = 'hide'
    output_notebook()
    show(fig)

def plot_traj_omega(traj_new):
    res_list = []
    ref_list = []
    for pt in traj_new:
        if 'omega' not in pt:
            return
        res_list.append(pt['omega'] * 180.0 / math.pi)
        if 'ref_omega' not in pt:
            return
        ref_list.append(pt['ref_omega'] * 180.0 / math.pi)    
        
    fig_source = ColumnDataSource({
                    'x': list(range(len(res_list))), 
                    'y': res_list
                    })

    ref_source = ColumnDataSource({
                'x': list(range(len(ref_list))), 
                'y': ref_list
                })

    fig = figure(width=800, height=150, match_aspect=True, x_range=[-1, 31],
                    y_axis_label="Omega / degree/s")

    fig.scatter('x', 'y', source=fig_source, line_width=2, size = 8 ,color = 'red', legend_label="res_omega")
    fig.line('x', 'y', source=fig_source)

    fig.scatter('x', 'y', source=ref_source, line_width=2, size = 8 ,color = 'blue', legend_label="ref_omega")
    fig.line('x', 'y', source=ref_source)

    hover_tool_res_v = HoverTool(tooltips=[("omega", "@y"),
                                            ("index", "@x"),
                                            ], mode='mouse')
    fig.add_tools(hover_tool_res_v)
    fig.toolbar.active_scroll = fig.select_one(WheelZoomTool)
    fig.legend.click_policy = 'hide'
    output_notebook()
    show(fig)

def plot_traj_curva(traj_new):
    res_list = []
    ref_list = []
    for pt in traj_new:
        if 'curva' not in pt:
            return
        res_list.append(pt['curva'])
    
        if 'ref_curva' not in pt:
            return
        ref_list.append(pt['ref_curva'])
        
    fig_source = ColumnDataSource({
                    'x': list(range(len(res_list))), 
                    'y': res_list
                    })
    
    ref_source = ColumnDataSource({
                'x': list(range(len(ref_list))), 
                'y': ref_list
                })

    fig = figure(width=800, height=150, match_aspect=True, x_range=[-1, 31])
    fig.scatter('x', 'y', source=fig_source, line_width=2, size = 8 ,color = 'red', legend_label="curva")
    fig.line('x', 'y', source=fig_source)

    fig.scatter('x', 'y', source=ref_source, line_width=2, size = 8 ,color = 'blue', legend_label="ref_curva")
    fig.line('x', 'y', source=ref_source)

    hover_tool_res_v = HoverTool(tooltips=[("curva", "@y"),
                                        ("index", "@x"),
                                        ], mode='mouse')
    fig.add_tools(hover_tool_res_v)
    fig.toolbar.active_scroll = fig.select_one(WheelZoomTool)
    fig.legend.click_policy = 'hide'
    output_notebook()
    show(fig)

def plot_traj_acc_lat(traj_new):
    res_list = []
    ref_list = []
    for pt in traj_new:
        if 'acc_lat' not in pt:
            return
        res_list.append(pt['acc_lat'])
        if 'acc_lat_ref' not in pt:
            return
        ref_list.append(pt['acc_lat_ref'])
        
    fig_source = ColumnDataSource({
                    'x': list(range(len(res_list))), 
                    'y': res_list
                    })
    fig_source_ref = ColumnDataSource({
                    'x': list(range(len(ref_list))), 
                    'y': ref_list
                    })
    
    fig = figure(width=800, height=150, match_aspect=True, x_range=[-1, 31])
    fig.scatter('x', 'y', source=fig_source, line_width=2, size = 8 ,color = 'red', legend_label="res_acc_lat")
    fig.line('x', 'y', source=fig_source)
    
    fig.scatter('x', 'y', source=fig_source_ref, line_width=2, size = 8 ,color = 'blue', legend_label="ref_acc_lat")
    fig.line('x', 'y', source=fig_source_ref)
    hover_tool_res_v = HoverTool( tooltips=[("acc_lat", "@y"),
                                   ("index", "@x"),
                                     ], mode='mouse')
    fig.add_tools(hover_tool_res_v)
    fig.toolbar.active_scroll = fig.select_one(WheelZoomTool)
    fig.legend.click_policy = 'hide'
    output_notebook()
    show(fig)
