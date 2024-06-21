import json
import math
from pprint import pprint
#planning channel
PLANNING_CHANNEL = '/nop/planning/trajectory'
XDEBUG_CHANNEL = '/xpilot/xdebug'

## 把类似
# data_set = [{"x":-35, 'y':-5},{"x":-30, "y":-10},{"x":-25, "y":-15}]
# 的格式，转化为
# data_set2 = {"x":[-35, -30, -25], "y":[-5, -10, -15]}
def transform_dataset_v1(data_set):
    if not data_set:
        return {}
    keys = data_set[0].keys()
    transformed = {key: [] for key in keys}
    for item in data_set:
        for key in keys:
            transformed[key].append(item[key])
    return transformed

## 把类似
# data_set = [{'x':-35, 'y':-5},{'x':-30, 'y':-10},{'x':-25, 'y':-15}]
# 的格式，转化为
# data_set2 = {"xs":[[-35, -30, -25]], 
#              "ys":[[-5, -10, -15]]}
def transform_dataset_v2(data_set):
    if not data_set:
        return {}

    keys = data_set[0].keys()
    new_keys = {key: key + 's' for key in keys}

    transformed = {new_key: [[]] for new_key in new_keys.values()}

    for item in data_set:
        for key in keys:
            transformed[new_keys[key]][0].append(item[key])

    return transformed

def read_planning_json(json):
    
    msg_json = json['traj']
    if not isinstance(msg_json, dict):
        raise ValueError("Input must be a dictionary representing JSON data.")
    data = {}
    data['traj'] = []
    try:
        if 'traj_point_array' not in msg_json or not isinstance(msg_json['traj_point_array'], list):
            raise ValueError(
                "The input data must contain a list named 'traj_point_array'.")
        for traj_point in msg_json['traj_point_array']:
            if not isinstance(traj_point, dict):
                raise ValueError(
                    "Each element in 'traj_point_array' must be a dictionary.")
            tmp_point = {}
            tmp_point['theta'] = traj_point.get('theta')
            tmp_point['timestampNs'] = traj_point.get('timestamp')
            tmp_point['velocity'] = traj_point.get('velocity')
            direction = traj_point.get('direction', {})
            tmp_point['x_direction'] = direction.get(
                'x') if isinstance(direction, dict) else None
            tmp_point['y_direction'] = direction.get(
                'y') if isinstance(direction, dict) else None
            # tmp_point['z_direction'] = direction.get(
            #     'z') if isinstance(direction, dict) else 0.0
            tmp_point['curv'] = traj_point.get('curvature')
            tmp_point['yawRate'] = traj_point.get('yaw_rate')
            position = traj_point.get('position', {})
            tmp_point['x'] = position.get(
                'x') if isinstance(position, dict) else None
            tmp_point['y'] = position.get(
                'y') if isinstance(position, dict) else None
            # tmp_point['z'] = position.get(
            #     'z') if isinstance(position, dict) else 0.0
            tmp_point['time_difference'] = traj_point.get('time_difference')
            tmp_point['sum_distance'] = traj_point.get('sum_distance')
            tmp_point['acceleration'] = traj_point.get('acceleration')
            data['traj'].append(tmp_point)
        # 0:none; 1:park; 2:reverse; 3:neutral; 4:drive; 5:low
        data['gear'] = msg_json.get('gear', None)
        # 0:no request; 1:both; 2:lat only; 3:long only; 4:error
        data['traj_state'] = msg_json.get('traj_state', None)
        data['traj_signal'] = msg_json.get('traj_signal', None)
        # data['trajectoryMode'] = msg_json.get('trajectoryMode', None)
        # # 0:none; 1:left; 2:right
        # data['turnSignalEnum'] = msg_json.get('turnSignalEnum', None)
        # data['updatedByVehicleStatus'] = 1 if msg_json.get(
        #     'updatedByVehicleStatus', False) else 0
        # data['traj_timestampNs'] = msg_json.get('trajTimestampNs', None)
    except Exception as e:
        print("An error occurred while processing the input data: {}".format(e))
    # pprint(data)
    return data

def read_xdebug_json(msg_json) -> dict:

    json_msg = msg_json
    data = {}

    def get_longi_debug(key):
        try:
            return json_msg['planner_debug']['longi_debug'][key]
        except KeyError:
            return None
        
    def get_lat_debug(key):
        try:
            return json_msg['planner_debug']['lat_debug'][key]
        except KeyError:
            return None
        
    def get_wm_debug(key):
        try:
            return json_msg['planner_debug']['world_model_debug'][key]
        except KeyError:
            return None
        
    def get_planning_debug(key):
        try:
            return json_msg['planner_debug'][key]
        except KeyError:
            return None
    
    def get_lat_lon_decider_debug():
        try:
            return json_msg['planner_debug']['lat_lon_search_debug']
        except KeyError:
            return None
    
    def get_lat_lon_motion_debug():
        try:
            return json_msg['planner_debug']['lat_lon_motion_debug']
        except KeyError:
            return None
    
    def get_lc_lon_search_debug():
        try:
            return json_msg['planner_debug']['lc_lon_search_debug']
        except KeyError:
            return None
    
    def get_lc_lat_search_debug():
        try:
            return json_msg['planner_debug']['lc_lat_search_debug']
        except KeyError:
            return None
        
    def get_osp_debug(key):
        try:
            return json_msg['planner_debug'][key]
        except KeyError:
            return None
    def get_osp_mgr_task_debug(key):
        try:
            return json_msg['planner_debug']['osp_mgr_task_debug'][key]
        except KeyError:
            return None
    def get_freespace_mgr_debug(key):
        try:
            return json_msg['planner_debug']['free_space_mgr_debug'][key]
        except KeyError:
            return None  

    def get_interaction_search_debug():
        try:
            return json_msg['planner_debug']['interaction_search_debug']
        except KeyError:
            return None
    
    def get_fake_sdmap_debug():
        try:
            return json_msg['planner_debug']['fake_sdmap_debug']
        except KeyError:
            return None 

    def get_nn_traj_debug():
        try:
            return json_msg['planner_debug']['nn_traj_debug']
        except KeyError:
            return None 

    data['osp_ref_point_by_ego_car_debug'] = get_osp_debug('osp_ref_point_by_ego_car_debug')
    data['osp_leader_car_history_pose_debug'] = get_osp_debug('osp_leader_car_history_pose_debug')
    data['osp_ref_point_by_leader_car_debug'] = get_osp_debug('osp_ref_point_by_leader_car_debug')
    data['osp_raw_routing_path_points'] = get_osp_mgr_task_debug('raw_routing_path_points')  
    data['osp_raw_routing_path_omega'] = get_osp_mgr_task_debug('raw_routing_path_omega')
    data['osp_motion_tree_points'] = get_osp_mgr_task_debug('motion_tree_points')
    data['osp_motion_tree_speed_limit_seq'] = get_osp_mgr_task_debug('motion_tree_speed_limit')
    data['fs_road_edges_points'] = get_freespace_mgr_debug('raw_road_edges')

    data['nop_counter'] = get_planning_debug('nop_counter')
    
    # read wm debug info
    data['lane_debug'] = get_wm_debug('lane_debug')
    data['obstacle_debug'] = get_wm_debug('obstacle_debug')
    data['ego_info_debug'] = get_wm_debug('ego_info_debug')
    
    # read longi debug info
    data['svlimit_s'] = get_longi_debug('svlimit_s')
    data['svlimit_v_upper'] = get_longi_debug('svlimit_v_upper')
    data['svlimit_v_lower'] = get_longi_debug('svlimit_v_lower')
    data['svlimit_ref_sv_v'] = get_longi_debug('svlimit_ref_sv_v')
    data['svlimit_ref_st_t'] = get_longi_debug('svlimit_ref_st_t')
    data['svlimit_ref_st_s'] = get_longi_debug('svlimit_ref_st_s')
    data['svlimit_ref_vt_v'] = get_longi_debug('svlimit_ref_vt_v')
    data['svlimit_ref_at_a'] = get_longi_debug('svlimit_ref_at_a')
    data['svlimit_v_by_curvature'] = get_longi_debug('svlimit_v_by_curvature')
    data['optimizer_v_ref'] = get_longi_debug('optimizer_v_ref')
    data['optimizer_v_upper'] = get_longi_debug('optimizer_v_upper')
    data['optimizer_s_ref'] = get_longi_debug('optimizer_s_ref')
    data['optimizer_s_upper'] = get_longi_debug('optimizer_s_upper')
    data['optimizer_a_ref'] = get_longi_debug('optimizer_a_ref')
    data['optimizer_a_lower'] = get_longi_debug('optimizer_a_lower')
    data['optimizer_a_upper'] = get_longi_debug('optimizer_a_upper')
    data['optimizer_jerk_upper'] = get_longi_debug('optimizer_jerk_upper')
    data['optimizer_jerk_lower'] = get_longi_debug('optimizer_jerk_lower')
    data['s_leader'] = get_longi_debug('s_leader')
    data['v_leader'] = get_longi_debug('v_leader')
    data['s_ref_follow'] = get_longi_debug('s_ref_follow')
    data['v_ref_follow'] = get_longi_debug('v_ref_follow')
    data['a_ref_follow'] = get_longi_debug('a_ref_follow')
    data['optimizer_jerk_out'] = get_longi_debug('optimizer_jerk_out')
    data['optimizer_a_out'] = get_longi_debug('optimizer_a_out')
    data['optimizer_v_out'] = get_longi_debug('optimizer_v_out')
    data['optimizer_s_out'] = get_longi_debug('optimizer_s_out')
    
    # read freespace debug info
    # data['raw_road_edge_seq'] = transform_dataset_v1(
    #     json_msg['planner_debug']['freespace_debug']['raw_road_edge'])
    # data['raw_convex_hull_seq'] = transform_dataset_v2(
    #     json_msg['planner_debug']['freespace_debug']['raw_convex_hull'])
    # pprint(json_msg['planner_debug']['freespace_debug']['raw_convex_hull']) 
 
    # read lat debug info
    data['traj_x'] = get_lat_debug('traj_x')
    data['traj_y'] = get_lat_debug('traj_y')
    data['traj_r_upper_x'] = get_lat_debug('traj_r_upper_x')
    data['traj_r_upper_y'] = get_lat_debug('traj_r_upper_y')
    data['traj_r_lower_x'] = get_lat_debug('traj_r_lower_x')
    data['traj_r_lower_y'] = get_lat_debug('traj_r_lower_y')
    data['traj_r_ref_x'] = get_lat_debug('traj_r_ref_x')
    data['traj_r_ref_y'] = get_lat_debug('traj_r_ref_y')
    data['traj_s'] = get_lat_debug('traj_s')
    data['traj_d'] = get_lat_debug('traj_d')
    data['traj_ref_d'] = get_lat_debug('traj_ref_d')
    data['traj_theta'] = get_lat_debug('traj_theta')
    data['traj_theta_error_ref'] = get_lat_debug('traj_theta_error_ref')
    data['traj_delta_ref'] = get_lat_debug('traj_delta_ref')
    data['traj_delta'] = get_lat_debug('traj_delta')
    # data['traj_curve_spline_ref'] = json_msg['planner_debug']['lat_debug']['traj_curve_spline_ref']

    # data['planning_origin'] = json_msg['planner_debug']['planning_origin_state_debug']

    # read lc search debug info
    data['lc_lon_search_debug'] = get_lc_lon_search_debug()
    data['lc_lat_search_debug'] = get_lc_lat_search_debug()

    # read lat_lon_decider debug info
    data['lat_lon_decider_debug'] = get_lat_lon_decider_debug()

    data['lat_lon_motion_debug'] = get_lat_lon_motion_debug()

    data['interaction_search_debug'] = get_interaction_search_debug()

    data['fake_sdmap_debug'] = get_fake_sdmap_debug()

    data['nn_traj_debug'] = get_nn_traj_debug()

    return data

json_callback_map = {
    PLANNING_CHANNEL: read_planning_json,
    XDEBUG_CHANNEL: read_xdebug_json,
}
