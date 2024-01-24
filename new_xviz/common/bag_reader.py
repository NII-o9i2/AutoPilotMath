import json
from pprint import pprint
#planning channel
PLANNING_CHANNEL = '/nop/planning/trajectory'
XDEBUG_CHANNEL = '/xpilot/xdebug'
#perception channel
PERCEPTION_CHANNEL = '/perception/fusion/object'
PERCEPTION_ROAD_GEOMETRY_CHANNEL = '/perception/center_camera_fov120/road_geometry'
#prediction channel
PREDICTION_CHANNEL = '/prediction/objects'
#vehicle state channel
VEHICLE_REPORT_CHANNEL = '/canbus/vehicle_report'
CHASSIS_A_CHANNEL = '/canbus/chassis_acan'
VEHICLE_COMMAND_CHANNEL = '/canbus/vehicle_command'
#control channel
CONTROL_DEBUG_CHANNEL = '/control/control_debug'
#localization channel
LOCALIZATION_CHANNEL = '/localization/odomstate_info'
#sdmap channel
SD_MAP_CHANNEL = '/ehr/sdmap'


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

def read_planning_msg(msg_json):
    if not isinstance(msg_json, dict):
        raise ValueError("Input must be a dictionary representing JSON data.")
    data = {}
    data['traj'] = []
    try:
        if 'trajPointArray' not in msg_json or not isinstance(msg_json['trajPointArray'], list):
            raise ValueError(
                "The input data must contain a list named 'trajPointArray'.")
        for traj_point in msg_json['trajPointArray']:
            if not isinstance(traj_point, dict):
                raise ValueError(
                    "Each element in 'trajPointArray' must be a dictionary.")
            tmp_point = {}
            tmp_point['theta'] = traj_point.get('theta')
            tmp_point['timestampNs'] = traj_point.get('timestampNs')
            tmp_point['velocity'] = traj_point.get('velocity')
            direction = traj_point.get('direction', {})
            tmp_point['x_direction'] = direction.get(
                'x') if isinstance(direction, dict) else None
            tmp_point['y_direction'] = direction.get(
                'y') if isinstance(direction, dict) else None
            tmp_point['z_direction'] = direction.get(
                'z') if isinstance(direction, dict) else None
            tmp_point['curv'] = traj_point.get('curvature')
            tmp_point['yawRate'] = traj_point.get('yawRate')
            position = traj_point.get('position', {})
            tmp_point['x'] = position.get(
                'x') if isinstance(position, dict) else None
            tmp_point['y'] = position.get(
                'y') if isinstance(position, dict) else None
            tmp_point['z'] = position.get(
                'z') if isinstance(position, dict) else None
            tmp_point['timeDifference'] = traj_point.get('timeDifference')
            tmp_point['sumDistance'] = traj_point.get('sumDistance')
            tmp_point['acceleration'] = traj_point.get('acceleration')
            data['traj'].append(tmp_point)
        # 0:none; 1:park; 2:reverse; 3:neutral; 4:drive; 5:low
        data['gearEnum'] = msg_json.get('gearEnum', None)
        # 0:no request; 1:both; 2:lat only; 3:long only; 4:error
        data['trajState'] = msg_json.get('trajState', None)
        data['trajectoryMode'] = msg_json.get('trajectoryMode', None)
        # 0:none; 1:left; 2:right
        data['turnSignalEnum'] = msg_json.get('turnSignalEnum', None)
        data['updatedByVehicleStatus'] = 1 if msg_json.get(
            'updatedByVehicleStatus', False) else 0
        data['trajTimestampNs'] = msg_json.get('trajTimestampNs', None)
    except Exception as e:
        print("An error occurred while processing the input data: {}".format(e))
    # pprint(data)
    return data

def read_xdebug_msg(msg_json) -> dict:

    json_msg = json.loads(msg_json['json'])
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
    # data['traj_x'] = json_msg['planner_debug']['lat_debug']['traj_x']
    # data['traj_y'] = json_msg['planner_debug']['lat_debug']['traj_y']
    data['traj_theta'] = get_lat_debug('traj_theta')
    data['traj_s'] = get_lat_debug('traj_s')
    data['traj_d'] = get_lat_debug('traj_d')
    data['traj_ref_d'] = get_lat_debug('traj_ref_d')
    # data['traj_r_upper_x'] = json_msg['planner_debug']['lat_debug']['traj_r_upper_x']
    # data['traj_r_upper_y'] = json_msg['planner_debug']['lat_debug']['traj_r_upper_y']
    # data['traj_r_lower_x'] = json_msg['planner_debug']['lat_debug']['traj_r_lower_x']
    # data['traj_r_lower_y'] = json_msg['planner_debug']['lat_debug']['traj_r_lower_y']
    # data['traj_r_ref_x'] = json_msg['planner_debug']['lat_debug']['traj_r_ref_x']
    # data['traj_r_ref_y'] = json_msg['planner_debug']['lat_debug']['traj_r_ref_y']
    data['traj_theta_error_ref'] = get_lat_debug('traj_theta_error_ref')
    data['traj_delta_ref'] = get_lat_debug('traj_delta_ref')
    data['traj_delta'] = get_lat_debug('traj_delta')
    # data['traj_curve_spline_ref'] = json_msg['planner_debug']['lat_debug']['traj_curve_spline_ref']



    # data['planning_origin'] = json_msg['planner_debug']['planning_origin_state_debug']

    # if 'lc_lon_search_debug' in json_msg['planner_debug']:
    #     data['lc_lon_search_debug'] = json_msg['planner_debug']['lc_lon_search_debug']
    # else:
    #     data['lc_lon_search_debug'] = {}
    # if 'lc_lat_search_debug' in json_msg['planner_debug']:
    #     data['lc_lat_search_debug'] = json_msg['planner_debug']['lc_lat_search_debug']
    # else:
    #     data['lc_lat_search_debug'] = {}

    # if 'lon_search_debug' in json_msg:
    #     data['lon_search_debug'] = json_msg['lon_search_debug']
    # else:
    #     data['lon_search_debug'] = {}
    # if 'lat_search_debug' in json_msg:
    #     data['lat_search_debug'] = json_msg['lat_search_debug']
    # else:
    #     data['lat_search_debug'] = {}

    # if 'lc_decider_lon_search_debug' in json_msg:
    #     data['lc_decider_lon_search_debug'] = json_msg['lc_decider_lon_search_debug']
    # else:
    #     data['lc_decider_lon_search_debug'] = {}
    # if 'lc_decider_lat_search_debug' in json_msg:
    #     data['lc_decider_lat_search_debug'] = json_msg['lc_decider_lat_search_debug']
    # else:
    #     data['lc_decider_lat_search_debug'] = {}


    # read lat_lon_decider debug info
    data['lat_lon_decider_debug'] = get_lat_lon_decider_debug()

    return data

def read_vehicle_report_msg(msg_json) -> dict:
    data = {}
    data['angleCommand'] = msg_json['steering']['angleCommand']
    data['accCommand'] = msg_json['brake']['decelCommand']
    data['torqueCommand'] = msg_json['brake']['torqueCommand']
    data['delta']=msg_json['steering']['angleActual']/12.88
    return data

def read_localization_msg(msg_json) -> dict:
    data = {}
    data['pos_x'] = msg_json['positionFlu']['x']
    data['pos_y'] = msg_json['positionFlu']['y']
    data['theta'] = msg_json['yaw']
    data['v'] = msg_json['linearVelocity']['x']
    return data

def read_sdmap_msg(msg_json) -> dict:
    data = {}
    geometry_list=[]
    for link in msg_json['links']:
        geometry_list.append(link['geometry']) 
    data['geometry']=geometry_list
    
    data['NavCoordinate']=msg_json['navPathInfo']['coordinates']
    return data

def read_road_geometry_msg(msg_json) -> dict:
    data = {}
    freespace_list=[]
    for freespace in msg_json['roadGeometry']['freespaceResults']:
        freespace_list.append(freespace['worldPoints'])
    data['freespace_list']=freespace_list
    return data

msg_callback_map = {
    PLANNING_CHANNEL: read_planning_msg,

    # PERCEPTION_CHANNEL: read_perception_msg,
    # PREDICTION_CHANNEL: read_prediction_msg,

    XDEBUG_CHANNEL: read_xdebug_msg,
    VEHICLE_REPORT_CHANNEL: read_vehicle_report_msg,
    LOCALIZATION_CHANNEL:read_localization_msg,
    SD_MAP_CHANNEL:read_sdmap_msg,
    PERCEPTION_ROAD_GEOMETRY_CHANNEL:read_road_geometry_msg
    # CHASSIS_A_CHANNEL: read_vehicle_a_channel_msg,
    # VEHICLE_COMMAND_CHANNEL: read_vehicle_command_msg,
    # CONTROL_DEBUG_CHANNEL: read_control_debug_msg
}
