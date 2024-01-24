import json

#decision channel
DECISION_DEBUG_CHANNEL = '/decision_planning/decision_debug'
DECISION_TARGET_CHANNEL = '/decision_planning/decision_target'
#planning channel
PLANNING_CHANNEL = '/nop/planning/trajectory'
PLANNING_DEBUG_CHANNEL = '/decision_planning/planning_debug'
XDEBUG_CHANNEL = '/xpilot/xdebug'
#perception channel
PERCEPTION_CHANNEL = '/perception/fusion/object'
#prediction channel
PREDICTION_CHANNEL = '/prediction/objects'
#vehicle state channel
VEHICLE_REPORT_CHANNEL = '/canbus/vehicle_report'
CHASSIS_A_CHANNEL = '/canbus/chassis_acan'
VEHICLE_COMMAND_CHANNEL = '/canbus/vehicle_command'
#control channel
CONTROL_DEBUG_CHANNEL = '/control/control_debug'


class ObjectType:
    UNKNOWN = 1
    PEDESTRIAN = 2
    VEHICLE = 14
    BIKE = 19


def read_perception_msg(msg_json) -> dict:
    data = {}
    data['obstacles'] = []
    if 'perceptionObjectList' not in msg_json.keys():
        return data
    for obstacle in msg_json['perceptionObjectList']:
        tmp_obj = {}
        tmp_obj['sensor_id'] = obstacle['sensorId']
        # 0 UNKNOWN;  1 PEDESTRIAN; 2-13 VEHICLE; 14-19 BIKE;
        if obstacle['label'] < 1:
            tmp_obj['type'] = ObjectType.UNKNOWN
        elif obstacle['label'] < 2:
            tmp_obj['type'] = ObjectType.PEDESTRIAN
        elif obstacle['label'] < 14:
            tmp_obj['type'] = ObjectType.VEHICLE
        else:
            tmp_obj['type'] = ObjectType.BIKE
        tmp_obj['id'] = obstacle['trackId']
        tmp_obj['polygon'] = []
        for point in obstacle['polygonBox']['polygonContour']:
            tmp_obj['polygon'].append({'x': point['x'], 'y': point['y']})
        tmp_obj['center'] = {'x': obstacle['motionInfo']['center']
                             ['x'], 'y': obstacle['motionInfo']['center']['y']}
        data['obstacles'].append(tmp_obj)
    return data


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
    # print(data)
    return data


def read_decision_debug_msg(msg_json) -> dict:
    data = {}
    data['dm_refline'] = []
    for refline in msg_json['reflines']:
        tmp_refline = {}
        tmp_refline['path_pts'] = []
        for pt in refline['pathPts']:
            tmp_point = {}
            tmp_point['x'] = pt['x']
            tmp_point['y'] = pt['y']
            tmp_refline['path_pts'].append(tmp_point)
    return data


def read_decision_target_msg(msg_json) -> dict:
    data = {}
    data['dm_current_refline'] = {}
    data['dm_current_refline']['path_pts'] = []
    for pt in msg_json['currentReflineEnu']:
        tmp_point = {}
        tmp_point['x'] = pt['x']
        tmp_point['y'] = pt['y']
        data['dm_current_refline']['path_pts'].append(tmp_point)
    data['dm_target_refline'] = {}
    data['dm_target_refline']['path_pts'] = []
    for pt in msg_json['targetReflineEnu']:
        tmp_point = {}
        tmp_point['x'] = pt['x']
        tmp_point['y'] = pt['y']
        data['dm_target_refline']['path_pts'].append(tmp_point)
    return data


def read_planning_debug_msg(msg_json) -> dict:
    data = {}
    data['lateral_lower_bound'] = {}
    data['lateral_upper_bound'] = {}
    data['lateral_r_ref'] = {}
    data['lateral_search_result'] = {}
    data['pp_refline'] = {}
    for line in msg_json['lines']:
        tmp_path_pts = []
        for pt in line['points']:
            tmp_point = {}
            tmp_point['x'] = pt['x']
            tmp_point['y'] = pt['y']
            tmp_path_pts.append(tmp_point)

        if line['type'] == 'lateral_lower_bound':
            data['lateral_lower_bound']['path_pts'] = tmp_path_pts
        elif line['type'] == 'lateral_upper_bound':
            data['lateral_upper_bound']['path_pts'] = tmp_path_pts
        elif line['type'] == 'lateral_r_ref':
            data['lateral_r_ref']['path_pts'] = tmp_path_pts
        elif line['type'] == 'lateral_search_result':
            data['lateral_search_result']['path_pts'] = tmp_path_pts
        elif line['type'] == 'refline':
            data['pp_refline']['path_pts'] = tmp_path_pts
    return data


def read_xdebug_msg(msg_json) -> dict:

    json_msg = json.loads(msg_json['json'])
    data = {}
    if 'common' in json_msg:
        data['veh_position_x'] = json_msg['common']['veh_position']['x']
        data['veh_position_y'] = json_msg['common']['veh_position']['y']
        common_debug = json_msg.get('common', {})
        keys_to_check = ['traj_point_x', 'traj_point_y']
        for key in keys_to_check:
            if key in common_debug:
                data[key] = common_debug[key]
            else:
                data[key] = []
    if 'lon_debug' in json_msg:
        data['leader_s'] = json_msg['lon_debug']['leader_s']
        data['leader_v'] = json_msg['lon_debug']['leader_v']
        data['optimizer_v_out'] = json_msg['lon_debug']['optimizer_v_out']
        data['optimizer_v_ref'] = json_msg['lon_debug']['optimizer_v_ref']
        data['optimizer_v_upper'] = json_msg['lon_debug']['optimizer_v_upper']
        data['optimizer_s_out'] = json_msg['lon_debug']['optimizer_s_out']
        data['optimizer_s_ref'] = json_msg['lon_debug']['optimizer_s_ref']
        data['optimizer_s_upper'] = json_msg['lon_debug']['optimizer_s_upper']
        lon_debug = json_msg.get('lon_debug', {})
        keys_to_check = ['optimizer_a_out', 'optimizer_a_ref', 'optimizer_a_upper',
                         'optimizer_a_lower', 'optimizer_jerk_out', 'optimizer_jerk_upper',
                         'optimizer_jerk_lower', 's_leader', 'v_leader', 's_ref_follow', 'v_ref_follow',
                         'curve_speed_limit', 'curve_speed_limit_s']
        for key in keys_to_check:
            if key in lon_debug:
                data[key] = lon_debug[key]
            else:
                data[key] = []
    if 'planner_debug' in json_msg:
        data['lane_debug'] = json_msg['planner_debug']['world_model_debug']['lane_debug']
        data['lane_debug_enu'] = json_msg['planner_debug']['world_model_debug']['lane_debug_enu']
        data['obstacle_debug'] = json_msg['planner_debug']['world_model_debug']['obstacle_debug']
        data['ego_info_debug'] = json_msg['planner_debug']['world_model_debug']['ego_info_debug']
        data['svlimit_s'] = json_msg['planner_debug']['longi_debug']['svlimit_s']
        data['svlimit_v_upper'] = json_msg['planner_debug']['longi_debug']['svlimit_v_upper']
        data['svlimit_v_lower'] = json_msg['planner_debug']['longi_debug']['svlimit_v_lower']
        data['svlimit_ref_sv_v'] = json_msg['planner_debug']['longi_debug']['svlimit_ref_sv_v']
        data['svlimit_ref_st_t'] = json_msg['planner_debug']['longi_debug']['svlimit_ref_st_t']
        data['svlimit_ref_st_s'] = json_msg['planner_debug']['longi_debug']['svlimit_ref_st_s']
        data['svlimit_ref_vt_v'] = json_msg['planner_debug']['longi_debug']['svlimit_ref_vt_v']
        data['svlimit_ref_at_a'] = json_msg['planner_debug']['longi_debug']['svlimit_ref_at_a']
        if 'svlimit_v_by_curvature' not in json_msg['planner_debug']['longi_debug']:
            data['svlimit_v_by_curvature'] = []
        else:
            data['svlimit_v_by_curvature'] = json_msg['planner_debug']['longi_debug']['svlimit_v_by_curvature']
        if 'traj_x' not in json_msg['planner_debug']['lat_debug'] or \
            'traj_y' not in json_msg['planner_debug']['lat_debug'] or \
            'traj_theta' not in json_msg['planner_debug']['lat_debug'] or \
            'traj_s' not in json_msg['planner_debug']['lat_debug'] or \
            'traj_d' not in json_msg['planner_debug']['lat_debug'] or \
            'traj_ref_d' not in json_msg['planner_debug']['lat_debug'] or \
            'traj_r_upper_x' not in json_msg['planner_debug']['lat_debug'] or \
            'traj_r_upper_y' not in json_msg['planner_debug']['lat_debug'] or \
            'traj_r_lower_x' not in json_msg['planner_debug']['lat_debug'] or \
            'traj_r_lower_y' not in json_msg['planner_debug']['lat_debug'] or \
            'traj_r_ref_x' not in json_msg['planner_debug']['lat_debug'] or \
            'traj_r_ref_y' not in json_msg['planner_debug']['lat_debug'] or \
            'traj_theta_error_ref' not in json_msg['planner_debug']['lat_debug'] or \
            'traj_delta_ref' not in json_msg['planner_debug']['lat_debug'] or \
                'traj_delta' not in json_msg['planner_debug']['lat_debug'] or \
            'path_bound_low_x' not in json_msg['planner_debug']['lat_debug'] or \
                'path_bound_low_y' not in json_msg['planner_debug']['lat_debug'] or \
                'path_bound_high_x' not in json_msg['planner_debug']['lat_debug'] or \
                'path_bound_high_y' not in json_msg['planner_debug']['lat_debug'] or \
                'static_obs_dodge_id' not in json_msg['planner_debug']['lat_debug'] or \
                'obstacle_debug' not in json_msg['planner_debug']['lat_debug']:

            # data['traj_x'] = []
            # data['traj_y'] = []
            # data['traj_theta'] = []
            # data['traj_s'] = []
            # data['traj_d'] = []
            # data['traj_ref_d'] = []
            # data['traj_r_upper_x'] = []
            # data['traj_r_upper_y'] = []
            # data['traj_r_lower_x'] = []
            # data['traj_r_lower_y'] = []
            # data['traj_r_ref_x'] = []
            # data['traj_r_ref_y'] = []
            # data['traj_theta_error_ref'] = []
            # data['traj_delta_ref'] = []
            # data['traj_delta'] = []
            # data['traj_curve_spline_ref'] = []
            data['path_bound_low_x'] = []
            data['path_bound_low_y'] = []
            data['path_bound_high_x'] = []
            data['path_bound_high_y'] = []
            data['static_obs_dodge_id'] = []
            data['nudge_obstacle_debug'] = []
        # else:
        data['traj_x'] = json_msg['planner_debug']['lat_debug']['traj_x']
        data['traj_y'] = json_msg['planner_debug']['lat_debug']['traj_y']
        data['traj_theta'] = json_msg['planner_debug']['lat_debug']['traj_theta']
        data['traj_s'] = json_msg['planner_debug']['lat_debug']['traj_s']
        data['traj_d'] = json_msg['planner_debug']['lat_debug']['traj_d']
        data['traj_ref_d'] = json_msg['planner_debug']['lat_debug']['traj_ref_d']
        data['traj_r_upper_x'] = json_msg['planner_debug']['lat_debug']['traj_r_upper_x']
        data['traj_r_upper_y'] = json_msg['planner_debug']['lat_debug']['traj_r_upper_y']
        data['traj_r_lower_x'] = json_msg['planner_debug']['lat_debug']['traj_r_lower_x']
        data['traj_r_lower_y'] = json_msg['planner_debug']['lat_debug']['traj_r_lower_y']
        data['traj_r_ref_x'] = json_msg['planner_debug']['lat_debug']['traj_r_ref_x']
        data['traj_r_ref_y'] = json_msg['planner_debug']['lat_debug']['traj_r_ref_y']
        data['traj_theta_error_ref'] = json_msg['planner_debug']['lat_debug']['traj_theta_error_ref']
        data['traj_delta_ref'] = json_msg['planner_debug']['lat_debug']['traj_delta_ref']
        data['traj_delta'] = json_msg['planner_debug']['lat_debug']['traj_delta']
        data['traj_curve_spline_ref'] = json_msg['planner_debug']['lat_debug']['traj_curve_spline_ref']

        # data['path_bound_low_x'] = json_msg['planner_debug']['lat_debug']['path_bound_low_x']
        # data['path_bound_low_y'] = json_msg['planner_debug']['lat_debug']['path_bound_low_y']
        # data['path_bound_high_x'] = json_msg['planner_debug']['lat_debug']['path_bound_high_x']
        # data['path_bound_high_y'] = json_msg['planner_debug']['lat_debug']['path_bound_high_y']
        # data['static_obs_dodge_id'] = json_msg['planner_debug']['lat_debug']['static_obs_dodge_id']
        # data['nudge_obstacle_debug'] = json_msg['planner_debug']['lat_debug']['obstacle_debug']

        data['nop_counter'] = json_msg['planner_debug']['nop_counter']
        data['optimizer_v_ref'] = json_msg['planner_debug']['longi_debug']['optimizer_v_ref']
        data['optimizer_v_upper'] = json_msg['planner_debug']['longi_debug']['optimizer_v_upper']
        data['optimizer_s_ref'] = json_msg['planner_debug']['longi_debug']['optimizer_s_ref']
        data['optimizer_s_upper'] = json_msg['planner_debug']['longi_debug']['optimizer_s_upper']
        data['optimizer_a_ref'] = json_msg['planner_debug']['longi_debug']['optimizer_a_ref']
        data['optimizer_a_lower'] = json_msg['planner_debug']['longi_debug']['optimizer_a_lower']
        data['optimizer_a_upper'] = json_msg['planner_debug']['longi_debug']['optimizer_a_upper']
        data['optimizer_jerk_upper'] = json_msg['planner_debug']['longi_debug']['optimizer_jerk_upper']
        data['optimizer_jerk_lower'] = json_msg['planner_debug']['longi_debug']['optimizer_jerk_lower']
        data['s_leader'] = json_msg['planner_debug']['longi_debug']['s_leader']
        data['v_leader'] = json_msg['planner_debug']['longi_debug']['v_leader']
        data['s_ref_follow'] = json_msg['planner_debug']['longi_debug']['s_ref_follow']
        data['v_ref_follow'] = json_msg['planner_debug']['longi_debug']['v_ref_follow']
        data['a_ref_follow'] = json_msg['planner_debug']['longi_debug']['a_ref_follow']
        data['optimizer_jerk_out'] = json_msg['planner_debug']['longi_debug']['optimizer_jerk_out']
        data['optimizer_a_out'] = json_msg['planner_debug']['longi_debug']['optimizer_a_out']
        data['optimizer_v_out'] = json_msg['planner_debug']['longi_debug']['optimizer_v_out']
        data['optimizer_s_out'] = json_msg['planner_debug']['longi_debug']['optimizer_s_out']
        data['planning_origin'] = json_msg['planner_debug']['planning_origin_state_debug']

        if 'lc_lon_search_debug' in json_msg['planner_debug']:
            data['lc_lon_search_debug'] = json_msg['planner_debug']['lc_lon_search_debug']
        else:
            data['lc_lon_search_debug'] = {}
        if 'lc_lat_search_debug' in json_msg['planner_debug']:
            data['lc_lat_search_debug'] = json_msg['planner_debug']['lc_lat_search_debug']
        else:
            data['lc_lat_search_debug'] = {}
    else:
        data['lane_debug'] = []
        data['lane_debug_enu'] = []
        data['ego_info_debug'] = {}
        data['lc_lon_search_debug'] = {}

    if 'lon_search_debug' in json_msg:
        data['lon_search_debug'] = json_msg['lon_search_debug']
    else:
        data['lon_search_debug'] = {}
    if 'lat_search_debug' in json_msg:
        data['lat_search_debug'] = json_msg['lat_search_debug']
    else:
        data['lat_search_debug'] = {}

    if 'lc_decider_lon_search_debug' in json_msg:
        data['lc_decider_lon_search_debug'] = json_msg['lc_decider_lon_search_debug']
    else:
        data['lc_decider_lon_search_debug'] = {}
    if 'lc_decider_lat_search_debug' in json_msg:
        data['lc_decider_lat_search_debug'] = json_msg['lc_decider_lat_search_debug']
    else:
        data['lc_decider_lat_search_debug'] = {}
    if 'report_data' in json_msg:
        data['ego_vel'] = json_msg['report_data']['ego_vel']
        data['set_speed'] = json_msg['report_data']['set_speed']
        # data['ego_acc'] = json_msg['report_data']['ego_acc']
        data['planning_acc'] = json_msg['report_data']['planning_acc']
        data['follow_dis_real'] = json_msg['report_data']['follow_dis_real']
        data['follow_dis_desire'] = json_msg['report_data']['follow_dis_desire']
        data['road_curva'] = json_msg['report_data']['road_curva']
        data['ego_steer_angle'] = json_msg['report_data']['ego_steer_angle']
        data['ego_heading'] = json_msg['report_data']['ego_heading']
        data['lateral_offset_error'] = json_msg['report_data']['lateral_offset_error']
        data['lateral_heading_error'] = json_msg['report_data']['lateral_heading_error']
        data['right_lane_x'] = json_msg['report_data']['left_nearest_lane_point']['x']
        data['right_lane_y'] = json_msg['report_data']['left_nearest_lane_point']['y']
        data['left_lane_x'] = json_msg['report_data']['right_nearest_lane_point']['x']
        data['left_lane_y'] = json_msg['report_data']['right_nearest_lane_point']['y']
        data['ego_position_x'] = json_msg['report_data']['ego_position']['x']
        data['ego_position_y'] = json_msg['report_data']['ego_position']['y']
        data['has_leader'] = json_msg['report_data']['has_leader']
        data['relative_speed'] = json_msg['report_data']['relative_speed']
        data['ego_yaw_rate'] = json_msg['report_data']['ego_yaw_rate']
    else:
        data['ego_vel'] = []
        data['set_speed'] = []
    return data


def read_prediction_msg(msg_json) -> dict:
    data = {}
    data['prediction_traj'] = []
    for PredictionObject in msg_json['objectList']:
        tmp_obj = {}
        tmp_obj['id'] = PredictionObject['id']
        tmp_obj['lengthM'] = PredictionObject['lengthM']
        tmp_obj['widthM'] = PredictionObject['widthM']
        tmp_obj['x'] = PredictionObject['position']['x']
        tmp_obj['y'] = PredictionObject['position']['y']
        tmp_obj['polygon'] = []
        for point in PredictionObject['polygonContour']:
            tmp_obj['polygon'].append({'x': point['x'], 'y': point['y']})
        tmp_obj['traj'] = []
        if PredictionObject['trajectoryArray'] and 'trajectoryPointArray' in PredictionObject['trajectoryArray'][0]:
            for traj_point in PredictionObject['trajectoryArray'][0]['trajectoryPointArray']:
                tmp_point = {}
                tmp_point['x'] = traj_point['position']['x']
                tmp_point['y'] = traj_point['position']['y']
                tmp_obj['traj'].append(tmp_point)
        data['prediction_traj'].append(tmp_obj)
    return data


def read_vehicle_report_msg(msg_json) -> dict:
    data = {}
    data['vehicleMps'] = []
    data['accelX'] = []
    data['vehicleMps'] = msg_json['chassis']['vehicleMps']
    data['accelX'] = msg_json['imu']['accelX']
    # print(data)
    return data


def read_vehicle_a_channel_msg(msg_json) -> dict:
    data = {}
    data['steering_torque_act'] = msg_json['eps2']['epsLkaCtrlDlvdValue']
    data['ego_acc'] = msg_json['bcs11A']['bcsActVehLongAccel']
    # print(data)
    return data


def read_vehicle_command_msg(msg_json) -> dict:
    data = {}
    data['steering_torque_command'] = msg_json['steering']['command']
    return data


def read_control_debug_msg(msg_json) -> dict:
    data = {}
    data['vehicleAutoEnable'] = []
    data['accCmd'] = msg_json['accCmd']
    data['velCmd'] = msg_json['velCmd']
    # print(data)
    return data


msg_callback_map = {
    PLANNING_CHANNEL: read_planning_msg,
    PLANNING_DEBUG_CHANNEL: read_planning_debug_msg,

    PERCEPTION_CHANNEL: read_perception_msg,
    PREDICTION_CHANNEL: read_prediction_msg,

    DECISION_DEBUG_CHANNEL: read_decision_debug_msg,
    DECISION_TARGET_CHANNEL: read_decision_target_msg,

    XDEBUG_CHANNEL: read_xdebug_msg,

    VEHICLE_REPORT_CHANNEL: read_vehicle_report_msg,
    CHASSIS_A_CHANNEL: read_vehicle_a_channel_msg,
    VEHICLE_COMMAND_CHANNEL: read_vehicle_command_msg,
    CONTROL_DEBUG_CHANNEL: read_control_debug_msg
}
