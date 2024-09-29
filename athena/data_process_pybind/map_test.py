import sys
sys.path.append('/home/SENSETIME/zhangjian3/ws/E2E/common_math/athena/build')
import pybind_dlp
# print(dir(pybind_dlp))
# print(dir(pybind_dlp.MapToTensor))
# print(dir(pybind_dlp.MapInfo))
import argparse
import json

lane_type_map = {
    'VEHICLE': pybind_dlp.LaneType.VEHICLE_LANE,
    'BIKE': pybind_dlp.LaneType.BIKE_LANE,
    'PEDESTRIAN': pybind_dlp.LaneType.PEDESTRIAN_LANE,
    'BUS': pybind_dlp.LaneType.BUS_LANE,
    'STOP_LANE': pybind_dlp.LaneType.STOP_LANE,
}

traffic_light_status_map = {
    'UNKNOWNSTATUS': pybind_dlp.TrafficLightStatus.UNKNOWNSTATUS,
    'INVALID': pybind_dlp.TrafficLightStatus.INVALID,
    'OFF': pybind_dlp.TrafficLightStatus.OFF,
    'GREEN': pybind_dlp.TrafficLightStatus.GREEN,
    'YELLOW': pybind_dlp.TrafficLightStatus.YELLOW,
    'RED': pybind_dlp.TrafficLightStatus.RED,
    'GREENFLASH': pybind_dlp.TrafficLightStatus.GREENFLASH,
    'YELLOWFLASH': pybind_dlp.TrafficLightStatus.YELLOWFLASH,
}

lane_mark_type_map = {
    'DASH_SOLID_YELLOW': pybind_dlp.LaneMarkType.DASH_SOLID_YELLOW,
    'DASH_SOLID_WHITE': pybind_dlp.LaneMarkType.DASH_SOLID_WHITE,
    'DASHED_WHITE': pybind_dlp.LaneMarkType.DASHED_WHITE,
    'DASHED_YELLOW': pybind_dlp.LaneMarkType.DASHED_YELLOW,
    'DOUBLE_SOLID_YELLOW': pybind_dlp.LaneMarkType.DOUBLE_SOLID_YELLOW,
    'DOUBLE_SOLID_WHITE': pybind_dlp.LaneMarkType.DOUBLE_SOLID_WHITE,
    'DOUBLE_DASH_YELLOW': pybind_dlp.LaneMarkType.DOUBLE_DASH_YELLOW,
    'DOUBLE_DASH_WHITE': pybind_dlp.LaneMarkType.DOUBLE_DASH_WHITE,
    'SOLID_YELLOW': pybind_dlp.LaneMarkType.SOLID_YELLOW,
    'SOLID_WHITE': pybind_dlp.LaneMarkType.SOLID_WHITE,
    'SOLID_DASH_WHITE': pybind_dlp.LaneMarkType.SOLID_DASH_WHITE,
    'SOLID_DASH_YELLOW': pybind_dlp.LaneMarkType.SOLID_DASH_YELLOW,
    'SOLID_BLUE': pybind_dlp.LaneMarkType.SOLID_BLUE,
    'NONE': pybind_dlp.LaneMarkType.NONE,
    'UNKNOWN': pybind_dlp.LaneMarkType.UNKNOWN,
    'CROSSWALK': pybind_dlp.LaneMarkType.CROSSWALK,
    'CENTERLINE': pybind_dlp.LaneMarkType.CENTERLINE
}

def load_json_file(file_path):
    """
    读取 .json 文件并返回数据。

    参数:
    file_path (str): .json 文件的路径

    返回:
    data: 从 .json 文件中加载的数据
    """
    with open(file_path, 'r', encoding='utf-8') as file:
        data = json.load(file)
    return data

def call_pybind_dlp(data):
    # map_info_manager = pybind_dlp.MapInfoManager
    map_info = pybind_dlp.MapInfo()
    drivable_out = []
    for key, value in data['drivable_areas'].items():
        # 创建 DrivableAreas 实例
        drivable_area = pybind_dlp.DrivableAreas()
        drivable_area.id = int(key)
        tmp = []
        for point in value['area_boundary']:
            point3d = pybind_dlp.Point3D()
            point3d.x = point['x']
            point3d.y = point['y']
            point3d.z = point['z']
            # 打印调试信息
            # print(f"Adding Point3D: ({point3d.x}, {point3d.y}, {point3d.z})")
            tmp.append(point3d)
            # print(f"length: {len(tmp)}")
            drivable_area.area_boundary_points = tmp
        drivable_out.append(drivable_area)
    map_info.drivable_areas = drivable_out
    
    lane_out = []
    for key, value in data['lane_segments'].items():
        lane_seg = pybind_dlp.LaneSegments()
        lane_seg.id = int(key)
        lane_seg.is_intersection = value['is_intersection']
        lane_seg.lane_type = lane_type_map.get(value['lane_type'], pybind_dlp.LaneType.VEHICLE_LANE)
        lane_seg.traffic_light_status = traffic_light_status_map.get(value['traffic_light_status'],
                                                                     pybind_dlp.TrafficLightStatus.UNKNOWNSTATUS)
        lane_seg.left_lane_mark_type = lane_mark_type_map.get(value['left_lane_mark_type'], 
                                                              pybind_dlp.LaneMarkType.UNKNOWN)
        lane_seg.left_neighbor_id = value['left_neighbor_id'] if value['left_neighbor_id'] is not None else -1

        lane_seg.right_lane_mark_type = lane_mark_type_map.get(value['right_lane_mark_type'], 
                                                               pybind_dlp.LaneMarkType.UNKNOWN)
        lane_seg.right_neighbor_id = value['right_neighbor_id'] if value['right_neighbor_id'] is not None else -1
        centerline = []
        for point in value['centerline']:
            point3d = pybind_dlp.Point3D()
            point3d.x = point['x']
            point3d.y = point['y']
            point3d.z = point['z']
            centerline.append(point3d)
        lane_seg.centerline = centerline
        left_bound_points = []
        for point in value['left_lane_boundary']:
            point3d = pybind_dlp.Point3D()
            point3d.x = point['x']
            point3d.y = point['y']
            point3d.z = point['z']
            left_bound_points.append(point3d)
        lane_seg.left_lane_boundary = left_bound_points
        # for list in value['left_lane_boundary']:
        #     for point in list:
        #         point3d = pybind_dlp.Point3D()
        #         point3d.x = point['x']
        #         point3d.y = point['y']
        #         point3d.z = point['z']
        #         left_bound_points.append(point3d)
        # lane_seg.left_lane_boundary = left_bound_points
        right_bound_points = []
        for point in value['right_lane_boundary']:
            point3d = pybind_dlp.Point3D()
            point3d.x = point['x']
            point3d.y = point['y']
            point3d.z = point['z']
            right_bound_points.append(point3d)
        lane_seg.right_lane_boundary = right_bound_points
        # for list in value['right_lane_boundary']:
        #     for point in list:
        #         point3d = pybind_dlp.Point3D()
        #         point3d.x = point['x']
        #         point3d.y = point['y']
        #         point3d.z = point['z']
        #         right_bound_points.append(point3d)
        # lane_seg.right_lane_boundary = right_bound_points
        prede = []
        for id in value['predecessors']:
            prede.append(id)
        lane_seg.predecessors = prede
        succe = []
        for id in value['successors']:
            succe.append(id)
        lane_seg.successors = succe
        lane_out.append(lane_seg)
    map_info.lane_segments = lane_out
        
    cross_out = []
    for key, value in data['pedestrian_crossings'].items():
        pedecrossing = pybind_dlp.PedestrianCrossings()
        pedecrossing.id = int(key)
        edge1 = []
        for pt in value['edge1']:
            point3d = pybind_dlp.Point3D()
            point3d.x = pt['x']
            point3d.y = pt['y']
            point3d.z = pt['z']
            edge1.append(point3d)
        pedecrossing.edge1 = edge1
        edge2 = []
        for pt in value['edge2']:
            point3d = pybind_dlp.Point3D()
            point3d.x = pt['x']
            point3d.y = pt['y']
            point3d.z = pt['z']
            edge2.append(point3d)
        pedecrossing.edge2 = edge2
        cross_out.append(pedecrossing)
    map_info.pedestrain_crossings = cross_out

    stop_out = []
    for key, value in data['stop_lines'].items():
        stopline = pybind_dlp.MapApiStopLines()
        stopline.id = int(key)
        points = []
        for pt in value['points']:
            point3d = pybind_dlp.Point3D()
            point3d.x = pt['x']
            point3d.y = pt['y']
            point3d.z = pt['z']
            points.append(point3d)
        stopline.points = points
        stop_out.append(stopline)
    map_info.stop_lines = stop_out

    params = pybind_dlp.MapDataParam()
    params.mode = pybind_dlp.MapInfoMode.M_Train# 确保正确实例化
    map_info_manager = pybind_dlp.MapInfoManager(params)

    map_info_manager.get_map_features(map_info)
    tensor_out = map_info_manager.get_map_tensor_out()
    map_pl_po = tensor_out.get_map_polygon_position().data()
    map_pl_or = tensor_out.get_map_polygon_orientation().data()
    map_pl_type = tensor_out.get_map_polygon_type().data()
    map_pl_isin = tensor_out.get_map_polygon_is_intersection().data()
    
    map_pt_po = tensor_out.get_map_point_position().data()
    map_pt_or = tensor_out.get_map_point_orientation().data()
    map_pt_mag = tensor_out.get_map_point_magnitude().data()
    map_pt_type = tensor_out.get_map_point_type().data()
    map_pt_side = tensor_out.get_map_point_side().data()
    
    return map_pl_po, map_pl_or, map_pl_type, map_pl_isin, \
            map_pt_po, map_pt_or, map_pt_mag, map_pt_type, map_pt_side
    
    

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="parse rsclbag",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "-s", "--input", dest="input", help="input map json file path", required=True
    )
    parser.add_argument(
        "-p", "--input1", dest="input1", help="input pkl file path", required=True
    )

    # test_args = ["-s", "/home/SENSETIME/zhangjian3/ws/E2E/common_math/athena/data_demo/raw/ee412d91202ae7352a09b135f5395a8543fddb41e86fde4d52d42d742f730ac7/log_map_archive_ee412d91202ae7352a09b135f5395a8543fddb41e86fde4d52d42d742f730ac7.json"]
    
    args = parser.parse_args()
    json_data = load_json_file(args.input)
    call_pybind_dlp(json_data)


