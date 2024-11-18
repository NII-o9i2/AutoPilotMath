import os
import sys
import math
from common.type import *
from metrics.utils import *
from metrics.metric_base import MetricBase

current_directory = os.getcwd()
relative_path1 = '../build'

file_path1 = os.path.join(current_directory, relative_path1)
sys.path.append(file_path1)
import pybind_dlp


class DrivableAreaMetric(MetricBase):
    def __init__(self, threshold=0.0):
        self.threshold_ = threshold
        self.drivable_area_score_ = 100.0
        self.away_from_edge_weight = 0.1

    def evaluate(self, ego_origin_state=None, trajectory=None, map_data=None, use_senseauto_flag=True):
        # calculate drivable area score
        if use_senseauto_flag:
            occ_raw_data = map_data['occ_data']
            occ_mgr = pybind_dlp.OccVectorGenerator()
            occ_mgr_raw_info = pybind_dlp.RawOccInfo()
            occ_mgr_raw_info.x_partition.num_partition = occ_raw_data[
                'xPartition_info']['numPartition']
            occ_mgr_raw_info.x_partition.ranges = occ_raw_data['xPartition_info']['ranges']
            occ_mgr_raw_info.x_partition.steps = occ_raw_data['xPartition_info']['steps']
            occ_mgr_raw_info.x_partition.num_voxels_per_range = occ_raw_data[
                'xPartition_info']['numVoxelsPerRange']

            occ_mgr_raw_info.y_partition.num_partition = occ_raw_data[
                'yPartition_info']['numPartition']
            occ_mgr_raw_info.y_partition.ranges = occ_raw_data['yPartition_info']['ranges']
            occ_mgr_raw_info.y_partition.steps = occ_raw_data['yPartition_info']['steps']
            occ_mgr_raw_info.y_partition.num_voxels_per_range = occ_raw_data[
                'yPartition_info']['numVoxelsPerRange']

            x_step = occ_mgr_raw_info.x_partition.steps
            y_step = occ_mgr_raw_info.y_partition.steps
            x_sum = occ_mgr_raw_info.x_partition.num_voxels_per_range[0]
            y_sum = occ_mgr_raw_info.y_partition.num_voxels_per_range[0]
            y_mid = y_sum // 2

            ego_x = ego_origin_state.position.x
            ego_y = ego_origin_state.position.y
            ego_head = ego_origin_state.theta

            occ_mgr_voxels = []
            for occ_raw_feature in occ_raw_data['occ_info']:
                if occ_raw_feature['type'] == 2:
                    voxel = pybind_dlp.OccVoxel()
                    voxel.i = occ_raw_feature['index'][0]
                    voxel.j = occ_raw_feature['index'][1]
                    voxel.v = occ_raw_feature['type']
                    occ_mgr_voxels.append(voxel)
            occ_mgr_raw_info.voxel_list = occ_mgr_voxels
            res = occ_mgr.build_occ_vector(occ_mgr_raw_info)

            # trans free space edges ego coord to local
            free_space_edges = []
            for item in res:
                free_space_edge = []
                for pt in item.free_space_edge:
                    pt_local = transform(
                        ego_x, ego_y, ego_head, pt[0], (pt[1] - y_mid), 0)
                    free_space_edge.append([pt_local[0], pt_local[1]])
                free_space_edges.append(free_space_edge)

            # todo:
            # 1.0 check for collision between ego traj and free space edges
            edges_segments = []
            for edge in free_space_edges:
                segments = points_to_segments(edge)
                edges_segments.append(segments)

            # 2.0 keep ego trajectory away from edges
            sum_dis = 0.0
            for edge in edges_pts:
                ans = find_closest_distance(edge, ego_traj_pose_list)
                sum_dis += ans[0]

            # all score
            if out_of_drivable_flag:
                self.drivable_area_score_ = 0
            else:
                max_distance = 100.0
                self.drivable_area_score_ = round(
                    min((sum_dis / max_distance) * 100, 100), 2)
        else:
            drivable_areas = map_data['drivable_areas']
            edges_pts = []
            for key, value in drivable_areas.items():
                edge = []
                for pt in value['area_boundary']:
                    ego_vec = Point2D(
                        math.cos(ego_origin_state.theta), math.sin(ego_origin_state.theta))
                    query_pt_vec = Point2D((pt['x'] - ego_origin_state.position.x),
                                           (pt['y'] - ego_origin_state.position.y))
                    dot_product = ego_vec.x * query_pt_vec.x + ego_vec.y * query_pt_vec.y
                    if dot_product < 0.0:
                        continue
                    else:
                        edge.append([pt['x'], pt['y']])
                edges_pts.append(edge)

            # generate edges segments and ego traj segments
            edges_segments = []
            ego_traj_pose_list = []
            ego_traj_segments = []

            for item in edges_pts:
                segments = points_to_segments(item)
                edges_segments.append(segments)

            for pt in trajectory:
                ego_traj_pose_list.append([pt['x'], pt['y']])
                ego_traj_segments = points_to_segments(ego_traj_pose_list)

            # 1.0 check for collision between ego traj and free space edges
            calc_num = 0
            out_of_drivable_flag = False
            for edge in edges_segments:
                for seg in edge:
                    for traj_seg in ego_traj_segments:
                        calc_num += 1
                        out_of_drivable_flag = intersects(traj_seg, seg)
                        if out_of_drivable_flag:
                            break

            # 2.0 keep ego trajectory away from edges
            sum_dis = 0.0
            for edge in edges_pts:
                ans = find_closest_distance(edge, ego_traj_pose_list)
                sum_dis += ans[0]

            # all score
            if out_of_drivable_flag:
                self.drivable_area_score_ = 0
            else:
                max_distance = 75.0
                self.drivable_area_score_ = round(
                    min((sum_dis / max_distance) * 100, 100), 2)
