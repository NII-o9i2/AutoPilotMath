import math
from shapely.geometry import Polygon
from metrics.utils import *
from metrics.metric_base import MetricBase


class CollisionMetric(MetricBase):
    def __init__(self, collision_threshold=0.0):
        self.collision_threshold = collision_threshold
        self.collision_score = 100.0

    def evaluate(self, trajectory=None, map_data=None, agent_data=None):
        # calculate collision score
        agents_info = {}
        for row in agent_data.itertuples(index=False, name='Pandas'):
            temp_data = {'veh_position': {'x': row.position_x, 'y': row.position_y},
                         'veh_yaw': row.heading,
                         'type': row.object_type,
                         'track_id': row.track_id}
            if row.timestep in agents_info:
                agents_info[row.timestep].append(temp_data)
            else:
                agents_info[row.timestep] = [temp_data]
        # we only consider agent that exists at timestep 50
        consider_agents = agents_info[50] if 50 in agents_info else None
        consider_track_ids = {agent['track_id'] for agent in consider_agents}

        for traj_pt_index, traj_point in enumerate(trajectory):
            filter_agents_info = []
            timestep = traj_pt_index + 50
            if timestep in agents_info:
                consider_agents_info = [
                    agent for agent in agents_info[timestep]
                    if agent['track_id'] in consider_track_ids
                ]

                # filter agent that is far away from ego car
                for each_agent in consider_agents_info:
                    if math.sqrt(
                            (each_agent['veh_position']['x'] - traj_point['x']) ** 2 + (
                                each_agent['veh_position']['y'] - traj_point['y']) ** 2) > 10.0:
                        continue
                    filter_agents_info.append(each_agent)

                # calculate overlap between ego car and remain agents
                ego_polygon_points = build_polygon_by_pos_and_heading(traj_point['x'], traj_point['y'],
                                                                      traj_point['heading'])
                ego_car_polygon = Polygon(ego_polygon_points)
                for each_agent in filter_agents_info:
                    if each_agent['type'] == 1:
                        # pedestrian
                        agent_polygon_points = build_polygon_by_pos_and_heading(each_agent['veh_position']['x'],
                                                                                each_agent['veh_position']['y'],
                                                                                each_agent['veh_yaw'], 0.5, 0.5)
                        agent_polygon = Polygon(agent_polygon_points)
                    elif each_agent['type'] == 14:
                        # bike
                        agent_polygon_points = build_polygon_by_pos_and_heading(each_agent['veh_position']['x'],
                                                                                each_agent['veh_position']['y'],
                                                                                each_agent['veh_yaw'], 0.5, 1.2)
                        agent_polygon = Polygon(agent_polygon_points)
                    elif each_agent['type'] in [6, 7, 8, 10, 11]:
                        # truck
                        agent_polygon_points = build_polygon_by_pos_and_heading(each_agent['veh_position']['x'],
                                                                                each_agent['veh_position']['y'],
                                                                                each_agent['veh_yaw'], 2.5, 7.0)
                        agent_polygon = Polygon(agent_polygon_points)
                    else:
                        # vehicle
                        agent_polygon_points = build_polygon_by_pos_and_heading(each_agent['veh_position']['x'],
                                                                                each_agent['veh_position']['y'],
                                                                                each_agent['veh_yaw'])
                        agent_polygon = Polygon(agent_polygon_points)
                    if ego_car_polygon.intersects(agent_polygon):
                        self.collision_score = 0.0
                        return
                # determine whether line segments intersect between ego car and remain agents
                current_ego_traj_pt = (
                    trajectory[traj_pt_index]['x'], trajectory[traj_pt_index]['y'])
                if traj_pt_index > 0:
                    last_ego_traj_pt = (trajectory[traj_pt_index-1]['x'], trajectory[traj_pt_index-1]
                                        ['y'])
                    for each_agent in filter_agents_info:
                        current_ego_traj_pt = (
                            each_agent['veh_position']['x'], each_agent['veh_position']['y'])
                        if timestep - 1 in agents_info and each_agent['track_id'] in agents_info[timestep - 1]:
                            last_agent_traj_pt = (agents_info[timestep - 1][each_agent['track_id']]['veh_position']
                                                  ['x'], agents_info[timestep - 1][each_agent['track_id']]['veh_position']['y'])
                            if is_intersect(last_ego_traj_pt,
                                            current_ego_traj_pt, last_agent_traj_pt, current_ego_traj_pt):
                                self.collision_score = 0.0
                                return
