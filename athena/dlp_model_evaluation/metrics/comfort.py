import numpy as np
from metrics.metric_base import MetricBase


class ComfortMetric(MetricBase):
    def __init__(self, accel_threshold=3.0, yawrate_threshold=1.5, jerk_threshold=2.0, weights=[0.4, 0.3, 0.3]):
        self.comfort_score = 100.0
        self.accel_threshold = accel_threshold
        self.yawrate_threshold = yawrate_threshold
        self.jerk_threshold = jerk_threshold
        self.weights = weights

    def evaluate(self, trajectory=None, map_data=None, agent_data=None):
        accel_counter = 0
        yawrate_counter = 0
        jerk_counter = 0
        for traj_pt_index, traj_point in enumerate(trajectory):
            if abs(traj_point['accel']) > self.accel_threshold:
                accel_counter += 1
            if abs(traj_point['yawrate']) > self.yawrate_threshold:
                yawrate_counter += 1
            if abs(traj_point['jerk']) > self.jerk_threshold:
                jerk_counter += 1

        accel_score = round((1 - accel_counter / len(trajectory)) * 100, 2)
        yawrate_score = round((1 - yawrate_counter / len(trajectory)) * 100, 2)
        jerk_score = round((1 - jerk_counter / len(trajectory)) * 100, 2)
        self.comfort_score = round(
            np.dot([accel_score, yawrate_score, jerk_score], self.weights), 2)
