//
// Created by SENSETIME\fengxiaotong on 23-12-11.
//
#include "obstacle.h"
#include "iostream"
#include "spline.h"

namespace EnvSim {
void Obstacle::construct(const LaneManager &lane_manager) {
  // 1. update trajectory
  std::vector<MathUtils::Point2D> raw_cartesian_points;
  raw_cartesian_points.clear();

  for (auto &pt : raw_trajectory_points_) {
    auto &goal_lane = lane_manager.get_lane(pt.lane_id);
    MathUtils::Point2D cart_point =
        goal_lane.frenet_to_cartesian(MathUtils::Point2D(pt.s, pt.l));
    raw_cartesian_points.emplace_back(cart_point);
  }
  std::vector<MathUtils::Point2D> smooth_cartesian_points;
  interpolate_points(raw_cartesian_points, smooth_cartesian_points, 50);

  double sum_t = 6.0;
  double tmp_t = 0.0;
  std::vector<double> raw_s;
  raw_s.clear();
  raw_s.reserve(raw_t_.size());
  raw_s.emplace_back(0.0);
  for (int i = 1; i < raw_t_.size(); i++) {
    double s_tmp =
        (raw_v_[i] + raw_v_[i - 1]) * 0.5 * (raw_t_[i] - raw_t_[i - 1]);
    raw_s.emplace_back(raw_s.back() + s_tmp);
  }
  double t_tmp = 0.0;
  for (int i = 1; i < raw_t_.size(); i++) {
    while (tmp_t < raw_t_[i]) {
      TrajectoryPoint tmp;
      double radio = (tmp_t - raw_t_[i - 1]) / (raw_t_[i] - raw_t_[i - 1]);
      double tmp_s = radio * (raw_s[i] - raw_s[i - 1]) + raw_s[i - 1];
      tmp.relative_time = tmp_t;
      tmp.v = radio * (raw_v_[i] - raw_v_[i - 1]) + raw_v_[i - 1];
      auto position_tmp = get_point_via_s(smooth_cartesian_points, tmp_s);
      tmp.position = position_tmp;
      trajectory_points.emplace_back(tmp);
      tmp_t += 0.2;
    }
  }

  if (trajectory_points.size() < 2) {
    trajectory_points.front().theta = init_theta_;
    std::cout << " fill trajectory :" << trajectory_points.size() << std::endl;
    return;
  }
  double delta_y =
      trajectory_points[1].position.y - trajectory_points[0].position.y;
  double delta_x =
      trajectory_points[1].position.x - trajectory_points[0].position.x;
  if (delta_x * delta_x + delta_y * delta_y < 1e-3) {
    trajectory_points.front().theta = init_theta_;
  } else {
    if (std::fabs(delta_x) < 1e-3) {
      trajectory_points.front().theta = delta_y > 0 ? M_PI * 0.5 : -M_PI * 0.5;
    } else {
      trajectory_points.front().theta = std::atan(delta_y / delta_x);
    }
  }
  for (int i = 1; i < trajectory_points.size() - 1; i++) {
    double delta_y =
        trajectory_points[i + 1].position.y - trajectory_points[i].position.y;
    double delta_x =
        trajectory_points[i + 1].position.x - trajectory_points[i].position.x;
    if (delta_x * delta_x + delta_y * delta_y < 1e-3) {
      trajectory_points[i].theta = trajectory_points[i - 1].theta;
    } else {
      if (std::fabs(delta_x) < 1e-3) {
        trajectory_points[i].theta = delta_y > 0 ? M_PI * 0.5 : -M_PI * 0.5;
      } else {
        trajectory_points[i].theta = std::atan2(delta_y , delta_x);
      }
    }
  }
}
}  // namespace EnvSim
