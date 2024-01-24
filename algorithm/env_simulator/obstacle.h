//
// Created by SENSETIME\fengxiaotong on 23-12-11.
//

#ifndef ALGORITHM_ALGORITHM_ENV_SIMULATOR_OBSTACLE_H_
#define ALGORITHM_ALGORITHM_ENV_SIMULATOR_OBSTACLE_H_

#include "lane.h"
#include "unordered_map"
#include "utils.h"
#include "vector"

namespace EnvSim {

struct TrajectoryPoint {
  double relative_time;
  double v;
  double theta;
  MathUtils::Point2D position;
  std::vector<MathUtils::Point2D> polygon;
};

class RawTrajectoryPoint {
public:
  RawTrajectoryPoint(int _id, double _s, double _l)
      : lane_id(_id), s(_s), l(_l){};
  int lane_id;
  double s;
  double l;
};

class Obstacle {
public:
  Obstacle(int _id, double _length, double _width, double _init_theta,
           const std::vector<double> &_raw_t, const std::vector<double> &_raw_v,
           const std::vector<RawTrajectoryPoint> &_raw_trajectory)
      : id(_id), length_(_length), width_(_width), init_theta_(_init_theta),
        raw_t_(_raw_t), raw_v_(_raw_v),
        raw_trajectory_points_(_raw_trajectory) {
    for (auto &v : raw_v_) {
      v = v / 3.6;
    }
  };

  void construct(const LaneManager &lane_manager);
  std::vector<TrajectoryPoint> trajectory_points;
  double get_length() const { return length_; };
  double get_width() const { return width_; };
  int get_id() const { return id; };

private:
  int id;
  double length_;
  double width_;
  double init_theta_;

  std::vector<double> raw_t_;
  std::vector<double> raw_v_;

  std::vector<RawTrajectoryPoint> raw_trajectory_points_;
};

class ObstacleManager {
public:
  std::unordered_map<int, Obstacle> obstacle_map;
};
} // namespace EnvSim

#endif // ALGORITHM_ALGORITHM_ENV_SIMULATOR_OBSTACLE_H_
