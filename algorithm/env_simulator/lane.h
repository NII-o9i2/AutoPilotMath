//
// Created by SENSETIME\fengxiaotong on 23-12-8.
//

#ifndef ALGORITHM_ALGORITHM_ENV_SIMULATER_LANE_H_
#define ALGORITHM_ALGORITHM_ENV_SIMULATER_LANE_H_
#include "utils.h"
#include "vector"

namespace EnvSim {
class Lane {
 public:
  Lane(const int &_relative_id,
       const std::vector<MathUtils::Point2D> &_raw_points)
      : raw_center_points_(_raw_points), relative_id_(_relative_id){};
  void update_center_points(const bool interpolate_flag);

  const int &get_id() const { return relative_id_; };

  const MathUtils::Point2D frenet_to_cartesian(
      const MathUtils::Point2D &frenet_point) const;

  const std::vector<MathUtils::Point2D> &get_center_points() const {
    return center_points_;
  };

  const std::vector<double> &get_curva() const { return curva_; };

  const std::vector<double> &get_speed_limit() const { return speed_limit_; };

  double get_s(const MathUtils::Point2D &point) const;


 private:
  void update_curvature();
  void update_s();
  void update_speed_limit(const bool interpolate_flag);

 private:
  std::vector<MathUtils::Point2D> center_points_;
  std::vector<MathUtils::Point2D> raw_center_points_;
  std::vector<double> center_points_s_;
  // -1 left 0 middle 1 right
  int relative_id_ = 0;
  std::vector<double> curva_;  // kappa for every point
  std::vector<double> speed_limit_; // speed limit for every point
};

class LaneManager {
 public:
  void update_lanes(const std::vector<Lane> &lanes);
  std::vector<std::vector<MathUtils::Point2D>> get_all_lanes_center_points();

  const Lane &get_lane(const int lane_id) const {
    for (auto &lane : lanes_) {
      if (lane.get_id() == lane_id) {
        return lane;
      }
    }
  }

 private:
  std::vector<Lane> lanes_;
};
}  // namespace EnvSim

#endif  // ALGORITHM_ALGORITHM_ENV_SIMULATER_LANE_H_
