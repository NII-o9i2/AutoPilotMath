#pragma once

#include <vector>
#include "utils.h"

namespace InteractionSearch {

struct OSPReflinePointInfo {
  OSPReflinePointInfo() = default;
  MathUtils::Point2D point;
  double theta = 0.0;
  double curvature = 0.0;
  bool is_on_left = true;
  double speed_limit = 16.67;
};

class OSPEnv {
 public:
  OSPEnv() = default;
  virtual ~OSPEnv() = default;

  virtual MathUtils::FrenetPoint2D cartesian_to_frenet(
      const MathUtils::Point2D &cartesian_point) = 0;

  virtual MathUtils::Point2D frenet_to_cartesian(
      const MathUtils::FrenetPoint2D &frenet_point) = 0;

  virtual OSPReflinePointInfo get_lane_point_info(
      const MathUtils::Point2D &cartesian_point) = 0;

  virtual double get_heading_at_s(const double s) = 0;

  virtual double get_lane_point_speed_limit(
      const MathUtils::Point2D &cartesian_point) = 0;

  virtual void max_min_sl(const std::vector<MathUtils::Point2D> &polygon,
                          double &max_s,
                          double &min_s,
                          double &max_l,
                          double &min_l) = 0;

  virtual bool check_freespace_collision(
      const MathUtils::Point2D &cartesian_point) = 0;
};

}  // namespace InteractionSearch
