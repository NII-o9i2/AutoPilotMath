#pragma once

#include "utils.h"
#include <vector>

namespace ILQR {

struct VehicleParam {
  double length;
  double width;
  double wheel_base;
  double max_delta;
  double r_4;
  std::vector<double> r_4_ratio = {3, 1, -1, -3};
};

struct ObstaclePoint {
  MathUtils::Point2D position;
  double theta = 0.0;
  double velocity = 0.0;
  double acceleration = 0.0;
  double relative_time = 0.0;
};
}