/*
 * Copyright (C) 2020 by SenseTime Group Limited. All rights reserved.
 * caoxiaoxu <caoxiaoxu@sensetime.com>
 */

#pragma once
#include <cmath>
#include <utility>
#include <vector>

namespace PolyFit {
    using  uint32_t = u_int32_t;
    using  uint64_t = u_int64_t;
struct PathPoint {
  PathPoint() {}
  PathPoint(double _x, double _y, double _theta, double _s)
      : x(_x), y(_y), theta(_theta), s(_s) {}

  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
  double s = 0.0;
};

class DiscretePath2d {
public:
  DiscretePath2d() = default;

  ~DiscretePath2d() = default;

  explicit DiscretePath2d(
      const std::vector<PathPoint> &discrete_path_points_2d);
  explicit DiscretePath2d(const DiscretePath2d &path_2d);

  PathPoint InterpolatePathByDistance(const double s);

  bool InterpPathByEqualDistance(const double intervel,
                                 std::vector<PathPoint> *equal_dist_path);

  DiscretePath2d *operator+=(const DiscretePath2d &new_path);
  DiscretePath2d *operator+=(const PathPoint &path_point);
  DiscretePath2d *operator+=(const std::vector<PathPoint> &path_points);
  // bool UpateThetaS(std::vector<PathPoint> *path_point_2d);
  std::vector<PathPoint> GetPathPoints() const;
  void ClearPoints();
  void SetPoint(const uint32_t index, const PathPoint &point);
  PathPoint GetPoint(const uint32_t index);
  double GetLengthS();
  unsigned int GetPathSize();

private:
  bool fast_search_in_array(const std::vector<double> &InputArray,
                            double targetvalue,
                            std::pair<uint32_t, uint32_t> *indexs);

  bool intepolate_between(const PathPoint &point1, const PathPoint &point2,
                          const double ratio, PathPoint *mid_point);
  bool UpateThetaS(std::vector<PathPoint> *path_point_2d);
  double InterpLine(double x, const std::vector<double> &xp,
                    const std::vector<double> &fp);

private:
  std::vector<PathPoint> path_point_2d_;
  const double s_epsilon_ = 0.01;
};

} // namespace PolyFit
