/*
 * Copyright (C) 2020 by SenseTime Group Limited. All rights reserved.
 * caoxiaoxu <caoxiaoxu@sensetime.com>
 */

#include "spline_smoothing/include/common/discrete_path2d.hpp"

namespace PolyFit {

DiscretePath2d::DiscretePath2d(
    const std::vector<PathPoint> &discrete_path_points_2d) {
  path_point_2d_.clear();
  path_point_2d_ = discrete_path_points_2d;
  UpateThetaS(&path_point_2d_);
}

DiscretePath2d::DiscretePath2d(const DiscretePath2d &path_2d) {
  path_point_2d_ = path_2d.path_point_2d_;
}
double DiscretePath2d::GetLengthS() { return path_point_2d_.back().s; }
unsigned int DiscretePath2d::GetPathSize() { return path_point_2d_.size(); }

bool DiscretePath2d::InterpPathByEqualDistance(
    const double intervel, std::vector<PathPoint> *equal_dist_path) {
  // check and ignore redundant points
  if (nullptr == equal_dist_path) {
    // AD_LERROR(PP_DiscretePath2d) << " euqal_dist_path nullptr";
  }
  equal_dist_path->clear();
  std::vector<PathPoint> raw_path_points;
  if (path_point_2d_.size() < 2) {
    // AD_LERROR(PP_DiscretePath2d) << " path point size too small: "
    //                              << path_point_2d_.size();
    return false;
  }
  raw_path_points.push_back(path_point_2d_.at(0));
  for (int i = 1; i < (path_point_2d_.size() - 1); i++) {
    if (std::abs(path_point_2d_.at(i).s - path_point_2d_.at(i - 1).s) <
        s_epsilon_) {
      // AD_LERROR(PP_DiscretePath2d) << "redundant points found, index: " << i;
      continue;
    }
    raw_path_points.push_back(path_point_2d_.at(i));
  }
  // interpolate
  std::vector<double> x_vec, y_vec, s_vec;
  for (unsigned int i = 0; i < raw_path_points.size(); i++) {
    x_vec.push_back(raw_path_points.at(i).x);
    y_vec.push_back(raw_path_points.at(i).y);
    s_vec.push_back(raw_path_points.at(i).s);
  }
  double s_value = 0.0;
  while (s_value < raw_path_points.back().s) {
    double x_tmp = InterpLine(s_value, s_vec, x_vec);
    double y_tmp = InterpLine(s_value, s_vec, y_vec);
    PathPoint tmp_path_point = {x_tmp, y_tmp, 0.0, s_value};
    equal_dist_path->push_back(tmp_path_point);
    s_value += intervel;
  }

  return true;
}

PathPoint DiscretePath2d::InterpolatePathByDistance(const double s) {
  PathPoint mid_point;
  std::vector<double> s_array;
  s_array.clear();
  for (auto &item : path_point_2d_) {
    s_array.push_back(item.s);
  }
  std::pair<uint32_t, uint32_t> index_pair;
  if (fast_search_in_array(s_array, s, &index_pair)) {
    PathPoint point1 = path_point_2d_[index_pair.first];
    PathPoint point2 = path_point_2d_[index_pair.second];

    double s1 = point1.s, s2 = point2.s;
    if (abs(s1 - s2) < s_epsilon_) {
      return point1;
    } else if (abs(s1 - s) < s_epsilon_) {
      return point1;
    } else if (abs(s2 - s) < s_epsilon_) {
      return point2;
    } else {
      double ratio = (s - s1) / (s2 - s1);

      if (intepolate_between(point1, point2, ratio, &mid_point)) {
        return mid_point;
      }
    }
    // AD_LERROR(PP_DiscretePath2d) << "abnormal s appear!";
    return mid_point;
  } else {
    // AD_LERROR(PP_DiscretePath2d) << "interpolate issue, please check";
    return mid_point;
  }
}

double DiscretePath2d::InterpLine(double x, const std::vector<double> &xp,
                                  const std::vector<double> &fp) {
  const size_t N = xp.size();
  size_t hi;
  for (hi = 0; hi < N && x > xp[hi]; hi++) {
  }

  if (hi == 0)
    return fp[0];

  if (hi == N && x > xp[N - 1])
    return fp[N - 1];

  const size_t low = hi - 1;
  const float xp_diff = xp[hi] - xp[low];
  if (xp_diff < 1e-5 && xp_diff > -1e-5)
    return fp[low];

  return (x - xp[low]) * (fp[hi] - fp[low]) / xp_diff + fp[low];
}

bool DiscretePath2d::fast_search_in_array(
    const std::vector<double> &InputArray, double targetvalue,
    std::pair<uint32_t, uint32_t> *indexs) {
  uint32_t iLength = InputArray.size();
  uint32_t low = 0, high = iLength - 1;
  if (indexs == nullptr) {
    // AD_LERROR(PP_DiscretePath2d) << "wrong input";
  }
  if (iLength < 1) {
    // AD_LERROR(PP_DiscretePath2d) << "input array less than 1";
    return false;
  }

  if (iLength == 1) {
    indexs->first = 0;
    indexs->second = 0;
    return true;
  }

  if (targetvalue <= InputArray[0]) {
    indexs->first = 0;
    indexs->second = 0;
    return true;
  } else if (targetvalue >= InputArray[iLength - 1]) {
    indexs->first = iLength - 1;
    indexs->second = iLength - 1;
    return iLength - 1;
  }

  while (low < high - 1) {
    uint32_t middle = (low + high) / 2; // 1.5-> 1
    if (targetvalue > InputArray[middle]) {
      low = middle;
    } else {
      high = middle;
    }
  }

  // check
  if (InputArray[low] < targetvalue && InputArray[high] > targetvalue) {
    indexs->first = low;
    indexs->second = high;
    return true;
  } else {
    // AD_LERROR(PP) << "discrete path 2d search wrong";
    return false;
  }
}

bool DiscretePath2d::intepolate_between(const PathPoint &point1,
                                        const PathPoint &point2,
                                        const double ratio,
                                        PathPoint *mid_point) {
  if (ratio < 0.0) {
    // AD_LERROR(PP_DiscretePath2d) << "ratio should in [0,1], current ratio: "
    //                              << ratio;
    *mid_point = point1;
    return false;
  } else if (ratio > 1.0) {
    // AD_LERROR(PP_DiscretePath2d) << "ratio should in [0,1], current ratio: "
    //  << ratio;
    *mid_point = point2;
    return false;
  } else {
    mid_point->x = (point2.x - point1.x) * ratio + point1.x;
    mid_point->y = (point2.y - point1.y) * ratio + point1.y;
    mid_point->s = (point2.s - point1.s) * ratio + point1.s;

    double tmp_theta_0 = point1.theta;
    double tmp_theta_1 = point2.theta;
    // for example angle = 170 and start angle =-170
    if ((tmp_theta_1 - tmp_theta_0) > M_PI) {
      tmp_theta_1 = tmp_theta_1 - 2 * M_PI;
    } else if ((tmp_theta_1 - tmp_theta_0) < -M_PI) {
      tmp_theta_1 = tmp_theta_1 + 2 * M_PI;
    }
    mid_point->theta = tmp_theta_0 * (1 - ratio) + tmp_theta_1 * ratio;
    return true;
  }
}

std::vector<PathPoint> DiscretePath2d::GetPathPoints() const {
  return path_point_2d_;
}
DiscretePath2d *DiscretePath2d::operator+=(const DiscretePath2d &new_path) {
  std::vector<PathPoint> tmp_new_path_points = new_path.GetPathPoints();
  for (auto &item : tmp_new_path_points) {
    this->path_point_2d_.push_back(item);
  }
  UpateThetaS(&path_point_2d_);
  // todo(cxx) modify s and theta
  return this;
}

DiscretePath2d *DiscretePath2d::operator+=(const PathPoint &new_path_point) {
  PathPoint input_path_point;
  input_path_point.x = new_path_point.x;
  input_path_point.y = new_path_point.y;
  if (path_point_2d_.empty()) {
    input_path_point.s = 0;
    this->path_point_2d_.push_back(input_path_point);
  } else {
    double d_x = input_path_point.x - path_point_2d_.back().x;
    double d_y = input_path_point.y - path_point_2d_.back().y;
    double delta_s = std::sqrt(d_x * d_x + d_y * d_y);
    double tmp_theta = std::atan2(d_y, d_x);

    input_path_point.s = path_point_2d_.back().s + delta_s;
    input_path_point.theta = tmp_theta;
    // modify the original path2d last theta
    this->path_point_2d_.back().theta = tmp_theta;

    this->path_point_2d_.push_back(input_path_point);

    // AD_LDEBUG(PP_DiscretePath2d)
    // << "x: " << new_path_point.x << ", y: " << new_path_point.y
    // << "path_point_2d_ size: " << path_point_2d_.size()
    // << ", length: " << path_point_2d_.back().s << ", delta_s: " << delta_s;
  }
  return this;
}

DiscretePath2d *DiscretePath2d::
operator+=(const std::vector<PathPoint> &path_points) {
  if (path_points.empty()) {
    // AD_LERROR(PP_DiscretePath2d) << "input empty";
    return this;
  } else {
    for (auto &path_point : path_points) {
      this->operator+=(path_point);
    }
    return this;
  }
}

void DiscretePath2d::ClearPoints() { this->path_point_2d_.clear(); }
bool DiscretePath2d::UpateThetaS(std::vector<PathPoint> *path_point_2d) {
  uint64_t size = path_point_2d->size();
  if (size < 2) {
    // AD_LERROR(PP_DiscretePath2d) << "wrong size";
    return false;
  }

  path_point_2d->at(0).s = 0;

  for (uint64_t i = 1; i < size; i++) {
    double d_x = path_point_2d->at(i).x - path_point_2d->at(i - 1).x;
    double d_y = path_point_2d->at(i).y - path_point_2d->at(i - 1).y;
    double delta_s = std::sqrt(d_x * d_x + d_y * d_y);
    double tmp_theta = std::atan2(d_y, d_x);
    path_point_2d->at(i).s = delta_s + path_point_2d->at(i - 1).s;
    path_point_2d->at(i - 1).theta = tmp_theta;
  }
  // the final direction is set to 0
  path_point_2d->back().theta = path_point_2d->at(size - 2).theta;
  return true;
}
PathPoint DiscretePath2d::GetPoint(const uint32_t index) {
  PathPoint tmp_point;
  if (index < path_point_2d_.size()) {
    tmp_point = path_point_2d_.at(index);
  } else {
    // AD_LERROR(PP_DiscretePath2d) << "wrong index input";
  }
  return tmp_point;
}
void DiscretePath2d::SetPoint(const uint32_t index, const PathPoint &point) {
  if (index >= path_point_2d_.size()) {
    // AD_LERROR(PP_DiscretePath2d) << "wrong index input";
  } else {
    path_point_2d_.at(index) = point;
  }
}

} // namespace PolyFit
