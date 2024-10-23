//
// Created by SENSETIME\fengxiaotong on 23-12-8.
//
#include "lane.h"
#include "spline.h"
#include <iostream>

namespace EnvSim {
void Lane::update_center_points(const bool interpolate_flag) {
  if (interpolate_flag) {
    interpolate_points(raw_center_points_, center_points_, 120);
  } else {
    center_points_ = raw_center_points_;;
  }
  update_speed_limit(interpolate_flag);
  update_s();
  update_curvature();
}

double Lane::get_s(const MathUtils::Point2D &point) const {
  int index = 0;
  double delta_x = center_points_.front().x - point.x;
  double delta_y = center_points_.front().y - point.y;
  double min_dis = delta_x * delta_x + delta_y * delta_y;

  for (int i = 1; i < center_points_.size(); i++) {
    double delta_x_t = center_points_[i].x - point.x;
    double delta_y_t = center_points_[i].y - point.y;
    double min_dis_t = delta_x_t * delta_x_t + delta_y_t * delta_y_t;
    if (min_dis_t < min_dis) {
      index = i;
    }
  }
  int left_index = 0;
  int right_index = 0;
  if (index > 0) {
    left_index = index - 1;
    right_index = index;
  } else {
    left_index = index;
    right_index = index + 1;
  }
  MathUtils::Point2D PA = {point.x - center_points_[left_index].x,
                           point.y - center_points_[left_index].y};
  MathUtils::Point2D BA = {
      center_points_[right_index].x - center_points_[left_index].x,
      center_points_[right_index].y - center_points_[left_index].y};

  double length_ba = std::hypot(BA.x, BA.y);
  double s_add = (PA.x * BA.x + PA.y * PA.y) / length_ba;
  return center_points_s_[left_index] + s_add;
}

void Lane::update_s() {
  center_points_s_.clear();
  center_points_s_.emplace_back(0.0);
  for (int i = 1; i < center_points_.size(); i++) {
    double delta_x = center_points_[i].x - center_points_[i - 1].x;
    double delta_y = center_points_[i].y - center_points_[i - 1].y;
    double s_tmp = std::hypot(delta_x, delta_y);
    center_points_s_.emplace_back(center_points_s_.back() + s_tmp);
  }
}

void Lane::update_speed_limit(const bool interpolate_flag) {
  speed_limit_.clear();
  if (interpolate_flag) {
    for (int i = 0; i < center_points_.size(); i++) {
      speed_limit_.emplace_back(16.67);
    }
  } else {
    for (int i = 0; i < center_points_.size(); i++) {
      double velocity = 0.0;
      if (center_points_.size() == 1) {
        velocity = 0.0;
      } else if (i + 1 < center_points_.size()) {
        velocity =
            std::hypot(
                center_points_[i + 1].x - center_points_[i].x,
                center_points_[i + 1].y - center_points_[i].y) /
            0.1;
      } else {
        velocity = speed_limit_[i - 1];
      }
      speed_limit_.emplace_back(velocity);
    }
  }
}

// Note: not precise
void Lane::update_curvature() {
  int n = center_points_.size();
  curva_.resize(n, 0.0);
  if (n < 3) {
    return;
  }
  for (int i = 1; i < n - 1; ++i) {
    auto &point_1 = center_points_[i - 1];
    auto &point_2 = center_points_[i];
    auto &point_3 = center_points_[i + 1];
    auto r = MathUtils::CalculateRadius(point_1, point_2, point_3);
    curva_[i] = 1.0 / r;
  }
  curva_[0] = curva_[1];
  curva_[n - 1] = curva_[n - 2];

  // for (auto c : curva_) {
  //   std::cout << "curva " << c << std::endl;
  // }
}

void LaneManager::update_lanes(const std::vector<Lane> &lanes) {
  lanes_.clear();
  lanes_ = lanes;
}

// Note: not precise
const MathUtils::Point2D Lane::frenet_to_cartesian(
    const MathUtils::Point2D &frenet_point) const {
  double s_sum = 0;
  int index_target;
  MathUtils::Point2D res_point;
  for (int i = 1; i < center_points_.size(); i++) {
    s_sum += std::hypot(center_points_[i].x - center_points_[i - 1].x,
                        center_points_[i].y - center_points_[i - 1].y);
    if (s_sum > frenet_point.x) {
      res_point.x = center_points_[i].x;
      res_point.y = center_points_[i].y;
      index_target = i;
      break;
    }
  }
  MathUtils::Point2D straight_vec = {
      center_points_[index_target].x - center_points_[index_target - 1].x,
      center_points_[index_target].y - center_points_[index_target - 1].y};

  MathUtils::Point2D left_vec = {-straight_vec.y, straight_vec.x};
  MathUtils::Point2D right_vec = {straight_vec.y, -straight_vec.x};

  if (frenet_point.y > 0) {
    // left
    double left_vec_length = std::hypot(left_vec.x, left_vec.y);
    res_point.x += left_vec.x * std::fabs(frenet_point.y) / left_vec_length;
    res_point.y += left_vec.y * std::fabs(frenet_point.y) / left_vec_length;
  } else {
    // right
    double right_vec_length = std::hypot(right_vec.x, right_vec.y);
    res_point.x += right_vec.x * std::fabs(frenet_point.y) / right_vec_length;
    res_point.y += right_vec.y * std::fabs(frenet_point.y) / right_vec_length;
  }
  return res_point;
}

std::vector<std::vector<MathUtils::Point2D>>
LaneManager::get_all_lanes_center_points() {
  std::vector<std::vector<MathUtils::Point2D>> lanes_pts;
  for (auto &lane : lanes_) {
    lanes_pts.emplace_back(lane.get_center_points());
  }
  return lanes_pts;
}
}  // namespace EnvSim