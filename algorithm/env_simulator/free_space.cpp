#include "free_space.h"

namespace EnvSim {

MathUtils::Point2D find_projection_point(const MathUtils::Point2D &point1,
                                         const MathUtils::Point2D &point2,
                                         double x,
                                         double y) {
  MathUtils::Point2D projPoint;

  // Compute vector v
  double v_x = point2.x - point1.x;
  double v_y = point2.y - point1.y;

  // Compute vector u
  double u_x = x - point1.x;
  double u_y = y - point1.y;

  // Compute dot product of u and v
  double dotProductUV = u_x * v_x + u_y * v_y;

  // Compute dot product of v and v
  double dotProductVV = v_x * v_x + v_y * v_y;

  // Calculate the projection ratio t
  double t = dotProductUV / dotProductVV;

  // Compute the projection point
  projPoint.x = point1.x + t * v_x;
  projPoint.y = point1.y + t * v_y;

  return projPoint;
}

double calculate_distance(const MathUtils::Point2D &p1,
                          const MathUtils::Point2D &p2) {
  return std::hypot(p2.x - p1.x, p2.y - p1.y);
}

std::vector<MathUtils::Point2D> equalDistanceInterpolation(
    const std::vector<MathUtils::Point2D> &points, double interval) {
  std::vector<MathUtils::Point2D> interpolatedPoints;

  for (size_t i = 0; i < points.size() - 1; ++i) {
    const MathUtils::Point2D &p1 = points[i];
    const MathUtils::Point2D &p2 = points[i + 1];

    // 计算两点之间的距离
    double distance = calculate_distance(p1, p2);

    // 计算需要插值的点数
    size_t numInterpolatedPoints = static_cast<size_t>(distance / interval);

    // 计算每个插值点的步长
    double stepX = (p2.x - p1.x) / (numInterpolatedPoints + 1);
    double stepY = (p2.y - p1.y) / (numInterpolatedPoints + 1);

    // 从第一个点开始插值
    double currentX = p1.x + stepX;
    double currentY = p1.y + stepY;
    interpolatedPoints.emplace_back(MathUtils::Point2D(p1.x, p1.y));
    // 生成插值点并添加到结果向量中
    for (size_t j = 0; j < numInterpolatedPoints; ++j) {
      interpolatedPoints.emplace_back(MathUtils::Point2D(currentX, currentY));
      currentX += stepX;
      currentY += stepY;
    }
  }

  return interpolatedPoints;
}

// 计算向量叉积
double crossProduct(const MathUtils::Point2D &A,
                    const MathUtils::Point2D &B,
                    const MathUtils::Point2D &C) {
  return (B.x - A.x) * (C.y - A.y) - (B.y - A.y) * (C.x - A.x);
}

// 判断两个线段是否相交
bool segmentsIntersect(const MathUtils::Point2D &A,
                       const MathUtils::Point2D &B,
                       const MathUtils::Point2D &C,
                       const MathUtils::Point2D &D) {
  double cross1 = crossProduct(A, B, C);
  double cross2 = crossProduct(A, B, D);
  double cross3 = crossProduct(C, D, A);
  double cross4 = crossProduct(C, D, B);

  // 如果 cross1 和 cross2 异号，并且 cross3 和 cross4
  // 也异号，则表示两个线段相交
  return ((cross1 > 0 && cross2 < 0) || (cross1 < 0 && cross2 > 0)) &&
         ((cross3 > 0 && cross4 < 0) || (cross3 < 0 && cross4 > 0));
}

// 判断两个点是否在同一侧
bool onSameSide(MathUtils::Point2D p1,
                MathUtils::Point2D p2,
                MathUtils::Point2D a,
                MathUtils::Point2D b) {
  double cp1 = (b.x - a.x) * (p1.y - a.y) - (b.y - a.y) * (p1.x - a.x);
  double cp2 = (b.x - a.x) * (p2.y - a.y) - (b.y - a.y) * (p2.x - a.x);
  return (cp1 * cp2) >= 0;
}

// 判断两条线段是否相交
bool areSegmentsIntersecting(MathUtils::Point2D p1,
                             MathUtils::Point2D p2,
                             MathUtils::Point2D q1,
                             MathUtils::Point2D q2) {
  bool isIntersecting = false;

  // 判断 p1 和 p2 在 q1 和 q2 的两侧
  bool p1p2q1 = onSameSide(p1, p2, q1, q2);
  bool p1p2q2 = onSameSide(p1, p2, q2, q1);

  // 判断 q1 和 q2 在 p1 和 p2 的两侧
  bool q1q2p1 = onSameSide(q1, q2, p1, p2);
  bool q1q2p2 = onSameSide(q1, q2, p2, p1);

  // 如果两条线段不在同一侧，则相交
  if (!p1p2q1 && !p1p2q2 && !q1q2p1 && !q1q2p2) {
    isIntersecting = true;
  }

  return isIntersecting;
}

bool FreeSpaceManager::is_point_in_ellipse(
    const MathUtils::Point2D &road_edge_point,
    const PlanningPoint &traj_point) {
  double road_edge_x = road_edge_point.x;
  double road_edge_y = road_edge_point.y;

  double h = traj_point.position.x;
  double k = traj_point.position.y;
  double theta = traj_point.theta;
  double ellipse_param_a = traj_point.velocity * 3.0;
  ellipse_param_a = std::max(ellipse_param_a, vehicle_info_.length + 5.0);
  double ellipse_param_b = ellipse_param_a * 0.1;
  ellipse_param_b = std::max(ellipse_param_b, vehicle_info_.width + 5.0);
  double condition =
      std::pow(
          (road_edge_x * std::cos(theta) + road_edge_y * std::sin(theta) - h) /
              ellipse_param_a,
          2) +
      std::pow(
          (-road_edge_x * std::sin(theta) + road_edge_y * std::cos(theta) - k) /
              ellipse_param_b,
          2);
  return condition < 1;
}

void FreeSpaceManager::update(
    const std::vector<std::vector<MathUtils::Point2D>> &road_edge_vec_input,
    const VehicleInfo &vehicle_info) {
  vehicle_info_ = vehicle_info;

  if (road_edge_vec_input.size() < 2) {
    std::cerr << "road_edge_vec_input size is less 2" << std::endl;
    return;
  }

  // save raw data
  raw_left_road_edge_.assign(road_edge_vec_input[0].begin(),
                             road_edge_vec_input[0].end());
  raw_right_road_edge_.assign(road_edge_vec_input[1].begin(),
                              road_edge_vec_input[1].end());

  // interpolate road edge raw point
  for (const auto &road_edge : road_edge_vec_input) {
    // interpolate_points(road_edge, road_edge_interp_points_, 60);
    // road_edge_processed_vec_.emplace_back(road_edge);
    // road_edge_processed_vec_.emplace_back(road_edge_interp_points_);
    road_edge_processed_vec_.emplace_back(
        equalDistanceInterpolation(road_edge, 2.0));
  }

  // save processed road edge point
  processed_left_road_edge_.assign(road_edge_processed_vec_[0].begin(),
                                   road_edge_processed_vec_[0].end());
  processed_right_road_edge_.assign(road_edge_processed_vec_[1].begin(),
                                    road_edge_processed_vec_[1].end());

  // construct freespace polygon
  for (const auto &point : road_edge_processed_vec_[0]) {
    freespace_polygon_.emplace_back(point);
  }
  for (auto it = road_edge_processed_vec_[1].rbegin();
       it != road_edge_processed_vec_[1].rend(); ++it) {
    freespace_polygon_.emplace_back(*it);
  }
}

std::map<Direction, std::vector<MathUtils::Point2D>>
FreeSpaceManager::find_closest_point_at_both_side(
    const MathUtils::Point2D &point) {
  std::map<Direction, std::vector<MathUtils::Point2D>> result;
  std::vector<MathUtils::Point2D> left_closest_points;
  std::vector<MathUtils::Point2D> right_closest_points;
  left_closest_points.emplace_back(
      find_closest_point_at_boundary(point, road_edge_processed_vec_[0]));
  right_closest_points.emplace_back(
      find_closest_point_at_boundary(point, road_edge_processed_vec_[1]));
  result.insert(std::make_pair(Direction::LEFT, left_closest_points));
  result.insert(std::make_pair(Direction::RIGHT, right_closest_points));
  return result;
}

MathUtils::Point2D FreeSpaceManager::find_closest_point_at_boundary(
    const MathUtils::Point2D &point,
    const std::vector<MathUtils::Point2D> &boundary) {
  // todo: 假设可能不成立, 规划轨迹point可能超出视觉车道线范围
  if (!is_point_in_freespace(point)) {
    // std::cout << "query point is not in freespace" << std::endl;
    return {0, 0};
  }
  auto func_distance_square = [](const MathUtils::Point2D &point,
                                 const double x, const double y) {
    double dx = point.x - x;
    double dy = point.y - y;
    return dx * dx + dy * dy;
  };

  double distance_min =
      func_distance_square(boundary.front(), point.x, point.y);
  std::size_t index_min = 0;

  for (std::size_t i = 1; i < boundary.size(); ++i) {
    double distance_temp = func_distance_square(boundary[i], point.x, point.y);
    if (distance_temp < distance_min) {
      distance_min = distance_temp;
      index_min = i;
    }
  }

  std::size_t index_start = (index_min == 0) ? index_min : index_min - 1;
  std::size_t index_end =
      (index_min + 1 == boundary.size()) ? index_min : index_min + 1;

  if (index_start == index_end) {
    return boundary[index_start];
  }

  return find_projection_point(boundary[index_start], boundary[index_end],
                               point.x, point.y);
}

std::vector<MathUtils::Point2D>
FreeSpaceManager::get_right_border_use_ellipse_model(
    const PlanningPoint &traj_point) {
  std::vector<MathUtils::Point2D> result;
  for (const auto &road_edge_pt : processed_right_road_edge_) {
    if (is_point_in_ellipse(road_edge_pt, traj_point)) {
      result.emplace_back(road_edge_pt);
    }
  }
  return result;
}

std::vector<MathUtils::Point2D>
FreeSpaceManager::get_left_border_use_ellipse_model(
    const PlanningPoint &traj_point) {
  std::vector<MathUtils::Point2D> result;
  for (const auto &road_edge_pt : processed_left_road_edge_) {
    if (is_point_in_ellipse(road_edge_pt, traj_point)) {
      result.emplace_back(road_edge_pt);
    }
  }
  return result;
}

std::vector<MathUtils::Point2D>
FreeSpaceManager::get_freespace_left_points_by_ttc(
    double time, const PlanningPoint &traj_point) {
  MathUtils::Point2D predict_position =
      predict_ego_position_at_time(time, traj_point);
  MathUtils::Point2D ego_position = traj_point.position;
  std::vector<MathUtils::Point2D> result;
  std::size_t size_step = 4;
  std::size_t check_index = 0;
  for (size_t i = 0; i < processed_left_road_edge_.size() - size_step;
       i = i + size_step) {
    check_index = i;
    const MathUtils::Point2D &p1 = processed_left_road_edge_[i];
    const MathUtils::Point2D &p2 = processed_left_road_edge_[i + size_step];
    if (areSegmentsIntersecting(predict_position, ego_position, p1, p2)) {
      std::copy(processed_left_road_edge_.begin() + i,
                processed_left_road_edge_.begin() + i + size_step,
                std::back_inserter(result));
      return result;
    }
  }
  if ((check_index + size_step) != (processed_left_road_edge_.size() - 1)) {
    const MathUtils::Point2D &p1 = processed_left_road_edge_[check_index];
    const MathUtils::Point2D &p2 = processed_left_road_edge_.back();
    if (areSegmentsIntersecting(predict_position, ego_position, p1, p2)) {
      std::copy(processed_left_road_edge_.begin() + check_index,
                processed_left_road_edge_.end(), std::back_inserter(result));
      return result;
    }
  }
  return result;
}

std::vector<MathUtils::Point2D>
FreeSpaceManager::get_freespace_right_points_by_ttc(
    double time, const PlanningPoint &traj_point) {
  std::vector<MathUtils::Point2D> result;

  MathUtils::Point2D predict_position =
      predict_ego_position_at_time(time, traj_point);

  MathUtils::Point2D ego_position = traj_point.position;
  std::size_t size_step = 4;
  std::size_t check_index = 0;
  for (size_t i = 0; i < processed_right_road_edge_.size() - size_step;
       i = i + size_step) {
    check_index = i;
    const MathUtils::Point2D &p1 = processed_right_road_edge_[i];
    const MathUtils::Point2D &p2 = processed_right_road_edge_[i + size_step];
    // todo: need more robust method to judge line segments intersect
    if (areSegmentsIntersecting(predict_position, ego_position, p1, p2)) {
      std::copy(processed_right_road_edge_.begin() + i,
                processed_right_road_edge_.begin() + i + size_step,
                std::back_inserter(result));
      // std::cerr << "[PART_1]segmentsIntersect is true" << std::endl;
      return result;
    }
  }
  if ((check_index + size_step) != (processed_right_road_edge_.size() - 1)) {
    const MathUtils::Point2D &p1 = processed_right_road_edge_[check_index];
    const MathUtils::Point2D &p2 = processed_right_road_edge_.back();
    if (areSegmentsIntersecting(predict_position, ego_position, p1, p2)) {
      std::copy(processed_right_road_edge_.begin() + check_index,
                processed_right_road_edge_.end(), std::back_inserter(result));
      // std::cerr << "[PART_2]segmentsIntersect is true" << std::endl;
      return result;
    }
  }
  // std::cerr << "segmentsIntersect is FALSE" << std::endl;

  return result;
}

MathUtils::Point2D FreeSpaceManager::predict_ego_position_at_time(
    double time, const PlanningPoint &traj_point) {
  MathUtils::Point2D prediction_position;

  double ego_speed = traj_point.velocity;
  double ego_theta = traj_point.theta;
  MathUtils::Point2D ego_position = traj_point.position;
  double ego_curva = traj_point.curva;
  double ego_a = traj_point.acceleration;

  double deltaTheta = ego_curva * ego_speed * time;
  double intermidiate_theta = ego_theta + 0.5 * time * ego_curva * ego_speed;

  prediction_position.x =
      ego_position.x +
      time * (ego_speed + 0.5 * time * ego_a) * std::cos(intermidiate_theta);
  prediction_position.y =
      ego_position.y +
      time * (ego_speed + 0.5 * time * ego_a) * std::sin(intermidiate_theta);

  return prediction_position;
}

bool FreeSpaceManager::is_point_in_freespace(const MathUtils::Point2D &point) {
  int n = freespace_polygon_.size();
  bool inside = false;
  for (int i = 0, j = n - 1; i < n; j = i++) {
    if (((freespace_polygon_[i].y > point.y) !=
         (freespace_polygon_[j].y > point.y)) &&
        (point.x < (freespace_polygon_[j].x - freespace_polygon_[i].x) *
                           (point.y - freespace_polygon_[i].y) /
                           (freespace_polygon_[j].y - freespace_polygon_[i].y) +
                       freespace_polygon_[i].x)) {
      inside = !inside;
    }
  }

  return inside;
}

}  // namespace EnvSim