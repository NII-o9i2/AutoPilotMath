//
// Created by SENSETIME\fengxiaotong on 23-12-8.
//
#include "env_simulator.h"
#include "3rd/json.hpp"
#include "fstream"
#include "iostream"
#include "obstacle.h"
namespace EnvSim {

void EnvSimulator::update_ref_points(const std::vector<MathUtils::Point2D>& ref_pathpoints) {
  if (ref_pathpoints.empty()) {
    return;
  }
  std::vector<Lane> lanes;
  Lane tmp_lane(0, ref_pathpoints);
  tmp_lane.update_center_points(false);
  lanes.emplace_back(tmp_lane);
  lane_manager_.update_lanes(lanes);
  return;
}

void EnvSimulator::update_case_data(const std::string &file_path) {
  std::ifstream file(file_path);
  if (!file.is_open()) {
    std::cerr << "file not exist." << std::endl;
  }
  nlohmann::json json_data;
  file >> json_data;

  std::string name = json_data["name"];
  std::cout << "---- loading " << name << " case ----" << std::endl;
  // 1. update lanes data
  std::vector<Lane> lanes;
  for (auto &lane : json_data["lanes"]) {
    std::vector<MathUtils::Point2D> raw_points;
    raw_points.clear();
    for (auto &pt : lane["center_points"]) {
      raw_points.emplace_back(pt.front(), pt.back());
    }
    Lane tmp_lane(lane["relative_id"], raw_points);
    tmp_lane.update_center_points(true);
    std::cout << "---- loading relative " << lane["relative_id"] << " lane "
              << std::endl;
    lanes.emplace_back(tmp_lane);
  }
  lane_manager_.update_lanes(lanes);

  // 2.0 update obstacle data
  obstacle_manager_.obstacle_map.clear();
  for (auto &obj : json_data["objects"]) {
    std::vector<RawTrajectoryPoint> tmp_traj;
    tmp_traj.clear();
    for (auto &pt : obj["position_std"]) {
      tmp_traj.emplace_back(pt["lane"], pt["s"], pt["l"]);
    }
    Obstacle tmp = {obj["id"],         obj["length"], obj["width"],
                    obj["init_theta"], obj["t_std"],  obj["v_std"],
                    tmp_traj};
    std::cout << "---- loading obstacle " << obj["id"] << std::endl;
    tmp.construct(lane_manager_);
    obstacle_manager_.obstacle_map.insert({obj["id"], tmp});
  }

  // 3. update road edge data
  std::vector<std::vector<MathUtils::Point2D>> road_edge_vec_;
  if (json_data.count("road_edges") == 0) {
    std::cout << "Field 'road_edges' does not exist." << std::endl;
    return;
  }
  for (const auto &road_edge : json_data["road_edges"]) {
    std::vector<MathUtils::Point2D> raw_points;
    for (const auto &pt : road_edge["points"]) {
      raw_points.emplace_back(pt.front(), pt.back());
    }
    road_edge_vec_.emplace_back(raw_points);
  }

  // set vehicle initial info
  VehicleInfo vehicle_info;
  vehicle_info.vehicle_speed = 10.0;
  vehicle_info.vehicle_position = {0, -2.0};
  vehicle_info.theta = 0.0;
  vehicle_info.length = 4.9;
  vehicle_info.width = 1.9;
  vehicle_info.curvature = 0.0;
  freespace_manager_.update(road_edge_vec_, vehicle_info);
  std::cerr << "load suceeful" << std::endl;
}

double EnvSimulator::get_lane_s(const MathUtils::Point2D &pos) {
  auto &lane = lane_manager_.get_lane(0);
  return lane.get_s(pos);
}

MathUtils::Point2D EnvSimulator::get_nearest_point(
    const MathUtils::Point2D &pos) {
  //  auto &lane = lane_manager_.get_lane(0);
  //  auto &center_points = lane.get_center_points();
  //  double min_dis = 0.0;
  //  MathUtils::Point2D res;
  //  res = center_points.front();
  //  min_dis =
  //      (res.x - pos.x) * (res.x - pos.x) + (res.y - pos.y) * (res.y - pos.y);
  //  for (auto &point : center_points) {
  //    double tmp = (point.x - pos.x) * (point.x - pos.x) +
  //                 (point.y - pos.y) * (point.y - pos.y);
  //    if (tmp < min_dis) {
  //      min_dis = tmp;
  //      res = point;
  //    }
  //  }
  //  return res;
  auto &lane = lane_manager_.get_lane(0);
  auto &center_points = lane.get_center_points();
  double min_dis = 0.0;
  MathUtils::Point2D res;
  res = center_points.front();
  min_dis =
      (res.x - pos.x) * (res.x - pos.x) + (res.y - pos.y) * (res.y - pos.y);
  int index_left = 0;
  int index_right = 0;
  for (int i = 0; i < center_points.size(); i++) {
    double tmp = (center_points[i].x - pos.x) * (center_points[i].x - pos.x) +
                 (center_points[i].y - pos.y) * (center_points[i].y - pos.y);
    if (tmp < min_dis) {
      min_dis = tmp;
      res = center_points[i];
      index_left = i;
    }
  }
  if (index_left + 1 < center_points.size()) {
    index_right = index_left + 1;
  } else {
    index_right = index_left;
    index_left--;
  }
  if (index_left < 0) {
    return res;
  }
  double pa_x = pos.x - center_points[index_left].x;
  double pa_y = pos.y - center_points[index_left].y;
  double ba_x = center_points[index_right].x - center_points[index_left].x;
  double ba_y = center_points[index_right].y - center_points[index_left].y;

  double ba_len = std::hypot(ba_x, ba_y);
  if (ba_len < 1e-5) {
    return res;
  }
  double ca_len = (pa_x * ba_x + pa_y * ba_y) / ba_len;
  res.x = center_points[index_left].x + ba_x * ca_len / ba_len;
  res.y = center_points[index_left].y + ba_y * ca_len / ba_len;

  return res;
}

PointInfo EnvSimulator::get_nearest_point_info(const MathUtils::Point2D &pos) {
  auto &lane = lane_manager_.get_lane(0);
  auto &center_points = lane.get_center_points();
  double min_dis = 0.0;
  PointInfo res;
  // get point
  res.point = center_points.front();
  min_dis = (res.point.x - pos.x) * (res.point.x - pos.x) +
            (res.point.y - pos.y) * (res.point.y - pos.y);
  int index_left = 0;
  int index_right = 0;
  for (int i = 0; i < center_points.size(); i++) {
    double tmp = (center_points[i].x - pos.x) * (center_points[i].x - pos.x) +
                 (center_points[i].y - pos.y) * (center_points[i].y - pos.y);
    if (tmp < min_dis) {
      min_dis = tmp;
      res.point = center_points[i];
      index_left = i;
    }
  }
  if (index_left + 1 < center_points.size()) {
    index_right = index_left + 1;
  } else {
    index_right = index_left;
    index_left--;
  }
  if (index_left < 0) {
    std::cout << "line too short!!" << std::endl;
    return res;
  }
  auto &curvature = lane.get_curva();
  auto &speed_limit = lane.get_speed_limit();
  if (std::fabs(center_points[index_right].x - center_points[index_left].x) <
      1e-6) {
    res.theta = center_points[index_right].y > center_points[index_left].y
                    ? 0.5 * M_PI
                    : -0.5 * M_PI;
  } else {
    res.theta =
        std::atan((center_points[index_right].y - center_points[index_left].y) /
                  (center_points[index_right].x - center_points[index_left].x));
  }
  res.curvature = curvature[index_right];
  res.speed_limit = speed_limit[index_right];
  double pa_x = pos.x - center_points[index_left].x;
  double pa_y = pos.y - center_points[index_left].y;
  double ba_x = center_points[index_right].x - center_points[index_left].x;
  double ba_y = center_points[index_right].y - center_points[index_left].y;

  res.is_on_left = ba_x * pa_y - ba_y * pa_x > 0;

  double ba_len = std::hypot(ba_x, ba_y);
  if (ba_len < 1e-5) {
    return res;
  }
  double ca_len = (pa_x * ba_x + pa_y * ba_y) / ba_len;
  res.point.x = center_points[index_left].x + ba_x * ca_len / ba_len;
  res.point.y = center_points[index_left].y + ba_y * ca_len / ba_len;

  return res;
}

double EnvSimulator::get_nearest_point_curva(const MathUtils::Point2D &pos) {
  auto &lane = lane_manager_.get_lane(0);
  auto &center_points = lane.get_center_points();
  auto &first_point = center_points.front();
  double min_dis = 0.0;
  int index = 0;
  min_dis = (first_point.x - pos.x) * (first_point.x - pos.x) +
            (first_point.y - pos.y) * (first_point.y - pos.y);
  for (int i = 1; i < center_points.size(); ++i) {
    auto &point = center_points[i];
    double tmp = (point.x - pos.x) * (point.x - pos.x) +
                 (point.y - pos.y) * (point.y - pos.y);
    if (tmp < min_dis) {
      min_dis = tmp;
      index = i;
    }
  }
  auto &curva = lane.get_curva();
  return curva[index];
}

double EnvSimulator::get_next_point_curva(const MathUtils::Point2D &pos) {
  auto &lane = lane_manager_.get_lane(0);
  auto &center_points = lane.get_center_points();
  auto &first_point = center_points.front();
  double min_dis = 0.0;
  int index = 0;
  min_dis = (first_point.x - pos.x) * (first_point.x - pos.x) +
            (first_point.y - pos.y) * (first_point.y - pos.y);
  for (int i = 1; i < center_points.size(); ++i) {
    auto &point = center_points[i];
    double tmp = (point.x - pos.x) * (point.x - pos.x) +
                 (point.y - pos.y) * (point.y - pos.y);
    if (tmp < min_dis) {
      min_dis = tmp;
      index = i;
    }
  }
  auto &curva = lane.get_curva();
  if (index < center_points.size() - 2) {
    return curva[index + 2];
  } else if (index < center_points.size() - 1) {
    return curva[index + 1];
  }
  return curva[index];
}
}  // namespace EnvSim