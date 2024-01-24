//
// Created by SENSETIME\fengxiaotong on 23-12-8.
//

#ifndef ALGORITHM_ALGORITHM_ENV_SIMULATER_ENV_SIMULATER_H_
#define ALGORITHM_ALGORITHM_ENV_SIMULATER_ENV_SIMULATER_H_

#include "free_space.h"
#include "lane.h"
#include "obstacle.h"

namespace EnvSim {
struct PointInfo{
  MathUtils::Point2D point;
  double theta = 0.0;
  double curvature = 0.0;
  bool is_on_left = true;
};
class EnvSimulator {
public:
  void update_case_data(const std::string &file_path);

  std::vector<std::vector<MathUtils::Point2D>> get_all_lanes_center_points() {
    return lane_manager_.get_all_lanes_center_points();
  };

  std::vector<Obstacle> get_all_obstacle() {
    std::vector<Obstacle> res;
    for (auto obj : obstacle_manager_.obstacle_map) {
      res.emplace_back(obj.second);
    }
    return res;
  }

  const std::vector<std::vector<MathUtils::Point2D>> &
  get_all_road_edge_points() {
    return freespace_manager_.get_all_road_edge_points();
  }

  const std::vector<MathUtils::Point2D> &get_freespace_points_by_roi() {
    return freespace_manager_.get_freespace_points_by_roi();
  }

  const EllipseParam &get_freespace_ellipse_param() {
    return freespace_manager_.get_ellipse_param();
  }

  const VehicleInfo &get_ego_car() {
    return freespace_manager_.get_vehicle_info();
  }

  std::vector<MathUtils::Point2D>
  get_freespace_right_points_by_ttc(double time,
                                    const PlanningPoint &traj_point) {
    return freespace_manager_.get_freespace_right_points_by_ttc(time,
                                                                traj_point);
  }

  std::vector<MathUtils::Point2D>
  get_freespace_left_points_by_ttc(double time,
                                   const PlanningPoint &traj_point) {
    return freespace_manager_.get_freespace_left_points_by_ttc(time,
                                                               traj_point);
  }

  std::vector<MathUtils::Point2D>
  get_right_border_use_ellipse_model(const PlanningPoint &traj_point) {
    return freespace_manager_.get_right_border_use_ellipse_model(traj_point);
  }

  std::vector<MathUtils::Point2D>
  get_left_border_use_ellipse_model(const PlanningPoint &traj_point) {
    return freespace_manager_.get_left_border_use_ellipse_model(traj_point);
  }

  MathUtils::Point2D
  get_predict_ego_position_at_time(double time,
                                   const PlanningPoint &traj_point) {
    return freespace_manager_.predict_ego_position_at_time(time, traj_point);
  }

  const ObstacleManager &get_obstacle_mgr() const { return obstacle_manager_; }

  FreeSpaceManager &get_freespace_manager() { return freespace_manager_; }

  MathUtils::Point2D get_nearest_point(const MathUtils::Point2D &pos);

  double get_lane_s(const MathUtils::Point2D &pos);

  double get_nearest_point_curva(const MathUtils::Point2D &pos);

  double get_next_point_curva(const MathUtils::Point2D &pos);

  PointInfo get_nearest_point_info(const MathUtils::Point2D &pos);
private:
  LaneManager lane_manager_;
  ObstacleManager obstacle_manager_;
  FreeSpaceManager freespace_manager_;
};
} // namespace EnvSim
#endif // ALGORITHM_ALGORITHM_ENV_SIMULATER_ENV_SIMULATER_H_
