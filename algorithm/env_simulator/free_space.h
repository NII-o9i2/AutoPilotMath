#pragma once

#include <algorithm>
#include <iostream>
#include <map>
#include <memory>


#include "spline.h"
#include "utils.h"

namespace EnvSim {

struct FreeSpaceMangerOutput {
  // todo
};

struct PlanningPoint {
  MathUtils::Point2D position;
  double theta = 0.0;
  double velocity = 0.0;
  double acceleration = 0.0;
  double omega = 0.0; // d(theta) / dt
  double curva = 0.0; // omega / velocity = curva
  double jerk = 0.0;
  double omega_dot = 0.0;
};

enum class Direction { UNKNOW = 0, LEFT = 1, RIGHT = 2 };

struct VehicleInfo {
  double vehicle_speed = 0.0;
  MathUtils::Point2D vehicle_position;
  double theta = 0.0;
  double length = 0.0;
  double width = 0.0;
  double curvature = 0.0;
};

struct EllipseParam {
  double a = 0.0;
  double b = 0.0;
  MathUtils::Point2D center_point;
};

class FreeSpaceManager {
public:
  FreeSpaceManager() { output_ = std::make_shared<FreeSpaceMangerOutput>(); }

  ~FreeSpaceManager() = default;

  const std::shared_ptr<FreeSpaceMangerOutput> &get_output() const {
    return output_;
  }

  void update(
      const std::vector<std::vector<MathUtils::Point2D>> &road_edge_vec_input,
      const VehicleInfo &vehicle_info);

  std::map<Direction, std::vector<MathUtils::Point2D>>
  update_freespace_ttc_point();

  bool is_point_in_freespace(const MathUtils::Point2D &point);

  MathUtils::Point2D find_closest_point_at_boundary(
      const MathUtils::Point2D &point,
      const std::vector<MathUtils::Point2D> &boundary);

  std::map<Direction, std::vector<MathUtils::Point2D>>
  find_closest_point_at_both_side(const MathUtils::Point2D &point);

  MathUtils::Point2D
  predict_ego_position_at_time(double time, const PlanningPoint &traj_point);

  std::vector<MathUtils::Point2D>
  get_right_border_use_ellipse_model(const PlanningPoint &traj_point);

  std::vector<MathUtils::Point2D>
  get_left_border_use_ellipse_model(const PlanningPoint &traj_point);

  bool is_point_in_ellipse(const MathUtils::Point2D &road_edge_point,
                           const PlanningPoint &traj_point);

  // accessor
  const std::vector<std::vector<MathUtils::Point2D>> &
  get_all_road_edge_points() {
    return road_edge_processed_vec_;
  }

  const std::vector<MathUtils::Point2D> &get_freespace_points_by_roi() {
    return freespace_points_by_roi_;
  }

  std::vector<MathUtils::Point2D>
  get_freespace_left_points_by_ttc(double time,
                                   const PlanningPoint &traj_point);

  std::vector<MathUtils::Point2D>
  get_freespace_right_points_by_ttc(double time,
                                    const PlanningPoint &traj_point);

  const EllipseParam &get_ellipse_param() { return ellipse_param_; }

  const VehicleInfo &get_vehicle_info() { return vehicle_info_; }

private:
  // intermediate results

  // index 0:left road edge; index 1:right road edge
  std::vector<std::vector<MathUtils::Point2D>> road_edge_processed_vec_;
  std::vector<MathUtils::Point2D> raw_left_road_edge_;
  std::vector<MathUtils::Point2D> raw_right_road_edge_;
  std::vector<MathUtils::Point2D> processed_left_road_edge_;
  std::vector<MathUtils::Point2D> processed_right_road_edge_;
  std::vector<MathUtils::Point2D> freespace_polygon_;
  std::vector<MathUtils::Point2D> road_edge_interp_points_;
  std::vector<MathUtils::Point2D> freespace_points_by_roi_;
  std::vector<MathUtils::Point2D> freespace_points_by_ttc_;
  EllipseParam ellipse_param_;

  // inputs
  VehicleInfo vehicle_info_;

  // output
  std::shared_ptr<FreeSpaceMangerOutput> output_;
};

} // namespace EnvSim