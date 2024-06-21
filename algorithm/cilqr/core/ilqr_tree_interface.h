//
// Created by SENSETIME\fengxiaotong on 24-1-26.
//

#pragma once
#include "vector"
#include "memory"
#include "unordered_map"
#include "utils.h"

namespace ILQR {

using FuncStatus = MathUtils::FuncStatus;
template <typename Section>
class FuncReturn {
 public:
  FuncStatus first;
  Section second;

  FuncReturn(const FuncStatus& f, const Section& s) : first(f), second(s) {}
};

struct InterfacePointInfo {
  InterfacePointInfo() = default;
  MathUtils::Point2D point;
  double theta = 0.0;
  double curvature = 0.0;
  bool is_on_left = true;
  double speed_limit = 16.67;
};

struct InterfaceVehicleInfo{
  InterfaceVehicleInfo() = default;
  MathUtils::Point2D point;
  double theta = 0.0;
  double default_omega = 0.0;
};

struct InterfacePursuitPointInfo{
  InterfacePursuitPointInfo() = default;
  MathUtils::Point2D point;
  double omega = 0.0;
  double speed_limit = 0.0;
};

enum ObstacleType{
  Real = 0,
  StopLine = 1,
  RoadEdge = 2
};

struct ILQRObstacleTrajectoryPoint {
  double relative_time;
  double theta;
  MathUtils::Point2D position;
  std::vector<MathUtils::Point2D> polygon;
  double belief = 0.0;
  double s = 0.0;
  double length = 0.0;
  double width = 0.0;
  MathUtils::Point2D v;
  MathUtils::Point2D a;
  ObstacleType type = Real;
};

class ILQRObstacleInterface {
 public:
  virtual ~ILQRObstacleInterface() = default;
  //  virtual FuncReturn<double> get_length() const = 0;
  //  virtual FuncReturn<double> get_width() const = 0;
  //  virtual FuncReturn<int> get_id() const = 0;
  std::vector<ILQRObstacleTrajectoryPoint> trajectory_points;
};

class ILQRObstacleMgrInterface
    : public std::unordered_map<int, std::shared_ptr<ILQRObstacleInterface>> {
 public:
};

class ILQREnvInterface {
 public:
  virtual ~ILQREnvInterface() = default;
  //  virtual FuncReturn<MathUtils::Point2D> get_nearest_point(const
  //  MathUtils::Point2D &pos) = 0;
  //  virtual FuncReturn<double> get_next_point_curva(const MathUtils::Point2D
  //  &pos) = 0;
  virtual FuncReturn<InterfacePointInfo> get_nearest_point_info(
      const MathUtils::Point2D& pos) = 0;
  virtual FuncReturn<double> get_lane_s(const MathUtils::Point2D& pos) = 0;
  virtual FuncReturn<InterfacePursuitPointInfo> get_pursuit_point_info(const InterfaceVehicleInfo& vehicle_info) = 0;
  std::shared_ptr<ILQRObstacleMgrInterface> obstacle_mgr_interface_ = nullptr;
};
}
