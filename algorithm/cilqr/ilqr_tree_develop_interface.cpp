//
// Created by SENSETIME\fengxiaotong on 24-1-30.
//
#include "ilqr_tree_develop_interface.h"
namespace ILQR {
void ILQR::ILQRDevelopEnvInterface::init(
    std::shared_ptr<EnvSim::EnvSimulator>& env_simulator) {
  env_simulator_ = env_simulator;
  auto develop_obs_ptr = std::make_shared<ILQRDevelopObstacleMgrInterface>();
  develop_obs_ptr->init(env_simulator);
  obstacle_mgr_interface_ =
      std::dynamic_pointer_cast<ILQRDevelopObstacleMgrInterface>(
          develop_obs_ptr);
}

// FuncReturn<double> ILQRDevelopEnvInterface::get_next_point_curva(const
// MathUtils::Point2D &pos) {
//  if (env_simulator_ == nullptr){
//    return FuncReturn<double>{FuncStatus::FuncFailed,0.0};
//  }
//  double curvature = env_simulator_->get_next_point_curva(pos);
//
//  return FuncReturn<double>{FuncStatus::FuncSucceed,curvature};
//}

FuncReturn<InterfacePointInfo> ILQRDevelopEnvInterface::get_nearest_point_info(
    const MathUtils::Point2D& pos) {
  InterfacePointInfo tmp;
  if (env_simulator_ == nullptr) {
    return FuncReturn<InterfacePointInfo>{FuncStatus::FuncFailed, tmp};
  }
  auto info = env_simulator_->get_nearest_point_info(pos);

  tmp.point = info.point;
  tmp.theta = info.theta;
  tmp.curvature = info.curvature;
  tmp.is_on_left = info.is_on_left;
  tmp.speed_limit = info.speed_limit;
  return FuncReturn<InterfacePointInfo>{FuncStatus::FuncSucceed, tmp};
}

// for adapt dlp interface, "frame_count" was not use
FuncReturn<InterfacePointInfo> ILQRDevelopEnvInterface::get_nearest_point_info(
    const MathUtils::Point2D &pos, const int &frame_count) {
  InterfacePointInfo tmp;
  if (env_simulator_ == nullptr) {
    return FuncReturn<InterfacePointInfo>{FuncStatus::FuncFailed, tmp};
  }
  auto info = env_simulator_->get_nearest_point_info(pos);

  tmp.point = info.point;
  tmp.theta = info.theta;
  tmp.curvature = info.curvature;
  tmp.is_on_left = info.is_on_left;
  tmp.speed_limit = info.speed_limit;
  return FuncReturn<InterfacePointInfo>{FuncStatus::FuncSucceed, tmp};
}

void ILQRDevelopObstacleInterface::init(int id,
                                        const EnvSim::Obstacle& obs,
                                        EnvSim::EnvSimulator& env_simulator) {
  source_ = std::make_shared<EnvSim::Obstacle>(obs);
  trajectory_points.clear();
  int index = 0;
  for (auto& pt : source_->trajectory_points) {
    ILQRObstacleTrajectoryPoint tmp;
    tmp.relative_time = pt.relative_time;
    tmp.theta = pt.theta;
    tmp.polygon = pt.polygon;
    tmp.position = pt.position;
    tmp.length = obs.get_length();
    tmp.width = obs.get_width();
    tmp.belief = 0.0;
    auto nearest_point = env_simulator.get_nearest_point(
        MathUtils::Point2D{pt.position.x, pt.position.y});
    double dis = std::hypot(pt.position.x - nearest_point.x,
                            pt.position.y - nearest_point.y);

    if (dis < 1.0) {
      tmp.belief = 1.0;
    }
    MathUtils::Point2D v;
    MathUtils::Point2D a;
    tmp.s = env_simulator.get_lane_s(
        MathUtils::Point2D{pt.position.x, pt.position.y});
    ;

    tmp.v.x = pt.v * std::cos(pt.theta);
    tmp.v.y = pt.v * std::sin(pt.theta);

    double tmp_a = 0.0;
    if (index < 30) {
      tmp_a = (source_->trajectory_points[index + 1].v -
               source_->trajectory_points[index].v) /
              0.2;
    }
    tmp.a.x = tmp_a * std::cos(pt.theta);
    tmp.a.y = tmp_a * std::sin(pt.theta);
    trajectory_points.emplace_back(tmp);
  }
}

// FuncReturn<MathUtils::Point2D>
// ILQRDevelopEnvInterface::get_nearest_point(const MathUtils::Point2D &pos) {
//  MathUtils::Point2D tmp;
//  if (env_simulator_ == nullptr){
//    return FuncReturn<MathUtils::Point2D>{FuncStatus::FuncFailed,tmp};
//  }
//  auto pt = env_simulator_->get_nearest_point(pos);
//
//  return FuncReturn<MathUtils::Point2D>{FuncStatus::FuncSucceed,pt};
//}

FuncReturn<double> ILQRDevelopEnvInterface::get_lane_s(
    const MathUtils::Point2D& pos) {
  if (env_simulator_ == nullptr) {
    return FuncReturn<double>{FuncStatus::FuncFailed, 0.0};
  }
  auto res = env_simulator_->get_lane_s(pos);

  return FuncReturn<double>{FuncStatus::FuncSucceed, res};
}
}
