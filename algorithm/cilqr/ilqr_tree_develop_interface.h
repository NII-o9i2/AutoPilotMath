//
// Created by SENSETIME\fengxiaotong on 24-1-30.
//
#pragma once
#include "unordered_map"
#include "core/ilqr_tree_interface.h"
#ifdef ENGINEER_OPTION

#else
#include "env_simulator.h"
#include "obstacle.h"
#endif

namespace ILQR {
class ILQRDevelopEnvInterface : public ILQREnvInterface {
 public:
  ILQRDevelopEnvInterface() = default;
  ~ILQRDevelopEnvInterface() override = default;
  void init(std::shared_ptr<EnvSim::EnvSimulator>& env_simulator);
  //  FuncReturn<double> get_next_point_curva(const MathUtils::Point2D &pos)
  //  override;
  FuncReturn<InterfacePointInfo> get_nearest_point_info(
      const MathUtils::Point2D& pos) override;
  //  FuncReturn<MathUtils::Point2D> get_nearest_point(const MathUtils::Point2D
  //  &pos) override;
  FuncReturn<double> get_lane_s(const MathUtils::Point2D& pos) override;

 private:
  std::shared_ptr<EnvSim::EnvSimulator> env_simulator_;
};

class ILQRDevelopObstacleInterface : public ILQRObstacleInterface {
 public:
  ~ILQRDevelopObstacleInterface() override = default;
  //  FuncReturn<double> get_length() const override{
  //    if (source_ == nullptr){
  //      return FuncReturn<double>{FuncStatus::FuncFailed,0.0};
  //    }
  //    return
  //    FuncReturn<double>{FuncStatus::FuncSucceed,source_->get_length()};
  //  };
  //  FuncReturn<double> get_width() const override{
  //    if (source_ == nullptr){
  //      return FuncReturn<double>{FuncStatus::FuncFailed,0.0};
  //    }
  //    return FuncReturn<double>{FuncStatus::FuncSucceed,source_->get_width()};
  //  };
  //  FuncReturn<int> get_id() const override{
  //    if (source_ == nullptr){
  //      return FuncReturn<int>{FuncStatus::FuncFailed,0};
  //    }
  //    return FuncReturn<int>{FuncStatus::FuncSucceed,source_->get_id()};
  //  }
  void init(int id,
            const EnvSim::Obstacle& obs,
            EnvSim::EnvSimulator& env_simulator);

 private:
  int id_ = 0;
  std::shared_ptr<EnvSim::Obstacle> source_;
};

class ILQRDevelopObstacleMgrInterface : public ILQRObstacleMgrInterface {
 public:
  ~ILQRDevelopObstacleMgrInterface() = default;

  void init(std::shared_ptr<EnvSim::EnvSimulator>& env_simulator) {
    env_simulator_ = env_simulator;
    auto obs = env_simulator_->get_obstacle_mgr();
    for (auto& ob : obs.obstacle_map) {
      auto tmp = std::make_shared<ILQRDevelopObstacleInterface>();
      tmp->init(ob.first, ob.second, *env_simulator);
      insert(std::pair<int, std::shared_ptr<ILQRDevelopObstacleInterface>>(
          ob.first, tmp));
    }
  };

 private:
  std::shared_ptr<EnvSim::EnvSimulator> env_simulator_;
};
}
