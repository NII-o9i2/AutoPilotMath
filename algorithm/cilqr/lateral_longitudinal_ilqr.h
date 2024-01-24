//
// Created by SENSETIME\fengxiaotong on 23-12-13.
//

#ifndef ALGORITHM_ALGORITHM_CILQR_LATERAL_LONGITUDINAL_ILQR_H_
#define ALGORITHM_ALGORITHM_CILQR_LATERAL_LONGITUDINAL_ILQR_H_
#include "ilqr_solver.h"
#include "ilqr_tree_solver.h"
#include "init_traj_generator.h"
#include "utils.h"

using PlanningPoint = ILQR::PlanningPoint;

struct LatLongiMotionIterationStatistic {
  std::vector<PlanningPoint> trajectory;
  double total_cost;
};

struct LateralLongitudinalMotionInput {
  EnvSim::EnvSimulator env;
  PlanningPoint planning_origin;
};

class LateralLongitudinalMotion {
public:
  void init(const std::string &file_path, const PlanningPoint &planning_point);

  const EnvSim::EnvSimulator &get_env() const { return input_.env; }
  const PlanningPoint &get_planning_origin() const {
    return input_.planning_origin;
  }
  const std::vector<PlanningPoint> &get_init_trajectory() const {
    return init_trajectory_;
  }
  const std::vector<PlanningPoint> &get_new_trajectory() const {
    return new_trajectory_;
  }
  const std::vector<std::vector<PlanningPoint>> &get_trajectory_tree() const {
    return trajectory_tree_;
  }
  std::vector<MathUtils::Point2D> get_match_point() {
    return solver_.get_l_condition_match_points();
  }

  std::vector<double> get_ref_kappa() {
    return solver_.get_l_condition_ref_curva();
  }
  std::vector<double> get_ref_omega() {
    return solver_.get_l_condition_ref_omega();
  }

  void use_idm_model_get_va_ref(const double &init_speed,
                                const double &init_acc,
                                const double &desired_spd);
  std::vector<double> get_ref_v() {
    std::vector<double> res;
    res.clear();
    for (auto &l_condition : solver_.get_l_condition_data()) {
      res.emplace_back(l_condition.v_ref);
    }
    return res;
  };

  std::vector<double> get_ref_a() {
    std::vector<double> res;
    res.clear();
    for (auto &l_condition : solver_.get_l_condition_data()) {
      res.emplace_back(l_condition.a_ref);
    }
    return res;
  };
  std::vector<std::vector<TreeILQR::LonDebugInfo>>& get_lon_debug_tree(){
    return lon_debug_tree_;
  }
  std::vector<std::vector<TreeILQR::LatDebugInfo>>& get_lat_debug_tree(){
    return lat_debug_tree_;
  }

  const std::vector<ILQR::StepCondition> &get_l_condition() const {
    return solver_.get_l_condition_data();
  }

  std::vector<LatLongiMotionIterationStatistic> get_iter_stat_list() {
    return iter_stat_list_;
  }

  std::vector<std::vector<PlanningPoint>> get_init_traj_gen_failed_path_set() {
    std::vector<std::vector<PlanningPoint>> res;
    const auto &failed_path_set = init_traj_generator_.get_failed_path_set();
    for (const auto &failed_path : failed_path_set) {
      std::vector<PlanningPoint> path;
      for (const auto &failed_point : failed_path) {
        PlanningPoint point;
        point.position.x = failed_point.state.position.x;
        point.position.y = failed_point.state.position.y;
        point.theta = failed_point.state.theta;
        point.velocity = failed_point.state.velocity;
        point.acceleration = failed_point.control.acceleration;
        point.omega = failed_point.control.omega;
        path.emplace_back(point);
      }
      res.emplace_back(path);
    }
    return res;
  }

  bool get_init_traj_gen_success() {
    return init_traj_generator_.get_search_success();
  }

  void execute();

  void execute_tree();

private:
  ILQR::ILQRSpace solver_;
  ILQR::InitialTrajectoryGenerator init_traj_generator_;
  LateralLongitudinalMotionInput input_;
  std::vector<PlanningPoint> init_trajectory_;
  std::vector<PlanningPoint> new_trajectory_;
  std::vector<LatLongiMotionIterationStatistic> iter_stat_list_;
  std::vector<double> v_ref_;
  std::vector<double> a_ref_;

  TreeILQR::TrajectoryTreeManager tree_solver_;
  std::vector<std::vector<PlanningPoint>> trajectory_tree_;
  std::vector<std::vector<TreeILQR::LonDebugInfo>> lon_debug_tree_;
  std::vector<std::vector<TreeILQR::LatDebugInfo>> lat_debug_tree_;
};

#endif // ALGORITHM_ALGORITHM_CILQR_LATERAL_LONGITUDINAL_ILQR_H_
