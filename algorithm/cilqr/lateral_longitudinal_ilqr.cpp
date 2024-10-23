//
// Created by SENSETIME\fengxiaotong on 23-12-13.
//
#include "memory"
#include "iostream"
#include "lateral_longitudinal_ilqr.h"
#include "ilqr_tree_develop_interface.h"

using FuncStatus = MathUtils::FuncStatus;
void LateralLongitudinalMotion::init(const std::string& file_path,
                                     const PlanningPoint& planning_point,
                                     bool enable_dodge) {
  enable_dodge_ = enable_dodge;
  input_.env.update_case_data(file_path);
  input_.planning_origin = planning_point;
};

void LateralLongitudinalMotion::init(const std::string& file_path,
                                     const PlanningPoint& planning_point) {
  enable_dodge_ = false;
  input_.env.update_case_data(file_path);
  input_.planning_origin = planning_point;
};

void LateralLongitudinalMotion::init(const std::vector<MathUtils::Point2D>& ref_pathpoints,
                                     const PlanningPoint& planning_point) {
  enable_dodge_ = false;
  input_.env.update_ref_points(ref_pathpoints);
  input_.planning_origin = planning_point;
};

void LateralLongitudinalMotion::use_idm_model_get_va_ref(
    const double& init_speed,
    const double& init_acc,
    const double& desired_speed) {
  v_ref_.clear();
  a_ref_.clear();
  v_ref_.resize(31);
  a_ref_.resize(31);
  for (size_t i = 0; i < 31; i++) {
    if (i == 0) {
      v_ref_.at(i) = init_speed;
      a_ref_.at(i) = init_acc;
    } else {
      double idm_acc =
          2.5 * (1.0 - std::pow(v_ref_.at(i - 1) / desired_speed, 4));
      idm_acc = std::max(-3.5, std::min(idm_acc, 3.5));
      a_ref_.at(i) = idm_acc;
      v_ref_.at(i) = v_ref_.at(i - 1) + a_ref_.at(i) * 0.2;
    }
  }
  if (!a_ref_.empty()) {
    a_ref_.erase(a_ref_.begin());
    a_ref_.push_back(a_ref_.back());
    //    for (int i = 0; i < a_ref_.size(); i++) {
    //        std::cout << "a ref is: " << a_ref_.at(i) << ", index :" << i <<
    //        std::endl;
    //    }
  }

  // for (size_t i = 0; i < v_ref_.size(); i++) {
  //   std::cout << "v ref is: " << v_ref_.at(i)
  //             << ", a ref is : " << a_ref_.at(i)
  //             << ", index : " << i << std::endl;
  // }
}

void LateralLongitudinalMotion::execute_tree() {
  // 1.0 update input
  auto interface_ptr = std::make_shared<ILQR::ILQRDevelopEnvInterface>();
  auto env_ptr = std::make_shared<EnvSim::EnvSimulator>(input_.env);
  interface_ptr->init(env_ptr);

  // 1.1 input planning origin
  // 1.2 input default action
  Eigen::VectorXd init_state;
  init_state.resize(6);
  init_state << input_.planning_origin.position.x,
      input_.planning_origin.position.y, input_.planning_origin.velocity,
      input_.planning_origin.theta, input_.planning_origin.acceleration,
      input_.planning_origin.omega;
  std::vector<Eigen::VectorXd> actions;

  for (int i = 0; i < 30; i++) {
    Eigen::VectorXd tmp;
    tmp.resize(2);
    tmp.setZero();
    actions.emplace_back(tmp);
  }

  // 1.3 init param
  ILQR::ILQRParam param;
  if (enable_dodge_) {
    param.solver_func_type = ILQR::SolverFuncType::LonLatWithDodge;
  } else {
    param.solver_func_type = ILQR::SolverFuncType::LonLatWithoutDodge;
  }

  // 1.4 init solver
  tree_solver_.init(interface_ptr, actions, init_state, param);
  auto begin = std::chrono::high_resolution_clock::now();
  // 2.0 solve
  if (tree_solver_.solve() != FuncStatus::FuncSucceed) {
    std::cout << "solve failed" << std::endl;
    return;
  }
  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> duration = end - begin;
  std::cout << " **************** ilqr tree time: " << duration.count() * 1000
            << " ms " << std::endl;

  // 3.0 update output
  auto state_space_list = tree_solver_.get_tree_x_space();
  auto action_space_list = tree_solver_.get_tree_u_space();
  trajectory_tree_.clear();
  lon_debug_tree_.clear();
  lon_debug_tree_ = tree_solver_.get_tree_lon_debug_info();
  lat_debug_tree_.clear();
  lat_debug_tree_ = tree_solver_.get_tree_lat_debug_info();
  dodge_debug_tree_.clear();
  dodge_debug_tree_ = tree_solver_.get_tree_dodge_debug_info();
  std::vector<PlanningPoint> trajectory;
  PlanningPoint pt = input_.planning_origin;
  for (int i = 0; i < state_space_list.size(); i++) {
    trajectory.clear();
    for (auto& state : state_space_list[i]) {
      pt.position.x = state[0];
      pt.position.y = state[1];
      pt.velocity = state[2];
      pt.theta = state[3];
      pt.acceleration = state[4];
      pt.omega = state[5];
      trajectory.emplace_back(pt);
    }
    for (int j = 0; j < action_space_list[i].size(); j++) {
      auto& control = action_space_list[i][j];
      trajectory[j].jerk = control[0];
      trajectory[j].omega_dot = control[1];
      if (trajectory[j].velocity > 1e-4) {
        trajectory[j].curva = trajectory[j].omega / trajectory[j].velocity;
      } else {
        trajectory[j].curva = 0.0;
      }
    }
    trajectory_tree_.emplace_back(trajectory);
  }

  std::cout << " tree trajectory finish" << std::endl;
}