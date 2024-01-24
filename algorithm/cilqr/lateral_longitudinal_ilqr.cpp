//
// Created by SENSETIME\fengxiaotong on 23-12-13.
//
#include "memory"
#include "iostream"
#include "lateral_longitudinal_ilqr.h"

void LateralLongitudinalMotion::init(const std::string& file_path, 
                                     const PlanningPoint &planning_point){
  input_.env.update_case_data(file_path);
  input_.planning_origin = planning_point;

};

void LateralLongitudinalMotion::use_idm_model_get_va_ref(const double & init_speed,
                                                         const double & init_acc,
                                                         const double & desired_speed) {
  v_ref_.clear();
  a_ref_.clear();
  v_ref_.resize(31);
  a_ref_.resize(31);
  for (size_t i = 0; i < 31; i++) {
    if (i == 0) {
      v_ref_.at(i) = init_speed;
      a_ref_.at(i) = init_acc;
    } else {
      double idm_acc = 2.5 * (1.0 - std::pow(v_ref_.at(i-1) / desired_speed, 4));
      idm_acc = std::max(-3.5, std::min(idm_acc, 3.5));
      a_ref_.at(i) = idm_acc;
      v_ref_.at(i) = v_ref_.at(i-1) + a_ref_.at(i) * 0.2;
    }
  }
  if (!a_ref_.empty()) {
    a_ref_.erase(a_ref_.begin());
    a_ref_.push_back(a_ref_.back());
//    for (int i = 0; i < a_ref_.size(); i++) {
//        std::cout << "a ref is: " << a_ref_.at(i) << ", index :" << i << std::endl;
//    }
  }
  
  // for (size_t i = 0; i < v_ref_.size(); i++) {
  //   std::cout << "v ref is: " << v_ref_.at(i)
  //             << ", a ref is : " << a_ref_.at(i)
  //             << ", index : " << i << std::endl;
  // }
}

void LateralLongitudinalMotion::execute() {

  // 1.0 init solver
  solver_.init();
  auto env_ptr = std::make_shared<EnvSim::EnvSimulator>(input_.env);
  solver_.set_env(env_ptr);
  solver_.set_obstacle_cost();
  solver_.set_va_refs(v_ref_, a_ref_);
  solver_.condition_init();

  // 2.0 input planning origin & default action
  // x, y, v, theta
  Eigen::VectorXd init_state;
  init_state.resize(4);
  init_state << input_.planning_origin.position.x,
      input_.planning_origin.position.y, input_.planning_origin.velocity,
      input_.planning_origin.theta;
  // // a, w
  std::vector<Eigen::VectorXd> actions;

  init_traj_generator_.init();
  ILQR::StateVariable init_state_var;
  init_state_var.position.x = input_.planning_origin.position.x;
  init_state_var.position.y = input_.planning_origin.position.y;
  init_state_var.theta = input_.planning_origin.theta;
  init_state_var.velocity = input_.planning_origin.velocity;
  init_traj_generator_.set_init_state(init_state_var);
  ILQR::ControlVariable init_control_var;
  init_control_var.acceleration = input_.planning_origin.acceleration;
  init_control_var.omega = input_.planning_origin.omega;
  init_traj_generator_.set_init_control(init_control_var);
  init_traj_generator_.set_env(env_ptr);
  init_traj_generator_.set_obstacle();
  if (init_traj_generator_.gen_traj() != FuncStatus::FuncSucceed) {
    Eigen::VectorXd init_action;
    init_action.resize(2);
    init_action << input_.planning_origin.acceleration,
        input_.planning_origin.omega;
    actions.resize(30, init_action);
  } else {
    auto init_traj = init_traj_generator_.get_traj();
    for (int i = 0; i < 30; ++i) {
      auto &traj_pt = init_traj[i];
      Eigen::VectorXd action;
      action.resize(2);
      action << traj_pt.control.acceleration, traj_pt.control.omega;
      actions.emplace_back(action);
    }
  }

  // 3.0 solve
  if (solver_.solve(init_state, actions) != FuncStatus::FuncSucceed) {
    std::cout<< "solve failed" << std::endl;
    return;
  }

  // 4.0 get result
  auto state_new_space = solver_.get_x_space();
  new_trajectory_.clear();
  PlanningPoint pt = input_.planning_origin;
  for(auto& state : state_new_space){
    pt.position.x = state[0];
    pt.position.y = state[1];
    pt.velocity = state[2];
    pt.theta = state[3];
    pt.acceleration = 0.0;
    pt.omega = 0.0;
    new_trajectory_.emplace_back(pt);
  }
  auto control_new_space = solver_.get_u_space();
  for (int i = 0; i < control_new_space.size(); ++i) {
    auto& control = control_new_space[i];
    auto& pt =  new_trajectory_[i];
    pt.acceleration = control[0];
    pt.omega = control[1];
    if (pt.velocity > 1e-4) {
      pt.curva = pt.omega / pt.velocity;
    } else {
      pt.curva = 0.0;
    }
  }
  std::cout << "solve finished, new trajectory size : "
            << new_trajectory_.size() << std::endl;

  auto init_space = solver_.get_x_init_space();
  init_trajectory_.clear();
  for(auto& state : init_space){
    pt.position.x = state[0];
    pt.position.y = state[1];
    pt.velocity = state[2];
    pt.theta = state[3];
    init_trajectory_.emplace_back(pt);
  }
  auto init_u_space = solver_.get_u_init_space();
  for (int i = 0; i < init_u_space.size(); ++i) {
    auto &control = init_u_space[i];
    auto &pt = init_trajectory_[i];
    pt.acceleration = control[0];
    pt.omega = control[1];
    if (pt.velocity > 1e-4) {
      pt.curva = pt.omega / pt.velocity;
    } else {
      pt.curva = 0.0;
    }
  }
  std::cout << "solve finished, init trajectory size : "
            << init_trajectory_.size() << std::endl;

  // 5.0 get statistic
  auto solver_stat_list = solver_.get_iter_stat_list();
  iter_stat_list_.clear();
  iter_stat_list_.reserve(solver_stat_list.size());
  for (const auto& solver_stat : solver_stat_list) {
    std::vector<PlanningPoint> new_trajectory;
    auto& state_space = solver_stat.x_space;
    PlanningPoint pt = input_.planning_origin;
    for(auto& state : state_space){
      pt.position.x = state[0];
      pt.position.y = state[1];
      pt.velocity = state[2];
      pt.theta = state[3];
      pt.acceleration = 0.0;
      pt.omega = 0.0;
      new_trajectory.emplace_back(pt);
    }
    auto& control_space = solver_stat.u_space;
    for (int i = 0; i < control_space.size(); ++i) {
      auto& control = control_space[i];
      auto &pt = new_trajectory[i];
      pt.acceleration = control[0];
      pt.omega = control[1];
      if (pt.velocity > 1e-4) {
        pt.curva = pt.omega / pt.velocity;
      } else {
        pt.curva = 0.0;
      }
    }

    LatLongiMotionIterationStatistic motion_stat;
    motion_stat.trajectory = std::move(new_trajectory);
    motion_stat.total_cost = solver_stat.total_cost;
    iter_stat_list_.emplace_back(motion_stat);
  }
}

void LateralLongitudinalMotion::execute_tree() {
  // 1.0 update input
  auto env_ptr = std::make_shared<EnvSim::EnvSimulator>(input_.env);

  // 1.1 input planning origin
  // 1.2 input default action
  Eigen::VectorXd init_state;
  init_state.resize(6);
  init_state << input_.planning_origin.position.x,
      input_.planning_origin.position.y, input_.planning_origin.velocity,
      input_.planning_origin.theta, input_.planning_origin.acceleration,
      input_.planning_origin.omega;
  std::vector<Eigen::VectorXd> actions;

  for (int i =0; i< 30 ; i++){
    Eigen::VectorXd tmp;
    tmp.resize(2);
    tmp.setZero();
    actions.emplace_back(tmp);
  }
//  init_traj_generator_.init();
//  ILQR::StateVariable init_state_var;
//  init_state_var.position.x = input_.planning_origin.position.x;
//  init_state_var.position.y = input_.planning_origin.position.y;
//  init_state_var.theta = input_.planning_origin.theta;
//  init_state_var.velocity = input_.planning_origin.velocity;
//  init_traj_generator_.set_init_state(init_state_var);
//  ILQR::ControlVariable init_control_var;
//  init_control_var.acceleration = input_.planning_origin.acceleration;
//  init_control_var.omega = input_.planning_origin.omega;
//  init_traj_generator_.set_init_control(init_control_var);
//  init_traj_generator_.set_env(env_ptr);
//  init_traj_generator_.set_obstacle();
//  if (init_traj_generator_.gen_traj() != FuncStatus::FuncSucceed) {
//    Eigen::VectorXd init_action;
//    init_action.resize(2);
//    init_action.setZero();
//    actions.resize(30, init_action);
//  } else {
//    const auto& init_traj = init_traj_generator_.get_traj();
//    for (int i = 0; i < 30; ++i) {
//      const auto &traj_pt = init_traj[i];
//      const auto &traj_pt_next = init_traj[i + 1];
//      double jerk = (traj_pt_next.control.acceleration - traj_pt.control.acceleration) / 0.2;
//      double omega_dot = (traj_pt_next.control.omega - traj_pt.control.omega) / 0.2;
//      Eigen::VectorXd action;
//      action.resize(2);
//      action << jerk, omega_dot;
//      actions.emplace_back(action);
//      // std::cout << "init jerk " << jerk << " omega_dot " << omega_dot << std::endl;
//    }
//  }

  // 1.3 init param
  ILQR::ILQRParam param;

  // 1.4 init solver
  tree_solver_.init(env_ptr,actions,init_state,param);
  auto begin = std::chrono::high_resolution_clock::now();
  // 2.0 solve
  if (tree_solver_.solve() != FuncStatus::FuncSucceed) {
    std::cout<< "solve failed" << std::endl;
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
  std::vector<PlanningPoint> trajectory;
  PlanningPoint pt = input_.planning_origin;
  for(int i=0; i< state_space_list.size(); i++){
    trajectory.clear();
    for(auto& state : state_space_list[i]){
      pt.position.x = state[0];
      pt.position.y = state[1];
      pt.velocity = state[2];
      pt.theta = state[3];
      pt.acceleration = state[4];
      pt.omega = state[5];
      trajectory.emplace_back(pt);
    }
    for (int j = 0; j< action_space_list[i].size(); j++) {
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

  std::cout<< " tree trajectory finish" << std::endl;
}