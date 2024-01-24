//
// Created by SENSETIME\fengxiaotong on 23-12-13.
//
#include "iostream"
#include "chrono"
#include "ilqr_solver.h"
#include "iostream"
#include "lateral_longitudinal_ilqr.h"
#include "ilqr_lon_condition.h"

int main(int argc, char* argv[]){
  ILQR::VehicleModelBicycle vehicle_model_bicycle;
  ILQR::ILQRParam param;
  param.delta_t = 0.2;

  vehicle_model_bicycle.update_parameter(param);

  Eigen::VectorXd state;
  state.resize(4);
  state << 3.0, 4.0, 20, 1.0;

  Eigen::VectorXd action;
  action.resize(2);
  action << 1.0, 0.02;
  Eigen::VectorXd next_state;
  next_state.resize(4);
  // steer angle model
  std::cout << " steer angle model " << std::endl;
  vehicle_model_bicycle.step(state, action, next_state);
  for (auto &p : next_state) {
    std::cout << " " << p << std::endl;
  }
  // kappa model
  action.resize(2);
  action << 1.0, 0.002;
  std::cout << " kappa model " << std::endl;
  vehicle_model_bicycle.step_kappa(state, action, next_state);
  for (auto &p : next_state) {
    std::cout << " " << p << std::endl;
  }

  LateralLongitudinalMotion motion;
  PlanningPoint point_init;
  point_init.position.x = 30.0;
  point_init.position.y = 5.8;
  point_init.theta = -0.1;
  point_init.velocity = 14.22;
  point_init.acceleration = 0.1;

  if (argc < 2){
    std::cout << "please input simulator case path" << std::endl;
    return 1;
  }
  auto start = std::chrono::high_resolution_clock::now();


  motion.init(argv[1],point_init);
  auto end_init = std::chrono::high_resolution_clock::now();
//  motion.execute();

  motion.execute_tree();
//  motion.init("/home/SENSETIME/fengxiaotong/ws/xviz/common_math/algorithm/env_simulator/data/test_case_2_straight_lanes_cruise.json",point_init);
//
//  motion.execute_tree();
  auto trajs_new = motion.get_trajectory_tree();

  auto end = std::chrono::high_resolution_clock::now();

  // 计算时间差
  std::chrono::duration<double> duration = end - start;
  std::chrono::duration<double> duration_init = end_init - start;

  // 输出执行时间
  std::cout << "执行时间：" << duration.count() << " 秒" << std::endl;
  std::cout << "init 执行时间：" << duration_init.count() << " 秒" << std::endl;


  ILQR::IDMParam idm_params;

  std::vector<double> cur_v = {40,20,80,100,80,60};
  std::vector<double> desired_v = {60,60,60,80,40,80};
  for (int i =0;i<cur_v.size();i++){
    idm_params.desired_spd = desired_v[i]/3.6;
    auto res = ILQR::LongitudinalOperator::calc_idm_acc(idm_params,0.0,cur_v[i]/3.6,0.0,0);
    std::cout << " no car, " << cur_v[i] << " -> " << desired_v[i]<< " res : " <<res << std::endl;
  }
  cur_v = {60,60,60,60,80,80,80,80};
  desired_v = {60,60,60,60,80,80,80,80};
  std::vector<double> obj_v = {60,60,60,61,80,85,81,81};
  std::vector<double> delta_s = {20,10,16.67,10,20,16,1,80};
  for (int i =0;i<cur_v.size();i++){
    idm_params.desired_spd = desired_v[i]/3.6;
    auto res = ILQR::LongitudinalOperator::calc_idm_acc(idm_params, delta_s[i], cur_v[i] / 3.6, (cur_v[i] - obj_v[i]) / 3.6, 1);
    std::cout << " has car, " << cur_v[i] << " -> desired spd is: " << desired_v[i]
    << " , obj_v is: " <<obj_v[i] << " , delta s is: "<< delta_s[i]<< " res : " <<res << std::endl;
    std::cout << "-------------------------------" << std::endl;
  }

  cur_v = {60,60,60,60};
  desired_v = {60,100,60,60};
  obj_v = {30,30,50,50};
  delta_s = {50,50,20,27};
  for (int i =0;i<cur_v.size();i++){
    idm_params.desired_spd = desired_v[i]/3.6;
    auto res = ILQR::LongitudinalOperator::calc_idm_acc(idm_params,delta_s[i] , cur_v[i] / 3.6 , (cur_v[i] - obj_v[i]) / 3.6, 1);
    std::cout << " has car, " << cur_v[i] << " -> desired spd is: " << desired_v[i]
    << " , obj_v is: " <<obj_v[i] << " , delta s is: "<< delta_s[i]<< " res : " <<res << std::endl;
    std::cout << "-------------------------------" << std::endl;
  }

}