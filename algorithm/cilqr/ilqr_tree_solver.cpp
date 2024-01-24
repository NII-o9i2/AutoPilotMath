//
// Created by SENSETIME\fengxiaotong on 24-1-5.
//
#include "iostream"
#include "ilqr_tree_solver.h"
#include "ilqr_lon_condition.h"

namespace TreeILQR {
static double search_time = 0.0;
void VehicleModelBicyclePlus::step(const Eigen::VectorXd& state,
          Eigen::VectorXd& action,
          Eigen::VectorXd& next_state) const{
  // state -> 0:x, 1:y, 2:v, 3:theta, 4:a, 5:omega
  // action -> 0:jerk, 1:omega_dot
  if (state.size() != state_size_ || action.size() != action_size_) {
    std::cout << " input state size not right " << std::endl;
    return;
  }

  next_state.resize(state_size_);
  double s = state[2]* param_.delta_t + 0.5 * state[4] * param_.delta_t * param_.delta_t;

  double next_x = state[0] + 0.5*(std::cos(state[3]) + std::cos(state[3] + state[5] * param_.delta_t))
      * s;
  double next_y = state[1] + 0.5*(std::sin(state[3]) + std::sin(state[3] + state[5] * param_.delta_t))
      * s;
  double next_v = std::max(0.0,state[2] + state[4] * param_.delta_t);

  double next_theta = state[3] + state[5] * param_.delta_t;
  double next_a = state[4] + action[0] * param_.delta_t;
  if (next_v < 1e-3){
    next_a = action[0] * param_.delta_t;
  }
  next_a = std::max(-6.0,next_a);
  next_a = std::min(2.0,next_a);
  action[0] = (next_a - state[4]) / param_.delta_t;

  double max_omega = next_v * param_.curvature_max;
  max_omega = std::min(max_omega,(param_.acc_lat_max-0.2)/state[2]);
  double next_omega = state[5] + action[1] * param_.delta_t;
  next_omega = std::max(-max_omega,next_omega);
  next_omega = std::min(max_omega,next_omega);
  action[1] = (next_omega - state[5]) / param_.delta_t;

  next_state << next_x, next_y, next_v, next_theta, next_a, next_omega;
}

std::vector<double> VehicleModelBicyclePlus::step_std(const std::vector<double>& state_std,
                                                  const std::vector<double>& action_std){
  Eigen::VectorXd state;
  Eigen::VectorXd action;
  Eigen::VectorXd next_state;
  std::vector<double> next_state_std;
  next_state_std.clear();
  if (state_std.size() != state_size_ || action_std.size() != action_size_) {
    std::cout << " input state size not right " << std::endl;
    return next_state_std;
  }
  state.resize(state_size_);
  for (int i = 0; i < state_size_; i++) {
    state[i] = state_std[i];
  }
  action.resize(action_size_);
  for (int i = 0; i < action_size_; i++) {
    action[i] = action_std[i];
  }
  next_state.resize(state_size_);
  step(state, action, next_state);

  for (int i = 0; i < state_size_; i++) {
    next_state_std.emplace_back(next_state[i]);
  }
  return next_state_std;
}

void VehicleModelBicyclePlus::condition_init(const int & frame_count,
                   const std::shared_ptr<EnvSim::EnvSimulator> &env_ptr,
                   ILQR::StepCondition & l_condition) const{

  auto obs = env_ptr->get_all_obstacle();
  l_condition.obstacle_belief_state_map.clear();
  // trajectory point interval time == param.delta_t
  for(auto& ob :obs){
    double x = ob.trajectory_points[frame_count].position.x;
    double y = ob.trajectory_points[frame_count].position.y;
    auto begin = std::chrono::high_resolution_clock::now();
    auto nearest_point = env_ptr->get_nearest_point(MathUtils::Point2D{x,y});
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> time = end - begin;
    search_time += time.count();
    double dis = std::hypot(x-nearest_point.x,y-nearest_point.y);

    ILQR::LonObstacleInfo info;
    if (dis < 3.0 ){
      info.belief = 1.0;
    }
    info.position.x = x;
    info.position.y = y;
    info.length = ob.get_length();
    info.s = env_ptr->get_lane_s(MathUtils::Point2D{x,y});
    info.v.x = ob.trajectory_points[frame_count].v * std::cos(ob.trajectory_points[frame_count].theta);
    info.v.y = ob.trajectory_points[frame_count].v * std::sin(ob.trajectory_points[frame_count].theta);
    double tmp_a = 0.0;
    if (frame_count< param_.horizon){
      tmp_a = (ob.trajectory_points[frame_count+1].v - ob.trajectory_points[frame_count].v) / param_.delta_t;
    }
    info.a.x = tmp_a * std::cos(ob.trajectory_points[frame_count].theta);
    info.a.y = tmp_a * std::sin(ob.trajectory_points[frame_count].theta);
    l_condition.obstacle_belief_state_map.insert({ob.get_id(),info});
  }
}

void VehicleModelBicyclePlus::get_l_lon_condition(const int & frame_count,
                                              const Eigen::VectorXd& pre_state,
                                              const Eigen::VectorXd& state,
                                              ILQR::StepCondition& pre_l_condition,
                                              const std::shared_ptr<EnvSim::EnvSimulator> &env_ptr,
                                              ILQR::StepCondition & l_condition) const{
  MathUtils::Point2D pos;
  pos.x = state[0];
  pos.y = state[1];
  double s_ego = env_ptr->get_lane_s(pos);
  double pre_v_ego;
  double pre_a_ego;
  double ref_a = 0.0;


  pre_v_ego = pre_state[2];
  if (param_.enable_hard_idm_model && pre_l_condition.has_ref_v){
    pre_v_ego = pre_l_condition.v_ref;
  }
  if (pre_l_condition.has_ref_a){
    pre_a_ego = pre_l_condition.a_ref;
  }else{
    pre_a_ego = pre_state[4];
  }
  double v_ego = pre_v_ego + pre_a_ego * param_.delta_t;
  ref_a = ILQR::LongitudinalOperator::calc_idm_acc(param_.idm_param,0.0,v_ego,0.0,0);
  l_condition.has_lead_one = false;
  double min_a = 0.0;
  double min_s = 0.0;
  for(auto& item : l_condition.obstacle_belief_state_map){
    if (item.second.belief < 1e-2){
      continue;
    }
    double v_obj = item.second.v.x * std::cos(state[3]) + item.second.v.y * std::sin(state[3]);
    double a_obj = item.second.a.x * std::cos(state[3]) + item.second.a.y * std::sin(state[3]);
    double delta_s =  item.second.s - 0.5 * item.second.length - 0.5 * param_.vehicle_param.length - s_ego;
    if (item.second.s - 0.5 * item.second.length < min_s || !l_condition.has_lead_one) {
      min_s = item.second.s - 0.5 * item.second.length;
      l_condition.has_lead_one = true;
    }
    ref_a = ILQR::LongitudinalOperator::calc_idm_acc(param_.idm_param,delta_s,v_ego,v_ego-v_obj,1);
  }
  if (l_condition.has_lead_one){
    l_condition.min_s = min_s;
  }


  auto begin = std::chrono::high_resolution_clock::now();
  auto info = env_ptr->get_nearest_point_info(pos);
  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> time = end - begin;
  search_time += time.count();
  double curvature = info.curvature;
  double ref_v_curvature = std::fabs(curvature) < 1e-3 ? v_ego : std::sqrt((param_.lat_acc_lon) / curvature);
  if (ref_v_curvature < v_ego){
    v_ego = pre_v_ego - std::min(pre_v_ego - ref_v_curvature, 1.0 * param_.delta_t);
    ref_a = (v_ego - pre_v_ego) / param_.delta_t;
  }
  l_condition.s_ego = s_ego;
  l_condition.pre_s_ego = pre_l_condition.s_ego;
  l_condition.has_ref_a = true;
  l_condition.a_ref = ref_a;
  l_condition.has_ref_v = true;
  l_condition.v_ref = v_ego;

}


void VehicleModelBicyclePlus::get_all_element(const int & frame_count,
                     const std::shared_ptr<EnvSim::EnvSimulator> &env_ptr,
                     const std::vector<ILQR::ILQRObstacleConstrain>& cost_list,
                     const std::shared_ptr<ILQR::SolverSpace>& pre_space_ptr,
                     std::shared_ptr<ILQR::SolverSpace>& space_ptr) const {
  auto& state = space_ptr->x_space;
  auto& action = space_ptr->u_space;
  // state -> 0:x, 1:y, 2:v, 3:theta, 4:a, 5:omega
  // action -> 0:jerk, 1:omega_dot
  double l = state[2] * param_.delta_t +
      0.5 * state[4] * param_.delta_t * param_.delta_t;
  double theta_add_omega_dot_t = state[3] + state[5] * param_.delta_t;
  double middle_sin = 0.5 * (std::sin(state[3]) +
      std::sin(theta_add_omega_dot_t));
  double middle_cos = 0.5 * (std::cos(state[3]) +
      std::cos(theta_add_omega_dot_t));
  // 1.0 get_f_x
  space_ptr->f_x_space.resize(param_.state_size,param_.state_size);
  space_ptr->f_x_space << 1.0, 0.0, param_.delta_t * middle_cos,  -middle_sin * l, middle_cos * 0.5 * param_.delta_t * param_.delta_t , l * 0.5 * param_.delta_t * (-std::sin( theta_add_omega_dot_t)),
      0.0, 1.0, param_.delta_t * middle_sin, middle_cos * l, middle_sin * 0.5 * param_.delta_t * param_.delta_t , l * 0.5 * param_.delta_t * std::cos(theta_add_omega_dot_t),
      0.0, 0.0, 1.0, 0.0, param_.delta_t , 0.0,
      0.0, 0.0, 0.0, 1.0, 0.0 , param_.delta_t,
      0.0, 0.0, 0.0, 0.0, 1.0 , 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0 , 1.0;

  // 2.0 get_f_xx
  space_ptr->f_xx_space.resize(param_.state_size,param_.state_size);
  if (param_.use_hessians){
    space_ptr->f_xx_space.setZero();
    std::cout<< " f_xx default! "<< std::endl;
  }else{
    space_ptr->f_xx_space.setZero();
  }

  // 3.0 get_f_u
  space_ptr->f_u_space.resize(param_.state_size,param_.action_size);
  space_ptr->f_u_space << 0.0 , 0.0,
                          0.0 , 0.0,
                          0.0 , 0.0,
                          0.0 , 0.0,
                          param_.delta_t, 0.0,
                          0.0, param_.delta_t;

  // 4.0 get_f_ux
  space_ptr->f_ux_space.resize(param_.action_size,param_.state_size);
  if (param_.use_hessians){
    space_ptr->f_ux_space.setZero();
    std::cout<< " f_ux default! "<< std::endl;
  }else{
    space_ptr->f_ux_space.setZero();
  }

  // 5.0 get_f_uu
  space_ptr->f_uu_space.resize(param_.action_size,param_.state_size);
  if (param_.use_hessians){
    space_ptr->f_uu_space.setZero();
    std::cout<< " f_uu default! "<< std::endl;
  }else{
    space_ptr->f_uu_space.setZero();
  }

  // 6.0 get l_condition
  if (env_ptr == nullptr){
    std::cout<< " nullptr !"<< std::endl;
    return;
  }
  // 6.1 get condition
  get_l_condition(frame_count,space_ptr->x_space,env_ptr, space_ptr->l_condition);

  // 6.2 get lon condition
  if (pre_space_ptr != nullptr){
    auto &pre_state = pre_space_ptr->x_space;
    auto &pre_condition = pre_space_ptr->l_condition;
    get_l_lon_condition(frame_count,pre_state, space_ptr->x_space, pre_condition,env_ptr,space_ptr->l_condition);
  }else{
    space_ptr->l_condition.v_ref = space_ptr->x_space[2];
    space_ptr->l_condition.a_ref = space_ptr->x_space[4];
    MathUtils::Point2D pos;
    pos.x = space_ptr->x_space[0];
    pos.y =  space_ptr->x_space[1];
    space_ptr->l_condition.s_ego = env_ptr->get_lane_s(pos);
    space_ptr->l_condition.has_ref_v = false;
    space_ptr->l_condition.has_ref_a = false;
  }

  // 7.0 get l
  space_ptr->l_space = 0.0;
  get_l(space_ptr->x_space,space_ptr->u_space, space_ptr->l_condition,frame_count,space_ptr->l_space,space_ptr->l_lat_space,space_ptr->l_lon_space);

  double acc_lat_real = state[2] * state[5];
  space_ptr->l_x_space.setZero();
  space_ptr->l_u_space.setZero();
  space_ptr->l_xx_space.setZero();
  space_ptr->l_ux_space.setZero();
  space_ptr->l_uu_space.setZero();

  // 8.0 ref jerk
  double exp_pos_jerk = std::exp( param_.jerk_param.q2 * (action[0]- param_.jerk_thr));
  double exp_neg_jerk = std::exp( param_.jerk_param.q2 * (param_.jerk_thr - action[0]));
  // 8.1 get_l_x
  // 8.2 get_l_u
  space_ptr->l_u_space[0] += 2.0  * param_.w_ref_jerk * action[0] ;
  space_ptr->l_u_space[0] += param_.jerk_param.q1 * param_.jerk_param.q2 *  exp_pos_jerk;
  space_ptr->l_u_space[0] += -param_.jerk_param.q1 * param_.jerk_param.q2 * exp_neg_jerk;
  // 8.3 get_l_xx
  // 8.4 get_l_ux
  // 8.5 get_l_uu
  space_ptr->l_uu_space(0,0) += 2.0 * param_.w_ref_jerk;
  space_ptr->l_uu_space(0,0) += param_.jerk_param.q1 * param_.jerk_param.q2 * param_.jerk_param.q2 * exp_pos_jerk;
  space_ptr->l_uu_space(0,0) += param_.jerk_param.q1 * param_.jerk_param.q2 * param_.jerk_param.q2 * exp_neg_jerk;


  // 9.0 ref omega_dot
  if (param_.enable_lat){
    // 9.1 get_l_x
    // 9.2 get_l_u
    space_ptr->l_u_space[1] += 2.0 * param_.w_ref_omega_dot * action[1];
    // 9.3 get_l_xx
    // 9.4 get_l_ux
    // 9.5 get_l_uu
    space_ptr->l_uu_space(1,1) += 2.0 * param_.w_ref_omega_dot;
  }


  // 10.0 ref v
  // 10.1 get_l_x
  if (space_ptr->l_condition.has_ref_v) {
    space_ptr->l_x_space[2] += 2.0 * param_.w_ref_v * (state[2] - space_ptr->l_condition.v_ref);
  }
  // 10.2 get_l_u
  // 10.3 get_l_xx
  if (space_ptr->l_condition.has_ref_v) {
    space_ptr->l_xx_space(2, 2) += param_.w_ref_v * 2.0; // v,v
  }
  // 10.4 get_l_ux
  // 10.5 get_l_uu


  // 11.0 ref a
  // 11.1 get_l_x
  if (space_ptr->l_condition.has_ref_a) {
    space_ptr->l_x_space[4] += 2.0 * param_.w_ref_a * (state[4]);
  }
  // 11.2 get_l_u
  // 11.3 get_l_xx
  if (space_ptr->l_condition.has_ref_a) {
    space_ptr->l_xx_space(4, 4) += param_.w_ref_a * 2.0; // a,a
  }
  // 11.4 get_l_ux
  // 11.5 get_l_uu


  // 12.0 constraint v
  if (space_ptr->l_condition.has_lead_one){
    // 12.1 get_l_x
    double s_relative = - space_ptr->l_condition.pre_s_ego + space_ptr->l_condition.min_s - param_.s_constraint_min + 0.5 * param_.vehicle_param.length;
    double g = state[2] * param_.delta_t - s_relative;
    if (g > 0.0){
      space_ptr->l_x_space[2] += param_.s_constraint_exp_param.q1 * param_.s_constraint_exp_param.q2 * param_.delta_t *
          std::min(1e9,std::exp(param_.s_constraint_exp_param.q2 * g));
    }else if (g > -1) {
      space_ptr->l_x_space[2] += param_.delta_t / (param_.s_constraint_param.t * (s_relative - param_.delta_t * state[2]));
    }
    // 12.2 get_l_u
    // 12.3 get_l_xx
    if (g > 0.0){
      space_ptr->l_xx_space(2,2) += param_.s_constraint_exp_param.q1 * param_.s_constraint_exp_param.q2 * param_.delta_t * param_.s_constraint_exp_param.q2 * param_.delta_t *
          std::min(1e9,std::exp(param_.s_constraint_exp_param.q2 * g));
    }else if (g > -1){
      space_ptr->l_xx_space(2,2) += param_.delta_t * param_.delta_t / (param_.s_constraint_param.t * (s_relative - param_.delta_t * state[2]) * (s_relative - param_.delta_t * state[2]));
    }
    // 12.4 get_l_ux
    // 12.5 get_l_uu
  }


  // 13.0 constraint a
  // 13.1 get_l_x
  // 13.2 get_l_u
  // 13.3 get_l_xx
  // 13.4 get_l_ux
  // 13.5 get_l_uu


  // 14.0 ref line point
  if (param_.enable_lat && space_ptr->l_condition.has_ref_point) {
    // 14.1 get_l_x
    double dis = state[2] * param_.consider_time + param_.vehicle_param.length * 0.5;
    double cos_theta_de_alpha = std::cos(state[3] - space_ptr->l_condition.ref_point_theta);
    double sin_theta_de_alpha = std::sin(state[3] - space_ptr->l_condition.ref_point_theta);
    double tmp = space_ptr->l_condition.ref_lat_dis + dis * sin_theta_de_alpha;
    space_ptr->l_x_space[3] += 2 * param_.w_ref_line * dis * cos_theta_de_alpha * tmp;
    // 14.2 get_l_u
    // 14.3 get_l_xx
    space_ptr->l_xx_space(3,3) += 2 * dis * dis * param_.w_ref_line * cos_theta_de_alpha * cos_theta_de_alpha
        - 2 * dis * param_.w_ref_line * tmp * sin_theta_de_alpha;
    // 14.4 get_l_ux
    // 14.5 get_l_uu
  }


  // 15.0 ref omega
  // 15.1 get_l_x
  if (space_ptr->l_condition.has_ref_omega) {
    space_ptr->l_x_space[5] += 2.0 * param_.w_ref_omega * (state[5] - space_ptr->l_condition.ref_omega);
  }
  // 15.2 get_l_u
  // 15.3 get_l_xx
  if (space_ptr->l_condition.has_ref_omega) {
    space_ptr->l_xx_space(5, 5) += param_.w_ref_omega * 2.0; // omega,omega
  }
  // 15.4 get_l_ux
  // 15.5 get_l_uu


  // 16.0 constraint lat acc
  if (space_ptr->l_condition.has_constraint_acc_lat) {
    double acc_lat = state[2] * state[5];
    double g = acc_lat - param_.acc_lat_max;
    double exp = std::min(1e9,std::exp(param_.lat_acc_constraint_exp_param.q2 * g));
    // 16.1 get_l_x
    if (g> 0){
      space_ptr->l_x_space[5] += param_.lat_acc_constraint_exp_param.q1 * param_.lat_acc_constraint_exp_param.q2 * state[2] * exp;
    }else {
      space_ptr->l_x_space[5] += - state[2] / (param_.lat_acc_constraint_param.t * g);
    }
    g = - acc_lat - param_.acc_lat_max;
    double exp_min = std::min(1e9,std::exp(param_.lat_acc_constraint_exp_param.q2 * g));
    if (g> 0){
      space_ptr->l_x_space[5] += -param_.lat_acc_constraint_exp_param.q1 * param_.lat_acc_constraint_exp_param.q2 * state[2] * exp_min;
    }else {
      space_ptr->l_x_space[5] += state[2] / (param_.lat_acc_constraint_param.t * g);
    }
    // 16.2 get_l_u
    // 16.3 get_l_xx
    g = acc_lat - param_.acc_lat_max;
    if (g> 0){
      space_ptr->l_xx_space(5,5) += param_.lat_acc_constraint_exp_param.q1 * param_.lat_acc_constraint_exp_param.q2 * state[2]
           * param_.lat_acc_constraint_exp_param.q2 * state[2] * exp;
    }else{
      space_ptr->l_xx_space(5,5)  += state[2] * state[2] / (param_.lat_acc_constraint_param.t * g * g);
    }
    g = - acc_lat - param_.acc_lat_max;
    if (g> 0){
      space_ptr->l_xx_space(5,5) += param_.lat_acc_constraint_exp_param.q1 * param_.lat_acc_constraint_exp_param.q2 * state[2]
          * param_.lat_acc_constraint_exp_param.q2 * state[2] * exp_min;
    }else {
      space_ptr->l_xx_space(5,5)  += state[2] * state[2] / (param_.lat_acc_constraint_param.t * g * g);
    }
    // 16.4 get_l_ux
    // 16.5 get_l_uu
  }


  // 17.0 lat_acc v
  if (param_.enable_lat){
    // 17.1 get_l_x
    space_ptr->l_x_space[2] += 2 * param_.w_lat_acc_v * state[2] * state[5] * state[5];
    // 17.2 get_l_u
    // 17.3 get_l_xx
    space_ptr->l_xx_space(2,2) += 2 * param_.w_lat_acc_v * state[5] * state[5];
    // 17.4 get_l_ux
    // 17.5 get_l_uu
  }


  // 18.0 lat_acc omega
  if (param_.enable_lat && space_ptr->l_condition.has_ref_acc_lat){
    // 18.1 get_l_x
    space_ptr->l_x_space[5] += 2 * param_.w_ref_acc_lat * (state[2] *state[5]- space_ptr->l_condition.acc_lat_ref) * state[2];
    // 18.2 get_l_u
    // 18.3 get_l_xx
    space_ptr->l_xx_space(5,5) += 2 * param_.w_ref_acc_lat * state[2] * state[2];
    // 18.4 get_l_ux
    // 18.5 get_l_uu
  }


  // 19.0 ref point
  if (param_.enable_lat && space_ptr->l_condition.has_ref_point){
    // 19.1 get_l_x
    space_ptr->l_x_space[0] += 2 * param_.w_ref_point * (state[0]  - space_ptr->l_condition.ref_point.x);
    space_ptr->l_x_space[1] += 2 * param_.w_ref_point * (state[1]  - space_ptr->l_condition.ref_point.y);
    // 19.2 get_l_u
    // 19.3 get_l_xx
    space_ptr->l_xx_space(0,0) += 2 * param_.w_ref_point;
    space_ptr->l_xx_space(1,1) += 2 * param_.w_ref_point;
    // 19.4 get_l_ux
    // 19.5 get_l_uu
  }


  // 13.0 obstacle constraint cost
//  for (ILQR::ILQRObstacleConstrain cost : cost_list) {
//    cost.get_l_condition(frame_count, space_ptr->x_space, env_ptr, space_ptr->l_condition);
//    cost.get_all_l_element(space_ptr->x_space,  space_ptr->l_condition, space_ptr->l_space, space_ptr->l_x_space,
//                           space_ptr->l_xx_space);
//    cost.get_l(space_ptr->l_condition, space_ptr->l_space);
//  }

}

void VehicleModelBicyclePlus::get_l_condition(const int & frame_count,
                     const Eigen::VectorXd& state,
                     const std::shared_ptr<EnvSim::EnvSimulator> &env_ptr,
                     ILQR::StepCondition & l_condition) const{
  // state : x, y, v, theta, a, omega
  if (env_ptr == nullptr){
    std::cout<< " nullptr !"<< std::endl;
  }

  if (param_.enable_lat){
    MathUtils::Point2D pos;
    pos.x = state[0];
    pos.y = state[1];
    auto begin = std::chrono::high_resolution_clock::now();
    auto info = env_ptr->get_nearest_point_info(pos);
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> time = end - begin;
    search_time += time.count();
    // 1.0 ref point
    l_condition.has_ref_point = true;
    l_condition.ref_point = info.point;
    l_condition.ref_lat_dis = info.is_on_left ? std::hypot(pos.x - info.point.x,pos.y - info.point.y)
        :- std::hypot(pos.x - info.point.x,pos.y - info.point.y);
    l_condition.ref_point_theta = info.theta;

    // 2.0 ref omega
    l_condition.has_ref_omega = true;
    l_condition.ref_curva = info.curvature;
    l_condition.ref_omega = state[2] * info.curvature;
    // 3.0 ref acc_lat
    l_condition.has_ref_acc_lat = false;
    l_condition.acc_lat_ref = 0.0;
  }else{
    l_condition.has_ref_point = false;
    l_condition.has_ref_omega = false;
  }

  // 4.0 ref v
  // 5.0 ref a

  // 6.0 constraint acc_lat
  l_condition.has_constraint_acc_lat = true;

  // 7.0 constraint omega
  l_condition.has_constraint_omega = false;

}

void VehicleModelBicyclePlus::get_l(const Eigen::VectorXd& state,
           const Eigen::VectorXd& action,
           const ILQR::StepCondition& condition,
           const int frame_count,
           double &l,
           double &l_lat,
           double &l_lon) const{
  l = 0.0;
  double last_l = 0;
  l_lon = 0.0;
  l_lat = 0.0;

  // 1.0 ref jerk
  l += param_.w_ref_jerk * (action[0] * action[0]);
  l += param_.jerk_param.q1 * std::exp( param_.jerk_param.q2 * (action[0]- param_.jerk_thr));
  l += param_.jerk_param.q1 * std::exp( param_.jerk_param.q2 * (param_.jerk_thr - action[0]));
  l_lon += l;

  // 2.0 ref omega_dot
  last_l = l;
  l += param_.w_ref_omega_dot * (action[1] * action[1]);
  l_lat += l - last_l;

  // 3.0 ref v
  last_l = l;
  if (condition.has_ref_v) {
    double delta_v = state[2] - condition.v_ref;
    l += param_.w_ref_v * (delta_v * delta_v);
    l_lon += l - last_l;
  }

  // 4.0 ref a
  last_l = l;
  if (condition.has_ref_a) {
    double delta_acc = state[4];
    l += param_.w_ref_a * (delta_acc * delta_acc);
    l_lon += l - last_l;
  }

  // 5.0 constraint v
  last_l = l;
  if (condition.has_lead_one){
    double g = state[2] * param_.delta_t + condition.pre_s_ego - condition.min_s + param_.s_constraint_min + 0.5 * param_.vehicle_param.length;
    if (g > 0.0){
      l += std::min(1e9,param_.s_constraint_exp_param.q1 * std::exp(param_.s_constraint_exp_param.q2 * g)) + 1e8;
    }else if (g > - 1){
      l += std::min(1e8, - std::log(-g) / param_.s_constraint_param.t);
    }
    l_lon += l - last_l;
  }

  // 6.0 constraint a
//  {
//    double g = state[4] - param_.a_constraint_max;
//    if (g > 0.0){
//      l += param_.a_max_constraint_exp_param.q1 * std::min(1e8,std::exp(param_.a_max_constraint_exp_param.q2 * g)) + 1e6;
//    }else if (g > - 1) {
//      l += std::min(1e6, -std::log(-g) / param_.a_max_constraint_param.t);
//    }
//  }

  last_l = l;
  if (param_.enable_lat){
    // 7.0 ref point lat
    if (condition.has_ref_point) {
      double tmp = condition.ref_lat_dis + std::sin(state[3] - condition.ref_point_theta) * (state[2]* param_.consider_time + param_.vehicle_param.length * 0.5);
      l += param_.w_ref_line * tmp * tmp;
      double delta_x = state[0] - condition.ref_point.x;
      double delta_y = state[1] - condition.ref_point.y;
      l += param_.w_ref_point * (delta_x * delta_x + delta_y * delta_y);
    }
    // 8.0 ref omega lat
    if (condition.has_ref_omega) {
      double delta_omega = state[5] - condition.ref_omega;
      l += param_.w_ref_omega * (delta_omega * delta_omega);
    }

    // 9.0 ref acc_lat
    //    double lat_acc = state[2] * state[5];
//    last_l = l;
//    if (condition.has_ref_acc_lat) {
//      double delta_acc_lat = lat_acc - condition.acc_lat_ref;
//      l += param_.w_ref_acc_lat * (delta_acc_lat * delta_acc_lat);
//      l_lat += l - last_l;
//    }
  }


  // 10.0 constraint acc_lat
  last_l = l;
  if (condition.has_constraint_acc_lat) {
    double acc_lat = state[2] * state[5];
    double g = acc_lat - param_.acc_lat_max;
    if (g > 0){
      l += std::min(1e9,param_.lat_acc_constraint_exp_param.q1 * std::exp(param_.lat_acc_constraint_exp_param.q2 * g)) + 1e8;
    }else if (g > -1){
      l += std::min(1e8,- std::log(- g)/ param_.lat_acc_constraint_param.t);
    }
    g = - param_.acc_lat_max - acc_lat;
    if (g > 0){
      l += std::min(1e9,param_.lat_acc_constraint_exp_param.q1 * std::exp(param_.lat_acc_constraint_exp_param.q2 * g)) + 1e8;
    }else if (g > -1){
      l += std::min(1e8,- std::log(- g)/ param_.lat_acc_constraint_param.t);
    }
    l_lat += l - last_l;
  }

  // 11.0 constraint omega

}


void VehicleModelBicyclePlus::get_l_f(const Eigen::VectorXd &state,
                                      const ILQR::StepCondition &condition, double &l_f) const {
  l_f = 0.0;
  // 1.0 ref point
//  if (condition.has_ref_point) {
//    double delta_x = state[0] - condition.ref_point.x;
//    double delta_y = state[1] - condition.ref_point.y;
//    l_f += param_.w_ref_line * (delta_x * delta_x + delta_y * delta_y);
//  }

  //  2.0 ref v
//  if (condition.has_ref_v) {
//    double delta_v = state[2] - condition.v_ref;
//    l_f += param_.w_ref_v * (delta_v * delta_v);
//  }

  // 3.0 ref a

  // 4.0 ref omega

  // 5.0 ref acc_lat

  // 6.0 constraint acc_lat

  // 7.0 constraint omega

}

void TrajectoryTreeNode::clear() {
  for (auto &node : next_node_) {
    node.clear();
  }
  next_node_.clear();
  space_ptr_.reset();
}

void TrajectoryTreeNode::get_space_list(std::vector<std::shared_ptr<ILQR::SolverSpace>> &s_list,
                                        std::vector<std::vector<std::shared_ptr<ILQR::SolverSpace>>> &s_list_list) {
  s_list.emplace_back(space_ptr_);
  if (index_ >= input_->get_param().horizon || s_list.size() > 50) {
    s_list_list.emplace_back(s_list);
  } else {
    for (auto &node : next_node_) {
      node.get_space_list(s_list, s_list_list);
    }
  }
  s_list.pop_back();
}

FuncStatus TrajectoryTreeNode::pre_process() {
  auto &env_ptr = input_->get_env_ptr();
  auto &param = input_->get_param();
  auto obs = env_ptr->get_all_obstacle();

  // 1.0 update lon condition
  space_ptr_->l_condition.obstacle_belief_state_map.clear();
  for (const auto &ob : obs) {
    double x = ob.trajectory_points[index_].position.x;
    double y = ob.trajectory_points[index_].position.y;
    auto begin = std::chrono::high_resolution_clock::now();
    auto nearest_point = env_ptr->get_nearest_point(MathUtils::Point2D{x, y});
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> time = end - begin;
    search_time += time.count();
    double dis = std::hypot(x - nearest_point.x, y - nearest_point.y);

    ILQR::LonObstacleInfo info;
    if (dis < 3.0) {
      info.belief = 1.0;
    }
    info.position.x = x;
    info.position.y = y;
    info.length = ob.get_length();
    info.s = env_ptr->get_lane_s(MathUtils::Point2D{x, y});
    info.v.x = ob.trajectory_points[index_].v * std::cos(ob.trajectory_points[index_].theta);
    info.v.y = ob.trajectory_points[index_].v * std::sin(ob.trajectory_points[index_].theta);
    double tmp_a = 0.0;
    if (index_ < param.horizon - 1) {
      tmp_a = (ob.trajectory_points[index_ + 1].v - ob.trajectory_points[index_].v) / param.delta_t;
    }
    info.a.x = tmp_a * std::cos(ob.trajectory_points[index_].theta);
    info.a.y = tmp_a * std::sin(ob.trajectory_points[index_].theta);
    space_ptr_->l_condition.obstacle_belief_state_map.insert({ob.get_id(), info});
  }
  return FuncStatus::FuncSucceed;
}

FuncStatus TrajectoryTreeNode::backward_pass() {
  auto &param = input_->get_param();
  std::shared_ptr<ILQR::SolverSpace> next_space_ptr_real = nullptr;
  Eigen::MatrixXd reg_add_vxx;
  reg_add_vxx.resize(param.state_size, param.state_size);

  if (index_ >= param.horizon) {
    space_ptr_->v_x_space = space_ptr_->l_x_space;
    space_ptr_->v_xx_space = space_ptr_->l_xx_space;
//    space_ptr_->v_x_space.setZero();
//    space_ptr_->v_xx_space.setZero();
//    return FuncStatus::FuncSucceed;
  } else {
    double sum = 1e-6;
    for (auto &node : next_node_) {
      node.backward_pass();
      sum += std::exp(node.get_space()->branch_belief);
    }
    for (auto &node : next_node_) {
      double belief = std::exp(node.get_space()->branch_belief) / sum;
      space_ptr_->math_belief = belief;
      space_ptr_->v_x_space = node.get_space()->v_x_space * belief;
      space_ptr_->v_xx_space = node.get_space()->v_xx_space * belief;
    }
  }

  Eigen::VectorXd q_x;
  Eigen::VectorXd q_u;
  Eigen::MatrixXd q_xx;
  Eigen::MatrixXd q_ux;
  Eigen::MatrixXd q_uu;
  q_x.resize(param.state_size);
  q_u.resize(param.action_size);
  q_xx.resize(param.state_size, param.state_size);
  q_uu.resize(param.action_size, param.action_size);
  q_ux.resize(param.action_size, param.state_size);
  Eigen::MatrixXd reg = param.mu * Eigen::MatrixXd::Identity(
      param.state_size, param.state_size);
  if (param.use_hessians) {
    // todo: @jojo
  } else {
    q_x = space_ptr_->l_x_space + space_ptr_->f_x_space.transpose() * space_ptr_->v_x_space;
    q_u = space_ptr_->l_u_space + space_ptr_->f_u_space.transpose() * space_ptr_->v_x_space;
    q_xx = space_ptr_->l_xx_space + space_ptr_->f_x_space.transpose() * space_ptr_->v_xx_space * space_ptr_->f_x_space;
    double max_value = 1e-6;
    for (int i=0; i<param.state_size;i++){
      for (int j=0; j<param.state_size;j++){
        if (std::fabs(space_ptr_->v_xx_space(i,j)) > max_value){
          max_value = std::fabs(space_ptr_->v_xx_space(i,j));
        }
      }
    }
    double radio = std::max(std::min(1e8,max_value),1e-6);
    reg_add_vxx = (space_ptr_->v_xx_space*radio/max_value);
    for (int i=0; i<param.state_size;i++){
        if (std::fabs(reg_add_vxx(i,i)) < param.mu){
         reg_add_vxx(i,i) = reg_add_vxx(i,i) < 0 ? -param.mu : param.mu;
        }
    }
    q_ux = space_ptr_->l_ux_space
        + space_ptr_->f_u_space.transpose() * (reg_add_vxx) * space_ptr_->f_x_space;
    q_uu = space_ptr_->l_uu_space
        + space_ptr_->f_u_space.transpose() * (reg_add_vxx) * space_ptr_->f_u_space;
  }

  Eigen::FullPivLU<Eigen::MatrixXd> lu(q_uu);

  if (!lu.isInvertible()) {
    std::cout << "index: " << index_ << " Matrix is not invertible.\n";
//    std::cout << "reg "<< reg_add_vxx<< std::endl;
//    std::cout << "q_uu "<< q_uu <<std::endl;
    return FuncStatus::FuncFailed;
  }
  space_ptr_->k_space = -lu.inverse() * q_u;
  for (int i= 0; i< param.action_size;i++){
    if (q_u[i] < 0){
      space_ptr_->k_space[i] = std::fabs(space_ptr_->k_space[i]);
    }else{
      space_ptr_->k_space[i] = -std::fabs(space_ptr_->k_space[i]);
    }
  }
  space_ptr_->k_matrix_space = -lu.inverse() * q_ux;
  double k_radio_1 = std::fabs(space_ptr_->k_space[0]) / param.k_radio[0];
  double k_radio_2 = std::fabs(space_ptr_->k_space[1]) / param.k_radio[1];
//  std::cout<< "index: "<< index_ << "l_x: "<< space_ptr_->l_x_space<< std::endl;
//  std::cout<< "index: "<< index_ << "q_x: "<< q_x<< std::endl;
//  std::cout<< "index: "<< index_ << "q_u: "<< q_u<< std::endl;
//  std::cout<< "index: "<< index_ << "q_ux: "<< q_ux<< std::endl;
//  std::cout<< "index: "<< index_ << "l_uu: "<< space_ptr_->l_uu_space<< std::endl;
//  std::cout<< "index: "<< index_ << "reg: "<< reg_add_vxx<< std::endl;
//  std::cout<< "index: "<< index_ << "q_uu: "<< q_uu<< std::endl;
//  std::cout<< "index: "<< index_ <<" before backward k0 " <<space_ptr_->k_space[0] << " before backward k1 "<<space_ptr_->k_space[1] << std::endl;

  space_ptr_->v_x_space = q_x + space_ptr_->k_matrix_space.transpose() * q_uu * space_ptr_->k_space;
  space_ptr_->v_x_space += space_ptr_->k_matrix_space.transpose() * q_u + q_ux.transpose() * space_ptr_->k_space;

  space_ptr_->v_xx_space = q_xx + space_ptr_->k_matrix_space.transpose() * (q_uu) * space_ptr_->k_matrix_space;
  space_ptr_->v_xx_space +=
      space_ptr_->k_matrix_space.transpose() * (q_ux) + q_ux.transpose() * space_ptr_->k_matrix_space;
  space_ptr_->v_xx_space = 0.5 * (space_ptr_->v_xx_space + space_ptr_->v_xx_space.transpose());
  if (k_radio_1 > 1.0){
    space_ptr_->k_space[0] = space_ptr_->k_space[0]/ k_radio_1;
  }
  if (k_radio_2 > 1.0){
    space_ptr_->k_space[1] = space_ptr_->k_space[1]/ k_radio_2;
  }
//  std::cout<< "index: "<< index_ << " backward k0 "<<space_ptr_->k_space[0] << " backward k1 "<<space_ptr_->k_space[1] << std::endl;
  return FuncStatus::FuncSucceed;
}

void TrajectoryTreeNode::search_max_k_radio(std::vector<double>& k_radio){
  auto &param = input_->get_param();
  double k_radio_1 = std::fabs(space_ptr_->k_space[0]) / param.k_radio[0];
  double k_radio_2 = std::fabs(space_ptr_->k_space[1]) / param.k_radio[1];
  if (std::fabs(k_radio_1) > k_radio[0]){
    k_radio[0] = k_radio_1;
  }
  if (std::fabs(k_radio_2) > k_radio[1]){
    k_radio[1] = k_radio_2;
  }
}
void TrajectoryTreeNode::reset_k(std::vector<double> &k_radio) {
  space_ptr_->k_space[0] = space_ptr_->k_space[0]/ k_radio[0];
  space_ptr_->k_space[1] = space_ptr_->k_space[1]/ k_radio[1];
}

FuncStatus TrajectoryTreeNode::forward_rollout(int index,
                                               const std::shared_ptr<SolverInput> &input,
                                               const std::shared_ptr<ILQR::SolverSpace> &pre_space_ptr) {

  // 1.0 init ILQR space
  index_ = index;
  input_ = input;
  space_ptr_ = std::make_shared<ILQR::SolverSpace>();
  next_node_.clear();
  auto param = input->get_param();
  space_ptr_->x_space.resize(param.state_size);
  space_ptr_->x_space_init.resize(param.state_size);
  space_ptr_->u_space.resize(param.action_size);

  space_ptr_->f_x_space.resize(param.state_size, param.state_size);
  space_ptr_->f_u_space.resize(param.state_size, param.action_size);
  space_ptr_->f_xx_space.resize(param.state_size, param.state_size);
  space_ptr_->f_ux_space.resize(param.action_size, param.state_size);
  space_ptr_->f_uu_space.resize(param.action_size, param.action_size);

//  space_ptr_->l_condition;
  space_ptr_->l_x_space.resize(param.state_size);
  space_ptr_->l_u_space.resize(param.action_size);
  space_ptr_->l_xx_space.resize(param.state_size, param.state_size);
  space_ptr_->l_ux_space.resize(param.action_size, param.state_size);
  space_ptr_->l_uu_space.resize(param.action_size, param.action_size);

  space_ptr_->k_space.resize(param.action_size);
  space_ptr_->k_matrix_space.resize(param.action_size, param.state_size);
  space_ptr_->v_x_space.resize(param.state_size);
  space_ptr_->v_xx_space.resize(param.state_size, param.state_size);

  TrajectoryTreeNode next_node;
  auto &init_state = input_->get_init_state();
  auto &init_action = input_->get_init_action();
  auto &model = input_->get_model_plus();
  auto &env_ptr = input_->get_env_ptr();
  auto &obstacle_cost = input_->get_obstacle_cost();
  pre_space_ptr_ = pre_space_ptr;
  model.condition_init(index,env_ptr,space_ptr_->l_condition);
  model.condition_init(index,env_ptr,space_ptr_->l_new_condition);
  // update lon condition
  pre_process();
  // 2.0 update x_space
  if (index == 0) { // without pre node
    // 2.1 update the first point space
    space_ptr_->u_space = init_action[index];
    space_ptr_->x_space = init_state;
    space_ptr_->x_space_init = space_ptr_->x_space;
    space_ptr_->l_condition.v_ref = init_state[2];
    model.get_all_element(index, env_ptr, obstacle_cost, pre_space_ptr_, space_ptr_);
    next_node.forward_rollout(index + 1, input, space_ptr_);
    next_node_.emplace_back(next_node);
    // 2.2 init first point l new condition
    space_ptr_->l_new_condition.v_ref = space_ptr_->x_space[2];
    space_ptr_->l_new_condition.a_ref = space_ptr_->x_space[4];
    MathUtils::Point2D pos;
    pos.x = space_ptr_->x_space[0];
    pos.y =  space_ptr_->x_space[1];
    space_ptr_->l_new_condition.s_ego = env_ptr->get_lane_s(pos);
    space_ptr_->l_new_condition.has_ref_v = false;
    space_ptr_->l_new_condition.has_ref_a = false;
  } else if (index >= param.horizon) { // without next node
    // 2.1 update the last point space
    space_ptr_->u_space = pre_space_ptr->u_space;
    model.step(pre_space_ptr_->x_space, pre_space_ptr_->u_space, space_ptr_->x_space);
    space_ptr_->x_space_init = space_ptr_->x_space;
    model.get_all_element(index, env_ptr, obstacle_cost, pre_space_ptr_, space_ptr_);
  } else { // with pre & next node
    // 2.1 update the middle point space
    space_ptr_->u_space = init_action[index];
    model.step(pre_space_ptr_->x_space, pre_space_ptr_->u_space, space_ptr_->x_space);
    space_ptr_->x_space_init = space_ptr_->x_space;
    model.get_all_element(index, env_ptr, obstacle_cost, pre_space_ptr_, space_ptr_);
    next_node.forward_rollout(index + 1, input, space_ptr_);
    next_node_.emplace_back(next_node);
  }
  return FuncStatus::FuncSucceed;
}

FuncStatus TrajectoryTreeNode::forward_pass() {
  auto param = input_->get_param();
  auto &model = input_->get_model_plus();
  auto &env_ptr = input_->get_env_ptr();
  auto &obstacle_cost = input_->get_obstacle_cost();
  space_ptr_->x_space = space_ptr_->x_new_space;
  space_ptr_->u_space = space_ptr_->u_new_space;
  space_ptr_->l_condition = space_ptr_->l_new_condition;

  if (index_ >= param.horizon) { // without next node
    // 2.1 update the last point space
    model.get_all_element(index_, env_ptr, obstacle_cost, pre_space_ptr_, space_ptr_);
  } else { // with next node
    // 2.1 update space
    model.get_all_element(index_, env_ptr, obstacle_cost, pre_space_ptr_, space_ptr_);
    for (auto &node : next_node_) {
      node.forward_pass();
    }
  }
  return FuncStatus::FuncSucceed;
}

double TrajectoryTreeNode::get_l_sum() {
  double add_sum = 0.0;
  for (auto &node : next_node_) {
    add_sum += node.get_l_sum();
  }
  add_sum += space_ptr_->l_space;
  return add_sum;
}

double TrajectoryTreeNode::control(const double &alpha) {
  auto &param = input_->get_param();
  auto &model = input_->get_model_plus();
  auto &obs_cost = input_->get_obstacle_cost();
  auto &env_ptr = input_->get_env_ptr();
  space_ptr_->l_new_space = 0.0;
  if (index_ == 0) {
    // 1.0 first point
    space_ptr_->x_new_space = space_ptr_->x_space;
    space_ptr_->u_new_space = space_ptr_->u_space + alpha * space_ptr_->k_space;
    double leave_l = 0.0;
    // 2.0 get new condition
    model.get_l_condition(index_, space_ptr_->x_new_space, env_ptr, space_ptr_->l_new_condition);
    // 3.0 recursive
    for (auto &node : next_node_) {
      leave_l = node.get_space()->math_belief * node.control(alpha);
    }
    // 4.0 get l
    model.get_l(space_ptr_->x_new_space, space_ptr_->u_new_space, space_ptr_->l_condition,index_, space_ptr_->l_new_space,space_ptr_->l_lat_space,space_ptr_->l_lon_space);
//    for (auto &cost : obs_cost) {
//      cost.get_l(space_ptr_->l_condition, space_ptr_->l_new_space);
//    }
    return leave_l + space_ptr_->l_new_space;
  } else if (index_ >= param.horizon) {
    // 1.0 last point
    model.step(pre_space_ptr_->x_new_space, pre_space_ptr_->u_new_space, space_ptr_->x_new_space);
    space_ptr_->u_new_space = space_ptr_->u_space;
    space_ptr_->u_new_space.setZero();
   // 2.0 get new condition
    model.get_l_condition(index_, space_ptr_->x_new_space, env_ptr, space_ptr_->l_new_condition);
    model.get_l_lon_condition(index_,
                              pre_space_ptr_->x_new_space,
                              space_ptr_->x_new_space,
                              pre_space_ptr_->l_new_condition,
                              env_ptr,
                              space_ptr_->l_new_condition);
    // 3.0 get l
    model.get_l(space_ptr_->x_new_space, space_ptr_->u_new_space, space_ptr_->l_new_condition,index_, space_ptr_->l_new_space,space_ptr_->l_lat_space,space_ptr_->l_lon_space);
//    for (auto &cost : obs_cost) {
//      cost.get_l(space_ptr_->l_new_condition, space_ptr_->l_new_space);
//    }
//    double l_f = 0.0;
//    model.get_l_f(space_ptr_->x_new_space, space_ptr_->l_condition, l_f);
//    space_ptr_->l_new_space += l_f;
    return space_ptr_->l_new_space;
  } else {
    // 1.0 middle point
    model.step(pre_space_ptr_->x_new_space, pre_space_ptr_->u_new_space, space_ptr_->x_new_space);

    if (index_ == param.horizon - 1){
      space_ptr_->u_new_space = pre_space_ptr_->u_new_space;
    }else{
      space_ptr_->u_new_space = space_ptr_->u_space + alpha * space_ptr_->k_space
          + space_ptr_->k_matrix_space * (space_ptr_->x_new_space - space_ptr_->x_space);
    }
    // 2.0 get new condition
    model.get_l_condition(index_, space_ptr_->x_new_space, env_ptr, space_ptr_->l_new_condition);
    model.get_l_lon_condition(index_,
                              pre_space_ptr_->x_new_space,
                              space_ptr_->x_new_space,
                              pre_space_ptr_->l_new_condition,
                              env_ptr,
                              space_ptr_->l_new_condition);
    // 3.0 recursive
    double leave_l = 0.0;
    for (auto &node : next_node_) {
      leave_l = node.get_space()->math_belief * node.control(alpha);
    }
    // 4.0 get l
    model.get_l(space_ptr_->x_new_space, space_ptr_->u_new_space, space_ptr_->l_new_condition,index_, space_ptr_->l_new_space,space_ptr_->l_lat_space,space_ptr_->l_lon_space);
    //    for (auto &cost : obs_cost) {
//      cost.get_l(space_ptr_->l_new_condition, space_ptr_->l_new_space);
//    }
    return leave_l + space_ptr_->l_new_space;
  }
}

void TrajectoryTreeManager::init(std::shared_ptr<EnvSim::EnvSimulator> &env_ptr,
                                 std::vector<Eigen::VectorXd> &init_actions,
                                 Eigen::VectorXd &init_state,
                                 ILQR::ILQRParam &param) {
  input_.set_env_ptr(env_ptr);
  input_.set_init_action(init_actions);
  input_.set_init_state(init_state);
  bool use_plus_model = true;
  param.vehicle_param.length = 4.9;
  param.vehicle_param.width = 1.9;
  param.vehicle_param.wheel_base = 2.8;
  param.vehicle_param.max_delta = 0.61;
  param.curvature_max = std::tan(param.vehicle_param.max_delta) / param.vehicle_param.wheel_base;
  param.vehicle_param.r_4 = std::sqrt(
      (param.vehicle_param.length * 0.5 * 0.25) *
          (param.vehicle_param.length * 0.5 * 0.25) +
          (param.vehicle_param.width * 0.5) * (param.vehicle_param.width * 0.5));
  std::cout << " r_4 is : " << param.vehicle_param.r_4 << std::endl;
  param.exp_model_param.q1 = 1.0;
  param.exp_model_param.q2 = 5.0;
  param.exp_model_param.q3 = 4.0;
  param.enable_log_model = true;
  param.log_model_param.t = 2;
  if (use_plus_model){
    param.state_size = 6;
  }else{
    param.state_size = 4;
  }

  input_.set_param(param);
  auto obs_mgr = env_ptr->get_obstacle_mgr();
  std::vector<ILQR::ILQRObstacleConstrain> obstacle_cost;
  obstacle_cost.clear();
  for (auto &obj : obs_mgr.obstacle_map) {
    ILQR::ILQRObstacleConstrain new_obj_cost;
    new_obj_cost.set_ilqr_param(param);
    new_obj_cost.init(obj.second);
    obstacle_cost.emplace_back(new_obj_cost);
  }
  input_.set_obstacle_cost(obstacle_cost);

  input_.init();

}

FuncStatus TrajectoryTreeManager::solve() {
  // 1.0 construct trajectory tree & update init tree
  std::vector<std::shared_ptr<ILQR::SolverSpace>> s_list;
  tree_node_list_.clear();
  auto input_ptr = std::make_shared<SolverInput>(input_);
  auto &param = input_.get_param();
  auto &alphas = input_.get_alpha();
  auto &mutable_param = input_.mutable_param();
  auto& model = input_ptr->mutable_model();
  std::vector<double> k_radio;
  mutable_param.enable_lat = false;
  model.update_parameter(mutable_param);
  tree_node_list_.forward_rollout(0, input_ptr, nullptr);

  // 2.0 loop dp until finish
  double j_opt = tree_node_list_.get_l_sum();
  std::cout << "j_opt: " << j_opt << std::endl;

  for (int i = 0; i < param.max_iter_num; i++) {
    if (i>param.max_iter_lon_num && !param.enable_lat){
      mutable_param.enable_lat = true;
      std::cout << "----------enable lat max lon" << i << std::endl;
      model.update_parameter(mutable_param);
      j_opt = 1e10;
    }
    // 2.1 backward to get k & k_matrix
    if (tree_node_list_.backward_pass() != FuncStatus::FuncSucceed) {
      // increase regularization term
      mutable_param.delta = std::max(1.0, mutable_param.delta) * mutable_param.delta_0;
      mutable_param.mu *= mutable_param.delta;
      if (mutable_param.mu >= mutable_param.mu_max) {
        mutable_param.mu = mutable_param.mu_max;
        std::cout << "[ERROR]: exceeded max regularization term " << mutable_param.mu_max << std::endl;
      }
      continue;
    }
    double j_alpha_best = j_opt;
    double best_alpha = 1.0;
    double j_new = 0.0;
    // 2.2 get best new state
    for (auto alpha : alphas) {
      j_new = tree_node_list_.control(alpha);
      if (j_new < j_alpha_best) {
        best_alpha = alpha;
        j_alpha_best = j_new;
      }
//      std::cout << " alpha: " << alpha << " j new: " << j_new << std::endl;
    }
    // 2.3 update best new state & space
//    std::cout << "-----------------j_opt: " << j_opt << " j_best_alpha: " << j_alpha_best
//              << std::endl;

//    std::cout << "------- best alpha: " << best_alpha << std::endl;
    if (std::fabs(j_opt - j_alpha_best) > std::min(10.0, j_opt *param.tol) &&
        j_opt - j_alpha_best > param.tol_abs) {
      j_new = tree_node_list_.control(best_alpha);
      tree_node_list_.forward_pass();
      if (i > 1){
        mutable_param.enable_hard_idm_model = true;
      }
      // decrease regularization term
      mutable_param.delta = std::min(1.0, mutable_param.delta) / mutable_param.delta_0;
      mutable_param.mu *= mutable_param.delta;
      if (mutable_param.mu <= mutable_param.mu_min) {
        mutable_param.mu = mutable_param.mu_min;
      }
      model.update_parameter(mutable_param);
      j_opt = j_new;
    } else {
      if (param.enable_lat){
        std::cout << "----------iter times: " << i << std::endl;
        std::cout << "----------stop search " << std::endl;
        break;
      }else{
        std::cout << "----------enable lat " << i << std::endl;
        mutable_param.enable_lat = true;
        model.update_parameter(mutable_param);
        j_opt = 1e10;
      }

    }
  }

  // 3.0 get result
  space_list_.clear();
  tree_node_list_.get_space_list(s_list, space_list_);
//  int i = 0;
//  for(auto & item : space_list_[0]){
//    std::cout<< "i: "<< i << " ref dis: " << item->l_condition.ref_lat_dis << " theta: "<< item->l_condition.ref_point_theta << " curva: "<< item->l_condition.ref_curva<< std::endl;
//    std::cout << i <<" tree x "<< item->x_space[0] << " y "<< item->x_space[1]<< " v "<< item->x_space[2] << " theta "<< item->x_space[3]
//        << " a "<< item->x_space[4]<< " o "<< item->x_space[5] << std::endl;
////    std::cout << i <<" tree j "<< item->u_space[0] << " k "<< item->u_space[1]<< std::endl;
////    std::cout  << i << " tree f_x "<< item->f_x_space << std::endl;
////    std::cout << "i: " << i<< "l: "<< item->l_space<<std::endl;
//    std::cout << " i: "<< i << " has lead one "<< item->l_condition.has_lead_one <<" delta_s: "<< item->l_condition.min_s - (item->l_condition.pre_s_ego + item->x_space[2] * param.delta_t) - param.s_constraint_min- 0.5 * param.vehicle_param.length << std::endl;
////    std::cout << " i: "<< i << " s: "<< item->l_condition.s_ego << " min_s" << item->l_condition.min_s << std::endl;
//    i++;
//  }
  std::cout<< "----------search time: "<< search_time * 1000 << " ms"<< std::endl;
  return FuncStatus::FuncSucceed;
}
}