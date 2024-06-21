#include "vector"
#include "memory"
#include "unordered_map"
#include <cmath>
#include <chrono>

#include "utils.h"
#include "nn_tool.h"
#include "interaction_search_traj.h"

namespace InteractionSearch {

void TrajNodeUtil::calc_cost(
    const std::shared_ptr<InteractionSearchCommonData>& common_data,
    const bool nn_traj,
    const int frame,
    const bool forwar_simu_success,
    const ControlVariable& control,
    const StateVariable& state,
    const MathUtils::FrenetPoint2D& ego_pos_frenet,
    const std::vector<MathUtils::Point2D>& ego_polygon,
    const MathUtils::Range& ego_s_range,
    const MathUtils::Range& ego_l_range,
    const double& pre_accumu_cost,
    double& cost,
    double& accumu_cost) {
  if (frame == 0) {
    cost = 0.0;
    accumu_cost = 0.0;
    return;
  }

  if (!forwar_simu_success) {
    cost = common_data->max_cost;
    accumu_cost = common_data->max_cost;
    return;
  }

  // // longi acc
  // if (control.acceleration > common_data->acc_longi_max ||
  //     control.acceleration < common_data->acc_longi_min) {
  //   cost = common_data->max_cost;
  //   accumu_cost = common_data->max_cost;
  //   if (kDebugPrint) {
  //     SolverInfoLog::Instance()->error("longi_acc failed, a " +
  //                                      std::to_string(control.acceleration)
  //                                      +
  //                                      " debug_info " + print_debug_info());
  //   }

  //   return;
  // }

  // // longi jerk
  // auto& pre_control = pre_node_ptr_->get_control();
  // double jerk = (control.acceleration - pre_control.acceleration) /
  // common_data->delta_t;
  // if (jerk > common_data->jerk_max || jerk < common_data->jerk_min) {
  //   cost = common_data->max_cost;
  //   accumu_cost = common_data->max_cost;
  //   SolverInfoLog::Instance()->error("longi_jerk failed, debug_info " +
  //                                    print_debug_info());
  //   return;
  // }

  // // lat acc
  // double acc_lat = state.velocity * control.omega;
  // if (acc_lat > common_data->acc_lat_max ||
  //     acc_lat < common_data->acc_lat_min) {
  //   cost = common_data->max_cost;
  //   accumu_cost = common_data->max_cost;
  //   if (kDebugPrint) {
  //     SolverInfoLog::Instance()->error(
  //         "lat_acc failed, a_lat " + std::to_string(acc_lat) + " vel " +
  //         std::to_string(state.velocity) + " omega " +
  //         std::to_string(control.omega) + " debug_info " +
  //         print_debug_info());
  //   }

  //   return;
  // }

  // lat dis
  if (frame <= 20 && std::abs(ego_pos_frenet.l) > 1.85) {
    cost = common_data->max_cost;
    accumu_cost = common_data->max_cost;
    if (kDebugPrint) {
      SolverInfoLog::Instance()->error("lat_dis failed, l " +
                                       std::to_string(ego_pos_frenet.l));
    }
    return;
  }

  double cost_obs_collision = 0.0;
  bool collision_with_static, collision_with_dynamic;
  check_collision(common_data, nn_traj, frame, control, state, ego_pos_frenet,
                  ego_polygon, ego_s_range, ego_l_range, collision_with_static,
                  collision_with_dynamic);
  if (collision_with_static) {
    cost = common_data->max_cost;
    accumu_cost = common_data->max_cost;
    return;
  }
  if (collision_with_dynamic) {
    if (nn_traj) {
      if (frame < 15) {
        cost = common_data->max_cost;
        accumu_cost = common_data->max_cost;
        return;
      } else {
        cost_obs_collision =
            common_data->cost_obs_collision_base * (31 - frame) / 16.0;
      }
    } else {
      if (frame < 23) {
        cost = common_data->max_cost;
        accumu_cost = common_data->max_cost;
        return;
      } else {
        cost_obs_collision =
            common_data->cost_obs_collision_base * (31 - frame) / 16.0;
      }
    }
  }

  double cost_lat_dis = 0.0;
  constexpr double kWeightCostLatDis = 1.0;
  if (std::abs(ego_pos_frenet.l) > 0.7) {
    cost_lat_dis = kWeightCostLatDis * std::abs(ego_pos_frenet.l);
    // if (kDebugPrint) {
    //   SolverInfoLog::Instance()->error("lat_dis diverges, l " +
    //                                    std::to_string(ego_pos_frenet.l)
    //                                    +
    //                                    " debug_info " + print_debug_info());
    // }
  }

  double cost_acc = 0.0;
  constexpr double kWeightCostAcc = 10.0;
  double abs_acc = std::abs(control.acceleration);
  if (abs_acc > 1.5) {
    cost_acc = kWeightCostAcc * abs_acc;
  }

  double cost_acc_lat = 0.0;
  constexpr double kWeightCostAccLat = 10.0;
  double abs_acc_lat = std::abs(state.velocity * control.omega);
  if (abs_acc_lat > 1.5) {
    cost_acc_lat = kWeightCostAccLat * abs_acc_lat;
  }

  double cost_s = 0.0;
  constexpr double kWeightCostS = -0.01;
  cost_s = kWeightCostS * ego_pos_frenet.s;

  accumu_cost = cost_lat_dis + cost_acc + cost_acc_lat + cost_obs_collision +
                pre_accumu_cost;
  cost = accumu_cost + cost_s;

  if (kDebugPrint) {
    SolverInfoLog::Instance()->error(
        "cost " + std::to_string(cost) + " cost_lat_dis " +
        std::to_string(cost_lat_dis) + " cost_s " + std::to_string(cost_s) +
        " cost_acc " + std::to_string(cost_acc) + " cost_acc_lat " +
        std::to_string(cost_acc_lat) + " cost_obs_collision " +
        std::to_string(cost_obs_collision));
  }
  return;
}

void TrajNodeUtil::check_collision(
    const std::shared_ptr<InteractionSearchCommonData>& common_data,
    const bool nn_traj,
    const int frame,
    const ControlVariable& control,
    const StateVariable& state,
    const MathUtils::FrenetPoint2D& ego_pos_frenet,
    const std::vector<MathUtils::Point2D>& ego_polygon,
    const MathUtils::Range& ego_s_range,
    const MathUtils::Range& ego_l_range,
    bool& collision_with_static,
    bool& collision_with_dynamic) {
  collision_with_static = false;
  collision_with_dynamic = false;
  if (frame == 0) {
    return;
  }

  for (auto& pt : ego_polygon) {
    if (common_data->osp_env->check_freespace_collision(pt)) {
      collision_with_static = true;
      if (kDebugPrint) {
        SolverInfoLog::Instance()->error("collision_with_static, pt_x " +
                                         std::to_string(pt.x) + " pt_y " +
                                         std::to_string(pt.y));
      }
      break;
    }
  }
  if (collision_with_static) {
    // early return
    return;
  }

  std::vector<int> check_obs;
  if (nn_traj) {
    for (auto& obs_pair : common_data->obs_info) {
      auto& obs_id = obs_pair.first;
      check_obs.emplace_back(obs_id);
    }
  } else {
    for (auto& agent_id : common_data->interact_target) {
      // only check interactive agent to speed up
      if (common_data->obs_info.find(agent_id) == common_data->obs_info.end()) {
        continue;
      }
      check_obs.emplace_back(agent_id);
    }
    for (auto& obs_pair : common_data->resoveld_agent_set) {
      auto& obs_id = obs_pair.first;
      auto& obs_decision = obs_pair.second;
      if (common_data->obs_info.find(obs_id) == common_data->obs_info.end()) {
        continue;
      }
      if (obs_decision == InteractionDecision::InteractionDecision_IGNORE) {
        continue;
      }
      check_obs.emplace_back(obs_id);
    }
  }

  for (auto& obs_id : check_obs) {
    auto& obs_info = common_data->obs_info[obs_id];
    auto& obs_traj = obs_info.traj;
    if (frame > static_cast<int>(obs_traj.size()) - 1) {
      continue;
    }
    auto& obs_pt = obs_traj[frame];
    if (check_obs_collision(ego_s_range, ego_l_range, obs_pt)) {
      collision_with_dynamic = true;
      if (kDebugPrint) {
        SolverInfoLog::Instance()->error(
            "collision_with_dynamic, obs_id " + std::to_string(obs_id) +
            " ego x " + std::to_string(state.position.x) + " y " +
            std::to_string(state.position.y) + " obs x " +
            std::to_string(obs_pt.position.x) + " y " +
            std::to_string(obs_pt.position.y));
      }

      break;
    }

    // todo: check obs ttc
  }
}

bool TrajNodeUtil::check_obs_collision(
    const MathUtils::Range& ego_s_range,
    const MathUtils::Range& ego_l_range,
    ObstacleTrajectoryPointInternal& obs_pt) {
  // check by sl overlap
  MathUtils::Range obs_s_range(obs_pt.min_s, obs_pt.max_s);
  MathUtils::Range obs_l_range(obs_pt.min_l, obs_pt.max_l);
  if (ego_s_range.IsOverlap(obs_s_range) &&
      ego_l_range.IsOverlap(obs_l_range)) {
    // todo: may need ellipse model to get more accurate res
    return true;
  }
  return false;
}

StateVariable TrajNodeUtil::step(const StateVariable& state,
                                 const ControlVariable& action,
                                 const double delta_t) {
  double end_vel = state.velocity + action.acceleration * delta_t;
  if (action.acceleration < 0.0) {
    if (state.velocity < 1e-2) {
      StateVariable next_state = state;
      next_state.velocity = 0.0;
      return next_state;
    }
    if (end_vel < 0.0) {
      double t = state.velocity / std::abs(action.acceleration);
      StateVariable next_state;
      next_state.position.x =
          state.position.x +
          0.5 * (std::cos(state.theta) +
                 std::cos(state.theta + action.omega * t)) *
              (state.velocity * t + 0.5 * action.acceleration * t * t);
      next_state.position.y =
          state.position.y +
          0.5 * (std::sin(state.theta) +
                 std::sin(state.theta + action.omega * t)) *
              (state.velocity * t + 0.5 * action.acceleration * t * t);
      next_state.velocity = 0.0;
      next_state.theta =
          MathUtils::normalize_angle(state.theta + action.omega * t);
      return next_state;
    }
  }

  StateVariable next_state;
  next_state.position.x = state.position.x +
                          0.5 *
                              (std::cos(state.theta) +
                               std::cos(state.theta + action.omega * delta_t)) *
                              (state.velocity * delta_t +
                               0.5 * action.acceleration * delta_t * delta_t);
  next_state.position.y = state.position.y +
                          0.5 *
                              (std::sin(state.theta) +
                               std::sin(state.theta + action.omega * delta_t)) *
                              (state.velocity * delta_t +
                               0.5 * action.acceleration * delta_t * delta_t);
  next_state.velocity = state.velocity + action.acceleration * delta_t;
  next_state.theta =
      MathUtils::normalize_angle(state.theta + action.omega * delta_t);

  return next_state;
}

void TrajNodeUtil::generate_corner_points_by_rear_center(
    std::vector<MathUtils::Point2D>& ego_polygon,
    const MathUtils::Point2D& position,
    const double& theta,
    const double& rear_wheel_center_to_front_right_angle,
    const double& rear_wheel_center_to_front_right_dis,
    const double& rear_wheel_center_to_rear_right_angle,
    const double& rear_wheel_center_to_rear_right_dis) {
  ego_polygon.clear();

  auto& rear_wheel_center_origin_point = position;
  auto& rear_wheel_center_origin_heading = theta;
  auto& rear_wheel_center_to_front_left_angle =
      rear_wheel_center_to_front_right_angle;
  auto& rear_wheel_center_to_front_left_dis =
      rear_wheel_center_to_front_right_dis;
  auto& rear_wheel_center_to_rear_left_angle =
      rear_wheel_center_to_rear_right_angle;
  auto& rear_wheel_center_to_rear_left_dis =
      rear_wheel_center_to_rear_right_dis;

  // front_right_corner
  double front_right_corner_angle =
      rear_wheel_center_to_front_right_angle - rear_wheel_center_origin_heading;
  MathUtils::Point2D front_right =
      rear_wheel_center_origin_point +
      rear_wheel_center_to_front_right_dis *
          MathUtils::Point2D(std::cos(front_right_corner_angle),
                             -std::sin(front_right_corner_angle));
  ego_polygon.emplace_back(front_right);

  // front_left_corner
  double front_left_corner_angle =
      rear_wheel_center_to_front_left_angle + rear_wheel_center_origin_heading;
  MathUtils::Point2D front_left =
      rear_wheel_center_origin_point +
      rear_wheel_center_to_front_left_dis *
          MathUtils::Point2D(std::cos(front_left_corner_angle),
                             std::sin(front_left_corner_angle));
  ego_polygon.emplace_back(front_left);

  // rear_left_corner
  double rear_left_corner_angle =
      rear_wheel_center_to_rear_left_angle - rear_wheel_center_origin_heading;
  MathUtils::Point2D rear_left =
      rear_wheel_center_origin_point +
      rear_wheel_center_to_rear_left_dis *
          MathUtils::Point2D(-std::cos(rear_left_corner_angle),
                             std::sin(rear_left_corner_angle));
  ego_polygon.emplace_back(rear_left);

  // rear_right_corner
  double rear_right_corner_angle =
      rear_wheel_center_to_rear_right_angle + rear_wheel_center_origin_heading;
  MathUtils::Point2D rear_right =
      rear_wheel_center_origin_point +
      rear_wheel_center_to_rear_right_dis *
          MathUtils::Point2D(-std::cos(rear_right_corner_angle),
                             -std::sin(rear_right_corner_angle));
  ego_polygon.emplace_back(rear_right);
}

// ======================

void ForwardSimuPursuitLine::update_free_path(double origin_s) {
  constexpr double kFollowPointInterval = 1.0;
  constexpr double kFollowLineLength = 50.0;

  follow_points_.clear();
  for (double s = origin_s; s <= origin_s + kFollowLineLength;
       s += kFollowPointInterval) {
    ForwardSimuPursuitPoint pt;
    pt.pos.s = s;
    pt.pos.l = 0.0;
    pt.type =
        ForwardSimuPursuitPointType::ForwardSimuPursuitPointType_BASE_LINE;
    follow_points_.emplace_back(pt);
  }
}

bool ForwardSimuPursuitLine::update_nudge_path(double start_s,
                                               double end_s,
                                               double ref_l,
                                               ForwardSimuPursuitPointType type,
                                               int obs_id) {
  if (follow_points_.empty()) {
    return true;
  }
  if (end_s < start_s) {
    return true;
  }
  if (start_s > follow_points_.back().pos.s) {
    return true;
  }

  // if (kDebugPrint) {
  //   SolverInfoLog::Instance()->error(
  //       "update_nudge_path, start_s " + std::to_string(start_s) + " end_s " +
  //       std::to_string(end_s) + " ref_l " + std::to_string(ref_l) + " type "
  //       +
  //       std::to_string(static_cast<int>(type)) + " obs_id " +
  //       std::to_string(obs_id));
  // }

  auto it = std::find_if(
      follow_points_.begin(), follow_points_.end(),
      [&](const ForwardSimuPursuitPoint& a) { return a.pos.s >= start_s; });
  int index = std::distance(follow_points_.begin(), it);
  while (index < follow_points_.size() - 1 &&
         follow_points_[index].pos.s < end_s) {
    // if (kDebugPrint) {
    //   SolverInfoLog::Instance()->error(
    //       "follow_points index " + std::to_string(index) + " s " +
    //       std::to_string(follow_points_[index].pos.s));
    // }

    auto& pt = follow_points_[index];
    if (type ==
        ForwardSimuPursuitPointType::ForwardSimuPursuitPointType_LEFT_NUDGE) {
      if (ref_l < pt.pos.l) {
        index++;
        continue;
      }
      if (pt.type == ForwardSimuPursuitPointType::
                         ForwardSimuPursuitPointType_RIGHT_NUDGE) {
        if (kDebugPrint) {
          SolverInfoLog::Instance()->error(
              "update_nudge_path failed, old_type " +
              std::to_string(static_cast<int>(pt.type)) + " new_type " +
              std::to_string(static_cast<int>(type)) + " old_l " +
              std::to_string(pt.pos.l) + " new_l " + std::to_string(ref_l) +
              " obs_id " + std::to_string(obs_id));
        }

        return false;
      }

      // if (kDebugPrint) {
      //   SolverInfoLog::Instance()->error(
      //       "update_nudge_path, old_type " +
      //       std::to_string(static_cast<int>(pt.type)) + " new_type " +
      //       std::to_string(static_cast<int>(type)) + " old_l " +
      //       std::to_string(pt.pos.l) + " new_l " + std::to_string(ref_l) +
      //       " obs_id " + std::to_string(obs_id));
      // }

      pt.pos.l = ref_l;
      pt.type =
          ForwardSimuPursuitPointType::ForwardSimuPursuitPointType_LEFT_NUDGE;
    } else if (type == ForwardSimuPursuitPointType::
                           ForwardSimuPursuitPointType_RIGHT_NUDGE) {
      if (ref_l > pt.pos.l) {
        index++;
        continue;
      }
      if (pt.type ==
          ForwardSimuPursuitPointType::ForwardSimuPursuitPointType_LEFT_NUDGE) {
        if (kDebugPrint) {
          SolverInfoLog::Instance()->error(
              "update_nudge_path failed, old_type " +
              std::to_string(static_cast<int>(pt.type)) + " new_type " +
              std::to_string(static_cast<int>(type)) + " old_l " +
              std::to_string(pt.pos.l) + " new_l " + std::to_string(ref_l) +
              " obs_id " + std::to_string(obs_id));
        }

        return false;
      }

      // if (kDebugPrint) {
      //   SolverInfoLog::Instance()->error(
      //       "update_nudge_path, old_type " +
      //       std::to_string(static_cast<int>(pt.type)) + " new_type " +
      //       std::to_string(static_cast<int>(type)) + " old_l " +
      //       std::to_string(pt.pos.l) + " new_l " + std::to_string(ref_l) +
      //       " obs_id " + std::to_string(obs_id));
      // }

      pt.pos.l = ref_l;
      pt.type =
          ForwardSimuPursuitPointType::ForwardSimuPursuitPointType_RIGHT_NUDGE;
    }

    // constexpr double kObsNudgeLateralDisMax = 4.0;
    // if (std::abs(pt.pos.l) > kObsNudgeLateralDisMax) {
    //   if (kDebugPrint) {
    //     SolverInfoLog::Instance()->error("update_nudge_path failed, ref_l " +
    //                                      std::to_string(pt.pos.l));
    //   }

    //   return false;
    // }
    index++;
  }
  return true;
}

// ======================

void InteractionTrajNode::init(
    int frame,
    const std::shared_ptr<std::unordered_map<int, InteractionDecision>>&
        obs_tag,
    InteractionTrajNode* pre_node_ptr,
    const std::shared_ptr<InteractionSearchCommonData>& common_data) {
  frame_ = frame;
  obs_tag_ = obs_tag;
  pre_node_ptr_ = pre_node_ptr;
  common_data_ = common_data;

  info_.reset();
  next_node_.clear();
}

void InteractionTrajNode::run(std::queue<InteractionTrajNode*>& node_queue) {
  forward_simu();

  compute_cost();

  gen_child(node_queue);

  save_end_node();
}

void InteractionTrajNode::forward_simu() {
  info_.forwar_simu_success = true;

  if (frame_ == 0) {
    info_.state = common_data_->planning_origin.state;
    info_.control = common_data_->planning_origin.control;

    ControlVariable temp_control;
    temp_control.acceleration = 0.0;
    temp_control.omega = 0.0;
    auto temp_state =
        TrajNodeUtil::step(info_.state, temp_control, common_data_->delta_t);
    next_speed_limit_ =
        common_data_->osp_env->get_lane_point_speed_limit(temp_state.position);
    return;
  }

  // auto ego_heading_bpp = MathUtils::normalize_angle(
  //     common_data_->osp_env->get_heading_at_s(ego_pos_frenet.s));
  // auto ego_speed_bpp =
  //     pre_state.velocity * math::cos(ego_heading_bpp - pre_state.theta);

  // acc
  info_.control.acceleration = forward_simu_acc(info_.forwar_simu_success);
  if (!info_.forwar_simu_success) {
    return;
  }

  // omega
  info_.control.omega = forward_simu_omega(info_.forwar_simu_success);
  if (!info_.forwar_simu_success) {
    return;
  }

  // step
  info_.state = TrajNodeUtil::step(pre_node_ptr_->get_state(), info_.control,
                                   common_data_->delta_t);

  // ego info for this frame
  info_.ego_pos_frenet =
      common_data_->osp_env->cartesian_to_frenet(info_.state.position);
  TrajNodeUtil::generate_corner_points_by_rear_center(
      info_.ego_polygon, info_.state.position, info_.state.theta,
      common_data_->vehicle_param.rear_wheel_center_to_front_right_angle,
      common_data_->vehicle_param.rear_wheel_center_to_front_right_dis,
      common_data_->vehicle_param.rear_wheel_center_to_rear_right_angle,
      common_data_->vehicle_param.rear_wheel_center_to_rear_right_dis);
  double ego_max_s, ego_min_s, ego_max_l, ego_min_l;
  common_data_->osp_env->max_min_sl(info_.ego_polygon, ego_max_s, ego_min_s,
                                    ego_max_l, ego_min_l);
  info_.ego_s_range = MathUtils::Range(ego_min_s, ego_max_s);
  info_.ego_l_range = MathUtils::Range(ego_min_l, ego_max_l);

  // pre aiming for speed_limit
  ControlVariable temp_control;
  temp_control.acceleration = 0.0;
  temp_control.omega = 0.0;
  auto temp_state =
      TrajNodeUtil::step(info_.state, temp_control, common_data_->delta_t);
  next_speed_limit_ =
      common_data_->osp_env->get_lane_point_speed_limit(temp_state.position);
}

double InteractionTrajNode::forward_simu_acc(bool& can_process) {
  double res_acc = 0.0;

  auto& pre_state = pre_node_ptr_->get_state();
  auto& pre_pos_frenet = pre_node_ptr_->get_pos_frenet();
  auto& pre_s_range = pre_node_ptr_->get_s_range();
  auto& speed_limit = pre_node_ptr_->get_next_speed_limit();
  auto free_acc_tmp = ILQR::LongitudinalOperator::calc_idm_acc(
      common_data_->idm_param, 0.0, pre_state.velocity, 0.0, speed_limit, 0);
  res_acc = free_acc_tmp.first;

  std::vector<int> follow_obs;
  auto& obs_tag = *obs_tag_;
  for (auto& obs_pair : obs_tag) {
    auto& obs_id = obs_pair.first;
    auto& obs_decision = obs_pair.second;
    if (common_data_->obs_info.find(obs_id) == common_data_->obs_info.end()) {
      continue;
    }
    if (obs_decision != InteractionDecision::InteractionDecision_YIELD) {
      continue;
    }
    auto& agent_info = common_data_->interact_target_info[obs_id];
    if (frame_ - 1 >= agent_info.involved_end_frame) {
      continue;
    }
    follow_obs.emplace_back(obs_id);
  }
  for (auto& obs_pair : common_data_->resoveld_agent_set) {
    auto& obs_id = obs_pair.first;
    auto& obs_decision = obs_pair.second;
    if (common_data_->obs_info.find(obs_id) == common_data_->obs_info.end()) {
      continue;
    }
    if (obs_decision != InteractionDecision::InteractionDecision_YIELD) {
      continue;
    }
    follow_obs.emplace_back(obs_id);
  }

  for (auto& obs_id : follow_obs) {
    auto& obs_info = common_data_->obs_info[obs_id];
    auto& obs_traj = obs_info.traj;
    if (frame_ > obs_traj.size()) {
      continue;
    }
    auto& obs_pt = obs_traj[frame_ - 1];
    auto& obs_pos_frenet = obs_pt.pos_frenet;
    if (obs_pos_frenet.s < pre_pos_frenet.s) {
      can_process = false;
      if (kDebugPrint) {
        SolverInfoLog::Instance()->error("forward_simu_acc failed, obs_id " +
                                         std::to_string(obs_id));
      }
      return res_acc;
      // continue;
    }
    double delta_s = std::max(1e-3, obs_pt.min_s - pre_s_range.high());
    auto obs_speed_bpp = obs_pt.v.dot(MathUtils::Point2D(
        std::cos(pre_state.theta), std::sin(pre_state.theta)));
    auto acc_tmp = ILQR::LongitudinalOperator::calc_idm_acc(
        common_data_->idm_param, delta_s, pre_state.velocity, obs_speed_bpp,
        speed_limit, 1);
    res_acc = std::min(res_acc, acc_tmp.first);

    if (kDebugPrint) {
      SolverInfoLog::Instance()->error(
          "forward_simu_acc, obs_id " + std::to_string(obs_id) + " ego_v " +
          std::to_string(pre_state.velocity) + " ego_theta " +
          std::to_string(pre_state.velocity) + " delta_s " +
          std::to_string(delta_s) + " obs_speed_bpp " +
          std::to_string(obs_speed_bpp) + " acc " +
          std::to_string(acc_tmp.first));
    }
  }

  res_acc = std::max(common_data_->acc_longi_min,
                     std::min(common_data_->acc_longi_max, res_acc));
  return res_acc;
}

double InteractionTrajNode::forward_simu_omega(bool& can_process) {
  double res_omega = 0.0;
  can_process = true;

  auto& pre_state = pre_node_ptr_->get_state();
  auto& pre_control = pre_node_ptr_->get_control();
  auto& pre_pos_frenet = pre_node_ptr_->get_pos_frenet();
  auto& pre_s_range = pre_node_ptr_->get_s_range();

  ForwardSimuPursuitLine follow_line;
  follow_line.update_free_path(pre_pos_frenet.s);

  // for (auto& pt : follow_line.get_points()) {
  //   if (kDebugPrint) {
  //     SolverInfoLog::Instance()->error(
  //         "follow_line s " + std::to_string(pt.pos.s) + " l " +
  //         std::to_string(pt.pos.l) + " type " +
  //         std::to_string(static_cast<int>(pt.type)));
  //   }
  // }

  // update follow_line by obs

  std::unordered_map<int, InteractionDecision> follow_obs;
  auto& obs_tag = *obs_tag_;
  for (auto& obs_pair : obs_tag) {
    auto& obs_id = obs_pair.first;
    auto& obs_decision = obs_pair.second;
    if (common_data_->obs_info.find(obs_id) == common_data_->obs_info.end()) {
      continue;
    }
    if (obs_decision != InteractionDecision::InteractionDecision_LEFT_NUDGE &&
        obs_decision != InteractionDecision::InteractionDecision_RIGHT_NUDGE) {
      continue;
    }
    follow_obs.insert(obs_pair);
  }
  for (auto& obs_pair : common_data_->resoveld_agent_set) {
    auto& obs_id = obs_pair.first;
    auto& obs_decision = obs_pair.second;
    if (common_data_->obs_info.find(obs_id) == common_data_->obs_info.end()) {
      continue;
    }
    if (obs_decision != InteractionDecision::InteractionDecision_LEFT_NUDGE &&
        obs_decision != InteractionDecision::InteractionDecision_RIGHT_NUDGE) {
      continue;
    }
    follow_obs.insert(obs_pair);
  }

  for (auto& obs_pair : follow_obs) {
    auto& obs_id = obs_pair.first;
    auto& obs_decision = obs_pair.second;
    auto& obs_info = common_data_->obs_info[obs_id];
    auto& obs_traj = obs_info.traj;

    constexpr int kObsNudgeConsiderHorizon = 20;
    int start_frame = frame_ - 1;
    int end_frame = std::min(start_frame + kObsNudgeConsiderHorizon,
                             common_data_->horizon + 1);
    for (int cur_frame = start_frame; cur_frame <= end_frame; ++cur_frame) {
      if (cur_frame >= obs_traj.size()) {
        break;
      }
      auto& obs_pt = obs_traj[cur_frame];
      auto& obs_pos_frenet = obs_pt.pos_frenet;
      MathUtils::Range obs_s_range(obs_pt.min_s, obs_pt.max_s);
      if (obs_pos_frenet.s < pre_pos_frenet.s &&
          !obs_s_range.IsOverlap(pre_s_range)) {
        if (kDebugPrint) {
          SolverInfoLog::Instance()->error(
              "forward_simu_omega longi over obs_id " + std::to_string(obs_id) +
              " ego_s low " + std::to_string(pre_s_range.low()) +
              " ego_s high " + std::to_string(pre_s_range.high()) +
              " obs_s low " + std::to_string(obs_s_range.low()) +
              " obs_s high " + std::to_string(obs_s_range.high()));
        }
        break;
      }

      constexpr double kObsNudgeLongiBufferFrontLength = 4.0;
      constexpr double kObsNudgeLongiBufferBackLength = 6.0;
      constexpr double kObsNudgeLateralBufferDis = 0.3;
      double kObsNudgeLateralRefDis =
          common_data_->vehicle_param.half_width + kObsNudgeLateralBufferDis;

      // if (kDebugPrint) {
      //   SolverInfoLog::Instance()->error(
      //       " obs_id " + std::to_string(obs_id) + " frame " +
      //       std::to_string(cur_frame) + " min_s " +
      //       std::to_string(obs_pt.min_s) + " max_s " +
      //       std::to_string(obs_pt.max_s) + " min_l " +
      //       std::to_string(obs_pt.min_l) + " max_l " +
      //       std::to_string(obs_pt.max_l));
      // }

      if (obs_decision == InteractionDecision::InteractionDecision_LEFT_NUDGE) {
        if (!follow_line.update_nudge_path(
                obs_pt.min_s - kObsNudgeLongiBufferBackLength,
                obs_pt.max_s + kObsNudgeLongiBufferFrontLength,
                obs_pt.max_l + kObsNudgeLateralRefDis,
                ForwardSimuPursuitPointType::
                    ForwardSimuPursuitPointType_LEFT_NUDGE,
                obs_id)) {
          can_process = false;
          if (kDebugPrint) {
            SolverInfoLog::Instance()->error(
                "forward_simu_omega failed, obs_id " + std::to_string(obs_id));
          }

          return res_omega;
        }
      } else {
        if (!follow_line.update_nudge_path(
                obs_pt.min_s - kObsNudgeLongiBufferBackLength,
                obs_pt.max_s + kObsNudgeLongiBufferFrontLength,
                obs_pt.min_l - kObsNudgeLateralRefDis,
                ForwardSimuPursuitPointType::
                    ForwardSimuPursuitPointType_RIGHT_NUDGE,
                obs_id)) {
          can_process = false;
          if (kDebugPrint) {
            SolverInfoLog::Instance()->error(
                "forward_simu_omega failed, obs_id " + std::to_string(obs_id));
          }

          return res_omega;
        }
      }
    }
  }

  // for (auto& pt : follow_line.get_points()) {
  //   if (kDebugPrint) {
  //     SolverInfoLog::Instance()->error(
  //         "follow_line s " + std::to_string(pt.pos.s) + " l " +
  //         std::to_string(pt.pos.l) + " type " +
  //         std::to_string(static_cast<int>(pt.type)));
  //   }
  // }

  // find target_point
  constexpr double kLookAheadDisMin = 5.0;
  constexpr double kLookAheadDisMax = 40.0;

  MathUtils::Point2D target_point;
  bool found_target_point = false;
  double look_ahead_dis = 2.0 * pre_state.velocity;
  look_ahead_dis =
      std::min(std::max(look_ahead_dis, kLookAheadDisMin), kLookAheadDisMax);
  auto& follow_points = follow_line.get_points();
  for (auto& pt : follow_points) {
    auto cart_point = common_data_->osp_env->frenet_to_cartesian(pt.pos);
    auto vec = cart_point - pre_state.position;
    auto vec_len = vec.norm();
    if (vec_len < kLookAheadDisMin) {
      continue;
    }
    if (vec_len > look_ahead_dis) {
      found_target_point = true;
      target_point = cart_point;
      if (kDebugPrint) {
        SolverInfoLog::Instance()->error(
            "look_ahead_dis " + std::to_string(look_ahead_dis) + " vec_len " +
            std::to_string(vec_len) + " target_point x " +
            std::to_string(target_point.x) + " y " +
            std::to_string(target_point.y) + " type " + "road" +
            " frenet_pos s " + std::to_string(pt.pos.s) + " l " +
            std::to_string(pt.pos.l));
      }
      break;
    }
    if (pt.type == ForwardSimuPursuitPointType_LEFT_NUDGE ||
        pt.type == ForwardSimuPursuitPointType_RIGHT_NUDGE) {
      found_target_point = true;
      target_point = cart_point;
      if (kDebugPrint) {
        SolverInfoLog::Instance()->error(
            "look_ahead_dis " + std::to_string(look_ahead_dis) + " vec_len " +
            std::to_string(vec_len) + " target_point x " +
            std::to_string(target_point.x) + " y " +
            std::to_string(target_point.y) + " type " +
            std::to_string(static_cast<int>(pt.type)) + " frenet_pos s " +
            std::to_string(pt.pos.s) + " l " + std::to_string(pt.pos.l));
      }
      break;
    }
  }
  if (!found_target_point) {
    target_point =
        common_data_->osp_env->frenet_to_cartesian(follow_points.back().pos);
  }

  // compute delta and omega
  double delta = TrajNodeUtil::pp_control(
      target_point, pre_state.position, pre_state.theta,
      common_data_->vehicle_param.wheel_base);
  res_omega = pre_state.velocity * std::tan(delta) /
              common_data_->vehicle_param.wheel_base;
  double max_omega_by_param;
  MathUtils::interpolate<double, double>(
      info_.state.velocity, max_omega_by_param, common_data_->omega_max_v_list,
      common_data_->omega_max_omega_list);
  double min_omega_by_param = -max_omega_by_param;
  double max_omega_by_acc_lat =
      common_data_->acc_lat_max / std::max(1e-4, info_.state.velocity);
  double min_omega_by_acc_lat =
      -common_data_->acc_lat_max / std::max(1e-4, info_.state.velocity);
  double max_omega_by_omega_dot =
      pre_control.omega + common_data_->omega_dot_max * common_data_->delta_t;
  double min_omega_by_omega_dot =
      pre_control.omega - common_data_->omega_dot_max * common_data_->delta_t;
  double min_omega =
      std::max(std::max(min_omega_by_acc_lat, min_omega_by_param),
               min_omega_by_omega_dot);
  double max_omega =
      std::min(std::min(max_omega_by_acc_lat, max_omega_by_param),
               max_omega_by_omega_dot);
  res_omega = std::min(std::max(res_omega, min_omega), max_omega);

  if (kDebugPrint) {
    SolverInfoLog::Instance()->error(
        "forward_simu_omega, target_point x " + std::to_string(target_point.x) +
        " y " + std::to_string(target_point.y) + " delta " +
        std::to_string(delta) + " res_omega " + std::to_string(res_omega));
  }

  return res_omega;
}

void InteractionTrajNode::compute_cost() {
  double pre_accumu_cost = 0.0;
  if (frame_ > 0) {
    pre_accumu_cost = pre_node_ptr_->get_accumu_cost();
  }
  TrajNodeUtil::calc_cost(
      common_data_, false, frame_, info_.forwar_simu_success, info_.control,
      info_.state, info_.ego_pos_frenet, info_.ego_polygon, info_.ego_s_range,
      info_.ego_l_range, pre_accumu_cost, info_.cost, info_.accumu_cost);
}

void InteractionTrajNode::gen_child(
    std::queue<InteractionTrajNode*>& node_queue) {
  if (frame_ == 0) {
    next_node_.clear();
    auto decision_combine_list = gen_decision_combine();
    for (auto& decision_combine : decision_combine_list) {
      next_node_.emplace_back(InteractionTrajNode());
      auto& back_node = next_node_.back();
      back_node.init(frame_ + 1, decision_combine, this, common_data_);
    }
    for (auto& node : next_node_) {
      node_queue.push(&node);
    }
    return;
  }
  if (frame_ >= common_data_->horizon) {
    return;
  }
  if (info_.cost > common_data_->max_cost_threshold) {
    return;
  }

  next_node_.clear();

  next_node_.emplace_back(InteractionTrajNode());
  auto& back_node = next_node_.back();
  back_node.init(frame_ + 1, obs_tag_, this, common_data_);
  node_queue.push(&back_node);
}

void InteractionTrajNode::save_end_node() {
  if (!next_node_.empty()) {
    return;
  }
  if (frame_ == 0) {
    return;
  }
  if (frame_ == common_data_->horizon &&
      info_.cost < common_data_->max_cost_threshold) {
    common_data_->success_end_node_list.emplace_back(this);
  } else {
    common_data_->failed_end_node_list.emplace_back(this);
  }
}

std::vector<std::shared_ptr<std::unordered_map<int, InteractionDecision>>>
InteractionTrajNode::gen_decision_combine() {
  std::vector<std::shared_ptr<std::unordered_map<int, InteractionDecision>>>
      res;

  std::vector<int> ids;
  for (auto& agent_id : common_data_->interact_target) {
    ids.emplace_back(agent_id);
  }
  if (ids.empty()) {
    return res;
  }
  std::unordered_map<int, InteractionDecision> cur_combine;
  gen_decision_combine_dfs(0, ids, cur_combine, res);
  return res;
}

void InteractionTrajNode::gen_decision_combine_dfs(
    int index,
    std::vector<int>& ids,
    std::unordered_map<int, InteractionDecision>& cur_combine,
    std::vector<std::shared_ptr<std::unordered_map<int, InteractionDecision>>>&
        res_combine_list) {
  if (index >= ids.size()) {
    auto one_combine =
        std::make_shared<std::unordered_map<int, InteractionDecision>>();
    *one_combine = cur_combine;
    res_combine_list.emplace_back(one_combine);
    return;
  }

  auto& id = ids[index];
  cur_combine[id] = InteractionDecision::InteractionDecision_YIELD;
  gen_decision_combine_dfs(index + 1, ids, cur_combine, res_combine_list);
  cur_combine.erase(id);

  cur_combine[id] = InteractionDecision::InteractionDecision_LEFT_NUDGE;
  gen_decision_combine_dfs(index + 1, ids, cur_combine, res_combine_list);
  cur_combine.erase(id);

  cur_combine[id] = InteractionDecision::InteractionDecision_RIGHT_NUDGE;
  gen_decision_combine_dfs(index + 1, ids, cur_combine, res_combine_list);
  cur_combine.erase(id);

  auto& target_info = common_data_->interact_target_info[id];
  if (target_info.involved_start_frame >= 5) {
    cur_combine[id] = InteractionDecision::InteractionDecision_SURPASS;
    gen_decision_combine_dfs(index + 1, ids, cur_combine, res_combine_list);
    cur_combine.erase(id);
  }
}

SampleTrajDebug InteractionTrajNode::gen_path_from_root_to_cur() {
  SampleTrajDebug res;
  res.obs_tag = *obs_tag_;

  InteractionTrajNode* cur_node = this;
  while (cur_node != nullptr) {
    SamplePointDebug pt;
    pt.state = cur_node->info_.state;
    pt.control = cur_node->info_.control;
    pt.forward_success = cur_node->info_.forwar_simu_success;
    res.traj_points.emplace_back(pt);
    cur_node = cur_node->pre_node_ptr_;
  }
  std::reverse(res.traj_points.begin(), res.traj_points.end());
  return res;
}

// ======================

void NNTrajNode::init(
    int frame,
    NNTrajNode* pre_node_ptr,
    const std::shared_ptr<InteractionSearchCommonData>& common_data,
    const SamplePoint& traj_point) {
  frame_ = frame;
  pre_node_ptr_ = pre_node_ptr;
  common_data_ = common_data;

  info_.reset();
  info_.state = traj_point.state;
  info_.control = traj_point.control;
}

void NNTrajNode::run() {
  forward_simu();

  compute_cost();
}

void NNTrajNode::forward_simu() {
  info_.forwar_simu_success = true;

  // todo: maybe need smooth traj

  info_.ego_pos_frenet =
      common_data_->osp_env->cartesian_to_frenet(info_.state.position);
  TrajNodeUtil::generate_corner_points_by_rear_center(
      info_.ego_polygon, info_.state.position, info_.state.theta,
      common_data_->vehicle_param.rear_wheel_center_to_front_right_angle,
      common_data_->vehicle_param.rear_wheel_center_to_front_right_dis,
      common_data_->vehicle_param.rear_wheel_center_to_rear_right_angle,
      common_data_->vehicle_param.rear_wheel_center_to_rear_right_dis);
  double ego_max_s, ego_min_s, ego_max_l, ego_min_l;
  common_data_->osp_env->max_min_sl(info_.ego_polygon, ego_max_s, ego_min_s,
                                    ego_max_l, ego_min_l);
  info_.ego_s_range = MathUtils::Range(ego_min_s, ego_max_s);
  info_.ego_l_range = MathUtils::Range(ego_min_l, ego_max_l);
}

void NNTrajNode::compute_cost() {
  double pre_accumu_cost = 0.0;
  if (frame_ > 0) {
    pre_accumu_cost = pre_node_ptr_->get_accumu_cost();
  }
  TrajNodeUtil::calc_cost(
      common_data_, true, frame_, info_.forwar_simu_success, info_.control,
      info_.state, info_.ego_pos_frenet, info_.ego_polygon, info_.ego_s_range,
      info_.ego_l_range, pre_accumu_cost, info_.cost, info_.accumu_cost);
}
}
