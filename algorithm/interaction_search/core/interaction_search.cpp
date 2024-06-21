#include "vector"
#include "memory"
#include "unordered_map"
#include <cmath>
#include <chrono>

#include "utils.h"
#include "nn_tool.h"
#include "interaction_search.h"

namespace InteractionSearch {

MathUtils::FuncStatus InteractionSearchRunner::run(
    const InteractionSearchInput& input) {
  input_ = input;
  if (input_.osp_env == nullptr) {
    return MathUtils::FuncStatus::FuncFailed;
  }

  preprocess();

  calc_decision();

  gen_search_output();

  handle_nn_traj();

  gen_output();

  return MathUtils::FuncStatus::FuncSucceed;
}

void InteractionSearchRunner::preprocess() {
  // 1. init common_data
  common_data_->reset();
  common_data_->osp_env = input_.osp_env;

  common_data_->obs_info.clear();
  for (auto& raw_obs_pair : input_.obs_info) {
    auto& obs_id = raw_obs_pair.first;
    auto& raw_obs_info = raw_obs_pair.second;
    ObstacleInfoInternal new_obs_info;
    new_obs_info.is_static = raw_obs_info.is_static;
    new_obs_info.length = raw_obs_info.length;
    new_obs_info.width = raw_obs_info.width;
    new_obs_info.decision = raw_obs_info.decision;
    new_obs_info.traj.clear();
    for (int i = 0; i < raw_obs_info.traj.size(); ++i) {
      auto& obs_pt = raw_obs_info.traj[i];
      ObstacleTrajectoryPointInternal new_obs_pt;
      new_obs_pt.relative_time = obs_pt.relative_time;
      new_obs_pt.theta = obs_pt.theta;
      new_obs_pt.position = obs_pt.position;
      new_obs_pt.polygon = obs_pt.polygon;
      new_obs_pt.v = obs_pt.v;
      new_obs_pt.a = obs_pt.a;
      new_obs_pt.pos_frenet =
          input_.osp_env->cartesian_to_frenet(obs_pt.position);
      input_.osp_env->max_min_sl(obs_pt.polygon, new_obs_pt.max_s,
                                 new_obs_pt.min_s, new_obs_pt.max_l,
                                 new_obs_pt.min_l);

      // if (kDebugPrint) {
      //   SolverInfoLog::Instance()->error(
      //       " obs_id " + std::to_string(obs_id) + " index " +
      //       std::to_string(i) + " min_s " + std::to_string(new_obs_pt.min_s)
      //       +
      //       " max_s " + std::to_string(new_obs_pt.max_s) + " min_l " +
      //       std::to_string(new_obs_pt.min_l) + " max_l " +
      //       std::to_string(new_obs_pt.max_l));
      // }
      // for (auto& poly_pt : obs_pt.polygon) {
      //   if (kDebugPrint) {
      //     SolverInfoLog::Instance()->error(" poly_pt x " +
      //                                      std::to_string(poly_pt.x) + " y "
      //                                      +
      //                                      std::to_string(poly_pt.y));
      //   }
      // }

      new_obs_info.traj.emplace_back(new_obs_pt);
    }
    common_data_->obs_info[obs_id] = new_obs_info;
  }

  common_data_->planning_origin = input_.planning_origin;
  common_data_->planning_origin.state.theta =
      MathUtils::normalize_angle(common_data_->planning_origin.state.theta);
  common_data_->planning_origin.state.velocity =
      std::max(0.0, common_data_->planning_origin.state.velocity);

  common_data_->idm_param = input_.idm_param;

  auto& vehicle_param = common_data_->vehicle_param;
  vehicle_param.length = 4.886;
  vehicle_param.width = 2.1;
  vehicle_param.half_width = 0.5 * vehicle_param.width;
  vehicle_param.body_center_to_rear_wheel_center = 1.405;
  vehicle_param.r_4 =
      std::sqrt((vehicle_param.length * 0.5 * 0.25) *
                    (vehicle_param.length * 0.5 * 0.25) +
                (vehicle_param.width * 0.5) * (vehicle_param.width * 0.5));
  vehicle_param.wheel_base = 2.92;
  vehicle_param.rear_wheel_center_to_front =
      vehicle_param.body_center_to_rear_wheel_center +
      vehicle_param.length * 0.5;
  vehicle_param.rear_wheel_center_to_rear =
      vehicle_param.length * 0.5 -
      vehicle_param.body_center_to_rear_wheel_center;
  vehicle_param.rear_wheel_center_to_front_right_angle = std::atan2(
      vehicle_param.width * 0.5, vehicle_param.rear_wheel_center_to_front);
  vehicle_param.rear_wheel_center_to_front_right_dis =
      std::sqrt(vehicle_param.rear_wheel_center_to_front *
                    vehicle_param.rear_wheel_center_to_front +
                vehicle_param.width * vehicle_param.width * 0.25);
  vehicle_param.rear_wheel_center_to_front_left_angle =
      vehicle_param.rear_wheel_center_to_front_right_angle;
  vehicle_param.rear_wheel_center_to_front_left_dis =
      vehicle_param.rear_wheel_center_to_front_right_dis;
  vehicle_param.rear_wheel_center_to_rear_left_angle = std::atan2(
      vehicle_param.width * 0.5, vehicle_param.rear_wheel_center_to_rear);
  vehicle_param.rear_wheel_center_to_rear_left_dis =
      std::sqrt(vehicle_param.rear_wheel_center_to_rear *
                    vehicle_param.rear_wheel_center_to_rear +
                vehicle_param.width * vehicle_param.width * 0.25);
  vehicle_param.rear_wheel_center_to_rear_right_angle =
      vehicle_param.rear_wheel_center_to_rear_left_angle;
  vehicle_param.rear_wheel_center_to_rear_right_dis =
      vehicle_param.rear_wheel_center_to_rear_left_dis;

  ego_origin_pos_frenet_ = input_.osp_env->cartesian_to_frenet(
      input_.planning_origin.state.position);
  TrajNodeUtil::generate_corner_points_by_rear_center(
      ego_origin_polygon_, input_.planning_origin.state.position,
      input_.planning_origin.state.theta,
      common_data_->vehicle_param.rear_wheel_center_to_front_right_angle,
      common_data_->vehicle_param.rear_wheel_center_to_front_right_dis,
      common_data_->vehicle_param.rear_wheel_center_to_rear_right_angle,
      common_data_->vehicle_param.rear_wheel_center_to_rear_right_dis);
  double ego_max_s, ego_min_s, ego_max_l, ego_min_l;
  input_.osp_env->max_min_sl(ego_origin_polygon_, ego_max_s, ego_min_s,
                             ego_max_l, ego_min_l);
  ego_origin_s_range_ = MathUtils::Range(ego_min_s, ego_max_s);
  ego_origin_l_range_ = MathUtils::Range(ego_min_l, ego_max_l);
  // auto ego_theta =
  // MathUtils::normalize_angle(input_.planning_origin.state.theta);

  // 2. pick resoveld_agent and unresolved_agent

  resoveld_agent_set_.clear();
  unresolved_agent_set_.clear();

  constexpr double kIgnoreObsByEuclideanDistThr = 200.0 * 200.0;

  for (auto& obs_pair : common_data_->obs_info) {
    auto& obs_id = obs_pair.first;
    auto& obs_info = obs_pair.second;
    auto& obs_traj = obs_info.traj;

    if (obs_info.decision != InteractionDecision::InteractionDecision_UNKNOWN) {
      resoveld_agent_set_[obs_id] = obs_info.decision;
      continue;
    }
    if (obs_traj.empty()) {
      resoveld_agent_set_[obs_id] =
          InteractionDecision::InteractionDecision_IGNORE;
      continue;
    }

    double delta_x =
        obs_traj.front().position.x - input_.planning_origin.state.position.x;
    double delta_y =
        obs_traj.front().position.y - input_.planning_origin.state.position.y;
    if (delta_x * delta_x + delta_y * delta_y > kIgnoreObsByEuclideanDistThr) {
      resoveld_agent_set_[obs_id] =
          InteractionDecision::InteractionDecision_IGNORE;
      continue;
    }

    auto& obs_pos_frenet = obs_traj.front().pos_frenet;
    MathUtils::Range obs_origin_s_range(obs_traj.front().min_s,
                                        obs_traj.front().max_s);

    if (kDebugPrint) {
      SolverInfoLog::Instance()->error(
          "obs " + std::to_string(obs_id) + " pos x " +
          std::to_string(obs_traj.front().position.x) + " pos y " +
          std::to_string(obs_traj.front().position.y) + " frenet_pos s " +
          std::to_string(obs_pos_frenet.s) + " frenet_pos l " +
          std::to_string(obs_pos_frenet.l));
    }

    if (obs_pos_frenet.s < ego_origin_pos_frenet_.s) {
      if (obs_origin_s_range.IsOverlap(ego_origin_s_range_)) {
        if (obs_pos_frenet.l > 0.0) {
          resoveld_agent_set_[obs_id] =
              InteractionDecision::InteractionDecision_RIGHT_NUDGE;
        } else {
          resoveld_agent_set_[obs_id] =
              InteractionDecision::InteractionDecision_LEFT_NUDGE;
        }
      } else {
        resoveld_agent_set_[obs_id] =
            InteractionDecision::InteractionDecision_IGNORE;
      }

      continue;
    }

    // todo: obs theta value not right?
    // constexpr double kHeadingSameDirectionMin = -90 * M_PI / 180;
    // constexpr double kHeadingSameDirectionMax = 90 * M_PI / 180;
    // auto baseline_theta = MathUtils::normalize_angle(
    //     input_.osp_env->get_heading_at_s(obs_pos_frenet.s));
    // auto obs_theta = MathUtils::normalize_angle(obs_traj.front().theta);
    // double delta_theta = MathUtils::normalize_angle(baseline_theta -
    // obs_theta);

    // *SolverInfoLog::Instance()
    //     << "baseline_theta " +
    //     std::to_string(input_.osp_env->get_heading_at_s(
    //                                obs_pos_frenet.s)) +
    //            " obs_theta " + std::to_string(obs_traj.front().theta) +
    //            " delta_theta " + std::to_string(baseline_theta - obs_theta)
    //            +
    //            "baseline_theta " + std::to_string(baseline_theta) +
    //            " obs_theta " + std::to_string(obs_theta) + " delta_theta "
    //            +
    //            std::to_string(delta_theta);

    unresolved_agent_set_.insert(obs_id);
  }

  // 3. select and sort agent

  auto selected_agents = select_agents(input_.osp_env, unresolved_agent_set_);

  constexpr double kIgnoreObsByLatDistBuffer = 1.5;
  double ignore_obs_lat_dist_thr =
      kIgnoreObsByLatDistBuffer + common_data_->vehicle_param.half_width;
  MathUtils::Range ignore_obs_lat_range(-ignore_obs_lat_dist_thr,
                                        ignore_obs_lat_dist_thr);

  std::vector<int> to_remove_ids;
  for (auto& obs_id : unresolved_agent_set_) {
    auto it =
        std::find_if(selected_agents.begin(), selected_agents.end(),
                     [&](const AgentSelectInfo& a) { return a.id == obs_id; });
    if (it == selected_agents.end()) {
      auto& obs_info = common_data_->obs_info[obs_id];
      auto& obs_traj = obs_info.traj;
      auto& obs_pos_frenet = obs_traj.front().pos_frenet;
      auto& max_l = obs_traj.front().max_l;
      auto& min_l = obs_traj.front().min_l;
      MathUtils::Range obs_lat_range(min_l, max_l);
      if (!obs_lat_range.IsOverlap(ignore_obs_lat_range)) {
        resoveld_agent_set_[obs_id] =
            InteractionDecision::InteractionDecision_IGNORE;
        to_remove_ids.emplace_back(obs_id);
        continue;
      }

      if (obs_pos_frenet.l > 0.0) {
        resoveld_agent_set_[obs_id] =
            InteractionDecision::InteractionDecision_RIGHT_NUDGE;
      } else {
        resoveld_agent_set_[obs_id] =
            InteractionDecision::InteractionDecision_LEFT_NUDGE;
      }
      to_remove_ids.emplace_back(obs_id);
    }
  }
  for (auto& obs_id : to_remove_ids) {
    unresolved_agent_set_.erase(obs_id);
  }
  common_data_->resoveld_agent_set = resoveld_agent_set_;

  if (kDebugPrint) {
    for (auto& agent_pair : resoveld_agent_set_) {
      SolverInfoLog::Instance()->error(
          "resoveld_agent " + std::to_string(agent_pair.first) + " tag " +
          std::to_string(static_cast<int>(agent_pair.second)));
    }
    for (auto& id : unresolved_agent_set_) {
      SolverInfoLog::Instance()->error("unresolved  " + std::to_string(id));
    }
  }

  constexpr int kConsiderAgentNumber = 4;
  common_data_->interact_target.clear();
  common_data_->interact_target_info.clear();
  for (int i = 0; i < selected_agents.size(); ++i) {
    if (i >= kConsiderAgentNumber) {
      break;
    }
    auto& obs_id = selected_agents[i].id;
    common_data_->interact_target.emplace_back(obs_id);
    common_data_->interact_target_info[obs_id] = selected_agents[i];
  }
}

void InteractionSearchRunner::calc_decision() {
  common_data_->success_end_node_list.clear();
  common_data_->failed_end_node_list.clear();
  search_traj_success_ = false;
  success_search_trajs_.clear();
  failed_search_trajs_.clear();

  if (common_data_->interact_target.empty()) {
    SolverInfoLog::Instance()->error("InteractionSearchRunner, no target");
    return;
  }

  // forward traj by bfs
  std::queue<InteractionTrajNode*> node_queue;
  root_node_.init(0, nullptr, nullptr, common_data_);
  node_queue.push(&root_node_);
  while (!node_queue.empty()) {
    auto& cur_node = node_queue.front();
    cur_node->run(node_queue);
    if (kDebugPrint) {
      SolverInfoLog::Instance()->error(cur_node->print_debug_info());
    }
    node_queue.pop();
  }

  // save res traj

  if (!common_data_->success_end_node_list.empty()) {
    search_traj_success_ = true;
  }

  std::vector<int> success_end_node_list_index;
  for (int i = 0; i < common_data_->success_end_node_list.size(); ++i) {
    success_end_node_list_index.emplace_back(i);
  }
  std::sort(success_end_node_list_index.begin(),
            success_end_node_list_index.end(),
            [&](const int& index_a, const int& index_b) -> bool {
              auto& node_a = common_data_->success_end_node_list[index_a];
              auto& node_b = common_data_->success_end_node_list[index_b];
              double cost_a = node_a->get_cost();
              double cost_b = node_b->get_cost();
              return cost_a < cost_b;
            });
  for (auto& index : success_end_node_list_index) {
    auto& node = common_data_->success_end_node_list[index];
    auto traj = node->gen_path_from_root_to_cur();
    success_search_trajs_.emplace_back(traj);
  }

  for (auto& node : common_data_->failed_end_node_list) {
    auto traj = node->gen_path_from_root_to_cur();
    failed_search_trajs_.emplace_back(traj);
  }
}

std::vector<AgentSelectInfo> InteractionSearchRunner::select_agents(
    const std::shared_ptr<OSPEnv>& osp_env,
    const std::unordered_set<int>& unresolved_agent_set) {
  std::vector<AgentSelectInfo> res;

  // 1.0 选取交互agent by 横向范围
  auto agent_by_lat_dis =
      select_agents_by_lat_dis(osp_env, unresolved_agent_set);

  for (auto& agent : agent_by_lat_dis) {
    SolverInfoLog::Instance()->error("agent_by_lat_dis " +
                                     std::to_string(agent.id) + " select_s " +
                                     std::to_string(agent.select_s));
  }

  // // 2.0 选取交互agent by 轨迹交汇时间差
  // auto next_agent_set = unresolved_agent_set;
  // for (auto& agent : agent_by_lat_dis) {
  //   if (next_agent_set.find(agent.id) != next_agent_set.end()) {
  //     next_agent_set.erase(agent.id);
  //   }
  // }
  // auto agent_by_traj_merge =
  //     select_agents_by_traj_merge(osp_env, next_agent_set);

  // for (auto& agent : agent_by_traj_merge) {
  //   *SolverInfoLog::Instance()
  //       << "agent_by_traj_merge " + std::to_string(agent.id) + " s " +
  //              std::to_string(agent.merge_point.merge_s) +
  //              " ego_time_to_merge " +
  //              std::to_string(agent.merge_point.ego_time_to_merge) +
  //              " obs_time_to_merge " +
  //              std::to_string(agent.merge_point.obs_time_to_merge) +
  //              " ttm_diff " + std::to_string(agent.merge_point.ttm_diff);
  // }

  // 3.0 计算agent重要性, 按重要性排序d
  res.insert(res.end(), agent_by_lat_dis.begin(), agent_by_lat_dis.end());
  // res.insert(res.end(), agent_by_traj_merge.begin(),
  // agent_by_traj_merge.end());
  std::sort(res.begin(), res.end(),
            [](const AgentSelectInfo& a, const AgentSelectInfo& b) -> bool {
              double time_a = a.select_s;
              double time_b = b.select_s;
              // if (a.select_type == AgentSelectType::AgentSelectType_BY_MERGE)
              // {
              //   time_a = a.merge_point.merge_s;
              // }
              // if (b.select_type == AgentSelectType::AgentSelectType_BY_MERGE)
              // {
              //   time_b = b.merge_point.merge_s;
              // }
              return time_a < time_b;
            });

  for (auto& agent : res) {
    SolverInfoLog::Instance()->error("sort_agent " + std::to_string(agent.id));
  }

  return res;
}

std::vector<AgentSelectInfo> InteractionSearchRunner::select_agents_by_lat_dis(
    const std::shared_ptr<OSPEnv>& osp_env,
    const std::unordered_set<int>& unresolved_agent_set) {
  std::vector<AgentSelectInfo> res;

  constexpr double kNotNudgeObsConsiderTime = 8.0;
  double not_nudge_obs_consider_dis =
      kNotNudgeObsConsiderTime * common_data_->planning_origin.state.velocity;
  not_nudge_obs_consider_dis = std::max(not_nudge_obs_consider_dis, 50.0);

  constexpr double kSelectAgentByLatDistBuffer = 0.5;
  double select_agent_by_lat_dist_thr =
      kSelectAgentByLatDistBuffer + common_data_->vehicle_param.half_width;
  MathUtils::Range select_range(-select_agent_by_lat_dist_thr,
                                select_agent_by_lat_dist_thr);

  for (auto& obs_id : unresolved_agent_set) {
    if (common_data_->obs_info.find(obs_id) == common_data_->obs_info.end()) {
      continue;
    }
    auto& obs_info = common_data_->obs_info[obs_id];
    auto& obs_traj = obs_info.traj;
    if (obs_traj.empty()) {
      continue;
    }

    bool obs_involved = false;
    int obs_involved_start_frame = 0;
    int obs_involved_end_frame = 0;
    AgentSelectInfo agent;
    for (int i = 0; i < obs_traj.size(); ++i) {
      auto& obs_pt = obs_traj[i];
      MathUtils::Range obs_range(obs_pt.min_l, obs_pt.max_l);
      if (obs_range.IsOverlap(select_range) &&
          obs_pt.pos_frenet.s > ego_origin_pos_frenet_.s) {
        obs_involved = true;
        obs_involved_start_frame = i;
        agent.id = obs_id;
        agent.select_type = AgentSelectType::AgentSelectType_BY_LAT_DIS;
        agent.select_s = obs_pt.pos_frenet.s;
        break;
      }
    }
    if (obs_involved) {
      auto& obs_first_pt = obs_info.traj.front();
      auto obs_speed_bpp = obs_first_pt.v.dot(MathUtils::Point2D(
          std::cos(common_data_->planning_origin.state.theta),
          std::sin(common_data_->planning_origin.state.theta)));
      if (obs_speed_bpp > 0.0 &&
          obs_first_pt.pos_frenet.s - ego_origin_pos_frenet_.s >
              not_nudge_obs_consider_dis) {
        SolverInfoLog::Instance()->error("filter target by longi_dis, id " +
                                         std::to_string(obs_id));
        continue;
      }

      obs_involved_end_frame = obs_traj.size();
      for (int i = obs_involved_start_frame + 1; i < obs_traj.size(); ++i) {
        auto& obs_pt = obs_traj[i];
        MathUtils::Range obs_range(obs_pt.min_l, obs_pt.max_l);
        if (!obs_range.IsOverlap(select_range) ||
            obs_pt.pos_frenet.s < ego_origin_pos_frenet_.s) {
          obs_involved_end_frame = i;
          break;
        }
      }
      agent.involved_start_frame = obs_involved_start_frame;
      agent.involved_end_frame = obs_involved_end_frame;
      res.emplace_back(agent);
    }
  }
  return res;
}

std::vector<AgentSelectInfo>
InteractionSearchRunner::select_agents_by_traj_merge(
    const std::shared_ptr<OSPEnv>& osp_env,
    const std::unordered_set<int>& unresolved_agent_set) {
  std::vector<AgentSelectInfo> res;

  for (auto& id : unresolved_agent_set) {
    if (input_.obs_info.find(id) == input_.obs_info.end()) {
      continue;
    }
    auto& obs_traj = input_.obs_info[id].traj;

    TrajMergePointInfo merge_point;
    if (check_merge_point_exist(osp_env, obs_traj, merge_point) &&
        std::abs(merge_point.ttm_diff) < 4.0) {
      AgentSelectInfo agent;
      agent.id = id;
      agent.select_type = AgentSelectType::AgentSelectType_BY_MERGE;
      agent.merge_point = merge_point;
      res.emplace_back(agent);
    }
  }

  return res;
}

bool InteractionSearchRunner::check_merge_point_exist(
    const std::shared_ptr<OSPEnv>& osp_env,
    const std::vector<ObstacleTrajectoryPoint>& obs_traj,
    TrajMergePointInfo& merge_point) {
  int n = obs_traj.size();
  if (n <= 1) {
    return false;
  }

  // step over obs的轨迹, check  (left/right) change to ego_traj,
  // get merge_point, check ttm
  int i = 0;
  int last_relative_pos = 0;
  bool found_merge_point = false;
  while (i < n) {
    auto& obs_pt = obs_traj[i].position;
    bool relative_left;

    auto obs_pos_frenet = osp_env->cartesian_to_frenet(obs_pt);
    if (obs_pos_frenet.l > 0) {
      relative_left = true;
    } else {
      relative_left = false;
    }

    int cur_relative_pos = relative_left ? 1 : -1;

    if (last_relative_pos != 0 && cur_relative_pos != last_relative_pos) {
      // relative_pos changes
      found_merge_point = true;
      break;
    }
    last_relative_pos = cur_relative_pos;
    ++i;
  }

  if (found_merge_point) {
    auto& obs_pt_pre = obs_traj[i - 1].position;
    auto& obs_pt = obs_traj[i].position;
    auto obs_pos_frenet = osp_env->cartesian_to_frenet(obs_pt);
    auto obs_pos_pre_frenet = osp_env->cartesian_to_frenet(obs_pt_pre);
    merge_point.merge_s = 0.5 * (obs_pos_frenet.s + obs_pos_pre_frenet.s);
    merge_point.ego_time_to_merge =
        (merge_point.merge_s - ego_origin_pos_frenet_.s) /
        std::abs(input_.planning_origin.state.velocity);
    merge_point.obs_time_to_merge = common_data_->delta_t * (i - 0.5);
    merge_point.ttm_diff =
        merge_point.obs_time_to_merge - merge_point.ego_time_to_merge;
    return true;
  }

  // pending: another 简化方案,
  // 只考虑agent当前时刻的位置和速度,投影到ego参考线上
  return false;
}

// return true means cur_pt is located left side with line_ab
bool InteractionSearchRunner::check_relative_pos(
    const MathUtils::Point2D& cur_pt,
    const MathUtils::Point2D& pt_a,
    const MathUtils::Point2D& pt_b) {
  double cross_product = (pt_b.x - pt_a.x) * (cur_pt.y - pt_a.y) -
                         (pt_b.y - pt_a.y) * (cur_pt.x - pt_a.x);
  if (cross_product > 0.0) {
    return true;
  }
  // AD_LINFO(TMP) << "cross_product " << cross_product;
  return false;
}

bool InteractionSearchRunner::get_merge_point(const MathUtils::Point2D& pt_a,
                                              const MathUtils::Point2D& pt_b,
                                              const MathUtils::Point2D& pt_c,
                                              const MathUtils::Point2D& pt_d,
                                              MathUtils::Point2D& merge_point) {
  bool line_1_perpendicular_to_x = MathUtils::is_float_equal(pt_a.x, pt_b.x);
  bool line_2_perpendicular_to_x = MathUtils::is_float_equal(pt_c.x, pt_d.x);
  if (line_1_perpendicular_to_x && line_2_perpendicular_to_x) {
    if (MathUtils::is_float_equal(pt_a.x, pt_c.x)) {
      merge_point = pt_a;
      return true;
    }
    return false;
  }
  if (line_1_perpendicular_to_x) {
    double k2 = (pt_d.y - pt_c.y) / (pt_d.x - pt_c.x);
    double b2 = pt_c.y - k2 * pt_c.x;
    merge_point.x = pt_a.x;
    merge_point.y = k2 * merge_point.x + b2;
    return true;
  }
  if (line_2_perpendicular_to_x) {
    double k1 = (pt_b.y - pt_a.y) / (pt_b.x - pt_a.x);
    double b1 = pt_a.y - k1 * pt_a.x;
    merge_point.x = pt_c.x;
    merge_point.y = k1 * merge_point.x + b1;
    return true;
  }

  double k1 = (pt_b.y - pt_a.y) / (pt_b.x - pt_a.x);
  double b1 = pt_a.y - k1 * pt_a.x;
  double k2 = (pt_d.y - pt_c.y) / (pt_d.x - pt_c.x);
  double b2 = pt_c.y - k2 * pt_c.x;
  if (MathUtils::is_float_equal(k1, k2)) {
    if (MathUtils::is_float_equal(b1, b2)) {
      merge_point = pt_a;
      return true;
    }
    return false;
  }
  merge_point.x = (b2 - b1) / (k1 - k2);
  merge_point.y = k1 * merge_point.x + b1;
  return true;
}

bool InteractionSearchRunner::check_obs_in_sector(
    const TrajectoryPoint& traj_point,
    const ObstacleTrajectoryPoint& obs_point) {
  auto section_info = get_roi_sector(traj_point);
  auto& sector_angle = section_info.first;
  auto& sector_l = section_info.second;
  auto norm_theta = MathUtils::normalize_angle(traj_point.theta);
  auto start_angle =
      MathUtils::normalize_angle(norm_theta - 0.5 * sector_angle);
  auto end_angle = MathUtils::normalize_angle(norm_theta + 0.5 * sector_angle);

  // center point
  if (is_point_in_sector(obs_point.position.x, obs_point.position.y,
                         traj_point.position.x, traj_point.position.y,
                         norm_theta, sector_l, start_angle, end_angle)) {
    return true;
  }
  // corner point
  for (auto& pt : obs_point.polygon) {
    if (is_point_in_sector(pt.x, pt.y, traj_point.position.x,
                           traj_point.position.y, norm_theta, sector_l,
                           start_angle, end_angle)) {
      return true;
    }
  }
  return false;
}

std::pair<double, double> InteractionSearchRunner::get_roi_sector(
    const TrajectoryPoint& traj_point) {
  std::pair<double, double> sector_info;

  std::vector<double> v_list = {20.0 / 3.6, 40.0 / 3.6, 60.0 / 3.6, 80.0 / 3.6};
  constexpr double kDegreeToRadian = 0.01745329251;
  std::vector<double> sector_angle_list = {
      100.0 * kDegreeToRadian, 60.0 * kDegreeToRadian, 30.0 * kDegreeToRadian,
      10.0 * kDegreeToRadian};
  std::vector<double> sector_length_list = {11.0, 15.0, 33.33, 44.44};

  double vel = std::fabs(traj_point.velocity);
  MathUtils::interpolate<double, double>(vel, sector_info.first, v_list,
                                         sector_angle_list);
  MathUtils::interpolate<double, double>(vel, sector_info.second, v_list,
                                         sector_length_list);
  return sector_info;
}

double InteractionSearchRunner::dist_point_to_vector(
    const MathUtils::Point2D& a, double theta, const MathUtils::Point2D& b) {
  MathUtils::Point2D vec_ab = b - a;
  MathUtils::Point2D vec_ac(std::cos(theta), std::sin(theta));
  double len_ad = vec_ab.dot(vec_ac);
  double len_ab = vec_ab.norm();
  double temp = len_ab * len_ab - len_ad * len_ad;
  if (temp < 0.0) {
    return 0.0;
  }
  return std::sqrt(temp);
}

bool InteractionSearchRunner::is_point_in_sector(double px,
                                                 double py,
                                                 double cx,
                                                 double cy,
                                                 double theta,
                                                 double r,
                                                 double start_angle,
                                                 double end_angle) {
  constexpr double kRoiPointFilterLatDis = 1.8;
  if (dist_point_to_vector(MathUtils::Point2D(cx, cy), theta,
                           MathUtils::Point2D(px, py)) >
      kRoiPointFilterLatDis) {
    return false;
  }

  // 计算点到扇形中心的距离
  double distance = std::hypot(px - cx, py - cy);

  // 检查距离是否在扇形半径内
  if (distance > r) {
    return false;  // 距离超过半径，点在扇形外
  }

  // 计算点相对于扇形中心的角度
  double rel_theta = MathUtils::normalize_angle(std::atan2(py - cy, px - cx));

  // 检查角度是否在扇形夹角范围内
  if (start_angle <= rel_theta && rel_theta <= end_angle) {
    return true;  // 角度在扇形夹角范围内，点在扇形内
  }

  return false;  // 角度不在扇形夹角范围内，点在扇形外
}

void InteractionSearchRunner::gen_search_output() {
  search_obs_traj_tag_.clear();

  std::unordered_map<int, InteractionDecision> res_tag = resoveld_agent_set_;
  if (!common_data_->interact_target.empty()) {
    if (!search_traj_success_ || success_search_trajs_.empty()) {
      SolverInfoLog::Instance()->error(
          "InteractionSearchRunner, search_traj failed");
    } else {
      SolverInfoLog::Instance()->error(
          "InteractionSearchRunner, search_traj_success");

      auto& search_tags = success_search_trajs_.front().obs_tag;
      for (auto& one_obs_tag : search_tags) {
        SolverInfoLog::Instance()->error(
            "InteractionSearchRunner search_res, id " +
            std::to_string(one_obs_tag.first) + " tag " +
            std::to_string(static_cast<int>(one_obs_tag.second)));
        res_tag.insert(one_obs_tag);
      }
    }
  }

  // fallback stratagy for remain unresolved obs

  constexpr double kConsiderCutinWidthBuffer = 0.4;
  double cutin_belief_thr_dist =
      kConsiderCutinWidthBuffer + common_data_->vehicle_param.half_width;
  MathUtils::Range cutin_range(-cutin_belief_thr_dist, cutin_belief_thr_dist);
  for (auto& obs_id : unresolved_agent_set_) {
    if (res_tag.find(obs_id) != res_tag.end()) {
      continue;
    }
    if (common_data_->obs_info.find(obs_id) == common_data_->obs_info.end()) {
      continue;
    }
    auto& obs_info = common_data_->obs_info[obs_id];
    auto& obs_traj = obs_info.traj;
    MathUtils::Range obs_origin_s_range(obs_traj.front().min_s,
                                        obs_traj.front().max_s);
    if (obs_origin_s_range.IsOverlap(ego_origin_s_range_)) {
      continue;
    }
    for (auto& pt : obs_traj) {
      if (pt.pos_frenet.s < ego_origin_pos_frenet_.s) {
        continue;
      }
      MathUtils::Range obs_range(pt.min_l, pt.max_l);
      if (obs_range.IsOverlap(cutin_range)) {
        SolverInfoLog::Instance()->error(
            "InteractionSearchRunner, fallback yield id " +
            std::to_string(obs_id));
        res_tag.insert(std::make_pair(
            obs_id, InteractionDecision::InteractionDecision_YIELD));
        break;
      }
    }
  }
  for (auto& obs_id : unresolved_agent_set_) {
    if (res_tag.find(obs_id) == res_tag.end()) {
      res_tag.insert(std::make_pair(
          obs_id, InteractionDecision::InteractionDecision_UNKNOWN));
      SolverInfoLog::Instance()->error(
          "InteractionSearchRunner, not handle id " + std::to_string(obs_id));
    }
  }

  for (auto& one_obs_tag : res_tag) {
    auto& obs_id = one_obs_tag.first;
    auto& single_tag = one_obs_tag.second;
    std::vector<InteractionDecision> traj_tag(common_data_->horizon + 1);
    if (single_tag == InteractionDecision::InteractionDecision_YIELD &&
        common_data_->interact_target_info.find(obs_id) !=
            common_data_->interact_target_info.end()) {
      auto& target_info = common_data_->interact_target_info[obs_id];
      for (int i = 0; i <= common_data_->horizon; ++i) {
        if (i < target_info.involved_end_frame) {
          traj_tag[i] = InteractionDecision::InteractionDecision_YIELD;
        } else {
          traj_tag[i] = InteractionDecision::InteractionDecision_UNKNOWN;
        }
      }
    } else {
      traj_tag.assign(common_data_->horizon + 1, single_tag);
    }
    search_obs_traj_tag_.insert(std::make_pair(obs_id, traj_tag));

    // for (int i = 0; i < traj_tag.size(); ++i) {
    //   SolverInfoLog::Instance()->error(
    //       "InteractionSearchRunner, traj_tag id " + std::to_string(obs_id) +
    //       " i " + std::to_string(i) + " res " +
    //       std::to_string(static_cast<int>(traj_tag[i])));
    // }
  }
}

void InteractionSearchRunner::handle_nn_traj() {
  success_nn_trajs_.clear();
  failed_nn_trajs_.clear();
  for (auto& nn_traj : input_.nn_trajs) {
    if (nn_traj.empty()) {
      continue;
    }
    std::vector<NNTrajNode> tmp_traj;
    bool traj_success = true;
    for (int i = 0; i < nn_traj.size(); ++i) {
      NNTrajNode* pre_node_ptr = nullptr;
      if (i > 0) {
        pre_node_ptr = &(tmp_traj.back());
      }

      tmp_traj.emplace_back(NNTrajNode());
      auto& back_node = tmp_traj.back();
      back_node.init(i, pre_node_ptr, common_data_, nn_traj[i]);
      back_node.run();

      if (back_node.get_cost() > common_data_->max_cost_threshold) {
        traj_success = false;
        break;
      }
    }
    if (!traj_success) {
      failed_nn_trajs_.emplace_back(tmp_traj);
      continue;
    }
    success_nn_trajs_.emplace_back(tmp_traj);
  }

  best_nn_traj_index_ = -1;
  double min_cost = std::numeric_limits<double>::max();
  for (int i = 0; i < success_nn_trajs_.size(); ++i) {
    auto& tmp_traj = success_nn_trajs_[i];
    if (tmp_traj.back().get_cost() < min_cost) {
      min_cost = tmp_traj.back().get_cost();
      best_nn_traj_index_ = i;
    }
  }
}

void InteractionSearchRunner::gen_output() {
  output_.reset();
  output_.search_traj_success = search_traj_success_;

  output_.use_nn_traj = false;
  if (best_nn_traj_index_ != -1) {
    output_.use_nn_traj = true;
  }

  if (output_.use_nn_traj) {
    auto& success_traj = success_nn_trajs_[best_nn_traj_index_];
    for (auto& pt : success_traj) {
      SamplePoint traj_pt;
      traj_pt.state = pt.get_state();
      traj_pt.control = pt.get_control();
      output_.best_traj.emplace_back(traj_pt);
    }

    // todo: temp output no any tag
    // output_.obs_tag;
  } else {
    if (output_.search_traj_success) {
      auto& success_traj = success_search_trajs_.front();
      for (auto& pt : success_traj.traj_points) {
        SamplePoint traj_pt;
        traj_pt.state = pt.state;
        traj_pt.control = pt.control;
        output_.best_traj.emplace_back(traj_pt);
      }
    }
    output_.obs_tag = search_obs_traj_tag_;
  }
}
}
