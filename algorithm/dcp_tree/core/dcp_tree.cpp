#include "core/dcp_tree.h"
#include "core/track_simulator.h"

namespace DCP_TREE {

void DCPTreeNode::calc_node_cost(double pre_cost, const double& init_ego_vel) {
  double weight_lc_count = 1.0;
  double weight_s = -2.0;
  double weight_v = -1.0;
  double cost_lc = 0.0;
  double cost_s = 0.0;
  double cost_v = 0.0;

  if (ongoing_action == DCPActionDir::LC_LEFT ||
      ongoing_action == DCPActionDir::LC_RIGHT) {
    cost_lc = weight_lc_count;
  }
  cost_s = weight_s * (data.accumulated_s / std::max(init_ego_vel, 1e-3));
  // cost_v = weight_v * (init_ego_vel - data.ego_pos.car_v);
  cost_v = weight_v * data.ego_pos.car_v;

  data.accumulated_cost = cost_lc + pre_cost;
  data.cost = data.accumulated_cost + cost_s + cost_v;

  data.debug_info.s = data.ego_pos.car_s;
  data.debug_info.v = data.ego_pos.car_v;
  data.debug_info.a = data.ego_pos.car_a;
  data.debug_info.cost_lc = cost_lc;
  data.debug_info.cost_s = cost_s;
  data.debug_info.cost_v = cost_v;
}

std::shared_ptr<DCPTreeNode> DCPTree::generate_next_node(
    DCPActionDir action,
    double next_t,
    std::shared_ptr<DCPTreeNode>& parent,
    DCP_TREE::IDMLongiSimulator& simulator,
    const DCP_TREE::Params& params) {
  if (parent == nullptr) {
    return nullptr;
  }
  DCPTreeNodeData data;
  data.obs_pos = simulator.get_update_lanes_obs();
  data.ego_pos = simulator.get_update_ego_info();
  // std::cout << "ego acc is:" << data.ego_pos.car_a << std::endl;
  data.accumulated_s = simulator.get_update_ego_info().car_s;
  data.ego_lane_id = static_cast<DCPLaneDir>(parent->data.ego_lane_id + action);
  data.tree_trace = parent->data.tree_trace;
  data.tree_trace.emplace_back(action);

  auto next_node = std::make_shared<DCPTreeNode>(data);
  next_node->parent = parent;
  next_node->current_time = next_t;
  next_node->ongoing_action = action;
  //      next_node->ego_position
  next_node->calc_node_cost(parent->data.accumulated_cost, params.init_ego_vel);
  parent->child_list.emplace_back(next_node);
  return next_node;
}

void DCPTree::update_tree(const DCPTreeNodeData& src) {
  auto root = std::make_shared<DCPTreeNode>(src);
  set_root(root);
  std::vector<std::shared_ptr<DCPTreeNode>> remain_nodes;
  std::vector<std::shared_ptr<DCPTreeNode>> next_remain_nodes;
  next_remain_nodes.emplace_back(root);
  DCP_TREE::IDMLongiSimulator idm_simulator;
  auto get_finish_count =
      [&](std::vector<std::shared_ptr<DCPTreeNode>>& node_list) {
        int i = 0;
        for (auto& node : node_list) {
          if (node->current_time < params_.max_time) {
            i++;
          }
        }
        return i;
      };
  // 1.0 set step time
  while (get_finish_count(next_remain_nodes) > 0) {
    // 2.0 time move to next action & update remain_nodes
    remain_nodes.swap(next_remain_nodes);
    next_remain_nodes.clear();
    for (auto& node : remain_nodes) {
      // 2.1 skip overtime action
      if (node->current_time > params_.max_time) {
        next_remain_nodes.emplace_back(node);
        continue;
      }
      double t = node->current_time + params_.delta_t;
      if (t - params_.max_time > 1e-2) continue;
      auto& pre_data = node->data;
      DCPActionDir next_action = LK;
      double next_action_dur_time = 0.0;

      // 2.2 skip no lead-one action, fill LK until over
      bool no_leader_car = std::all_of(
          pre_data.obs_pos[pre_data.ego_lane_id].begin(),
          pre_data.obs_pos[pre_data.ego_lane_id].end(),
          [ego_s = pre_data.ego_pos.car_s](const ILQR::IDMCarInfo& car_info) {
            return ego_s > car_info.car_s;
          });
      if (no_leader_car) {
        next_action = LK;
        next_action_dur_time = params_.max_time - t + 1e-2;
      }

      // 2.3 simulate LK action
      double belief = 0.0;
      idm_simulator.init(pre_data.obs_pos, pre_data.ego_pos,
                         pre_data.ego_lane_id, next_action, params_);
      belief =
          idm_simulator.forward_simu(next_action_dur_time + params_.delta_t);
      // double idm_confidence = idm_simulator.get_belief();
      auto next_node =
          generate_next_node(DCPActionDir::LK, t + next_action_dur_time, node,
                             idm_simulator, params_);
      next_remain_nodes.emplace_back(next_node);

      // 2.4 if LK time less max time than 1.0s , simulate LC action
      double lc_time = 4.0;
      if (t + next_action_dur_time < params_.max_time &&
          t + lc_time < params_.max_time) {
        // 2.4 simulate LC action LEFT
        // todo: @jojo normal lc & aggressive lc
        next_action = DCPActionDir::LC_LEFT;
        idm_simulator.init(pre_data.obs_pos, pre_data.ego_pos,
                           pre_data.ego_lane_id, next_action, params_);
        belief = idm_simulator.forward_simu(lc_time + params_.delta_t);
        if (belief > params_.confidence_th) {
          // 2.5 LC LEFT succeed
          auto next_lc_node = generate_next_node(
              DCPActionDir::LC_LEFT, t + lc_time, node, idm_simulator, params_);
          next_remain_nodes.emplace_back(next_lc_node);
        }

        // 2.6 simulate LC action RIGHT
        next_action = DCPActionDir::LC_RIGHT;
        idm_simulator.init(pre_data.obs_pos, pre_data.ego_pos,
                           pre_data.ego_lane_id, next_action, params_);
        belief = idm_simulator.forward_simu(lc_time + params_.delta_t);
        if (belief > params_.confidence_th) {
          // 2.7 LC RIGHT succeed
          auto next_lc_node =
              generate_next_node(DCPActionDir::LC_RIGHT, t + lc_time, node,
                                 idm_simulator, params_);
          next_remain_nodes.emplace_back(next_lc_node);
        }
      }
    }
  }
}

void DCPTree::set_IDM_params(const double& delta_t,
                             const int& lanes_size,
                             const double& init_ego_vel,
                             const double& desired_speed) {
  params_.delta_t = delta_t;
  params_.consider_lanes_size = lanes_size;
  params_.init_ego_vel = init_ego_vel;
  params_.idm_params.desired_spd = desired_speed;
}

void DCPTreeNode::show_tree_list(std::string trace,
                                 DCP_TREE::Path& path,
                                 std::vector<Path>& all_paths) {
  path.push_back(std::make_tuple(data.cost, current_time, ongoing_action,
                                 data.debug_info));
  std::string cur_str = std::to_string(static_cast<int>(current_time)) +
                        "_action:" + std::to_string(ongoing_action) + " " +
                        "_cost:" + std::to_string(data.cost);
  // for (auto& node : child_list) {
  //   node->show_tree_list(trace + cur_str);
  // }
  if (child_list.empty()) {
    double single_path_cost = 0.0;
    for (auto& item : path) {
      single_path_cost += std::get<0>(item);
    }
    all_paths.push_back(path);
    // std::cout << "path is:" << trace + cur_str << std::endl;
  } else {
    for (auto& node : child_list) {
      DCP_TREE::Path new_path = path;
      node->show_tree_list(trace +
                               std::to_string(static_cast<int>(current_time)) +
                               "_action:" + std::to_string(ongoing_action) +
                               "_cost:" + std::to_string(data.cost) + " ",
                           new_path, all_paths);
    }
  }
}

MathUtils::FuncStatus DCPTreeRunner::init(const DCPTreeInput& input) {
  if (input.obstacle_list.empty()) {
    return MathUtils::FuncStatus::FuncFailed;
  }
  input_ = input;
  return MathUtils::FuncStatus::FuncSucceed;
}

MathUtils::FuncStatus DCPTreeRunner::run() {
  DCP_TREE::DCPTree test_tree;
  DCP_TREE::DCPTreeNodeData root_node;
  root_node.obs_pos = input_.obstacle_list;
  root_node.ego_pos = input_.ego_info;
  test_tree.set_IDM_params(
      input_.dcp_params.delta_t, input_.dcp_params.consider_lanes_size,
      input_.ego_info.car_v, input_.dcp_params.idm_params.desired_spd);
  root_node.tree_trace.clear();
  test_tree.update_tree(root_node);
  test_tree.show();
  // set_output_and_debug info
  auto all_path = test_tree.get_all_path();
  size_t min_cost_path_index = 0;
  double min_cost = std::get<0>(all_path[0].back());

  for (size_t i = 1; i < all_path.size(); i++) {
    double current_path_last_cost = std::get<0>(all_path[i].back());
    if (current_path_last_cost < min_cost) {
      min_cost = current_path_last_cost;
      min_cost_path_index = i;
    }
  }
  DCP_TREE::SolverInfoLog::Instance()->log(
      "Path with minimum last step cost (Path number " +
      std::to_string(min_cost_path_index + 1) + "):");
  for (const auto& step : all_path[min_cost_path_index]) {
    DCP_TREE::SolverInfoLog::Instance()->log(
        "Time: " + std::to_string(std::get<1>(step)) +
        ", Action: " + std::to_string(std::get<2>(step)) +
        ", Cost: " + std::to_string(std::get<0>(step)));
  }
  DCP_TREE::SolverInfoLog::Instance()->log("Minimum last step cost: " +
                                           std::to_string(min_cost));
  // if (all_path[min_cost_path_index].size() > 1) {
  //   auto& min_path_second_elem = all_path[min_cost_path_index][1];
  //   output_.suggest_action_confidence = std::get<0>(min_path_second_elem);
  //   output_.suggest_action_relative_time = std::get<1>(min_path_second_elem);
  //   output_.suggest_action = std::get<2>(min_path_second_elem);
  // } else {
  //   DCP_TREE::SolverInfoLog::Instance()->error(
  //       "The minimum cost path does not have enough elements.");
  // }
  if (all_path[min_cost_path_index].size() > 1) {
    bool found_non_lk = false;
    size_t non_lk_index = 0;
    for (size_t i = 0; i < all_path[min_cost_path_index].size(); ++i) {
      auto action = std::get<2>(all_path[min_cost_path_index][i]);
      if (action != DCPActionDir::LK) {
        found_non_lk = true;
        non_lk_index = i;
        break;
      }
    }
    if (found_non_lk) {
      if (non_lk_index == 1) {
        output_.suggest_action_relative_time = 1;
      } else if (non_lk_index > 0) {
        auto& prev_elem = all_path[min_cost_path_index][non_lk_index - 1];
        output_.suggest_action_relative_time = std::get<1>(prev_elem);
      } else {
        output_.suggest_action_relative_time = 0;
      }
      auto& non_lk_elem = all_path[min_cost_path_index][non_lk_index];
      output_.suggest_action_confidence = std::get<0>(non_lk_elem);
      output_.suggest_action = std::get<2>(non_lk_elem);
    } else {
      output_.suggest_action_confidence = 0.0;
      output_.suggest_action = DCPActionDir::LK;
      output_.suggest_action_relative_time = 0.0;
      DCP_TREE::SolverInfoLog::Instance()->error(
          "The minimum cost path does not have an action different from LK.");
    }
  } else {
    DCP_TREE::SolverInfoLog::Instance()->error(
        "The minimum cost path does not have enough elements.");
  }
  // debug_.reset();
  // for (auto& item : all_path) {
  //   auto path = item.second;
  //   std::vector<int> node_id_list;
  //   int node_index = 0;
  //   for (auto& node : path) {
  //     DCP_TREE::DebugNode debug_node;
  //     debug_node.cost = std::get<0>(node);
  //     debug_node.time = std::get<1>(node);
  //     debug_node.action = std::get<2>(node);
  //     debug_node.id = node_index++;
  //     node_id_list.push_back(node_index);
  //     node_index++;
  //     debug_.node_debug_list[debug_node.id] = debug_node;
  //   }
  //   debug_.node_path_debug_list.push_back(node_id_list);
  // }
  all_paths_ = std::move(all_path);

  return MathUtils::FuncStatus::FuncSucceed;
}
}  // namespace DCP_TREE