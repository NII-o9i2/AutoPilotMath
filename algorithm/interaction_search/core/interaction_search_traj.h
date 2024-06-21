#pragma once

#include "vector"
#include "memory"
#include "unordered_map"
#include <unordered_set>
#include <algorithm>
#include <queue>
#include <cmath>
#include "utils.h"
#include "interval.h"
#include "nn_tool.h"
#include "interaction_search_common.h"
#include <sstream>

namespace InteractionSearch {

class TrajNodeUtil {
 public:
  static void calc_cost(
      const std::shared_ptr<InteractionSearchCommonData> &common_data,
      const bool nn_traj,
      const int frame,
      const bool forwar_simu_success,
      const ControlVariable &control,
      const StateVariable &state,
      const MathUtils::FrenetPoint2D &ego_pos_frenet,
      const std::vector<MathUtils::Point2D> &ego_polygon,
      const MathUtils::Range &ego_s_range,
      const MathUtils::Range &ego_l_range,
      const double &pre_accumu_cost,
      double &cost,
      double &accumu_cost);

  static void check_collision(
      const std::shared_ptr<InteractionSearchCommonData> &common_data,
      const bool nn_traj,
      const int frame,
      const ControlVariable &control,
      const StateVariable &state,
      const MathUtils::FrenetPoint2D &ego_pos_frenet,
      const std::vector<MathUtils::Point2D> &ego_polygon,
      const MathUtils::Range &ego_s_range,
      const MathUtils::Range &ego_l_range,
      bool &collision_with_static,
      bool &collision_with_dynamic);

  static bool check_obs_collision(const MathUtils::Range &ego_s_range,
                                  const MathUtils::Range &ego_l_range,
                                  ObstacleTrajectoryPointInternal &obs_pt);

  static double pp_control(const MathUtils::Point2D &target_pose,
                           const MathUtils::Point2D &current_pose,
                           const double &current_theta,
                           const double &wheel_base) {
    double ld = (current_pose - target_pose).norm();
    double alfa = std::atan2(target_pose.y - current_pose.y,
                             target_pose.x - current_pose.x) -
                  current_theta;

    return std::atan(2 * wheel_base * std::sin(alfa) / ld);
  }

  static void generate_corner_points_by_rear_center(
      std::vector<MathUtils::Point2D> &ego_polygon,
      const MathUtils::Point2D &position,
      const double &theta,
      const double &rear_wheel_center_to_front_right_angle,
      const double &rear_wheel_center_to_front_right_dis,
      const double &rear_wheel_center_to_rear_right_angle,
      const double &rear_wheel_center_to_rear_right_dis);

  static StateVariable step(const StateVariable &state,
                            const ControlVariable &action,
                            const double delta_t);
};

class ForwardSimuPursuitLine {
 public:
  ForwardSimuPursuitLine() = default;
  ~ForwardSimuPursuitLine() = default;

  void update_free_path(double origin_s);

  bool update_nudge_path(double start_s,
                         double end_s,
                         double ref_l,
                         ForwardSimuPursuitPointType type,
                         int obs_id);

  const std::vector<ForwardSimuPursuitPoint> &get_points() {
    return follow_points_;
  }

 private:
  std::vector<ForwardSimuPursuitPoint> follow_points_;
};

class InteractionTrajNode {
 public:
  InteractionTrajNode() = default;
  ~InteractionTrajNode() = default;

  void init(int frame,
            const std::shared_ptr<std::unordered_map<int, InteractionDecision>>
                &obs_tag,
            InteractionTrajNode *pre_node_ptr,
            const std::shared_ptr<InteractionSearchCommonData> &common_data);

  void run(std::queue<InteractionTrajNode *> &node_queue);

  int get_frame() const { return frame_; }

  double get_cost() const { return info_.cost; }

  double get_accumu_cost() const { return info_.accumu_cost; }

  const StateVariable &get_state() const { return info_.state; }

  const ControlVariable &get_control() const { return info_.control; }

  const MathUtils::FrenetPoint2D &get_pos_frenet() const {
    return info_.ego_pos_frenet;
  }

  const double &get_next_speed_limit() const { return next_speed_limit_; }

  const MathUtils::Range &get_s_range() const { return info_.ego_s_range; }

  SampleTrajDebug gen_path_from_root_to_cur();

  const std::shared_ptr<std::unordered_map<int, InteractionDecision>>
      &get_obs_tag() const {
    return obs_tag_;
  }

  std::string print_debug_info() const {
    std::stringstream ss;
    ss << "frame " << frame_;

    std::stringstream ss_obs_tag;
    if (obs_tag_ != nullptr) {
      for (auto &obs_tag : *obs_tag_) {
        ss_obs_tag << " id " << obs_tag.first << " tag "
                   << static_cast<int>(obs_tag.second);
      }
    }
    ss << " obs_tag " << ss_obs_tag.str();
    ss << " forwar_success " << info_.forwar_simu_success;
    ss << " cost " << info_.cost << " acc_cost " << info_.accumu_cost;

    std::stringstream ss_next_node;
    for (auto &next_node : next_node_) {
      ss_next_node << " frame " << next_node.get_frame();
    }
    ss << " next_node " << ss_next_node.str();
    return ss.str();
  }

 private:
  void forward_simu();

  double forward_simu_acc(bool &can_process);

  double forward_simu_omega(bool &can_process);

  void compute_cost();

  void gen_child(std::queue<InteractionTrajNode *> &node_queue);

  void save_end_node();

  std::vector<std::shared_ptr<std::unordered_map<int, InteractionDecision>>>
  gen_decision_combine();

  void gen_decision_combine_dfs(
      int index,
      std::vector<int> &ids,
      std::unordered_map<int, InteractionDecision> &cur_combine,
      std::vector<std::shared_ptr<std::unordered_map<int, InteractionDecision>>>
          &res_combine_list);

 private:
  int frame_;
  std::shared_ptr<std::unordered_map<int, InteractionDecision>> obs_tag_ =
      nullptr;
  InteractionTrajNode *pre_node_ptr_ = nullptr;
  std::shared_ptr<InteractionSearchCommonData> common_data_ = nullptr;

  SamplePointInfo info_;
  double next_speed_limit_;

  std::vector<InteractionTrajNode> next_node_;
};

class NNTrajNode {
 public:
  NNTrajNode() = default;
  ~NNTrajNode() = default;

  void init(int frame,
            NNTrajNode *pre_node_ptr,
            const std::shared_ptr<InteractionSearchCommonData> &common_data,
            const SamplePoint &traj_point);

  void run();

  int get_frame() const { return frame_; }

  double get_cost() const { return info_.cost; }

  double get_accumu_cost() const { return info_.accumu_cost; }

  const StateVariable &get_state() const { return info_.state; }

  const ControlVariable &get_control() const { return info_.control; }

  const MathUtils::FrenetPoint2D &get_pos_frenet() const {
    return info_.ego_pos_frenet;
  }

  const MathUtils::Range &get_s_range() const { return info_.ego_s_range; }

  std::string print_debug_info() const {
    std::stringstream ss;
    ss << "frame " << frame_;
    ss << " cost " << info_.cost << " acc_cost " << info_.accumu_cost;
    return ss.str();
  }

 private:
  void forward_simu();

  void compute_cost();

 private:
  int frame_;
  NNTrajNode *pre_node_ptr_ = nullptr;
  std::shared_ptr<InteractionSearchCommonData> common_data_ = nullptr;

  SamplePointInfo info_;
};
}
