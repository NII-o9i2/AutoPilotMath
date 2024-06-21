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
#include "interaction_search_traj.h"
#include <sstream>

namespace InteractionSearch {

class InteractionSearchRunner {
 public:
  InteractionSearchRunner() {
    common_data_ = std::make_shared<InteractionSearchCommonData>();
    reset();
  };
  ~InteractionSearchRunner() = default;

  MathUtils::FuncStatus run(const InteractionSearchInput &input);

  const InteractionSearchOutput &get_output() const { return output_; };

  const std::vector<SampleTrajDebug> &get_success_trajs() const {
    return success_search_trajs_;
  };

  const std::vector<SampleTrajDebug> &get_failed_trajs() const {
    return failed_search_trajs_;
  };

  const std::vector<std::vector<NNTrajNode>> &get_success_nn_trajs() const {
    return success_nn_trajs_;
  };

  const std::vector<std::vector<NNTrajNode>> &get_failed_nn_trajs() const {
    return failed_nn_trajs_;
  };

  void reset() {
    input_.reset();
    output_.reset();
  }

 private:
  void preprocess();

  std::vector<AgentSelectInfo> select_agents(
      const std::shared_ptr<OSPEnv> &osp_env,
      const std::unordered_set<int> &unresolved_agent_set);

  void calc_decision();

  void gen_output();

  void gen_search_output();

  void handle_nn_traj();

  std::vector<AgentSelectInfo> select_agents_by_lat_dis(
      const std::shared_ptr<OSPEnv> &osp_env,
      const std::unordered_set<int> &unresolved_agent_set);

  std::vector<AgentSelectInfo> select_agents_by_traj_merge(
      const std::shared_ptr<OSPEnv> &osp_env,
      const std::unordered_set<int> &unresolved_agent_set);

  bool check_merge_point_exist(
      const std::shared_ptr<OSPEnv> &osp_env,
      const std::vector<ObstacleTrajectoryPoint> &obs_traj,
      TrajMergePointInfo &merge_point);

  bool check_obs_in_sector(const TrajectoryPoint &traj_point,
                           const ObstacleTrajectoryPoint &obs_point);

  std::pair<double, double> get_roi_sector(const TrajectoryPoint &traj_point);

  double dist_point_to_vector(const MathUtils::Point2D &a,
                              double theta,
                              const MathUtils::Point2D &b);

  bool is_point_in_sector(double px,
                          double py,
                          double cx,
                          double cy,
                          double theta,
                          double r,
                          double start_angle,
                          double end_angle);

  bool check_relative_pos(const MathUtils::Point2D &cur_pt,
                          const MathUtils::Point2D &pt_a,
                          const MathUtils::Point2D &pt_b);

  bool get_merge_point(const MathUtils::Point2D &pt_a,
                       const MathUtils::Point2D &pt_b,
                       const MathUtils::Point2D &pt_c,
                       const MathUtils::Point2D &pt_d,
                       MathUtils::Point2D &merge_point);

 private:
  InteractionSearchInput input_;
  InteractionSearchOutput output_;

  std::unordered_map<int, InteractionDecision> resoveld_agent_set_;
  std::unordered_set<int> unresolved_agent_set_;
  MathUtils::FrenetPoint2D ego_origin_pos_frenet_;
  std::vector<MathUtils::Point2D> ego_origin_polygon_;
  MathUtils::Range ego_origin_s_range_;
  MathUtils::Range ego_origin_l_range_;
  std::shared_ptr<InteractionSearchCommonData> common_data_ = nullptr;

  bool search_traj_success_ = false;
  InteractionTrajNode root_node_;
  std::vector<SampleTrajDebug> success_search_trajs_;
  std::vector<SampleTrajDebug> failed_search_trajs_;
  std::unordered_map<int, std::vector<InteractionDecision>>
      search_obs_traj_tag_;

  std::vector<std::vector<NNTrajNode>> success_nn_trajs_;
  std::vector<std::vector<NNTrajNode>> failed_nn_trajs_;
  int best_nn_traj_index_ = -1;
};
}
