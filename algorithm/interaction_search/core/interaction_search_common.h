#pragma once

#include "vector"
#include "memory"
#include "unordered_map"
#include "utils.h"
#include "osp_env.h"
#include "core/ilqr_tree_interface.h"
#include "core/ilqr_tree_solver.h"

#ifdef ENGINEER_OPTION
#include "ad_log/ad_log.hpp"
#endif

namespace InteractionSearch {

constexpr bool kDebugPrint = false;

using FuncStatus = MathUtils::FuncStatus;
template <typename Section>
class FuncReturn {
 public:
  FuncStatus first;
  Section second;

  FuncReturn(const FuncStatus &f, const Section &s) : first(f), second(s) {}
};

struct ObstacleTrajectoryPoint {
  double relative_time;
  double theta;
  MathUtils::Point2D position;
  std::vector<MathUtils::Point2D> polygon;
  MathUtils::Point2D v;
  MathUtils::Point2D a;
};

enum InteractionDecision {
  InteractionDecision_UNKNOWN = 0,
  InteractionDecision_YIELD = 1,
  InteractionDecision_SURPASS,
  InteractionDecision_LEFT_NUDGE,
  InteractionDecision_RIGHT_NUDGE,
  InteractionDecision_IGNORE
};

struct ObstacleInfo {
  bool is_static;
  double length = 0.0;
  double width = 0.0;
  InteractionDecision decision =
      InteractionDecision::InteractionDecision_UNKNOWN;
  std::vector<ObstacleTrajectoryPoint> traj;
};

struct ObstacleTrajectoryPointInternal {
  double relative_time;
  double theta;
  MathUtils::Point2D position;
  std::vector<MathUtils::Point2D> polygon;
  MathUtils::Point2D v;
  MathUtils::Point2D a;

  MathUtils::FrenetPoint2D pos_frenet;
  double max_s;
  double min_s;
  double max_l;
  double min_l;
};

struct ObstacleInfoInternal {
  bool is_static;
  double length = 0.0;
  double width = 0.0;
  InteractionDecision decision =
      InteractionDecision::InteractionDecision_UNKNOWN;
  std::vector<ObstacleTrajectoryPointInternal> traj;
};

struct StateVariable {
  MathUtils::Point2D position;
  double theta = 0.0;
  double velocity = 0.0;
};

struct ControlVariable {
  double acceleration = 0.0;
  double omega = 0.0;  // d(theta) / dt
};

struct SamplePoint {
  StateVariable state;
  ControlVariable control;
};

struct SamplePointDebug {
  StateVariable state;
  ControlVariable control;
  bool forward_success;
};

struct SampleTrajDebug {
  std::vector<SamplePointDebug> traj_points;
  std::unordered_map<int, InteractionDecision> obs_tag;
};

struct SamplePointInfo {
  bool forwar_simu_success = false;
  ControlVariable control;
  StateVariable state;
  MathUtils::FrenetPoint2D ego_pos_frenet;
  std::vector<MathUtils::Point2D> ego_polygon;
  MathUtils::Range ego_s_range;
  MathUtils::Range ego_l_range;
  double cost = 0.0;
  double accumu_cost = 0.0;

  void reset() {
    forwar_simu_success = false;
    cost = 0.0;
    accumu_cost = 0.0;
  }
};

struct InteractionSearchInput {
  SamplePoint planning_origin;
  std::shared_ptr<OSPEnv> osp_env = nullptr;
  std::unordered_map<int, ObstacleInfo> obs_info;
  ILQR::IDMParam idm_param;
  std::vector<std::vector<SamplePoint>> nn_trajs;

  void reset() {
    osp_env.reset();
    obs_info.clear();
    nn_trajs.clear();
  }
};

struct InteractionSearchOutput {
  bool search_traj_success = false;
  bool use_nn_traj = false;
  std::vector<SamplePoint> best_traj;
  std::unordered_map<int, std::vector<InteractionDecision>> obs_tag;

  void reset() {
    search_traj_success = false;
    use_nn_traj = false;
    best_traj.clear();
    obs_tag.clear();
  }
};

struct TrajectoryPoint {
  MathUtils::Point2D position = MathUtils::Point2D(0.0, 0.0);
  double theta = 0.0;
  double velocity = 0.0;
  double curvature = 0.0;
  double yaw_rate = 0.0;
  double acceleration = 0.0;
  double relative_time = 0.0;  // unit: s
};

struct TrajMergePointInfo {
  double merge_s = 0.0;
  double ego_time_to_merge = 0.0;
  double obs_time_to_merge = 0.0;
  double ttm_diff = 0.0;
};

enum AgentSelectType {
  AgentSelectType_UNKNOWN = 0,
  AgentSelectType_BY_LAT_DIS = 1,
  AgentSelectType_BY_MERGE
};

struct AgentSelectInfo {
  int id;
  AgentSelectType select_type;

  // select by lat dis
  double select_s = 0.0;

  // close left and open right
  int involved_start_frame = 0;
  int involved_end_frame = 0;

  // select by merge
  TrajMergePointInfo merge_point;
};

enum ForwardSimuPursuitPointType {
  ForwardSimuPursuitPointType_BASE_LINE = 0,
  ForwardSimuPursuitPointType_LEFT_NUDGE,
  ForwardSimuPursuitPointType_RIGHT_NUDGE,
};

struct ForwardSimuPursuitPoint {
  MathUtils::FrenetPoint2D pos;
  ForwardSimuPursuitPointType type;
};

struct VehicleParam {
  double length;
  double width;
  double half_width;
  double wheel_base;
  double r_4;
  std::vector<double> r_4_ratio = {3, 1, -1, -3};
  double body_center_to_rear_wheel_center;
  double rear_wheel_center_to_front;
  double rear_wheel_center_to_rear;
  double rear_wheel_center_to_front_right_angle;
  double rear_wheel_center_to_front_right_dis;
  double rear_wheel_center_to_front_left_angle;
  double rear_wheel_center_to_front_left_dis;
  double rear_wheel_center_to_rear_left_angle;
  double rear_wheel_center_to_rear_left_dis;
  double rear_wheel_center_to_rear_right_angle;
  double rear_wheel_center_to_rear_right_dis;
};

class InteractionTrajNode;

struct InteractionSearchCommonData {
  // param
  ILQR::IDMParam idm_param;
  VehicleParam vehicle_param;
  int horizon = 30;
  double delta_t = 0.2;  // for check
  double acc_longi_max = 2.0;
  double acc_longi_min = -5.0;
  double jerk_max = 5.0;
  double jerk_min = -10.0;
  double acc_lat_max = 3.5;
  std::vector<double> omega_max_v_list = {0.0, 2.78, 5.56, 11.11};
  std::vector<double> omega_max_omega_list = {1.57, 1.047, 0.785, 0.5};
  double omega_dot_max = 0.16;
  double cost_obs_collision_base = 1e4;
  double max_cost = 1e10;
  double max_cost_threshold = 1e9;

  // input
  std::shared_ptr<OSPEnv> osp_env = nullptr;
  std::unordered_map<int, ObstacleInfoInternal> obs_info;
  SamplePoint planning_origin;
  std::unordered_map<int, InteractionDecision> resoveld_agent_set;
  std::vector<int> interact_target;
  std::unordered_map<int, AgentSelectInfo> interact_target_info;

  // output
  std::vector<InteractionTrajNode *> success_end_node_list;
  std::vector<InteractionTrajNode *> failed_end_node_list;

  void reset() {
    osp_env = nullptr;
    obs_info.clear();
    resoveld_agent_set.clear();
    interact_target.clear();
    interact_target_info.clear();
    success_end_node_list.clear();
    failed_end_node_list.clear();
  }
};

class SolverInfoLog {
 public:
  static SolverInfoLog *Instance() {
    static SolverInfoLog instance;
    return &instance;
  }

 private:
  SolverInfoLog() = default;

 public:
  SolverInfoLog(SolverInfoLog const &) = delete;
  void operator=(SolverInfoLog const &) = delete;

 public:
  void clear() {
    source_.clear();
    err_.clear();
  }

  friend SolverInfoLog &operator<<(SolverInfoLog &log,
                                   const std::string &value) {
#ifdef ENGINEER_OPTION
    AD_LINFO(InteractionSearch) << value;
#else
    std::cout << value << " " << std::endl;
#endif
    return log;
  }

  friend SolverInfoLog &operator<<(SolverInfoLog &log, const int value) {
#ifdef ENGINEER_OPTION
    AD_LINFO(InteractionSearch) << value;
#else
    std::cout << value << " " << std::endl;
#endif
    return log;
  }

  friend SolverInfoLog &operator<<(SolverInfoLog &log, const double value) {
#ifdef ENGINEER_OPTION
    AD_LINFO(InteractionSearch) << value;
#else
    std::cout << value << " " << std::endl;
#endif

    return log;
  }

  std::string dump_log() {
    std::string res = source_;
    source_.clear();
    return res;
  }

  std::string dump_error() {
    std::string res = err_;
    err_.clear();
    return res;
  }

  void log(const std::string &tmp) {
#ifdef ENGINEER_OPTION
    AD_LINFO(InteractionSearch) << tmp;
#endif
    // source_ += tmp;
  }

  void error(const std::string &tmp) {
#ifdef ENGINEER_OPTION
    AD_LERROR(InteractionSearch) << tmp;
#endif
    // err_ += tmp;
  }

  void log(double tmp) {
#ifdef ENGINEER_OPTION
    AD_LINFO(InteractionSearch) << tmp;
#endif
    // std::ostringstream stream;
    // stream << std::fixed << std::setprecision(2) << tmp;
    // source_ += stream.str();
  }

 private:
  std::string source_;
  std::string err_;
};
}
