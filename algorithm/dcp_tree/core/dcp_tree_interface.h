#pragma once
#include <vector>
#include <unordered_map>
#include "core/ilqr_lon_condition.h"
#include "utils.h"
#ifdef ENGINEER_OPTION
#include "ad_log/ad_log.hpp"
#endif
namespace DCP_TREE {

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
    AD_LINFO(SolverInfoLog) << value;
#else
    std::cout << value << " " << std::endl;
#endif
    return log;
  }

  friend SolverInfoLog &operator<<(SolverInfoLog &log, const int value) {
#ifdef ENGINEER_OPTION
    AD_LINFO(SolverInfoLog) << value;
#else
    std::cout << value << " " << std::endl;
#endif
    return log;
  }

  friend SolverInfoLog &operator<<(SolverInfoLog &log, const double value) {
#ifdef ENGINEER_OPTION
    AD_LINFO(SolverInfoLog) << value;
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
    AD_LINFO(SolverInfoLog) << tmp;
#endif
    // source_ += tmp;
  }

  void error(const std::string &tmp) {
#ifdef ENGINEER_OPTION
    AD_LERROR(SolverInfoLog) << tmp;
#endif
    // err_ += tmp;
  }

  void log(double tmp) {
#ifdef ENGINEER_OPTION
    AD_LINFO(SolverInfoLog) << tmp;
#endif
    // std::ostringstream stream;
    // stream << std::fixed << std::setprecision(2) << tmp;
    // source_ += stream.str();
  }

 private:
  std::string source_;
  std::string err_;
};

enum DCPActionDir {
  LC_LEFT = -1,
  LK = 0,
  LC_RIGHT = 1,
};

struct DCPTreeNodeDebugInfo {
  double cost_lc = 0.0;
  double cost_s = 0.0;
  double cost_v = 0.0;
  double s = 0.0;
  double v = 0.0;
  double a = 0.0;
};

using Path =
    std::vector<std::tuple<double, double, DCPActionDir, DCPTreeNodeDebugInfo>>;

enum DCPLaneDir {
  ABSOLUTE_LEFT = -1,
  ABSOLUTE_MIDDLE = 0,
  ABSOLUTE_RIGHT = 1,
};

struct DCPLaneDirHash {
  std::size_t operator()(const DCPLaneDir &dir) const {
    return std::hash<int>()(static_cast<int>(dir));
  }
};

struct Params {
  double delta_t = 1.0;  // longi profile time
  int max_time = 12;
  int max_debug_tree_print = 5;
  double confidence_th = 0.8;
  ILQR::IDMParam idm_params;
  int consider_lanes_size = 0;
  double init_ego_vel = 0.0;

  Params(const Params &other) {
    delta_t = other.delta_t;
    idm_params = other.idm_params;
    consider_lanes_size = other.consider_lanes_size;
  }

  explicit Params(const double &delta_t,
                  const ILQR::IDMParam &idm_params,
                  const int &consider_lanes_size)
      : delta_t(delta_t),
        idm_params(idm_params),
        consider_lanes_size(consider_lanes_size){};
  Params() = default;
};

struct DCPTreeInput {
  std::unordered_map<DCPLaneDir, std::vector<ILQR::IDMCarInfo>, DCPLaneDirHash>
      obstacle_list;
  ILQR::IDMCarInfo ego_info;
  DCPActionDir init_action_dir;
  Params dcp_params;

  void reset() {
    obstacle_list.clear();
    ego_info = ILQR::IDMCarInfo();
    init_action_dir = DCPActionDir::LK;
    dcp_params = Params();
  }
};

struct DCPTreeOutput {
  DCPActionDir suggest_action;
  double suggest_action_relative_time;
  double suggest_action_confidence;

  void reset() {
    suggest_action = DCPActionDir::LK;
    suggest_action_relative_time = 0.0;
    suggest_action_confidence = 0.0;
  }
};

struct DebugNode {
  double time;
  double cost;
  DCPActionDir action;
  int id;

  void reset() {
    time = 0.0;
    cost = 0.0;
    action = DCPActionDir::LK;
    id = 0;
  }
};

struct DCPTreeDebug {
  std::unordered_map<int, DebugNode> node_debug_list;
  std::vector<std::vector<int>> node_path_debug_list;

  void reset() {
    node_debug_list.clear();
    node_path_debug_list.clear();
  }
};

class DCPTreeRunner {
 public:
  DCPTreeRunner() {
    input_.reset();
    output_.reset();
    debug_.reset();
  };
  ~DCPTreeRunner() = default;
  MathUtils::FuncStatus init(const DCPTreeInput &input);
  MathUtils::FuncStatus run();
  const DCPTreeOutput &get_output() const { return output_; };
  const DCPTreeDebug &get_debug() const { return debug_; };
  const std::vector<Path> get_all_path() const { return all_paths_; };

 private:
  DCPTreeInput input_;
  DCPTreeOutput output_;
  DCPTreeDebug debug_;
  std::vector<DCP_TREE::Path> all_paths_;
};

}  // namespace DCP_TREE