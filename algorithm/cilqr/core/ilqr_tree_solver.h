//
// Created by SENSETIME\fengxiaotong on 24-1-5.
//
#pragma once
#include <iostream>
#include <iomanip>
#include <sstream>
#include "vector"
#include "chrono"
#include "memory"
#include "utils.h"
#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/LU"
#include "common_struct.h"
#include "core/ilqr_tree_interface.h"
#include "core/ilqr_lon_condition.h"
#ifdef ENGINEER_OPTION
#include "ad_log/ad_log.hpp"
#endif
namespace ILQR {

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

struct LonObstacleInfo {
  double belief = 0.0;
  double s = 0.0;
  double length = 0.0;
  double theta = 0.0;
  bool is_behind_car = false;
  bool is_reverse = false;
  MathUtils::Point2D position;
  MathUtils::Point2D v;
  MathUtils::Point2D a;
  double ellipse_a = 0.0;
  double ellipse_b = 0.0;
  std::vector<double> dodge_g;
  bool overlap = false;
  ObstacleType type = Real;
};

struct StepCondition {
  bool has_ref_point = false;
  MathUtils::Point2D ref_point;
  double ref_point_theta;
  double ref_lat_dis = 0.0;
  std::vector<MathUtils::Point2D> ego_cycle_centers;
  int frame_cnt;
  std::unordered_map<int, bool> use_exp_model_map;
  std::unordered_map<int, LonObstacleInfo> obstacle_belief_state_map;

  bool has_ref_omega = true;
  double ref_curva;
  double ref_curva_v;
  double ref_omega;

  bool has_lead_one = false;
  double pre_s_ego = 0.0;
  double s_ego = 0.0;
  double min_s = 0.0;

  bool has_ref_v = false;
  double v_ref;

  bool has_ref_a = false;
  double a_ref;

  bool has_ref_acc_lat = false;
  double acc_lat_ref;

  bool has_constraint_acc_lat = false;
  bool has_constraint_omega = false;

  double speed_limit = 16.67;

  // for V2 model
  double pursuit_omega = 0.0;
  MathUtils::Point2D pursuit_point;
};

class SolverSpace {
 public:
  Eigen::VectorXd init_state;
  int not_converged_counter = 0;
  Eigen::VectorXd x_space_init;
  Eigen::VectorXd x_space;
  Eigen::VectorXd u_space;
  Eigen::VectorXd x_new_space;
  Eigen::VectorXd u_new_space;
  Eigen::MatrixXd f_x_space;
  Eigen::MatrixXd f_u_space;
  Eigen::MatrixXd f_xx_space;
  Eigen::MatrixXd f_ux_space;
  Eigen::MatrixXd f_uu_space;
  StepCondition l_condition;
  StepCondition l_new_condition;
  double l_space;
  double l_lat_space;
  double l_lon_space;
  double l_new_space;
  Eigen::VectorXd l_x_space;
  Eigen::VectorXd l_u_space;
  Eigen::MatrixXd l_xx_space;
  Eigen::MatrixXd l_ux_space;
  Eigen::MatrixXd l_uu_space;

  Eigen::VectorXd k_space;
  Eigen::MatrixXd k_matrix_space;
  Eigen::VectorXd v_x_space;
  Eigen::MatrixXd v_xx_space;

  double branch_belief = 1.0;
  double math_belief = 1.0;
};

struct ExpModelParam {
  double q1;
  double q2;
  double q3;
};

struct LogModelParam {
  double t;
};

struct SoftConstraintParam {
  double q1 = 1.0;
  double q2 = 1.0;

  SoftConstraintParam(const double &q1_, const double &q2_)
      : q1(q1_), q2(q2_) {}
};

struct HardConstraintParam {
  double t = 0.0;

  HardConstraintParam(const double &t_) : t(t_) {}
};

enum FuncTypeBit { Lon = 1, Lat = 2, Dodge = 4 };

enum SolverFuncType {
  OnlyLon = 1,
  LonLatWithoutDodge = 3,
  LonLatWithDodge = 7
};

struct ILQRParam {
  IDMParam idm_param;
  VehicleParam vehicle_param;
  // POILQR param
  bool enable_multi_mode = false;
  int multi_horizon = 15;
  // solver param
  SolverFuncType solver_func_type = SolverFuncType::LonLatWithoutDodge;
  double delta_t = 0.2;
  int horizon = 30;
  int max_iter_num = 50;
  int max_iter_lon_num = 2;
  int max_iter_lat_lon_num = 6;
  double tol = 1e-3;
  double tol_abs = 1.0;
  bool use_hessians = false;
  double mu = 1e-6;
  double mu_min = 1e-9;
  double mu_max = 1e9;
  double delta_0 = 2.0;
  double delta = 2.0;
  std::vector<double> k_radio = {1000.0, 30.0};
  int max_not_converged_time = 2;
  // model param
  bool enable_lat = true;
  bool enable_hard_idm_model = true;
  int state_size = 4;
  int action_size = 2;
  // lon param
  bool use_extern_ref = false;
  double w_ref_v = 25.0;
  double w_ref_a = 1.0;
  SoftConstraintParam jerk_param{0.0, 0.0};
  double jerk_thr = 10.0;
  double w_ref_jerk = 1.0;
  double w_lat_acc_v = 15.0;
  double lat_acc_lon = 1.5;
  HardConstraintParam s_constraint_param{5.0};
  SoftConstraintParam s_constraint_exp_param{1000.0, 100.0};
  double s_constraint_min = 1.5;
  double cut_in_belief_thr = 1e-2;
  // lat param
  double consider_time = 1.0;
  double consider_min_dis = 10.0;
  std::vector<double> w_v_std_list = {5.0, 20.0};
  std::vector<double> w_ref_line_list = {0.0, 0.0};
  std::vector<double> w_ref_point_list = {6.0, 4.0};
  std::vector<double> w_ref_dis_std_list = {10.0, 8.0};
  std::vector<double> w_ref_dis_max_list = {6.0, 6.0};
  std::vector<double> w_ref_point_dis_max_list = {10.0, 10.0};
  double w_ref_omega = 2.0;
  double w_ref_omega_dot = 10.0;
  double curvature_max = 0.5;
  double acc_lat_max = 2.5;
  double omega_dot_thr = 0.030461742;       // 0.17 * 0.17 (10 deg)
  double consider_omega_dot_thr = 0.00761;  // 0.087 * 0.087 (5 deg)
  HardConstraintParam lat_acc_constraint_param{0.08};
  SoftConstraintParam lat_acc_constraint_exp_param{1, 100.0};
  HardConstraintParam omega_dot_constraint_param{1.0};
  SoftConstraintParam omega_dot_constraint_exp_param{10.0, 100.0};
  // comfort param
  double w_ref_acc_lat = 0.0;
  // dodge param
  bool enable_dodge = false;
  double consider_g_thr = -0.5;
  HardConstraintParam dodge_pose_constraint_param{0.5};
  SoftConstraintParam dodge_pose_constraint_exp_param{20.0, 10.0};
  double dodge_lat_dis = 0.5;
};

struct LonDebugInfo {
  double min_s = 0.0;
  double v_ref = 0.0;
  double a_ref = 0.0;
  double jerk = 0.0;
  double v = 0.0;
  double a = 0.0;
};

struct LatDebugInfo {
  MathUtils::Point2D match_point;
  double ref_omega = 0.0;
  double lat_dis_to_ref_line = 0.0;
  double ref_lat_acc = 0.0;
  double ref_point_theta = 0.0;
};

struct DodgeDebugInfo {
  std::unordered_map<int, bool> overlap_map;
};

class VehicleModelBicyclePlus {
 public:
  void update_parameter(const ILQR::ILQRParam &_param) {
    param_ = _param;
    if (state_size_ != param_.state_size ||
        action_size_ != param_.action_size) {
      SolverInfoLog::Instance()->error(
          " --------init error! parameter not match !--------- \n");
      // std::cout<< " --------init error! parameter not match !---------" <<
      // std::endl;
    }
  };

  void step(Eigen::VectorXd &state,
            Eigen::VectorXd &action,
            Eigen::VectorXd &next_state) const;

  std::vector<double> step_std(const std::vector<double> &state_std,
                               const std::vector<double> &action_std);

  virtual void get_all_element(const int &frame_count,
                       const std::shared_ptr<ILQREnvInterface> &env_ptr,
                       const std::shared_ptr<ILQR::SolverSpace> &pre_space_ptr,
                       std::shared_ptr<ILQR::SolverSpace> &space_ptr) const;

  virtual void condition_init(const int &frame_count,
                      const std::shared_ptr<ILQREnvInterface> &env_ptr,
                      ILQR::StepCondition &l_condition,
                      MathUtils::Point2D ego_init_position) const;

  virtual void get_l_condition(const int &frame_count,
                       const Eigen::VectorXd &pre_state,
                       const Eigen::VectorXd &state,
                       ILQR::StepCondition &pre_l_condition,
                       const std::shared_ptr<ILQREnvInterface> &env_ptr,
                       ILQR::StepCondition &l_condition) const;

  virtual void get_l(const Eigen::VectorXd &state,
             const Eigen::VectorXd &action,
             const ILQR::StepCondition &condition,
             int frame_count,
             double &l,
             double &l_lat,
             double &l_lon) const;

  virtual void get_l_f(const Eigen::VectorXd &state,
               const ILQR::StepCondition &condition,
               double &l_f) const;

  const ILQR::ILQRParam &get_param() const { return param_; }

 protected:
  ILQR::ILQRParam param_;
  int state_size_ = 6;
  int action_size_ = 2;
};

class VehicleModelBicyclePlusV2 : public VehicleModelBicyclePlus {
 public:
  void get_all_element(const int &frame_count,
                       const std::shared_ptr<ILQREnvInterface> &env_ptr,
                       const std::shared_ptr<ILQR::SolverSpace> &pre_space_ptr,
                       std::shared_ptr<ILQR::SolverSpace> &space_ptr) const override;

  void condition_init(const int &frame_count,
                      const std::shared_ptr<ILQREnvInterface> &env_ptr,
                      ILQR::StepCondition &l_condition,
                      MathUtils::Point2D ego_init_position) const override;

  void get_l_condition(const int &frame_count,
                       const Eigen::VectorXd &pre_state,
                       const Eigen::VectorXd &state,
                       ILQR::StepCondition &pre_l_condition,
                       const std::shared_ptr<ILQREnvInterface> &env_ptr,
                       ILQR::StepCondition &l_condition) const override;

  void get_l(const Eigen::VectorXd &state,
             const Eigen::VectorXd &action,
             const ILQR::StepCondition &condition,
             int frame_count,
             double &l,
             double &l_lat,
             double &l_lon) const override;

//  void get_l_f(const Eigen::VectorXd &state,
//               const ILQR::StepCondition &condition,
//               double &l_f) const override;


};

class SolverInput {
 public:
  void init() {
    //    model_.update_parameter(param_);
    model_plus_.update_parameter(param_);
  }
  const std::vector<double> &get_alpha() const { return alpha_; }
  const std::shared_ptr<ILQREnvInterface> &get_env_ptr() const {
    return env_ptr_;
  }
  //  const std::vector<ILQR::ILQRObstacleConstrain>& get_obstacle_cost() const
  //  {
  //    return obstacle_cost_;
  //  }
  const Eigen::VectorXd &get_init_state() const { return init_state_; }
  const std::vector<Eigen::VectorXd> &get_init_action() const {
    return init_actions_;
  }
  const ILQR::ILQRParam &get_param() const { return param_; }

  ILQR::ILQRParam &mutable_param() { return param_; }

  const VehicleModelBicyclePlus &get_model_plus() const { return model_plus_; }
  VehicleModelBicyclePlus &mutable_model() { return model_plus_; }
  void set_env_ptr(const std::shared_ptr<ILQREnvInterface> &env_ptr) {
    env_ptr_ = env_ptr;
  }
  void set_init_action(const std::vector<Eigen::VectorXd> &init_action) {
    init_actions_ = init_action;
  }
  void set_init_state(const Eigen::VectorXd &init_state) {
    init_state_ = init_state;
  }
  void set_param(const ILQR::ILQRParam &param) { param_ = param; }
  //  void set_obstacle_cost(const std::vector<ILQR::ILQRObstacleConstrain>&
  //  cost){
  //    obstacle_cost_ = cost;
  //  }

 private:
  const std::vector<double> alpha_ = {
      1.00000000e+00, 9.09090909e-01, 6.83013455e-01, 4.24097618e-01,
      2.17629136e-01, 9.22959982e-02, 3.23491843e-02, 9.37040641e-03,
      2.24320079e-03, 4.43805318e-04};
  std::shared_ptr<ILQREnvInterface> env_ptr_;
  //  std::vector<ILQR::ILQRObstacleConstrain> obstacle_cost_;
  std::vector<Eigen::VectorXd> init_actions_;
  VehicleModelBicyclePlus model_plus_;
  Eigen::VectorXd init_state_;
  ILQR::ILQRParam param_;
};

class SolveStage {
 public:
  enum StageType { LonStage = 0, LatLonStage = 1, DodgeLatLonStage = 2 };
  explicit SolveStage(StageType _type, int _max_iter) {
    type = _type;
    max_iter = _max_iter;
  }
  StageType type;
  int max_iter;
  void update_mutable_parameter(ILQR::ILQRParam &param) const {
    switch (type) {
      case LonStage:
        param.enable_lat = false;
        param.enable_dodge = false;
        break;
      case LatLonStage:
        param.enable_lat = true;
        param.enable_dodge = false;
        break;
      case DodgeLatLonStage:
        param.enable_lat = true;
        param.enable_dodge = true;
    }
  }
};

class TrajectoryTreeNode {
 public:
  FuncStatus forward_rollout(
      int index,
      const std::shared_ptr<SolverInput> &input,
      const std::shared_ptr<ILQR::SolverSpace> &pre_space_ptr);
  FuncStatus forward_pass();
  FuncStatus backward_pass();

  FuncStatus pre_process();
  //  FuncStatus process(int index){};

  double get_l_sum();
  double control(const double &alpha);

  const std::shared_ptr<ILQR::SolverSpace> &get_space() const {
    return space_ptr_;
  }

  void get_space_list(
      std::vector<std::shared_ptr<ILQR::SolverSpace>> &s_list,
      std::vector<std::vector<std::shared_ptr<ILQR::SolverSpace>>>
          &s_list_list);

  void search_max_k_radio(std::vector<double> &k_radio);
  void reset_k(std::vector<double> &k_radio);
  void clear();

 private:
  int index_;

  std::vector<TrajectoryTreeNode> next_node_;

  std::shared_ptr<ILQR::SolverSpace> space_ptr_;
  std::shared_ptr<ILQR::SolverSpace> pre_space_ptr_;
  std::shared_ptr<SolverInput> input_;
};

class TrajectoryTreeManager {
 public:
  void init(std::shared_ptr<ILQR::ILQREnvInterface> env_ptr,
            std::vector<Eigen::VectorXd> &init_actions,
            Eigen::VectorXd &init_state,
            ILQR::ILQRParam &param);
  FuncStatus solve();

  std::vector<std::vector<Eigen::VectorXd>> get_tree_x_space() {
    std::vector<std::vector<Eigen::VectorXd>> res;
    std::vector<Eigen::VectorXd> list;
    for (auto &s_list : space_list_) {
      list.clear();
      for (auto &space : s_list) {
        list.emplace_back(space->x_space);
      }
      res.emplace_back(list);
    }
    return res;
  }

  std::vector<std::vector<Eigen::VectorXd>> get_tree_u_space() {
    std::vector<std::vector<Eigen::VectorXd>> res;
    std::vector<Eigen::VectorXd> list;
    for (auto &s_list : space_list_) {
      list.clear();
      for (auto &space : s_list) {
        list.emplace_back(space->u_space);
      }
      res.emplace_back(list);
    }
    return res;
  }
  std::vector<std::vector<LonDebugInfo>> get_tree_lon_debug_info() {
    std::vector<std::vector<LonDebugInfo>> res;
    std::vector<LonDebugInfo> list;
    for (auto &s_list : space_list_) {
      list.clear();
      for (auto &space : s_list) {
        LonDebugInfo tmp;
        tmp.a = space->x_space[4];
        tmp.v = space->x_space[2];
        tmp.v_ref = space->l_condition.v_ref;
        tmp.a_ref = space->l_condition.a_ref;
        tmp.min_s = space->l_condition.min_s;
        list.emplace_back(tmp);
      }
      res.emplace_back(list);
    }
    return res;
  }

  std::vector<std::vector<LatDebugInfo>> get_tree_lat_debug_info() {
    std::vector<std::vector<LatDebugInfo>> res;
    std::vector<LatDebugInfo> list;
    for (auto &s_list : space_list_) {
      list.clear();
      for (auto &space : s_list) {
        LatDebugInfo tmp;
        tmp.match_point = space->l_condition.ref_point;
        tmp.ref_omega = space->l_condition.ref_omega;
        tmp.lat_dis_to_ref_line = space->l_condition.ref_lat_dis;
        tmp.ref_lat_acc = space->l_condition.acc_lat_ref;
        tmp.ref_point_theta = space->l_condition.ref_point_theta;
        list.emplace_back(tmp);
      }
      res.emplace_back(list);
    }
    return res;
  }

  std::vector<std::vector<DodgeDebugInfo>> get_tree_dodge_debug_info() {
    std::vector<std::vector<DodgeDebugInfo>> res;
    std::vector<DodgeDebugInfo> list;
    for (auto &s_list : space_list_) {
      list.clear();
      for (auto &space : s_list) {
        DodgeDebugInfo tmp;
        tmp.overlap_map.clear();
        tmp.overlap_map = space->l_condition.use_exp_model_map;
        list.emplace_back(tmp);
      }
      res.emplace_back(list);
    }
    return res;
  }

  double get_tree_l_sum() { return tree_node_list_.get_l_sum(); }

 private:
  int not_converged_counter_ = 0;
  TrajectoryTreeNode tree_node_list_;
  SolverInput input_;
  std::vector<std::vector<std::shared_ptr<ILQR::SolverSpace>>> space_list_;
  std::vector<SolveStage> solve_stage_mgr_;
};
}