//
// Created by SENSETIME\fengxiaotong on 23-12-13.
//

#ifndef ALGORITHM_ALGORITHM_CILQR_ILQR_SOLVER_H_
#define ALGORITHM_ALGORITHM_CILQR_ILQR_SOLVER_H_

#include <utility>
#include <random>
#include "memory"
#include "Eigen/Eigen"
#include "utils.h"
#include "iostream"
#include "env_simulator.h"
#include "obstacle.h"
#include "common_struct.h"
#include "ilqr_lon_condition.h"

namespace ILQR{

struct PlanningPoint {
  MathUtils::Point2D position;
  double theta = 0.0;
  double velocity = 0.0;
  double acceleration = 0.0;
  double omega = 0.0; // d(theta) / dt
  double curva = 0.0; // omega / velocity = curva
  double jerk = 0.0;
  double omega_dot = 0.0;
};
struct ExpModelParam{
  double q1;
  double q2;
  double q3;
};

struct LogModelParam{
  double t;
};
struct SoftConstraintParam{
  double q1 = 1.0;
  double q2 = 1.0;
};
struct HardConstraintParam{
  double t = 0.0;
};

struct ILQRParam{
  IDMParam idm_param;
  VehicleParam vehicle_param;
  ExpModelParam exp_model_param;
  LogModelParam log_model_param;
  // solver param
  double delta_t = 0.2;
  int horizon =  30;
  int max_iter_num = 50;
  int max_iter_lon_num = 2;
  double tol = 1e-3;
  double tol_abs = 1.0;
  double kappa_thr = 1e-5;
  bool use_hessians = false;
  double mu = 1e-6;
  double mu_min = 1e-9;
  double mu_max = 1e9;
  double delta_0 = 2.0;
  double delta = 2.0;
  std::vector<double> k_radio = {1000.0,500.0};
  int max_not_converged_time = 2;
  // model param
  bool enable_log_model = true;
  bool enable_lat = true;
  bool enable_hard_idm_model = true;
  int state_size = 4;
  int action_size = 2;
  // lon param
  double w_ref_v = 25.0;
  double w_ref_a = 1.0;
  SoftConstraintParam jerk_param{0.0,0.0};
  double jerk_thr = 10.0;
  double w_ref_jerk = 1.0;
  double w_lat_acc_v = 15.0;
  double lat_acc_lon = 1.5;
  HardConstraintParam s_constraint_param{5.0};
  SoftConstraintParam s_constraint_exp_param{1000.0,100.0};
  double s_constraint_min = 0.5;
  // lat param
  double consider_time = 1.0;
  double w_ref_line = 10.0;
  double w_ref_point = 8.0;
  double w_ref_omega = 0.5;
  double w_ref_omega_dot = 5.0;
  double curvature_max = 0.5;
  double acc_lat_max = 2.5;
  HardConstraintParam lat_acc_constraint_param{0.08};
  SoftConstraintParam lat_acc_constraint_exp_param{1,100.0};
  // comfort param
  double w_ref_acc_lat = 0.0;
;
};

// Statistics for each iteration
struct IterationStatistic{
  std::vector<Eigen::VectorXd> x_space;
  std::vector<Eigen::VectorXd> u_space;
  double total_cost;

  IterationStatistic() = default;
  IterationStatistic(const std::vector<Eigen::VectorXd>& x_space_, 
                     const std::vector<Eigen::VectorXd>& u_space_,
                     const double& total_cost_) : x_space(x_space_),
                                                  u_space(u_space_),
                                                  total_cost(total_cost_) {};
};

struct LonObstacleInfo{
  double belief = 0.0;
  double s = 0.0;
  double length = 0.0;
  MathUtils::Point2D position;
  MathUtils::Point2D v;
  MathUtils::Point2D a;
};

struct StepCondition{
  bool has_ref_point = false;
  MathUtils::Point2D ref_point;
  double ref_point_theta;
  double ref_lat_dis = 0.0;
  std::vector<MathUtils::Point2D> ego_cycle_centers;
  int frame_cnt;
  std::unordered_map<int,bool> use_exp_model_map;
  std::unordered_map<int,LonObstacleInfo> obstacle_belief_state_map;

  // omega is d(theta)/d(t)
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
};


class SolverSpace{
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

class ILQRCost{
 public:
  virtual void get_l_condition(int frame_cnt, const Eigen::VectorXd& state,
                               const std::shared_ptr<EnvSim::EnvSimulator> &env_ptr,
                               StepCondition & l_condition) const {
  };
  virtual void get_l(const StepCondition & l_condition, double & l) const {
  };
  virtual void get_l_x(const StepCondition & l_condition, Eigen::VectorXd& l_x){
  };
  virtual void get_l_u(Eigen::VectorXd& l_u){
  };
  virtual void get_l_xx(Eigen::MatrixXd& l_xx){
  };
  virtual void get_l_ux(Eigen::MatrixXd& l_ux){
  };
  virtual void get_l_uu(Eigen::MatrixXd& l_uu){
  };
  void set_ilqr_param(const ILQRParam param){
    param_ = param;
  };
  ILQRParam param_;
};

class ILQRObstacleConstrain: public ILQRCost{
 public:
  void get_l_condition(int frame_cnt, const Eigen::VectorXd& state,
                       const std::shared_ptr<EnvSim::EnvSimulator> &env_ptr,
                       StepCondition & l_condition) const override;
  virtual void get_l(const StepCondition & l_condition, double & l) const override;
  virtual void get_l_x(const StepCondition & l_condition,Eigen::VectorXd& l_x)override;
  virtual void get_l_u(Eigen::VectorXd& l_u)override{};
  virtual void get_l_xx(Eigen::MatrixXd& l_xx)override{};
  virtual void get_l_ux(Eigen::MatrixXd& l_ux)override{};
  virtual void get_l_uu(Eigen::MatrixXd& l_uu)override{};

  void get_all_l_element( const Eigen::VectorXd& state,const StepCondition & l_condition, double & l,Eigen::VectorXd& l_x,Eigen::MatrixXd& l_xx);

  void init(const EnvSim::Obstacle& obstacle);
 private:
  std::vector<ObstaclePoint> obstacle_trajectory_;
  double length_;
  double width_;
  double ellipse_a_;
  double ellipse_b_;
  int id_;
};

class VehicleModelBicycle{
 public:
  void update_parameter( const ILQRParam& _param){
    param_ = _param;
    state_size_ = _param.state_size;
    action_size_ = _param.action_size;
    if (state_size_ != 4 || action_size_!= 2){
      std::cout<< " --------init error! parameter not match !---------" << std::endl;
    }
  };

  void step(const Eigen::VectorXd& state,
            const Eigen::VectorXd& action,
            Eigen::VectorXd& next_state) const;

  std::vector<double> step_std(const std::vector<double>& state_std,
            const std::vector<double>& action_std);

  void step_kappa(const Eigen::VectorXd& state,
            const Eigen::VectorXd& action,
            Eigen::VectorXd& next_state);
  void  step_kappa_fuzzy(const Eigen::VectorXd& state,
                         const Eigen::VectorXd& action,
                         Eigen::VectorXd& next_state);
  void get_all_element(const int & frame_count,
                       const std::shared_ptr<EnvSim::EnvSimulator> &env_ptr,
                       const std::vector<ILQRObstacleConstrain>& cost_list,
                       const std::shared_ptr<SolverSpace>& pre_space_ptr,
                       std::shared_ptr<SolverSpace>& space_ptr) const;
  void get_f_x(const Eigen::VectorXd& state,
               const Eigen::VectorXd& action,
               Eigen::MatrixXd& f_x);
  void get_f_u(const Eigen::VectorXd& state,
               const Eigen::VectorXd& action,
               Eigen::MatrixXd& u_x);
  void get_f_xx(const Eigen::VectorXd& state,
               const Eigen::VectorXd& action,
               Eigen::MatrixXd& f_xx);
  void get_f_ux(const Eigen::VectorXd& state,
                const Eigen::VectorXd& action,
                Eigen::MatrixXd& f_ux);
  void get_f_uu(const Eigen::VectorXd& state,
                const Eigen::VectorXd& action,
                Eigen::MatrixXd& f_uu);
  void get_l_condition(const int & frame_count,
                       const Eigen::VectorXd& state,
                       const std::shared_ptr<EnvSim::EnvSimulator> &env_ptr,
                       StepCondition & l_condition) const;
  void get_l_lon_condition(const int & frame_count,
                       const Eigen::VectorXd& pre_state,
                       const StepCondition& pre_l_condition,
                       const std::shared_ptr<EnvSim::EnvSimulator> &env_ptr,
                       StepCondition & l_condition) const;
  void get_l(const Eigen::VectorXd& state,
             const Eigen::VectorXd& action,
             const StepCondition& condition,
             double &l) const;

  void get_l_x(const Eigen::VectorXd& state,
             const Eigen::VectorXd& action,
             const StepCondition& condition,
             Eigen::VectorXd& l_x);

  void get_l_u(const Eigen::VectorXd& state,
               const Eigen::VectorXd& action,
               const StepCondition& condition,
               Eigen::VectorXd& l_u);

  void get_l_xx(const Eigen::VectorXd& state,
           const Eigen::VectorXd& action,
           const StepCondition& condition,
           Eigen::MatrixXd& l_xx);
  void get_l_ux(const Eigen::VectorXd& state,
                const Eigen::VectorXd& action,
                const StepCondition& condition,
                Eigen::MatrixXd& l_ux);
  void get_l_uu(const Eigen::VectorXd& state,
                const Eigen::VectorXd& action,
                const StepCondition& condition,
                Eigen::MatrixXd& l_uu);
  void get_l_f(const Eigen::VectorXd& state,
             const StepCondition& condition,
             double &l_f) const;
  void get_l_f_x(const Eigen::VectorXd& state,
               const StepCondition& condition,
               Eigen::VectorXd &l_f_x);
  void get_l_f_xx(const Eigen::VectorXd& state,
                 const StepCondition& condition,
                  Eigen::MatrixXd&l_f_xx);
  void get_f_x_kappa(const Eigen::VectorXd& state,
                     const Eigen::VectorXd& action,
                     Eigen::MatrixXd& f_x);
  std::vector<double> step_kappa_std(const std::vector<double>& state_std,
                               const std::vector<double>& action_std);
  std::vector<double> step_kappa_fuzzy_std(const std::vector<double>& state_std,
                                     const std::vector<double>& action_std);
                                     
  void set_v_refs(std::vector<double>_v_refs) {v_ref_.assign(_v_refs.begin(), _v_refs.end());}

  void set_a_refs(std::vector<double>_a_refs) {a_ref_.assign(_a_refs.begin(), _a_refs.end());}
 private:
  ILQRParam param_;
  int state_size_ = 0;
  int action_size_ = 0;
  std::vector<double> v_ref_;
  std::vector<double> a_ref_;
};

class ILQRSpace{
 public:
  ILQRSpace() = default;
  void init();

  void set_env(std::shared_ptr<EnvSim::EnvSimulator>& env_ptr){
    env_ptr_ = env_ptr;
  }

  void condition_init();

  void set_obstacle_cost();

  const std::vector<Eigen::VectorXd>& get_x_space() const {
    return x_space_;
  };

  const std::vector<Eigen::VectorXd>& get_x_init_space() const {
    return x_space_init_;
  };

  const std::vector<Eigen::VectorXd>& get_u_space() const {
    return u_space_;
  };

  const std::vector<Eigen::VectorXd> &get_u_init_space() const {
    return u_space_init_;
  };

  const std::vector<StepCondition>& get_l_condition_data() const {
    return l_condition_;
  };

  FuncStatus forward_rollout(std::vector<Eigen::VectorXd> &default_actions);
  FuncStatus backward_pass();
  FuncStatus forward_pass();
  FuncStatus control(const double &alpha);
  double get_l_sum(const std::vector<Eigen::VectorXd> &x_space,
                   const std::vector<Eigen::VectorXd>& u_space,
                   const std::vector<StepCondition>& l_condition);
  FuncStatus solve(const Eigen::VectorXd & init_state, std::vector<Eigen::VectorXd> &default_actions);

  std::vector<MathUtils::Point2D> get_l_condition_match_points() const {
    std::vector<MathUtils::Point2D> res;
    for (auto &condition : l_condition_){
      res.emplace_back(condition.ref_point);
    }
    return res;
  }

  std::vector<double> get_l_condition_ref_curva() const {
    std::vector<double> res;
    for (auto &condition : l_condition_){
      res.emplace_back(condition.ref_curva);
    }
    return res;
  }
  std::vector<double> get_l_condition_ref_omega() const {
    std::vector<double> res;
    for (auto &condition : l_condition_){
      res.emplace_back(condition.ref_omega);
    }
    return res;
  }

  void set_va_refs(std::vector<double>v_refs,
                   std::vector<double>a_refs) {
    vehicle_model_.set_v_refs(v_refs); 
    vehicle_model_.set_a_refs(a_refs);              
  }

  std::vector<IterationStatistic> get_iter_stat_list() {
    return iter_stat_list_;
  }
  
 private:
  VehicleModelBicycle vehicle_model_;
  ILQRParam param_;
  Eigen::VectorXd init_state_;
  int not_converged_counter_ = 0;
  std::vector<Eigen::VectorXd> x_space_init_;
  std::vector<Eigen::VectorXd> u_space_init_;
  std::vector<Eigen::VectorXd> x_space_;
  std::vector<Eigen::VectorXd> u_space_;
  std::vector<Eigen::VectorXd> x_new_space_;
  std::vector<Eigen::VectorXd> u_new_space_;
  std::vector<Eigen::MatrixXd> f_x_space_;
  std::vector<Eigen::MatrixXd> f_u_space_;
  std::vector<Eigen::MatrixXd> f_xx_space_;
  std::vector<Eigen::MatrixXd> f_ux_space_;
  std::vector<Eigen::MatrixXd> f_uu_space_;
  std::vector<StepCondition> l_condition_;
  std::vector<StepCondition> l_new_condition_;
  std::vector<double> l_space_;
  std::vector<Eigen::VectorXd> l_x_space_;
  std::vector<Eigen::VectorXd> l_u_space_;
  std::vector<Eigen::MatrixXd> l_xx_space_;
  std::vector<Eigen::MatrixXd> l_ux_space_;
  std::vector<Eigen::MatrixXd> l_uu_space_;

  std::vector<Eigen::VectorXd> k_space_;
  std::vector<Eigen::MatrixXd> k_matrix_space_;
  // env var : input
  std::shared_ptr<EnvSim::EnvSimulator> env_ptr_;
  std::vector<ILQRObstacleConstrain> obstacle_cost_;

  const std::vector<double> alpha_ = {1.00000000e+00, 
                                      9.09090909e-01, 
                                      6.83013455e-01, 
                                      4.24097618e-01,
                                      2.17629136e-01, 
                                      9.22959982e-02, 
                                      3.23491843e-02, 
                                      9.37040641e-03,
                                      2.24320079e-03, 
                                      4.43805318e-04};
  std::vector<IterationStatistic> iter_stat_list_;
};

}
#endif //ALGORITHM_ALGORITHM_CILQR_ILQR_SOLVER_H_
