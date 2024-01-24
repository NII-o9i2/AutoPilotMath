//
// Created by SENSETIME\fengxiaotong on 24-1-5.
//
#include "vector"
#include "chrono"
#include "ilqr_solver.h"
#pragma once
namespace TreeILQR{
struct LonDebugInfo{
  double v_ref = 0.0;
  double a_ref = 0.0;
  double jerk = 0.0;
  double v = 0.0;
  double a = 0.0;
};
struct LatDebugInfo{
  MathUtils::Point2D match_point;
  double ref_omega = 0.0;
  double lat_dis_to_ref_line = 0.0;
  double ref_lat_acc = 0.0;
};
class VehicleModelBicyclePlus{
 public:
  void update_parameter( const ILQR::ILQRParam& _param){
    param_ = _param;
    if (state_size_ != param_.state_size || action_size_!= param_.action_size){
      std::cout<< " --------init error! parameter not match !---------" << std::endl;
    }
  };

  void step(const Eigen::VectorXd& state,
            Eigen::VectorXd& action,
            Eigen::VectorXd& next_state) const;
  std::vector<double> step_std(const std::vector<double>& state_std,
                               const std::vector<double>& action_std);
  void get_all_element(const int & frame_count,
                       const std::shared_ptr<EnvSim::EnvSimulator> &env_ptr,
                       const std::vector<ILQR::ILQRObstacleConstrain>& cost_list,
                       const std::shared_ptr<ILQR::SolverSpace>& pre_space_ptr,
                       std::shared_ptr<ILQR::SolverSpace>& space_ptr) const;
  void get_l_condition(const int & frame_count,
                       const Eigen::VectorXd& state,
                       const std::shared_ptr<EnvSim::EnvSimulator> &env_ptr,
                       ILQR::StepCondition & l_condition) const;

  void condition_init(const int & frame_count,
                      const std::shared_ptr<EnvSim::EnvSimulator> &env_ptr,
                       ILQR::StepCondition & l_condition) const;
  void get_l_lon_condition(const int & frame_count,
                           const Eigen::VectorXd& pre_state,
                           const Eigen::VectorXd& state,
                           ILQR::StepCondition& pre_l_condition,
                           const std::shared_ptr<EnvSim::EnvSimulator> &env_ptr,
                           ILQR::StepCondition & l_condition) const;
  void get_l(const Eigen::VectorXd& state,
             const Eigen::VectorXd& action,
             const ILQR::StepCondition& condition,
             const int frame_count,
             double &l,
             double &l_lat,
             double &l_lon) const;
  void get_l_f(const Eigen::VectorXd &state,
               const ILQR::StepCondition &condition, double &l_f) const;
 private:
  ILQR::ILQRParam param_;
  int state_size_ = 6;
  int action_size_ = 2;
};

class SolverInput{
 public:
  void init(){
//    model_.update_parameter(param_);
    model_plus_.update_parameter(param_);
  }
  const std::vector<double>& get_alpha() const {
    return alpha_;
  }
  const std::shared_ptr<EnvSim::EnvSimulator>& get_env_ptr() const{
    return env_ptr_;
  }
  const std::vector<ILQR::ILQRObstacleConstrain>& get_obstacle_cost() const {
    return obstacle_cost_;
  }
  const Eigen::VectorXd& get_init_state() const {
    return init_state_;
  }
  const std::vector<Eigen::VectorXd>& get_init_action() const {
    return init_actions_;
  }
  const ILQR::ILQRParam& get_param() const{
    return param_;
  }

  ILQR::ILQRParam& mutable_param(){
    return param_;
  }

  const ILQR::VehicleModelBicycle& get_model() const {
    return model_;
  }
  const VehicleModelBicyclePlus& get_model_plus() const {
    return model_plus_;
  }
  VehicleModelBicyclePlus& mutable_model(){
    return model_plus_;
  }
  void set_env_ptr(const std::shared_ptr<EnvSim::EnvSimulator>& env_ptr){
    env_ptr_ = env_ptr;
  }
  void set_init_action(const std::vector<Eigen::VectorXd>& init_action){
    init_actions_ = init_action;
  }
  void set_init_state(const Eigen::VectorXd& init_state){
    init_state_ = init_state;
  }
  void set_param(const ILQR::ILQRParam& param){
    param_ = param;
  }
  void set_obstacle_cost(const std::vector<ILQR::ILQRObstacleConstrain>& cost){
    obstacle_cost_ = cost;
  }

 private:
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
  std::shared_ptr<EnvSim::EnvSimulator> env_ptr_;
  std::vector<ILQR::ILQRObstacleConstrain> obstacle_cost_;
  std::vector<Eigen::VectorXd> init_actions_;
  ILQR::VehicleModelBicycle model_;
  VehicleModelBicyclePlus model_plus_;
  Eigen::VectorXd init_state_;
  ILQR::ILQRParam param_;
};


class TrajectoryTreeNode{
 public:
  FuncStatus forward_rollout(int index,
                             const std::shared_ptr<SolverInput>& input,
                             const std::shared_ptr<ILQR::SolverSpace>& pre_space_ptr);
  FuncStatus forward_pass();
  FuncStatus backward_pass();

  FuncStatus pre_process();
//  FuncStatus process(int index){};

  double get_l_sum();
  double control(const double& alpha);

  const std::shared_ptr<ILQR::SolverSpace>& get_space() const {
    return space_ptr_;
  }

  void get_space_list(std::vector<std::shared_ptr<ILQR::SolverSpace>>& s_list,
                      std::vector<std::vector<std::shared_ptr<ILQR::SolverSpace>>>& s_list_list);

  void search_max_k_radio(std::vector<double>& k_radio);
  void reset_k(std::vector<double>& k_radio);
  void clear();
 private:
  int index_;

  std::vector<TrajectoryTreeNode> next_node_;

  std::shared_ptr<ILQR::SolverSpace> space_ptr_;
  std::shared_ptr<ILQR::SolverSpace> pre_space_ptr_;
  std::shared_ptr<SolverInput> input_;
};

class TrajectoryTreeManager{
 public:
  void init(std::shared_ptr<EnvSim::EnvSimulator>& env_ptr,
            std::vector<Eigen::VectorXd>& init_actions,
            Eigen::VectorXd& init_state,
            ILQR::ILQRParam& param);
  FuncStatus solve();

  std::vector<std::vector<Eigen::VectorXd>> get_tree_x_space(){
    std::vector<std::vector<Eigen::VectorXd>> res;
    std::vector<Eigen::VectorXd> list;
    for(auto& s_list :space_list_){
      list.clear();
      for(auto& space: s_list){
        list.emplace_back(space->x_space);
      }
      res.emplace_back(list);
    }
    return res;
  }

  std::vector<std::vector<Eigen::VectorXd>> get_tree_u_space(){
    std::vector<std::vector<Eigen::VectorXd>> res;
    std::vector<Eigen::VectorXd> list;
    for(auto& s_list :space_list_){
      list.clear();
      for(auto& space: s_list){
        list.emplace_back(space->u_space);
      }
      res.emplace_back(list);
    }
    return res;
  }
  std::vector<std::vector<LonDebugInfo>> get_tree_lon_debug_info(){
    std::vector<std::vector<LonDebugInfo>> res;
    std::vector<LonDebugInfo> list;
    for(auto& s_list :space_list_){
      list.clear();
      for(auto& space: s_list){
        LonDebugInfo tmp;
        tmp.a = space->x_space[4];
        tmp.v = space->x_space[2];
        tmp.v_ref = space->l_condition.v_ref;
        tmp.a_ref = space->l_condition.a_ref;
        list.emplace_back(tmp);
      }
      res.emplace_back(list);
    }
    return res;
  }

  std::vector<std::vector<LatDebugInfo>> get_tree_lat_debug_info(){
    std::vector<std::vector<LatDebugInfo>> res;
    std::vector<LatDebugInfo> list;
    for(auto& s_list :space_list_){
      list.clear();
      for(auto& space: s_list){
        LatDebugInfo tmp;
        tmp.match_point = space->l_condition.ref_point;
        tmp.ref_omega = space->l_condition.ref_omega;
        tmp.lat_dis_to_ref_line = space->l_condition.ref_lat_dis;
        tmp.ref_lat_acc = space->l_condition.acc_lat_ref;
        list.emplace_back(tmp);
      }
      res.emplace_back(list);
    }
    return res;
  }

 private:
  int not_converged_counter_ = 0;
  TrajectoryTreeNode tree_node_list_;
  SolverInput input_;
  std::vector<std::vector<std::shared_ptr<ILQR::SolverSpace>>> space_list_;
};

}