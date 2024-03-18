//
// Created by SENSETIME\fengxiaotong on 24-1-30.
//

#ifndef ALGORITHM_ALGORITHM_CILQR_OLD_VERSION_SPACE_BICYCLE_VERSION_H_
#define ALGORITHM_ALGORITHM_CILQR_OLD_VERSION_SPACE_BICYCLE_VERSION_H_

class VehicleModelBicycle {
 public:
  void update_parameter(const ILQRParam& _param) {
    param_ = _param;
    state_size_ = _param.state_size;
    action_size_ = _param.action_size;
    if (state_size_ != 4 || action_size_ != 2) {
      std::cout << " --------init error! parameter not match !---------"
                << std::endl;
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
  void step_kappa_fuzzy(const Eigen::VectorXd& state,
                        const Eigen::VectorXd& action,
                        Eigen::VectorXd& next_state);
  void get_all_element(const int& frame_count,
                       const std::shared_ptr<ILQREnvInterface>& env_ptr,
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
  void get_l_condition(const int& frame_count,
                       const Eigen::VectorXd& state,
                       const std::shared_ptr<ILQREnvInterface>& env_ptr,
                       StepCondition& l_condition) const;
  void get_l_lon_condition(const int& frame_count,
                           const Eigen::VectorXd& pre_state,
                           const StepCondition& pre_l_condition,
                           const std::shared_ptr<ILQREnvInterface>& env_ptr,
                           StepCondition& l_condition) const;
  void get_l(const Eigen::VectorXd& state,
             const Eigen::VectorXd& action,
             const StepCondition& condition,
             double& l) const;

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
               double& l_f) const;
  void get_l_f_x(const Eigen::VectorXd& state,
                 const StepCondition& condition,
                 Eigen::VectorXd& l_f_x);
  void get_l_f_xx(const Eigen::VectorXd& state,
                  const StepCondition& condition,
                  Eigen::MatrixXd& l_f_xx);
  void get_f_x_kappa(const Eigen::VectorXd& state,
                     const Eigen::VectorXd& action,
                     Eigen::MatrixXd& f_x);
  std::vector<double> step_kappa_std(const std::vector<double>& state_std,
                                     const std::vector<double>& action_std);
  std::vector<double> step_kappa_fuzzy_std(
      const std::vector<double>& state_std,
      const std::vector<double>& action_std);

  void set_v_refs(std::vector<double> _v_refs) {
    v_ref_.assign(_v_refs.begin(), _v_refs.end());
  }

  void set_a_refs(std::vector<double> _a_refs) {
    a_ref_.assign(_a_refs.begin(), _a_refs.end());
  }

 private:
  ILQRParam param_;
  int state_size_ = 0;
  int action_size_ = 0;
  std::vector<double> v_ref_;
  std::vector<double> a_ref_;
};

class ILQRSpace {
 public:
  ILQRSpace() = default;
  void init();

  void set_interface(std::shared_ptr<ILQREnvInterface> env_interface) {
    interface_ptr_ = env_interface;
  }

  void condition_init();

  void set_obstacle_cost();

  const std::vector<Eigen::VectorXd>& get_x_space() const { return x_space_; };

  const std::vector<Eigen::VectorXd>& get_x_init_space() const {
    return x_space_init_;
  };

  const std::vector<Eigen::VectorXd>& get_u_space() const { return u_space_; };

  const std::vector<Eigen::VectorXd>& get_u_init_space() const {
    return u_space_init_;
  };

  const std::vector<StepCondition>& get_l_condition_data() const {
    return l_condition_;
  };

  FuncStatus forward_rollout(std::vector<Eigen::VectorXd>& default_actions);
  FuncStatus backward_pass();
  FuncStatus forward_pass();
  FuncStatus control(const double& alpha);
  double get_l_sum(const std::vector<Eigen::VectorXd>& x_space,
                   const std::vector<Eigen::VectorXd>& u_space,
                   const std::vector<StepCondition>& l_condition);
  FuncStatus solve(const Eigen::VectorXd& init_state,
                   std::vector<Eigen::VectorXd>& default_actions);

  std::vector<MathUtils::Point2D> get_l_condition_match_points() const {
    std::vector<MathUtils::Point2D> res;
    for (auto& condition : l_condition_) {
      res.emplace_back(condition.ref_point);
    }
    return res;
  }

  std::vector<double> get_l_condition_ref_curva() const {
    std::vector<double> res;
    for (auto& condition : l_condition_) {
      res.emplace_back(condition.ref_curva);
    }
    return res;
  }
  std::vector<double> get_l_condition_ref_omega() const {
    std::vector<double> res;
    for (auto& condition : l_condition_) {
      res.emplace_back(condition.ref_omega);
    }
    return res;
  }

  void set_va_refs(std::vector<double> v_refs, std::vector<double> a_refs) {
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
  // input var : input
  std::shared_ptr<ILQREnvInterface> interface_ptr_;
  std::vector<ILQRObstacleConstrain> obstacle_cost_;

  const std::vector<double> alpha_ = {
      1.00000000e+00, 9.09090909e-01, 6.83013455e-01, 4.24097618e-01,
      2.17629136e-01, 9.22959982e-02, 3.23491843e-02, 9.37040641e-03,
      2.24320079e-03, 4.43805318e-04};
  std::vector<IterationStatistic> iter_stat_list_;
};

class ILQRObstacleConstrain : public ILQRCost {
 public:
  void get_l_condition(int frame_cnt,
                       const Eigen::VectorXd& state,
                       const std::shared_ptr<ILQREnvInterface>& env_ptr,
                       StepCondition& l_condition) const override;
  virtual void get_l(const StepCondition& l_condition,
                     double& l) const override;
  virtual void get_l_x(const StepCondition& l_condition,
                       Eigen::VectorXd& l_x) override;
  virtual void get_l_u(Eigen::VectorXd& l_u) override{};
  virtual void get_l_xx(Eigen::MatrixXd& l_xx) override{};
  virtual void get_l_ux(Eigen::MatrixXd& l_ux) override{};
  virtual void get_l_uu(Eigen::MatrixXd& l_uu) override{};

  void get_all_l_element(const Eigen::VectorXd& state,
                         const StepCondition& l_condition,
                         double& l,
                         Eigen::VectorXd& l_x,
                         Eigen::MatrixXd& l_xx);

  void init(const ILQRObstacleInterface& obstacle);

 private:
  std::vector<ObstaclePoint> obstacle_trajectory_;
  double length_;
  double width_;
  double ellipse_a_;
  double ellipse_b_;
  int id_;
};

class ILQRCost {
 public:
  virtual void get_l_condition(int frame_cnt,
                               const Eigen::VectorXd& state,
                               const std::shared_ptr<ILQREnvInterface>& env_ptr,
                               StepCondition& l_condition) const {};
  virtual void get_l(const StepCondition& l_condition, double& l) const {};
  virtual void get_l_x(const StepCondition& l_condition,
                       Eigen::VectorXd& l_x){};
  virtual void get_l_u(Eigen::VectorXd& l_u){};
  virtual void get_l_xx(Eigen::MatrixXd& l_xx){};
  virtual void get_l_ux(Eigen::MatrixXd& l_ux){};
  virtual void get_l_uu(Eigen::MatrixXd& l_uu){};
  void set_ilqr_param(const ILQRParam param) { param_ = param; };
  ILQRParam param_;
};

#endif  // ALGORITHM_ALGORITHM_CILQR_OLD_VERSION_SPACE_BICYCLE_VERSION_H_
