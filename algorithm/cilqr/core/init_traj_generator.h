#pragma once

#include "common_struct.h"
#include "env_simulator.h"
#include "obstacle.h"
#include "utils.h"
#include <iostream>
#include <sstream>

namespace ILQR {

struct InitTrajParam {
  VehicleParam vehicle_param;
  int horizon = 30;
  double delta_t = 0.2;         // for check
  double sample_delta_t = 0.8;  // for sample

  double acc_lat_max = 4.0;
  double acc_lat_min = -4.0;
  double omega_max = 0.5;
  double omega_min = -0.5;
  double omega_dot_max = 1.0;
  double omega_dot_min = -1.0;
  double acc_longi_max = 2.0;
  double acc_longi_min = -5.0;
  double jerk_max = 5.0;
  double jerk_min = -10.0;
  std::vector<double> delta_acc_list = {0.0, -3.0, 2.0};
  std::vector<double> delta_w_list = {0.0, -0.1, 0.1};

  double dist_2_obs_square_filter = 15.0 * 15.0;

  int max_check_count = 3000;
};

struct SampleControlPoint {
  double delta_acceleration = 0.0;
  double delta_omega = 0.0;
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
  double t = 0.0;
  StateVariable state;
  ControlVariable control;
};

class ObstacleConstrain {
 public:
  bool check_collision(
      int frame_cnt,
      const StateVariable &state,
      const std::vector<MathUtils::Point2D> &ego_cycle_centers);

  void init(const EnvSim::Obstacle &obstacle);

  void set_param(const InitTrajParam &param) { param_ = param; };

  int get_id() { return id_; }

 private:
  InitTrajParam param_;
  std::vector<ObstaclePoint> obstacle_trajectory_;
  double length_;
  double width_;
  double ellipse_a_;
  double ellipse_b_;
  int id_;
};

class InitialTrajectoryGenerator {
 public:
  InitialTrajectoryGenerator() = default;

  void init();

  void set_init_state(const StateVariable &state) { init_state_ = state; }

  void set_init_control(const ControlVariable &control) {
    init_control_ = control;
  }

  void set_env(std::shared_ptr<EnvSim::EnvSimulator> &env_ptr) {
    env_ptr_ = env_ptr;
  }

  void set_obstacle();

  FuncStatus gen_traj();

  std::vector<SamplePoint> get_traj() { return traj_; }

  const std::vector<std::vector<SamplePoint>> &get_failed_path_set() {
    return failed_path_set_;
  }

  bool get_search_success() { return search_success_; }

 private:
  StateVariable step(const StateVariable &state,
                     const ControlVariable &action,
                     const double &delta_t);

  void search_node(std::vector<SamplePoint> &cur_path,
                   std::vector<SamplePoint> &res_path,
                   std::vector<std::vector<SamplePoint>> &failed_path_set,
                   bool &find_res,
                   int &count,
                   int index,
                   StateVariable cur_state,
                   ControlVariable last_control);

  bool check_sample_point(const double &t,
                          const StateVariable &last_state,
                          const ControlVariable &control,
                          const StateVariable &new_state);

  bool check_freespace_collision(
      const StateVariable &last_state,
      const ControlVariable &control,
      const StateVariable &new_state,
      const std::vector<MathUtils::Point2D> &ego_circle_centers);

  bool check_freespace_ttc(const StateVariable &state,
                           const ControlVariable &control);

  bool check_freespace_bound(
      const StateVariable &state,
      const std::vector<MathUtils::Point2D> &ego_circle_centers);

  bool check_freespace_bound_points(
      const std::vector<MathUtils::Point2D> &ego_circle_centers,
      const std::vector<MathUtils::Point2D> &bound_points);

 private:
  InitTrajParam param_;
  double horizon_end_t_ = 6.0;
  std::vector<SampleControlPoint> sample_list_;
  double r_4_square_ = 0.0;
  double l_4_ = 0.0;

  StateVariable init_state_;
  ControlVariable init_control_;
  std::shared_ptr<EnvSim::EnvSimulator> env_ptr_;
  std::vector<ObstacleConstrain> obstacle_constrain_;

  bool search_success_ = false;
  std::vector<SamplePoint> traj_;
  std::vector<std::vector<SamplePoint>> failed_path_set_;
};
}