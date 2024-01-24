#include "init_traj_generator.h"
#include "ilqr_solver.h"
#include "chrono"
#include <iostream>

namespace ILQR {

constexpr double kEps = 1e-4;

void InitialTrajectoryGenerator::init() {
  param_.vehicle_param.length = 4.9;
  param_.vehicle_param.width = 1.9;
  param_.vehicle_param.r_4 = std::sqrt(
      (param_.vehicle_param.length * 0.5 * 0.25) *
          (param_.vehicle_param.length * 0.5 * 0.25) +
      (param_.vehicle_param.width * 0.5) * (param_.vehicle_param.width * 0.5));
  r_4_square_ = param_.vehicle_param.r_4 * param_.vehicle_param.r_4;
  l_4_ = param_.vehicle_param.length * 0.125;

  horizon_end_t_ = param_.horizon * param_.delta_t;

  sample_list_.clear();
  for (auto &delta_w : param_.delta_w_list) {
    for (auto &delta_acc : param_.delta_acc_list) {
      SampleControlPoint sample_point;
      sample_point.delta_acceleration = delta_acc;
      sample_point.delta_omega = delta_w;
      sample_list_.emplace_back(sample_point);
    }
  }
}

void InitialTrajectoryGenerator::set_obstacle() {
  if (env_ptr_ == nullptr) {
    std::cout << "env_ptr_ nullptr" << std::endl;
    return;
  }

  auto &obs_mgr = env_ptr_->get_obstacle_mgr();
  obstacle_constrain_.clear();
  for (auto &obj : obs_mgr.obstacle_map) {
    ObstacleConstrain new_obj_const;
    new_obj_const.set_param(param_);
    new_obj_const.init(obj.second);
    obstacle_constrain_.emplace_back(new_obj_const);
  }
}

FuncStatus InitialTrajectoryGenerator::gen_traj() {
  if (env_ptr_ == nullptr) {
    std::cout << "env_ptr_ nullptr" << std::endl;
    return FuncStatus::FuncFailed;
  }

  auto time_start = std::chrono::high_resolution_clock::now();

  traj_.clear();
  std::vector<SamplePoint> cur_path;
  std::vector<SamplePoint> res_path;
  std::vector<std::vector<SamplePoint>> failed_path_set;
  bool find_res{false};
  int count = 0;
  search_node(cur_path, res_path, failed_path_set, find_res, count, 0,
              init_state_, init_control_);

  if (count >= param_.max_check_count) {
    std::cout << "gen init_traj reach max_check_count" << std::endl;
  }
  // for (int i = 0; i < failed_path_set.size(); ++i) {
  //   const auto &path = failed_path_set[i];
  //   std::cout << "failed_path i " << i << std::endl;
  //   for (const auto &point : path) {
  //     std::cout << "point a " << point.control.acceleration << " w "
  //               << point.control.omega << " t " << point.t << std::endl;
  //   }
  // }
  if (find_res) {
    search_success_ = true;
    std::cout << "gen init_traj success" << std::endl;
    // for (const auto &point : res_path) {
    //   std::cout << "path_point a " << point.control.acceleration << " w "
    //             << point.control.omega << " t " << point.t << std::endl;
    // }
  } else {
    search_success_ = false;
    std::cout << "gen init_traj search_node failed" << std::endl;
  }

  auto time_end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> time_duration = time_end - time_start;
  std::cout << " **************** init traj gen time:："
            << time_duration.count() * 1000 << " ms "
            << " check_point_count " << count << std::endl;

  traj_ = std::move(res_path);
  failed_path_set_ = std::move(failed_path_set);
  if (!find_res) {
    return FuncStatus::FuncFailed;
  }
  return FuncStatus::FuncSucceed;
}

// 深度优先搜索, 递归实现
void InitialTrajectoryGenerator::search_node(
    std::vector<SamplePoint> &cur_path, std::vector<SamplePoint> &res_path,
    std::vector<std::vector<SamplePoint>> &failed_path_set, bool &find_res,
    int &count, int index, StateVariable cur_state,
    ControlVariable last_control) {
  // std::cout << "cur_index " << index << std::endl;

  double sample_start_t = index * param_.sample_delta_t;
  double sample_end_t = sample_start_t + param_.sample_delta_t;

  // std::cout << "sample_start_t " << sample_start_t << " sample_end_t "
  //           << sample_end_t << std::endl;

  if (find_res) {
    return;
  }
  if (sample_start_t >= horizon_end_t_ - kEps) {
    SamplePoint end_point;
    end_point.state = cur_state;
    end_point.control = last_control;
    end_point.t = horizon_end_t_;
    cur_path.emplace_back(end_point);
    std::copy(cur_path.begin(), cur_path.end(), std::back_inserter(res_path));
    find_res = true;
    cur_path.pop_back();
    return;
  }
  if (count >= param_.max_check_count) {
    return;
  }

  // sample next
  for (auto &sample_point : sample_list_) {
    double start_acc = last_control.acceleration;
    double end_acc = start_acc + sample_point.delta_acceleration;
    bool increase_acc = sample_point.delta_acceleration > 0 ? true : false;
    double start_omega = last_control.omega;
    double end_omega = start_omega + sample_point.delta_omega;
    bool increase_omega = sample_point.delta_omega > 0 ? true : false;

    // check acc_longi / omega
    end_acc =
        std::max(std::min(param_.acc_longi_max, end_acc), param_.acc_longi_min);
    end_omega =
        std::max(std::min(param_.omega_max, end_omega), param_.omega_min);

    // 递推每0.2s所有中间状态量, 并检查约束是否满足, 最终得到下一采样点状态量
    bool all_check_success = true;
    StateVariable temp_last_state = cur_state;
    ControlVariable temp_last_control = last_control;
    ControlVariable new_control;
    StateVariable new_state;
    std::vector<SamplePoint> temp_sample_point;
    bool first_point = index == 0 ? true : false;
    for (double check_t = sample_start_t; check_t < sample_end_t - kEps;
         check_t += param_.delta_t) {
      // std::cout << "check_t " << check_t << std::endl;

      if (check_t >= horizon_end_t_ - kEps) {
        SamplePoint end_point;
        end_point.state = temp_last_state;
        end_point.control = temp_last_control;
        end_point.t = horizon_end_t_;
        std::vector<SamplePoint> temp_path = cur_path;
        std::copy(temp_sample_point.begin(), temp_sample_point.end(),
                  std::back_inserter(temp_path));
        temp_path.emplace_back(end_point);
        std::copy(temp_path.begin(), temp_path.end(),
                  std::back_inserter(res_path));
        find_res = true;
        return;
      }

      if (first_point) {
        new_control = temp_last_control;
        first_point = false;
      } else {
        if (increase_acc) {
          if (temp_last_control.acceleration + param_.jerk_max * param_.delta_t <
              end_acc) {
            new_control.acceleration =
                temp_last_control.acceleration + param_.jerk_max * param_.delta_t;
          } else {
            new_control.acceleration = end_acc;
          }
        } else {
          if (temp_last_control.acceleration + param_.jerk_min * param_.delta_t <
              end_acc) {
            new_control.acceleration = end_acc;
          } else {
            new_control.acceleration =
                temp_last_control.acceleration + param_.jerk_min * param_.delta_t;
          }
        }
        if (increase_omega) {
          if (temp_last_control.omega + param_.omega_dot_max * param_.delta_t <
              end_omega) {
            new_control.omega =
                temp_last_control.omega + param_.omega_dot_max * param_.delta_t;
          } else {
            new_control.omega = end_omega;
          }
        } else {
          if (temp_last_control.omega + param_.omega_dot_min * param_.delta_t <
              end_omega) {
            new_control.omega = end_omega;
          } else {
            new_control.omega =
                temp_last_control.omega + param_.omega_dot_min * param_.delta_t;
          }
        }
      }

      // std::cout << "count " << count << " check_t " << check_t << " a "
      //           << new_control.acceleration << " w " << new_control.omega
      //           << std::endl;
      count++;
      if (count >= param_.max_check_count) {
        return;
      }

      new_state = step(temp_last_state, new_control, param_.delta_t);

      // std::cout << "cur_t " << check_t << " x " << temp_last_state.position.x
      //           << " y " << temp_last_state.position.y << " a "
      //           << new_control.acceleration << " w " << new_control.omega
      //           << " new_x " << new_state.position.x << " new_y "
      //           << new_state.position.y << std::endl;

      SamplePoint check_point;
      check_point.state = temp_last_state;
      check_point.control = new_control;
      check_point.t = check_t;
      if (!check_sample_point(check_t + param_.delta_t, temp_last_state,
                              new_control, new_state)) {
        all_check_success = false;
        std::vector<SamplePoint> temp_path = cur_path;
        std::copy(temp_sample_point.begin(), temp_sample_point.end(),
                  std::back_inserter(temp_path));
        temp_path.emplace_back(check_point);
        // for (const auto &point : temp_path) {
        //   std::cout << "temp point a " << point.control.acceleration << " w "
        //             << point.control.omega << " t " << point.t << std::endl;
        // }
        failed_path_set.emplace_back(temp_path);
        break;
      }

      temp_sample_point.emplace_back(check_point);
      temp_last_state = new_state;
      temp_last_control = new_control;
    }

    if (!all_check_success) {
      continue;
    }
    std::copy(temp_sample_point.begin(), temp_sample_point.end(),
              std::back_inserter(cur_path));

    // sample success
    search_node(cur_path, res_path, failed_path_set, find_res, count, index + 1,
                new_state, new_control);

    for (double check_t = sample_start_t; check_t < sample_end_t - kEps;
         check_t += param_.delta_t) {
      cur_path.pop_back();
    }

    if (find_res) {
      return;
    }
  }
}

StateVariable InitialTrajectoryGenerator::step(const StateVariable &state,
                                               const ControlVariable &action,
                                               const double &delta_t) {
  double end_vel = state.velocity + action.acceleration * delta_t;
  if (action.acceleration < 0.0) {
    if (state.velocity < 1e-2) {
      StateVariable next_state = state;
      next_state.velocity = 0.0;
      return next_state;
    }
    if (end_vel < 0.0) {
      double t = state.velocity / std::abs(action.acceleration);
      StateVariable next_state;
      next_state.position.x =
          state.position.x +
          0.5 * (std::cos(state.theta) +
                 std::cos(state.theta + action.omega * t)) *
              (state.velocity * t + 0.5 * action.acceleration * t * t);
      next_state.position.y =
          state.position.y +
          0.5 * (std::sin(state.theta) +
                 std::sin(state.theta + action.omega * t)) *
              (state.velocity * t + 0.5 * action.acceleration * t * t);
      next_state.velocity = 0.0;
      next_state.theta = state.theta + action.omega * t;
      return next_state;
    }
  }

  StateVariable next_state;
  next_state.position.x = state.position.x +
                          0.5 *
                              (std::cos(state.theta) +
                               std::cos(state.theta + action.omega * delta_t)) *
                              (state.velocity * delta_t +
                               0.5 * action.acceleration * delta_t * delta_t);
  next_state.position.y = state.position.y +
                          0.5 *
                              (std::sin(state.theta) +
                               std::sin(state.theta + action.omega * delta_t)) *
                              (state.velocity * delta_t +
                               0.5 * action.acceleration * delta_t * delta_t);
  next_state.velocity = state.velocity + action.acceleration * delta_t;
  next_state.theta = state.theta + action.omega * delta_t;

  return next_state;
}

bool InitialTrajectoryGenerator::check_sample_point(
    const double &t, const StateVariable &last_state,
    const ControlVariable &control, const StateVariable &new_state) {
  // std::cout << "check_t " << t << std::endl;

  // check acc lat
  double acc_lat = last_state.velocity * control.omega;
  if (acc_lat > param_.acc_lat_max || acc_lat < param_.acc_lat_min) {
    return false;
  }

  if (new_state.velocity < 0.0) {
    return false;
  }

  std::vector<MathUtils::Point2D> ego_circle_centers;
  auto &ratio = param_.vehicle_param.r_4_ratio;
  for (auto &r : ratio) {
    MathUtils::Point2D center_point;
    center_point.x =
        new_state.position.x + r * l_4_ * std::cos(new_state.theta);
    center_point.y =
        new_state.position.y + r * l_4_ * std::sin(new_state.theta);
    ego_circle_centers.emplace_back(center_point);
  }

  if (!check_freespace_collision(last_state, control, new_state,
                                 ego_circle_centers)) {
    return false;
  }

  // check obs collision
  int frame_cnt = static_cast<int>(std::floor((t + kEps) / param_.delta_t));
  // std::cout << "frame_cnt: " << frame_cnt << ", t: " << point.t
  // << ", delta_t:" << param_.delta_t << std::endl;
  for (auto &obs : obstacle_constrain_) {
    // std::cout << "frame_cnt: " << frame_cnt << ", t: " << point.t
    //           << ", obs_id:" << obs.get_id() << std::endl;
    if (!obs.check_collision(frame_cnt, new_state, ego_circle_centers)) {
      return false;
    }
  }

  // if (frame_cnt == param_.horizon) {
  //   return false;
  // }
  return true;
}

bool InitialTrajectoryGenerator::check_freespace_collision(
    const StateVariable &last_state, const ControlVariable &control,
    const StateVariable &new_state,
    const std::vector<MathUtils::Point2D> &ego_circle_centers) {
  if (!check_freespace_ttc(last_state, control)) {
    return false;
  }
  if (!check_freespace_bound(new_state, ego_circle_centers)) {
    return false;
  }
  return true;
}

bool InitialTrajectoryGenerator::check_freespace_ttc(
    const StateVariable &state, const ControlVariable &control) {
  double check_time = param_.delta_t + 0.05;
  EnvSim::PlanningPoint traj_point;
  traj_point.position = state.position;
  traj_point.theta = state.theta;
  traj_point.velocity = state.velocity;
  traj_point.acceleration = control.acceleration;
  if (state.velocity > 1e-4) {
    traj_point.curva = control.omega / state.velocity;
  } else {
    traj_point.curva = 0.0;
  }

  auto &freespace_mgr = env_ptr_->get_freespace_manager();
  auto left_ttc_pts =
      freespace_mgr.get_freespace_left_points_by_ttc(check_time, traj_point);
  if (!left_ttc_pts.empty()) {
    return false;
  }
  auto right_ttc_pts =
      freespace_mgr.get_freespace_right_points_by_ttc(check_time, traj_point);
  if (!right_ttc_pts.empty()) {
    return false;
  }
  return true;
}

bool InitialTrajectoryGenerator::check_freespace_bound(
    const StateVariable &state,
    const std::vector<MathUtils::Point2D> &ego_circle_centers) {
  // // 1. fixed bound_points for test
  // std::vector<MathUtils::Point2D> bound_points =
  // {
  //     {50, -1.75},  {52, -1.75},  {55, -1.75},  {60, -1.75},  {65, -1.75},
  //     {70, -1.75},  {72, -1.75},  {75, -1.75},  {80, -1.75},  {85, -1.75},
  //     {90, -1.75},  {95, -1.75},  {100, -1.75}, {105, -1.75}, {110, -1.75},
  //     {115, -1.75}, {120, -1.75}, {50, 9.25},   {52, 9.25},   {55, 9.25},
  //     {60, 9.25},   {65, 9.25},   {70, 9.25},   {72, 9.25},   {75, 9.25},
  //     {80, 9.25},   {85, 9.25},   {90, 9.25},   {95, 9.25},   {100, 9.25},
  //     {105, 9.25},  {110, 9.25},  {115, 9.25},  {120, 9.25}};

  // 2. get bound point by closest point
  std::vector<MathUtils::Point2D> bound_points;
  auto &freespace_mgr = env_ptr_->get_freespace_manager();
  auto bound_map =
      freespace_mgr.find_closest_point_at_both_side(state.position);
  for (const auto &bound_pair : bound_map) {
    auto &point_vec = bound_pair.second;
    std::copy(point_vec.begin(), point_vec.end(),
              std::back_inserter(bound_points));
  }
  if (!check_freespace_bound_points(ego_circle_centers, bound_points)) {
    return false;
  }

  // // 3. get bound point by ellipse model
  // auto &freespace_mgr = env_ptr_->get_freespace_manager();
  // PlanningPoint traj_point;
  // traj_point.position = state.position;
  // traj_point.theta = state.theta;
  // traj_point.velocity = state.velocity;
  // auto left_bound_pts =
  //     freespace_mgr.get_left_border_use_ellipse_model(traj_point);
  // if (!check_freespace_bound_points(ego_circle_centers, left_bound_pts)) {
  //   return false;
  // }
  // auto right_bound_pts =
  //     freespace_mgr.get_right_border_use_ellipse_model(traj_point);
  // if (!check_freespace_bound_points(ego_circle_centers, right_bound_pts)) {
  //   return false;
  // }

  return true;
}

bool InitialTrajectoryGenerator::check_freespace_bound_points(
    const std::vector<MathUtils::Point2D> &ego_circle_centers,
    const std::vector<MathUtils::Point2D> &bound_points) {
  for (const auto &bound_pt : bound_points) {
    auto &x0 = bound_pt.x;
    auto &y0 = bound_pt.y;
    for (auto &pt : ego_circle_centers) {
      double x_c = pt.x;
      double y_c = pt.y;
      double g = std::pow(x_c - x0, 2) + std::pow(y_c - y0, 2);

      if (g < r_4_square_) {
        return false;
      }
    }
  }
  return true;
}

void ObstacleConstrain::init(const EnvSim::Obstacle &obstacle) {
  length_ = obstacle.get_length();
  width_ = obstacle.get_width();
  id_ = obstacle.get_id();
  double a_safe = 0.2 + 0.7 * length_;
  double r_4 = param_.vehicle_param.r_4;
  ellipse_a_ = 0.5 * (length_ + r_4 * 2 + a_safe);
  double b_safe = 0.2 + 0.15 * width_;
  ellipse_b_ = 0.5 * (width_ + r_4 * 2 + b_safe);
  double max_time = obstacle.trajectory_points.back().relative_time;
  if (param_.delta_t * param_.horizon > max_time) {
    std::cout << " obstacle predict too short! " << std::endl;
  }
  obstacle_trajectory_.clear();
  for (int i = 0; i < param_.horizon + 1; i++) {
    ObstaclePoint tmp_pt;
    tmp_pt.relative_time = obstacle.trajectory_points[i].relative_time;
    tmp_pt.position = obstacle.trajectory_points[i].position;
    tmp_pt.velocity = obstacle.trajectory_points[i].v;
    tmp_pt.theta = obstacle.trajectory_points[i].theta;
    obstacle_trajectory_.emplace_back(tmp_pt);
  }
  for (int i = 0; i < param_.horizon; i++) {
    obstacle_trajectory_[i].acceleration =
        (obstacle_trajectory_[i + 1].velocity -
         obstacle_trajectory_[i].velocity) /
        param_.delta_t;
  }
  obstacle_trajectory_[param_.horizon].acceleration =
      obstacle_trajectory_[param_.horizon - 1].acceleration;
}

bool ObstacleConstrain::check_collision(
    int frame_cnt, const StateVariable &state,
    const std::vector<MathUtils::Point2D> &ego_circle_centers) {
  double x0 = obstacle_trajectory_[frame_cnt].position.x;
  double y0 = obstacle_trajectory_[frame_cnt].position.y;

  // 通过距离预先过滤
  const double &x_e = state.position.x;
  const double &y_e = state.position.y;
  double dist_2_obs_square = std::pow(x_e - x0, 2) + std::pow(y_e - y0, 2);

  // std::cout << "frame_cnt " << frame_cnt << " x0 " << x0 << " y0 " << y0
  //           << " x_e " << x_e << " y_e " << y_e << " dist_2_obs_square "
  //           << dist_2_obs_square << " filter "
  //           << param_.dist_2_obs_square_filter << std::endl;

  if (dist_2_obs_square > param_.dist_2_obs_square_filter) {
    return true;
  }

  for (auto &pt : ego_circle_centers) {
    double x_c = pt.x;
    double y_c = pt.y;
    double g = 1.0;

    double alpha = obstacle_trajectory_[frame_cnt].theta;
    g -= std::pow((x_c - x0) * std::cos(alpha) + (y_c - y0) * std::sin(alpha),
                  2.0) /
         (ellipse_a_ * ellipse_a_);
    g -= std::pow((x_c - x0) * std::sin(alpha) - (y_c - y0) * std::cos(alpha),
                  2.0) /
         (ellipse_b_ * ellipse_b_);
    if (g > 0.0) {
      return false;
    }
  }

  return true;
}
}