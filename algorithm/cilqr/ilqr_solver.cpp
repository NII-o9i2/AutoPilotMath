//
// Created by SENSETIME\fengxiaotong on 23-12-13.
//
#include "ilqr_solver.h"
#include "chrono"
#include "iostream"
#include <cmath>
#include <iomanip>

namespace ILQR {

void ILQRObstacleConstrain::init(const EnvSim::Obstacle &obstacle) {
  length_ = obstacle.get_length();
  width_ = obstacle.get_width();
  id_ = obstacle.get_id();
  double a_safe = 0.2 + 0.7 * length_;
  double r_4 = param_.vehicle_param.r_4;
  ellipse_a_ = 0.5 * (length_ + r_4 * 2 + a_safe);
  double b_safe = 0.2 + 0.15 * width_;
  ellipse_b_ = 0.5 * (width_ + r_4 * 2 + b_safe);
  double max_time = obstacle.trajectory_points.back().relative_time;
  if (param_.delta_t * param_.horizon > max_time){
    std::cout << " obstacle predict too short! "<< std::endl;
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
  for(int i= 0; i< param_.horizon; i++){
    obstacle_trajectory_[i].acceleration = (obstacle_trajectory_[i+1].velocity - obstacle_trajectory_[i].velocity) / param_.delta_t;
  }
  obstacle_trajectory_[param_.horizon].acceleration = obstacle_trajectory_[param_.horizon - 1].acceleration;

}

void ILQRObstacleConstrain::get_l_condition(int frame_cnt, const Eigen::VectorXd& state,
                                            const std::shared_ptr<EnvSim::EnvSimulator> &env_ptr,
                                            StepCondition & l_condition) const{
  // state : x, y, v, theta
  l_condition.frame_cnt = frame_cnt;
  // 1.0 calculate ego cycle centers
  l_condition.ego_cycle_centers.clear();
  std::vector<double> ratio = param_.vehicle_param.r_4_ratio;
  double l_4 = param_.vehicle_param.length * 0.125;
  for (auto& r : ratio){
    MathUtils::Point2D center_point;
    center_point.x = state[0] + r * l_4 * std::cos(state[3]);
    center_point.y = state[1] + r * l_4 * std::sin(state[3]);
    l_condition.ego_cycle_centers.emplace_back(center_point);
  }
  bool use_exp = false;
  double x0 = obstacle_trajectory_[frame_cnt].position.x;
  double y0 = obstacle_trajectory_[frame_cnt].position.y;
  for (auto& pt: l_condition.ego_cycle_centers){
    double x_c = pt.x;
    double y_c = pt.y;
    double g = 1.0;

    // todo: 椭圆公式符号问题 & 导数
    double alpha = obstacle_trajectory_[frame_cnt].theta;
    g -= std::pow((x_c - x0) * std::cos(alpha) + (y_c - y0) * std::sin(alpha),2.0) / (ellipse_a_ * ellipse_a_);
    g -= std::pow((x_c - x0) * std::sin(alpha) + (y_c - y0) * std::cos(alpha),2.0) / (ellipse_b_ * ellipse_b_);
    use_exp = g>0.0;
    if (use_exp){
      break;
    }
  }
  if (l_condition.use_exp_model_map.count(id_) < 1) {
    l_condition.use_exp_model_map.insert({id_, use_exp});
  } else {
    l_condition.use_exp_model_map[id_] = use_exp;
  }
}

void ILQRObstacleConstrain::get_l(const StepCondition &l_condition, double &l) const {
  double cos_alpha = std::cos(obstacle_trajectory_[l_condition.frame_cnt].theta);
  double sin_alpha = std::sin(obstacle_trajectory_[l_condition.frame_cnt].theta);
  double x0 = obstacle_trajectory_[l_condition.frame_cnt].position.x;
  double y0 = obstacle_trajectory_[l_condition.frame_cnt].position.y;

  bool use_exp_model = true;
  if (param_.enable_log_model && (l_condition.use_exp_model_map.count(id_) < 1 ||
      !l_condition.use_exp_model_map.at(id_))) {
    use_exp_model = false;
  }
  // std::cout << "use_exp_model " << use_exp_model << std::endl;

  if (use_exp_model){
    for (auto& pt: l_condition.ego_cycle_centers){
      double g = 1.0;
      double x_c = pt.x;
      double y_c = pt.y;
      g -= std::pow((x_c - x0) * cos_alpha + (y_c - y0) * sin_alpha,2.0) / (ellipse_a_ * ellipse_a_);
      g -= std::pow((x_c - x0) * sin_alpha - (y_c - y0) * cos_alpha,2.0) / (ellipse_b_ * ellipse_b_);
      l += param_.exp_model_param.q1 * std::exp(param_.exp_model_param.q2 * g + param_.exp_model_param.q3);
    }
  }else {
    // log
    for (auto& pt: l_condition.ego_cycle_centers){
      double g = 1.0;
      double x_c = pt.x;
      double y_c = pt.y;

      // get l
      g -= std::pow((x_c - x0) * cos_alpha + (y_c - y0) * sin_alpha, 2.0) /
           (ellipse_a_ * ellipse_a_);
      g -= std::pow((x_c - x0) * sin_alpha - (y_c - y0) * cos_alpha, 2.0) /
           (ellipse_b_ * ellipse_b_);
      if (g > 0.0) {
        std::cout << "ERROR: invalid g, can not happen";
        continue;
      }
      double log_g = std::max(0.0,-1.0 / param_.log_model_param.t * std::log(-g));
      l += log_g;
//       std::cout << "g: " << g << ", log_g: " << log_g << std::endl;
    }
  }

}

void ILQRObstacleConstrain::get_l_x(const StepCondition &l_condition,
                                    Eigen::VectorXd &l_x) {}

void ILQRObstacleConstrain::get_all_l_element(const Eigen::VectorXd &state,
                                              const StepCondition &l_condition,
                                              double &l, Eigen::VectorXd &l_x,
                                              Eigen::MatrixXd &l_xx) {
  double alpha = obstacle_trajectory_[l_condition.frame_cnt].theta;
  double cos_alpha =
      std::cos(obstacle_trajectory_[l_condition.frame_cnt].theta);
  double sin_alpha =
      std::sin(obstacle_trajectory_[l_condition.frame_cnt].theta);
  double x0 = obstacle_trajectory_[l_condition.frame_cnt].position.x;
  double y0 = obstacle_trajectory_[l_condition.frame_cnt].position.y;
  double q1 = param_.exp_model_param.q1;
  double q2 = param_.exp_model_param.q2;
  double q3 = param_.exp_model_param.q3;
  double t = param_.log_model_param.t;
  double l_4 = param_.vehicle_param.length * 0.125;
  double cos_theta = std::cos(state[3]);
  double sin_theta = std::sin(state[3]);
  double ellipse_a_2 = ellipse_a_ * ellipse_a_;
  double ellipse_b_2 = ellipse_b_ * ellipse_b_;
  double x = state[0];
  double y = state[1];
  double theta = state[3];

  bool use_exp_model = true;
  if (param_.enable_log_model && (l_condition.use_exp_model_map.count(id_) < 1 ||
      !l_condition.use_exp_model_map.at(id_))) {
    use_exp_model = false;
  }
  // std::cout << "use_exp_model: " << use_exp_model << std::endl;

  if (use_exp_model) {
    for (int i = 0; i < l_condition.ego_cycle_centers.size(); i++) {
      double g = 1.0;
      double x_c = l_condition.ego_cycle_centers[i].x;
      double y_c = l_condition.ego_cycle_centers[i].y;
      double r = param_.vehicle_param.r_4_ratio[i];

      // get l
      // todo: 椭圆公式符号问题 & 导数
      g -= std::pow((x_c - x0) * cos_alpha + (y_c - y0) * sin_alpha, 2.0) /
           ellipse_a_2;
      g -= std::pow((x_c - x0) * sin_alpha - (y_c - y0) * cos_alpha, 2.0) /
           ellipse_b_2;
      double exp_g =
          param_.exp_model_param.q1 *
          std::exp(param_.exp_model_param.q2 * g + param_.exp_model_param.q3);
      l += exp_g;

      // get l_x
      double exp_g_dot_x =
          q2 * (-2 * ((y_c - y0) * cos_alpha + (x_c - x0) * sin_alpha) *
                    sin_alpha / ellipse_b_2 -
                2 * ((y_c - y0) * sin_alpha + (x_c - x0) * cos_alpha) *
                    cos_alpha / ellipse_a_2) *
          exp_g;
      double exp_g_dot_y =
          q2 * (-2 * ((y_c - y0) * cos_alpha + (x_c - x0) * sin_alpha) *
                    cos_alpha / ellipse_b_2 -
                2 * ((y_c - y0) * sin_alpha + (x_c - x0) * cos_alpha) *
                    sin_alpha / ellipse_a_2) *
          exp_g;
      double exp_g_dot_theta =
          q2 * (-((y_c - y0) * cos_alpha + (x_c - x0) * sin_alpha) *
                    (-2 * l_4 * r * sin_alpha * sin_theta +
                     2 * l_4 * r * cos_alpha * cos_theta) /
                    ellipse_b_2 -
                ((y_c - y0) * sin_alpha + (x_c - x0) * cos_alpha) *
                    (2 * l_4 * r * sin_alpha * cos_theta -
                     2 * l_4 * r * sin_theta * cos_alpha) /
                    ellipse_a_2) *
          exp_g;

      l_x[0] += exp_g_dot_x;
      l_x[1] += exp_g_dot_y;
      l_x[2] += 0;
      l_x[3] += exp_g_dot_theta;

      // get l_u

      // get l_xx
      double exp_g_dot_x_x =
          q2 * q2 *
              std::pow((-2 * ((y_c - y0) * cos_alpha + (x_c - x0) * sin_alpha) *
                            sin_alpha / ellipse_b_2 -
                        2 * ((y_c - y0) * sin_alpha + (x_c - x0) * cos_alpha) *
                            cos_alpha / ellipse_a_2),
                       2) *
              exp_g +
          q2 * (-2 * sin_alpha * sin_alpha / ellipse_b_2 -
                2 * cos_alpha * cos_alpha / ellipse_a_2) *
              exp_g;
      double exp_g_dot_x_y =
          q2 * q2 * (-2 * ((y_c - y0) * cos_alpha + (x_c - x0) * sin_alpha) *
                         sin_alpha / ellipse_b_2 -
                     2 * ((y_c - y0) * sin_alpha + (x_c - x0) * cos_alpha) *
                         cos_alpha / ellipse_a_2) *
              (-2 * ((y_c - y0) * cos_alpha + (x_c - x0) * sin_alpha) *
                   cos_alpha / ellipse_b_2 -
               2 * ((y_c - y0) * sin_alpha + (x_c - x0) * cos_alpha) *
                   sin_alpha / ellipse_a_2) *
              exp_g +
          q2 * (-2 * sin_alpha * cos_alpha / ellipse_b_2 -
                2 * sin_alpha * cos_alpha / ellipse_a_2) *
              exp_g;
      double exp_g_dot_x_theta =
          q2 * q2 * (-((y_c - y0) * cos_alpha + (x_c - x0) * sin_alpha) *
                         (-2 * l_4 * r * sin_alpha * sin_theta +
                          2 * l_4 * r * cos_alpha * cos_theta) /
                         ellipse_b_2 -
                     ((y_c - y0) * sin_alpha + (x_c - x0) * cos_alpha) *
                         (2 * l_4 * r * sin_alpha * cos_theta -
                          2 * l_4 * r * sin_theta * cos_alpha) /
                         ellipse_a_2) *
              (-2 * ((y_c - y0) * cos_alpha + (x_c - x0) * sin_alpha) *
                   sin_alpha / ellipse_b_2 -
               2 * ((y_c - y0) * sin_alpha + (x_c - x0) * cos_alpha) *
                   cos_alpha / ellipse_a_2) *
              exp_g +
          q2 * (-2 * (-l_4 * r * sin_alpha * sin_theta +
                      l_4 * r * cos_alpha * cos_theta) *
                    sin_alpha / ellipse_b_2 -
                2 * (l_4 * r * sin_alpha * cos_theta -
                     l_4 * r * sin_theta * cos_alpha) *
                    cos_alpha / ellipse_a_2) *
              exp_g;

      double exp_g_dot_y_x =
          q2 * q2 * (-2 * ((y_c - y0) * cos_alpha + (x_c - x0) * sin_alpha) *
                         sin_alpha / ellipse_b_2 -
                     2 * ((y_c - y0) * sin_alpha + (x_c - x0) * cos_alpha) *
                         cos_alpha / ellipse_a_2) *
              (-2 * ((y_c - y0) * cos_alpha + (x_c - x0) * sin_alpha) *
                   cos_alpha / ellipse_b_2 -
               2 * ((y_c - y0) * sin_alpha + (x_c - x0) * cos_alpha) *
                   sin_alpha / ellipse_a_2) *
              exp_g +
          q2 * (-2 * sin_alpha * cos_alpha / ellipse_b_2 -
                2 * sin_alpha * cos_alpha / ellipse_a_2) *
              exp_g;
      double exp_g_dot_y_y =
          q2 * q2 *
              std::pow((-2 * ((y_c - y0) * cos_alpha + (x_c - x0) * sin_alpha) *
                            cos_alpha / ellipse_b_2 -
                        2 * ((y_c - y0) * sin_alpha + (x_c - x0) * cos_alpha) *
                            sin_alpha / ellipse_a_2),
                       2) *
              exp_g +
          q2 * (-2 * cos_alpha * cos_alpha / ellipse_b_2 -
                2 * sin_alpha * sin_alpha / ellipse_a_2) *
              exp_g;
      double exp_g_dot_y_theta =
          q2 * q2 * (-((y_c - y0) * cos_alpha + (x_c - x0) * sin_alpha) *
                         (-2 * l_4 * r * sin_alpha * sin_theta +
                          2 * l_4 * r * cos_alpha * cos_theta) /
                         ellipse_b_2 -
                     ((y_c - y0) * sin_alpha + (x_c - x0) * cos_alpha) *
                         (2 * l_4 * r * sin_alpha * cos_theta -
                          2 * l_4 * r * sin_theta * cos_alpha) /
                         ellipse_a_2) *
              (-2 * ((y_c - y0) * cos_alpha + (x_c - x0) * sin_alpha) *
                   cos_alpha / ellipse_b_2 -
               2 * ((y_c - y0) * sin_alpha + (x_c - x0) * cos_alpha) *
                   sin_alpha / ellipse_a_2) *
              exp_g +
          q2 * (-2 * (-l_4 * r * sin_alpha * sin_theta +
                      l_4 * r * cos_alpha * cos_theta) *
                    cos_alpha / ellipse_b_2 -
                2 * (l_4 * r * sin_alpha * cos_theta -
                     l_4 * r * sin_theta * cos_alpha) *
                    sin_alpha / ellipse_a_2) *
              exp_g;

      double exp_g_dot_theta_x =
          q2 * q2 * (-((y_c - y0) * cos_alpha + (x_c - x0) * sin_alpha) *
                         (-2 * l_4 * r * sin_alpha * sin_theta +
                          2 * l_4 * r * cos_alpha * cos_theta) /
                         ellipse_b_2 -
                     ((y_c - y0) * sin_alpha + (x_c - x0) * cos_alpha) *
                         (2 * l_4 * r * sin_alpha * cos_theta -
                          2 * l_4 * r * sin_theta * cos_alpha) /
                         ellipse_a_2) *
              (-2 * ((y_c - y0) * cos_alpha + (x_c - x0) * sin_alpha) *
                   sin_alpha / ellipse_b_2 -
               2 * ((y_c - y0) * sin_alpha + (x_c - x0) * cos_alpha) *
                   cos_alpha / ellipse_a_2) *
              exp_g +
          q2 * (-(-2 * l_4 * r * sin_alpha * sin_theta +
                  2 * l_4 * r * cos_alpha * cos_theta) *
                    sin_alpha / ellipse_b_2 -
                (2 * l_4 * r * sin_alpha * cos_theta -
                 2 * l_4 * r * sin_theta * cos_alpha) *
                    cos_alpha / ellipse_a_2) *
              exp_g;
      double exp_g_dot_theta_y =
          q2 * q2 * (-((y_c - y0) * cos_alpha + (x_c - x0) * sin_alpha) *
                         (-2 * l_4 * r * sin_alpha * sin_theta +
                          2 * l_4 * r * cos_alpha * cos_theta) /
                         ellipse_b_2 -
                     ((y_c - y0) * sin_alpha + (x_c - x0) * cos_alpha) *
                         (2 * l_4 * r * sin_alpha * cos_theta -
                          2 * l_4 * r * sin_theta * cos_alpha) /
                         ellipse_a_2) *
              (-2 * ((y_c - y0) * cos_alpha + (x_c - x0) * sin_alpha) *
                   cos_alpha / ellipse_b_2 -
               2 * ((y_c - y0) * sin_alpha + (x_c - x0) * cos_alpha) *
                   sin_alpha / ellipse_a_2) *
              exp_g +
          q2 * (-(-2 * l_4 * r * sin_alpha * sin_theta +
                  2 * l_4 * r * cos_alpha * cos_theta) *
                    cos_alpha / ellipse_b_2 -
                (2 * l_4 * r * sin_alpha * cos_theta -
                 2 * l_4 * r * sin_theta * cos_alpha) *
                    sin_alpha / ellipse_a_2) *
              exp_g;

      double exp_g_dot_theta_theta =
          q2 * q2 *
              std::pow((-((y_c - y0) * cos_alpha + (x_c - x0) * sin_alpha) *
                            (-2 * l_4 * r * sin_alpha * sin_theta +
                             2 * l_4 * r * cos_alpha * cos_theta) /
                            ellipse_b_2 -
                        ((y_c - y0) * sin_alpha + (x_c - x0) * cos_alpha) *
                            (2 * l_4 * r * sin_alpha * cos_theta -
                             2 * l_4 * r * sin_theta * cos_alpha) /
                            ellipse_a_2),
                       2) *
              exp_g +
          q2 * (-((y_c - y0) * cos_alpha + (x_c - x0) * sin_alpha) *
                    (-2 * l_4 * r * sin_alpha * cos_theta -
                     2 * l_4 * r * sin_theta * cos_alpha) /
                    ellipse_b_2 -
                (-2 * l_4 * r * sin_alpha * sin_theta +
                 2 * l_4 * r * cos_alpha * cos_theta) *
                    (-l_4 * r * sin_alpha * sin_theta +
                     l_4 * r * cos_alpha * cos_theta) /
                    ellipse_b_2 -
                ((y_c - y0) * sin_alpha + (x_c - x0) * cos_alpha) *
                    (-2 * l_4 * r * sin_alpha * sin_theta -
                     2 * l_4 * r * cos_alpha * cos_theta) /
                    ellipse_a_2 -
                (l_4 * r * sin_alpha * cos_theta -
                 l_4 * r * sin_theta * cos_alpha) *
                    (2 * l_4 * r * sin_alpha * cos_theta -
                     2 * l_4 * r * sin_theta * cos_alpha) /
                    ellipse_a_2) *
              exp_g;

      l_xx(0, 0) += exp_g_dot_x_x;
      l_xx(0, 1) += exp_g_dot_x_y;
      l_xx(0, 3) += exp_g_dot_x_theta;
      l_xx(1, 0) += exp_g_dot_y_x;
      l_xx(1, 1) += exp_g_dot_y_y;
      l_xx(1, 3) += exp_g_dot_y_theta;
      l_xx(3, 0) += exp_g_dot_theta_x;
      l_xx(3, 1) += exp_g_dot_theta_y;
      l_xx(3, 3) += exp_g_dot_theta_theta;
      //      std::cout << "get l all elements" << std::endl;
    }
  } else {
    // log
    for (int i = 0; i < l_condition.ego_cycle_centers.size(); i++) {
      double g = 1.0;
      double x_c = l_condition.ego_cycle_centers[i].x;
      double y_c = l_condition.ego_cycle_centers[i].y;
      double r = param_.vehicle_param.r_4_ratio[i];

      // get l
      g -= std::pow((x_c - x0) * cos_alpha + (y_c - y0) * sin_alpha, 2.0) /
           ellipse_a_2;
      g -= std::pow((x_c - x0) * sin_alpha - (y_c - y0) * cos_alpha, 2.0) /
           ellipse_b_2;
      if (g > 0.0) {
        std::cout << "ERROR: invalid g, can not happen";
        continue;
      }
      double log_g = std::max(0.0,-1.0 / param_.log_model_param.t * std::log(-g));
      l += log_g;
      // get l_x
      double log_g_dot_x =
          -(2 * ((y_c - y0) * cos_alpha + (x_c - x0) * sin_alpha) * sin_alpha /
                ellipse_b_2 +
            2 * ((y_c - y0) * sin_alpha + (x_c - x0) * cos_alpha) * cos_alpha /
                ellipse_a_2) /
          (t * (-1 +
                std::pow((y_c - y0) * cos_alpha + (x_c - x0) * sin_alpha, 2) /
                    ellipse_b_2 +
                std::pow((y_c - y0) * sin_alpha + (x_c - x0) * cos_alpha, 2) /
                    ellipse_a_2));
      double log_g_dot_y =
          -(2 * ((y_c - y0) * cos_alpha + (x_c - x0) * sin_alpha) * cos_alpha /
                ellipse_b_2 +
            2 * ((y_c - y0) * sin_alpha + (x_c - x0) * cos_alpha) * sin_alpha /
                ellipse_a_2) /
          (t * (-1 +
                std::pow((y_c - y0) * cos_alpha + (x_c - x0) * sin_alpha, 2) /
                    ellipse_b_2 +
                std::pow((y_c - y0) * sin_alpha + (x_c - x0) * cos_alpha, 2) /
                    ellipse_a_2));
      double log_g_dot_theta =
          -(((y_c - y0) * cos_alpha + (x_c - x0) * sin_alpha) *
                (-2 * l_4 * r * sin_alpha * sin_theta +
                 2 * l_4 * r * cos_alpha * cos_theta) /
                ellipse_b_2 +
            ((y_c - y0) * sin_alpha + (x_c - x0) * cos_alpha) *
                (2 * l_4 * r * sin_alpha * cos_theta -
                 2 * l_4 * r * sin_theta * cos_alpha) /
                ellipse_a_2) /
          (t * (-1 +
                std::pow((y_c - y0) * cos_alpha + (x_c - x0) * sin_alpha, 2) /
                    ellipse_b_2 +
                std::pow((y_c - y0) * sin_alpha + (x_c - x0) * cos_alpha, 2) /
                    ellipse_a_2));

      l_x[0] += log_g_dot_x;
      l_x[1] += log_g_dot_y;
      l_x[3] += log_g_dot_theta;

      // get l_u

      // get l_xx
      double log_g_dot_x_x =
          -(2 * sin_alpha * sin_alpha / ellipse_b_2 +
            2 * cos_alpha * cos_alpha / ellipse_a_2) /
              (t *
               (-1 +
                std::pow((y_c - y0) * cos_alpha + (x_c - x0) * sin_alpha, 2) /
                    ellipse_b_2 +
                std::pow((y_c - y0) * sin_alpha + (x_c - x0) * cos_alpha, 2) /
                    ellipse_a_2)) -
          (-2 * ((y_c - y0) * cos_alpha + (x_c - x0) * sin_alpha) * sin_alpha /
               ellipse_b_2 -
           2 * ((y_c - y0) * sin_alpha + (x_c - x0) * cos_alpha) * cos_alpha /
               ellipse_a_2) *
              (2 * ((y_c - y0) * cos_alpha + (x_c - x0) * sin_alpha) *
                   sin_alpha / ellipse_b_2 +
               2 * ((y_c - y0) * sin_alpha + (x_c - x0) * cos_alpha) *
                   cos_alpha / ellipse_a_2) /
              (t *
               std::pow(
                   -1 +
                       std::pow((y_c - y0) * cos_alpha + (x_c - x0) * sin_alpha,
                                2) /
                           ellipse_b_2 +
                       std::pow((y_c - y0) * sin_alpha + (x_c - x0) * cos_alpha,
                                2) /
                           ellipse_a_2,
                   2));
      double log_g_dot_x_y =
          -(2 * ((y_c - y0) * cos_alpha + (x_c - x0) * sin_alpha) * sin_alpha /
                ellipse_b_2 +
            2 * ((y_c - y0) * sin_alpha + (x_c - x0) * cos_alpha) * cos_alpha /
                ellipse_a_2) *
              (-2 * ((y_c - y0) * cos_alpha + (x_c - x0) * sin_alpha) *
                   cos_alpha / ellipse_b_2 -
               2 * ((y_c - y0) * sin_alpha + (x_c - x0) * cos_alpha) *
                   sin_alpha / ellipse_a_2) /
              (t *
               std::pow(
                   -1 +
                       std::pow((y_c - y0) * cos_alpha + (x_c - x0) * sin_alpha,
                                2) /
                           ellipse_b_2 +
                       std::pow((y_c - y0) * sin_alpha + (x_c - x0) * cos_alpha,
                                2) /
                           ellipse_a_2,
                   2)) -
          (2 * sin_alpha * cos_alpha / ellipse_b_2 +
           2 * sin_alpha * cos_alpha / ellipse_a_2) /
              (t *
               (-1 +
                std::pow((y_c - y0) * cos_alpha + (x_c - x0) * sin_alpha, 2) /
                    ellipse_b_2 +
                std::pow((y_c - y0) * sin_alpha + (x_c - x0) * cos_alpha, 2) /
                    ellipse_a_2));
      double log_g_dot_x_theta =
          -(-((y_c - y0) * cos_alpha + (x_c - x0) * sin_alpha) *
                (-2 * l_4 * r * sin_alpha * sin_theta +
                 2 * l_4 * r * cos_alpha * cos_theta) /
                ellipse_b_2 -
            ((y_c - y0) * sin_alpha + (x_c - x0) * cos_alpha) *
                (2 * l_4 * r * sin_alpha * cos_theta -
                 2 * l_4 * r * sin_theta * cos_alpha) /
                ellipse_a_2) *
              (2 * ((y_c - y0) * cos_alpha + (x_c - x0) * sin_alpha) *
                   sin_alpha / ellipse_b_2 +
               2 * ((y_c - y0) * sin_alpha + (x_c - x0) * cos_alpha) *
                   cos_alpha / ellipse_a_2) /
              (t *
               std::pow(
                   -1 +
                       std::pow((y_c - y0) * cos_alpha + (x_c - x0) * sin_alpha,
                                2) /
                           ellipse_b_2 +
                       std::pow((y_c - y0) * sin_alpha + (x_c - x0) * cos_alpha,
                                2) /
                           ellipse_a_2,
                   2)) -
          (2 * (-l_4 * r * sin_alpha * sin_theta +
                l_4 * r * cos_alpha * cos_theta) *
               sin_alpha / ellipse_b_2 +
           2 * (l_4 * r * sin_alpha * cos_theta -
                l_4 * r * sin_theta * cos_alpha) *
               cos_alpha / ellipse_a_2) /
              (t *
               (-1 +
                std::pow((y_c - y0) * cos_alpha + (x_c - x0) * sin_alpha, 2) /
                    ellipse_b_2 +
                std::pow((y_c - y0) * sin_alpha + (x_c - x0) * cos_alpha, 2) /
                    ellipse_a_2));

      double log_g_dot_y_x =
          -(-2 * ((y_c - y0) * cos_alpha + (x_c - x0) * sin_alpha) * sin_alpha /
                ellipse_b_2 -
            2 * ((y_c - y0) * sin_alpha + (x_c - x0) * cos_alpha) * cos_alpha /
                ellipse_a_2) *
              (2 * ((y_c - y0) * cos_alpha + (x_c - x0) * sin_alpha) *
                   cos_alpha / ellipse_b_2 +
               2 * ((y_c - y0) * sin_alpha + (x_c - x0) * cos_alpha) *
                   sin_alpha / ellipse_a_2) /
              (t *
               std::pow(
                   -1 +
                       std::pow((y_c - y0) * cos_alpha + (x_c - x0) * sin_alpha,
                                2) /
                           ellipse_b_2 +
                       std::pow((y_c - y0) * sin_alpha + (x_c - x0) * cos_alpha,
                                2) /
                           ellipse_a_2,
                   2)) -
          (2 * sin_alpha * cos_alpha / ellipse_b_2 +
           2 * sin_alpha * cos_alpha / ellipse_a_2) /
              (t *
               (-1 +
                std::pow((y_c - y0) * cos_alpha + (x_c - x0) * sin_alpha, 2) /
                    ellipse_b_2 +
                std::pow((y_c - y0) * sin_alpha + (x_c - x0) * cos_alpha, 2) /
                    ellipse_a_2));
      double log_g_dot_y_y =
          -(2 * cos_alpha * cos_alpha / ellipse_b_2 +
            2 * sin_alpha * sin_alpha / ellipse_a_2) /
              (t *
               (-1 +
                std::pow((y_c - y0) * cos_alpha + (x_c - x0) * sin_alpha, 2) /
                    ellipse_b_2 +
                std::pow((y_c - y0) * sin_alpha + (x_c - x0) * cos_alpha, 2) /
                    ellipse_a_2)) -
          (-2 * ((y_c - y0) * cos_alpha + (x_c - x0) * sin_alpha) * cos_alpha /
               ellipse_b_2 -
           2 * ((y_c - y0) * sin_alpha + (x_c - x0) * cos_alpha) * sin_alpha /
               ellipse_a_2) *
              (2 * ((y_c - y0) * cos_alpha + (x_c - x0) * sin_alpha) *
                   cos_alpha / ellipse_b_2 +
               2 * ((y_c - y0) * sin_alpha + (x_c - x0) * cos_alpha) *
                   sin_alpha / ellipse_a_2) /
              (t *
               std::pow(
                   -1 +
                       std::pow((y_c - y0) * cos_alpha + (x_c - x0) * sin_alpha,
                                2) /
                           ellipse_b_2 +
                       std::pow((y_c - y0) * sin_alpha + (x_c - x0) * cos_alpha,
                                2) /
                           ellipse_a_2,
                   2));
      double log_g_dot_y_theta =
          -(-((y_c - y0) * cos_alpha + (x_c - x0) * sin_alpha) *
                (-2 * l_4 * r * sin_alpha * sin_theta +
                 2 * l_4 * r * cos_alpha * cos_theta) /
                ellipse_b_2 -
            ((y_c - y0) * sin_alpha + (x_c - x0) * cos_alpha) *
                (2 * l_4 * r * sin_alpha * cos_theta -
                 2 * l_4 * r * sin_theta * cos_alpha) /
                ellipse_a_2) *
              (2 * ((y_c - y0) * cos_alpha + (x_c - x0) * sin_alpha) *
                   cos_alpha / ellipse_b_2 +
               2 * ((y_c - y0) * sin_alpha + (x_c - x0) * cos_alpha) *
                   sin_alpha / ellipse_a_2) /
              (t *
               std::pow(
                   -1 +
                       std::pow((y_c - y0) * cos_alpha + (x_c - x0) * sin_alpha,
                                2) /
                           ellipse_b_2 +
                       std::pow((y_c - y0) * sin_alpha + (x_c - x0) * cos_alpha,
                                2) /
                           ellipse_a_2,
                   2)) -
          (2 * (-l_4 * r * sin_alpha * sin_theta +
                l_4 * r * cos_alpha * cos_theta) *
               cos_alpha / ellipse_b_2 +
           2 * (l_4 * r * sin_alpha * cos_theta -
                l_4 * r * sin_theta * cos_alpha) *
               sin_alpha / ellipse_a_2) /
              (t *
               (-1 +
                std::pow((y_c - y0) * cos_alpha + (x_c - x0) * sin_alpha, 2) /
                    ellipse_b_2 +
                std::pow((y_c - y0) * sin_alpha + (x_c - x0) * cos_alpha, 2) /
                    ellipse_a_2));

      double log_g_dot_theta_x =
          -(((y_c - y0) * cos_alpha + (x_c - x0) * sin_alpha) *
                (-2 * l_4 * r * sin_alpha * sin_theta +
                 2 * l_4 * r * cos_alpha * cos_theta) /
                ellipse_b_2 +
            ((y_c - y0) * sin_alpha + (x_c - x0) * cos_alpha) *
                (2 * l_4 * r * sin_alpha * cos_theta -
                 2 * l_4 * r * sin_theta * cos_alpha) /
                ellipse_a_2) *
              (-2 * ((y_c - y0) * cos_alpha + (x_c - x0) * sin_alpha) *
                   sin_alpha / ellipse_b_2 -
               2 * ((y_c - y0) * sin_alpha + (x_c - x0) * cos_alpha) *
                   cos_alpha / ellipse_a_2) /
              (t *
               std::pow(
                   -1 +
                       std::pow((y_c - y0) * cos_alpha + (x_c - x0) * sin_alpha,
                                2) /
                           ellipse_b_2 +
                       std::pow((y_c - y0) * sin_alpha + (x_c - x0) * cos_alpha,
                                2) /
                           ellipse_a_2,
                   2)) -
          ((-2 * l_4 * r * sin_alpha * sin_theta +
            2 * l_4 * r * cos_alpha * cos_theta) *
               sin_alpha / ellipse_b_2 +
           (2 * l_4 * r * sin_alpha * cos_theta -
            2 * l_4 * r * sin_theta * cos_alpha) *
               cos_alpha / ellipse_a_2) /
              (t *
               (-1 +
                std::pow((y_c - y0) * cos_alpha + (x_c - x0) * sin_alpha, 2) /
                    ellipse_b_2 +
                std::pow((y_c - y0) * sin_alpha + (x_c - x0) * cos_alpha, 2) /
                    ellipse_a_2));
      double log_g_dot_theta_y =
          -(((y_c - y0) * cos_alpha + (x_c - x0) * sin_alpha) *
                (-2 * l_4 * r * sin_alpha * sin_theta +
                 2 * l_4 * r * cos_alpha * cos_theta) /
                ellipse_b_2 +
            ((y_c - y0) * sin_alpha + (x_c - x0) * cos_alpha) *
                (2 * l_4 * r * sin_alpha * cos_theta -
                 2 * l_4 * r * sin_theta * cos_alpha) /
                ellipse_a_2) *
              (-2 * ((y_c - y0) * cos_alpha + (x_c - x0) * sin_alpha) *
                   cos_alpha / ellipse_b_2 -
               2 * ((y_c - y0) * sin_alpha + (x_c - x0) * cos_alpha) *
                   sin_alpha / ellipse_a_2) /
              (t *
               std::pow(
                   -1 +
                       std::pow((y_c - y0) * cos_alpha + (x_c - x0) * sin_alpha,
                                2) /
                           ellipse_b_2 +
                       std::pow((y_c - y0) * sin_alpha + (x_c - x0) * cos_alpha,
                                2) /
                           ellipse_a_2,
                   2)) -
          ((-2 * l_4 * r * sin_alpha * sin_theta +
            2 * l_4 * r * cos_alpha * cos_theta) *
               cos_alpha / ellipse_b_2 +
           (2 * l_4 * r * sin_alpha * cos_theta -
            2 * l_4 * r * sin_theta * cos_alpha) *
               sin_alpha / ellipse_a_2) /
              (t *
               (-1 +
                std::pow((y_c - y0) * cos_alpha + (x_c - x0) * sin_alpha, 2) /
                    ellipse_b_2 +
                std::pow((y_c - y0) * sin_alpha + (x_c - x0) * cos_alpha, 2) /
                    ellipse_a_2));
      double log_g_dot_theta_theta =
          -(-((y_c - y0) * cos_alpha + (x_c - x0) * sin_alpha) *
                (-2 * l_4 * r * sin_alpha * sin_theta +
                 2 * l_4 * r * cos_alpha * cos_theta) /
                ellipse_b_2 -
            ((y_c - y0) * sin_alpha + (x_c - x0) * cos_alpha) *
                (2 * l_4 * r * sin_alpha * cos_theta -
                 2 * l_4 * r * sin_theta * cos_alpha) /
                ellipse_a_2) *
              (((y_c - y0) * cos_alpha + (x_c - x0) * sin_alpha) *
                   (-2 * l_4 * r * sin_alpha * sin_theta +
                    2 * l_4 * r * cos_alpha * cos_theta) /
                   ellipse_b_2 +
               ((y_c - y0) * sin_alpha + (x_c - x0) * cos_alpha) *
                   (2 * l_4 * r * sin_alpha * cos_theta -
                    2 * l_4 * r * sin_theta * cos_alpha) /
                   ellipse_a_2) /
              (t *
               std::pow(
                   -1 +
                       std::pow((y_c - y0) * cos_alpha + (x_c - x0) * sin_alpha,
                                2) /
                           ellipse_b_2 +
                       std::pow((y_c - y0) * sin_alpha + (x_c - x0) * cos_alpha,
                                2) /
                           ellipse_a_2,
                   2)) -
          (((y_c - y0) * cos_alpha + (x_c - x0) * sin_alpha) *
               (-2 * l_4 * r * sin_alpha * cos_theta -
                2 * l_4 * r * sin_theta * cos_alpha) /
               ellipse_b_2 +
           (-2 * l_4 * r * sin_alpha * sin_theta +
            2 * l_4 * r * cos_alpha * cos_theta) *
               (-l_4 * r * sin_alpha * sin_theta +
                l_4 * r * cos_alpha * cos_theta) /
               ellipse_b_2 +
           ((y_c - y0) * sin_alpha + (x_c - x0) * cos_alpha) *
               (-2 * l_4 * r * sin_alpha * sin_theta -
                2 * l_4 * r * cos_alpha * cos_theta) /
               ellipse_a_2 +
           (l_4 * r * sin_alpha * cos_theta - l_4 * r * sin_theta * cos_alpha) *
               (2 * l_4 * r * sin_alpha * cos_theta -
                2 * l_4 * r * sin_theta * cos_alpha) /
               ellipse_a_2) /
              (t *
               (-1 +
                std::pow((y_c - y0) * cos_alpha + (x_c - x0) * sin_alpha, 2) /
                    ellipse_b_2 +
                std::pow((y_c - y0) * sin_alpha + (x_c - x0) * cos_alpha, 2) /
                    ellipse_a_2));

      l_xx(0, 0) += log_g_dot_x_x;
      l_xx(0, 1) += log_g_dot_x_y;
      l_xx(0, 3) += log_g_dot_x_theta;
      l_xx(1, 0) += log_g_dot_y_x;
      l_xx(1, 1) += log_g_dot_y_y;
      l_xx(1, 3) += log_g_dot_y_theta;
      l_xx(3, 0) += log_g_dot_theta_x;
      l_xx(3, 1) += log_g_dot_theta_y;
      l_xx(3, 3) += log_g_dot_theta_theta;

      // std::cout << "x: " << x << std::endl;
      // std::cout << "y: " << y << std::endl;
      // std::cout << "theta: " << theta << std::endl;
      // std::cout << "x0: " << x0 << std::endl;
      // std::cout << "y0: " << y0 << std::endl;
      // std::cout << "a: " << ellipse_a_ << std::endl;
      // std::cout << "b: " << ellipse_b_ << std::endl;
      // std::cout << "alpha: " << alpha << std::endl;
      // std::cout << "r: " << r << std::endl;
      // std::cout << "l_4: " << l_4 << std::endl;
      // std::cout << "t: " << t << std::endl;
      // std::cout << std::fixed
      //           << std::setprecision(std::numeric_limits<double>::digits10 +
      //           1)
      //           << "log_g: " << log_g << std::endl;
      // std::cout << "log_g_dot_x: " << log_g_dot_x << std::endl;
      // std::cout << "log_g_dot_y: " << log_g_dot_y << std::endl;
      // std::cout << "log_g_dot_theta: " << log_g_dot_theta << std::endl;
      // std::cout << "log_g_dot_x_x: " << log_g_dot_x_x << std::endl;
      // std::cout << "log_g_dot_x_y: " << log_g_dot_x_y << std::endl;
      // std::cout << "log_g_dot_x_theta: " << log_g_dot_x_theta << std::endl;
      // std::cout << "log_g_dot_y_x: " << log_g_dot_y_x << std::endl;
      // std::cout << "log_g_dot_y_y: " << log_g_dot_y_y << std::endl;
      // std::cout << "log_g_dot_y_theta: " << log_g_dot_y_theta << std::endl;
      // std::cout << "log_g_dot_theta_x: " << log_g_dot_theta_x << std::endl;
      // std::cout << "log_g_dot_theta_y: " << log_g_dot_theta_y << std::endl;
      // std::cout << "log_g_dot_theta_theta: " << log_g_dot_theta_theta
      //           << std::endl;
    }
  }
}

void VehicleModelBicycle::step(const Eigen::VectorXd &state,
                               const Eigen::VectorXd &action,
                               Eigen::VectorXd &next_state) const {
  // state : x, y, v, theta
  if (state.size() != state_size_ || action.size() != action_size_) {
    std::cout << " input state size not right " << std::endl;
    return;
  }

//  Eigen::Matrix<double,4,4> a;
//  Eigen::Matrix<double,4,2> b;
//  a.resize(state_size_,state_size_);
//  a << 1.0 , 0.0, param_.delta_t * std::cos(state[3]) , - param_.delta_t * state[2] * std::sin(state[3]),
//      0.0 , 1.0, param_.delta_t * std::sin(state[3]) ,  param_.delta_t * state[2] * std::cos(state[3]),
//      0.0 , 0.0 , 1.0 , 0.0,
//      0.0 , 0.0 , 0.0 , 1.0;
//  b.resize(state_size_,action_size_);
//  b << 0.0, 0.0,
//      0.0, 0.0,
//      param_.delta_t, 0.0,
//      0.0, param_.delta_t;

  next_state.resize(state_size_);

  double next_x = state[0] + 0.5*(std::cos(state[3]) + std::cos(state[3] + action[1] * param_.delta_t))
      * (state[2]* param_.delta_t + 0.5 * action[0] * param_.delta_t * param_.delta_t);
  double next_y = state[1] + 0.5*(std::sin(state[3]) + std::sin(state[3] + action[1] * param_.delta_t))
      * (state[2]* param_.delta_t + 0.5 * action[0] * param_.delta_t * param_.delta_t);
  double next_v = state[2] + action[0] * param_.delta_t;
  double next_theta = state[3] + action[1] * param_.delta_t;

  next_state << next_x, next_y, next_v, next_theta;
}

std::vector<double> VehicleModelBicycle::step_std(const std::vector<double>& state_std,
              const std::vector<double>& action_std){
  Eigen::VectorXd state;
  Eigen::VectorXd action;
  Eigen::VectorXd next_state;
  std::vector<double> next_state_std;
  next_state_std.clear();
  if (state_std.size() != state_size_ || action_std.size() != action_size_) {
    std::cout << " input state size not right " << std::endl;
    return next_state_std;
  }
  state.resize(state_size_);
  for (int i = 0; i < state_size_; i++) {
    state[i] = state_std[i];
  }
  action.resize(action_size_);
  for (int i = 0; i < action_size_; i++) {
    action[i] = action_std[i];
  }
  next_state.resize(state_size_);
  step(state, action, next_state);

  for (int i = 0; i < state_size_; i++) {
    next_state_std.emplace_back(next_state[i]);
  }
  return next_state_std;
}
void VehicleModelBicycle::step_kappa(const Eigen::VectorXd &state,
                                     const Eigen::VectorXd &action,
                                     Eigen::VectorXd &next_state) {
  // state : x, y, v, kappa
  if (state.size() != state_size_ || action.size() != action_size_) {
    std::cout << " input state size not right " << std::endl;
    return;
  }

  next_state.resize(state_size_);
  double next_v = state[2] + action[0] * param_.delta_t;
  double l = 0.5 * (next_v + state[2]) * param_.delta_t;
  double next_theta = state[3] + action[1] * l;
  double next_x = state[0];
  double next_y = state[1];
  if (action[1] * l < param_.kappa_thr) {
    next_x += std::cos(state[3]) * l;
    next_y += std::sin(state[3]) * l;
  } else {
    next_x +=
        (std::sin(-state[3]) + std::sin(state[3] + action[1] * l)) / action[1];
    next_y +=
        (std::cos(state[3]) - std::cos(state[3] + action[1] * l)) / action[1];
  }

  next_state << next_x, next_y, next_v, next_theta;
}

void VehicleModelBicycle::step_kappa_fuzzy(const Eigen::VectorXd &state,
                                           const Eigen::VectorXd &action,
                                           Eigen::VectorXd &next_state) {
  // state : x, y, v, kappa
  if (state.size() != state_size_ || action.size() != action_size_) {
    std::cout << " input state size not right " << std::endl;
    return;
  }

  next_state.resize(state_size_);
  double next_v = state[2] + action[0] * param_.delta_t;
  double l = 0.5 * (next_v + state[2]) * param_.delta_t;
  double next_theta = state[3] + action[1] * l;
  double next_x = state[0];
  double next_y = state[1];
  next_x += 0.5 * (std::cos(state[3]) + std::cos(state[3] + action[1] * l)) * l;
  next_y += 0.5 * (std::sin(state[3]) + std::sin(state[3] + action[1] * l)) * l;

  next_state << next_x, next_y, next_v, next_theta;
}

std::vector<double>
VehicleModelBicycle::step_kappa_std(const std::vector<double> &state_std,
                                    const std::vector<double> &action_std) {
  Eigen::VectorXd state;
  Eigen::VectorXd action;
  Eigen::VectorXd next_state;
  std::vector<double> next_state_std;
  next_state_std.clear();
  if (state_std.size() != state_size_ || action_std.size() != action_size_) {
    std::cout << " input state size not right " << std::endl;
    return next_state_std;
  }
  state.resize(state_size_);
  for (int i = 0; i < state_size_; i++) {
    state[i] = state_std[i];
  }
  action.resize(action_size_);
  for (int i = 0; i < action_size_; i++) {
    action[i] = action_std[i];
  }
  next_state.resize(state_size_);
  step_kappa(state, action, next_state);

  for (int i = 0; i < state_size_; i++) {
    next_state_std.emplace_back(next_state[i]);
  }
  return next_state_std;
}

std::vector<double> VehicleModelBicycle::step_kappa_fuzzy_std(
    const std::vector<double> &state_std,
    const std::vector<double> &action_std) {
  Eigen::VectorXd state;
  Eigen::VectorXd action;
  Eigen::VectorXd next_state;
  std::vector<double> next_state_std;
  next_state_std.clear();
  if (state_std.size() != state_size_ || action_std.size() != action_size_) {
    std::cout << " input state size not right " << std::endl;
    return next_state_std;
  }
  state.resize(state_size_);
  for (int i = 0; i < state_size_; i++) {
    state[i] = state_std[i];
  }
  action.resize(action_size_);
  for (int i = 0; i < action_size_; i++) {
    action[i] = action_std[i];
  }
  next_state.resize(state_size_);
  step_kappa_fuzzy(state, action, next_state);

  for (int i = 0; i < state_size_; i++) {
    next_state_std.emplace_back(next_state[i]);
  }
  return next_state_std;
}

void VehicleModelBicycle::get_all_element(const int & frame_count,
                                          const std::shared_ptr<EnvSim::EnvSimulator> &env_ptr,
                                          const std::vector<ILQRObstacleConstrain>& cost_list,
                                          const std::shared_ptr<SolverSpace>& pre_space_ptr,
                                          std::shared_ptr<SolverSpace>& space_ptr) const {
  auto& state = space_ptr->x_space;
  auto& action = space_ptr->u_space;
  // x, y, v, theta
  double l = state[2] * param_.delta_t +
      0.5 * action[0] * param_.delta_t * param_.delta_t;
  double middle_sin = 0.5 * (std::sin(state[3]) +
      std::sin(state[3] + action[1] * param_.delta_t));
  double middle_cos = 0.5 * (std::cos(state[3]) +
      std::cos(state[3] + action[1] * param_.delta_t));

  // 1.0 get_f_x
  space_ptr->f_x_space.resize(param_.state_size,param_.state_size);
  space_ptr->f_x_space << 1.0, 0.0, param_.delta_t * middle_cos,  -middle_sin * l,
                          0.0, 1.0, param_.delta_t * middle_sin, middle_cos * l,
                          0.0, 0.0, 1.0, 0.0,
                          0.0, 0.0, 0.0, 1.0;
  // 2.0 get_f_xx
  space_ptr->f_xx_space.resize(param_.state_size,param_.state_size);
  space_ptr->f_xx_space << 0.0, 0.0, 0.0, middle_cos * l,
                           0.0, 0.0, 0.0, middle_sin * l,
                           0.0, 0.0, 0.0, 0.0,
                           0.0, 0.0, 0.0, 0.0;
  // 3.0 get_f_u
  space_ptr->f_u_space.resize(param_.state_size,param_.action_size);
  space_ptr->f_u_space << middle_cos * 0.5 * param_.delta_t * param_.delta_t, l * 0.5 * param_.delta_t * (-std::sin( param_.delta_t * action[1] + state[3])),
                          middle_sin * 0.5 * param_.delta_t * param_.delta_t, l * 0.5 * param_.delta_t * std::cos(param_.delta_t * action[1] + state[3]),
                          param_.delta_t, 0.0,
                          0.0, param_.delta_t;
  // 4.0 get_f_ux
  space_ptr->f_ux_space.resize(param_.action_size,param_.state_size);
  space_ptr->f_ux_space << 0.0, 0.0,
                           0.0, 0.0,
                           0.0, 0.0,
                           0.0, 0.0;
  // 5.0 get_f_uu
  space_ptr->f_uu_space.resize(param_.action_size,param_.state_size);
  space_ptr->f_uu_space << 0.0, 0.5 * l * param_.delta_t* param_.delta_t * (-std::cos(state[3] + action[1] * param_.delta_t)),
                           0.0, 0.5 * l * param_.delta_t* param_.delta_t * (-std::sin(state[3] + action[1] * param_.delta_t)),
                           0.0, 0.0,
                           0.0, 0.0;

  // 6.0 get l_condition
  if (env_ptr == nullptr){
    std::cout<< " nullptr !"<< std::endl;
    return;
  }

  // 6.1 ref point
  MathUtils::Point2D pos;
  pos.x = state[0];
  pos.y = state[1];
  auto match_point = env_ptr->get_nearest_point(pos);
  space_ptr->l_condition.has_ref_point = true;
  space_ptr->l_condition.ref_point = match_point;

  // 6.2 ref omega
  // todo: @jojo remove acc_lat ref
  double ref_curva = env_ptr->get_next_point_curva(pos);
  double acc_lat_ref = 1.0;
  double ref_curva_v = 0.0;
  if (std::abs(ref_curva) > 1e-4) {
    ref_curva_v = std::sqrt(acc_lat_ref / std::abs(ref_curva));
  }
  space_ptr->l_condition.has_ref_omega = true;
  space_ptr->l_condition.ref_curva = ref_curva;
  space_ptr->l_condition.ref_curva_v = ref_curva_v;
  space_ptr->l_condition.ref_omega = ref_curva * ref_curva_v;


  // 6.3 ref v
  // 6.4 ref a
  if (pre_space_ptr != nullptr){
    auto &pre_state = pre_space_ptr->x_space;
    auto &pre_condition = pre_space_ptr->l_condition;
    get_l_lon_condition(frame_count,pre_state,pre_condition,env_ptr,space_ptr->l_condition);
  }

  // 6.5 ref acc_lat
  space_ptr->l_condition.has_ref_acc_lat = false;
  space_ptr->l_condition.acc_lat_ref = 0.0;

  // 6.6 constraint acc_lat
  space_ptr->l_condition.has_constraint_acc_lat = false;

  // 6.7 constraint omega
  space_ptr->l_condition.has_constraint_omega = false;


  // 7.0 get l
  space_ptr->l_space = 0.0;
  // 7.1 ref point
  if (space_ptr->l_condition.has_ref_point) {
    double delta_x = state[0] - space_ptr->l_condition.ref_point.x;
    double delta_y = state[1] - space_ptr->l_condition.ref_point.y;
    space_ptr->l_space += param_.w_ref_line * (delta_x * delta_x + delta_y * delta_y);
  }

  // 7.2 ref v
  if (space_ptr->l_condition.has_ref_v) {
    double delta_v = state[2] - space_ptr->l_condition.v_ref;
    space_ptr->l_space += param_.w_ref_v * (delta_v * delta_v);
  }

  // 7.3 ref a
  if (space_ptr->l_condition.has_ref_a) {
    double delta_acc = action[0] - space_ptr->l_condition.a_ref;
    space_ptr->l_space += param_.w_ref_a * (delta_acc * delta_acc);
  }

  // 7.4 ref omega
  if (space_ptr->l_condition.has_ref_omega) {
    double delta_omega = action[1] - space_ptr->l_condition.ref_omega;
    space_ptr->l_space += param_.w_ref_omega * (delta_omega * delta_omega);
  }

  // 7.5 ref acc_lat
  double acc_lat_real = state[2] * action[1];
  if (space_ptr->l_condition.has_ref_acc_lat) {
    double delta_acc_lat = acc_lat_real - space_ptr->l_condition.acc_lat_ref;
    space_ptr->l_space += param_.w_ref_acc_lat * (delta_acc_lat * delta_acc_lat);
  }

  // 7.6 constraint acc_lat
//  if (space_ptr->l_condition.has_constraint_acc_lat) {
//    space_ptr->l_space += param_.q1_acc_lat_max *
//        std::exp(param_.q2_acc_lat_max * (acc_lat_real - param_.acc_lat_max));
//    space_ptr->l_space += param_.q1_acc_lat_min *
//        std::exp(param_.q2_acc_lat_min * (param_.acc_lat_min - acc_lat_real));
//  }

  // 7.7 constraint omega
//  if (space_ptr->l_condition.has_constraint_omega) {
//    space_ptr->l_space += param_.q1_omega_max *
//        std::exp(param_.q2_omega_max * (action[1] - param_.omega_max));
//    space_ptr->l_space += param_.q1_omega_min *
//        std::exp(param_.q2_omega_min * (param_.omega_min - action[1]));
//  }
  // 8.0 get l_x
  // 8.1 ref line
  space_ptr->l_x_space.resize(param_.state_size);
  space_ptr->l_x_space << param_.w_ref_line * 2.0 * (state[0] - space_ptr->l_condition.ref_point.x),
                          param_.w_ref_line * 2.0 * (state[1] - space_ptr->l_condition.ref_point.y), 0.0, 0.0;

  // 8.2 ref v
  if (space_ptr->l_condition.has_ref_v) {
    space_ptr->l_x_space[2] += 2.0 * param_.w_ref_v * (state[2] - space_ptr->l_condition.v_ref);
  }

  // 8.3 ref acc_lat
  if (space_ptr->l_condition.has_ref_acc_lat) {
    space_ptr->l_x_space[2] += 2.0 * param_.w_ref_acc_lat * (state[2] * action[1] * action[1] -
        space_ptr->l_condition.acc_lat_ref * action[1]);
  }

  // 8.4 constraint acc_lat
//  if (space_ptr->l_condition.has_constraint_acc_lat) {
//    space_ptr->l_x_space[2] += param_.q1_acc_lat_max * param_.q2_acc_lat_max * action[1] *
//        std::exp(param_.q2_acc_lat_max * (acc_lat_real - param_.acc_lat_max));
//    space_ptr->l_x_space[2] += -param_.q1_acc_lat_min * param_.q2_acc_lat_min * action[1] *
//        std::exp(param_.q2_acc_lat_min * (param_.acc_lat_min - acc_lat_real));
//  }
  // 9.0 get l_u
  space_ptr->l_u_space.resize(param_.action_size);
  space_ptr->l_u_space << 0.0, 0.0;

  // 9.1 ref omega
  if (space_ptr->l_condition.has_ref_omega) {
    space_ptr->l_u_space[1] += 2.0 * param_.w_ref_omega * (action[1] - space_ptr->l_condition.ref_omega);
  }

  // 9.2 ref acc
  if (space_ptr->l_condition.has_ref_a) {
    space_ptr->l_u_space[0] += 2.0 * param_.w_ref_a * (action[0] - space_ptr->l_condition.a_ref);
  }

  // 9.3 ref acc_lat
  if (space_ptr->l_condition.has_ref_acc_lat) {
    space_ptr->l_u_space[1] += 2.0 * param_.w_ref_acc_lat * (action[1] * state[2] * state[2] -
        space_ptr->l_condition.acc_lat_ref * state[2]);
  }

  // 9.4 constraint acc_lat
//  if (space_ptr->l_condition.has_constraint_acc_lat) {
//    space_ptr->l_u_space[1] += param_.q1_acc_lat_max * param_.q2_acc_lat_max * state[2] *
//        std::exp(param_.q2_acc_lat_max * (acc_lat_real - param_.acc_lat_max));
//    space_ptr->l_u_space[1] += -param_.q1_acc_lat_min * param_.q2_acc_lat_min * state[2] *
//        std::exp(param_.q2_acc_lat_min * (param_.acc_lat_min - acc_lat_real));
//  }

  // 9.5 constraint omega
//  if (space_ptr->l_condition.has_constraint_omega) {
//    space_ptr->l_u_space[1] += param_.q1_omega_max * param_.q2_omega_max *
//        std::exp(param_.q2_omega_max * (action[1] - param_.omega_max));
//    space_ptr->l_u_space[1] += -param_.q1_omega_min * param_.q2_omega_min *
//        std::exp(param_.q2_omega_min * (param_.omega_min - action[1]));
//  }
  // 10.0 get l_xx
  space_ptr->l_xx_space.resize(param_.state_size,param_.state_size);
  // 10.1 ref line
  space_ptr->l_xx_space<< param_.w_ref_line * 2.0, 0.0, 0.0 ,0.0,
                          0.0, param_.w_ref_line * 2.0, 0.0 ,0.0,
                          0.0, 0.0, 0.0 ,0.0,
                          0.0, 0.0, 0.0 ,0.0;

  // 10.2 ref v
  if (space_ptr->l_condition.has_ref_v) {
    space_ptr->l_xx_space(2, 2) += 2.0 * param_.w_ref_v;
  }

  // 10.3 ref acc_lat
  if (space_ptr->l_condition.has_ref_acc_lat) {
    space_ptr->l_xx_space(2, 2) += 2.0 * param_.w_ref_acc_lat * action[1] * action[1];
  }

  // 10.4 constraint acc_lat
//  if (space_ptr->l_condition.has_constraint_acc_lat) {
//    space_ptr->l_xx_space(2, 2) +=
//        param_.q1_acc_lat_max * param_.q2_acc_lat_max * param_.q2_acc_lat_max *
//            action[1] * action[1] *
//            std::exp(param_.q2_acc_lat_max * (acc_lat_real - param_.acc_lat_max));
//    space_ptr->l_xx_space(2, 2) +=
//        param_.q1_acc_lat_min * param_.q2_acc_lat_min * param_.q2_acc_lat_min *
//            action[1] * action[1] *
//            std::exp(param_.q2_acc_lat_min * (param_.acc_lat_min - acc_lat_real));
//  }

  // 11.0 get l_ux
  space_ptr->l_ux_space.resize(param_.action_size,param_.state_size);
  space_ptr->l_ux_space << 0.0, 0.0,
                           0.0, 0.0,
                           0.0, 0.0,
                           0.0, 0.0;

  // 11.1 ref acc_lat
  if (space_ptr->l_condition.has_ref_acc_lat) {
    space_ptr->l_ux_space(1, 2) += 2.0 * param_.w_ref_acc_lat *
        (2.0 * state[2] * action[1] - space_ptr->l_condition.acc_lat_ref);
  }

  // 11.2 constraint acc_lat
//  if (space_ptr->l_condition.has_constraint_acc_lat) {
//    space_ptr->l_ux_space(1, 2) +=
//        param_.q1_acc_lat_max * param_.q2_acc_lat_max *
//            (1.0 + param_.q2_acc_lat_max * acc_lat_real) *
//            std::exp(param_.q2_acc_lat_max * (acc_lat_real - param_.acc_lat_max));
//    space_ptr->l_ux_space(1, 2) +=
//        param_.q1_acc_lat_min * param_.q2_acc_lat_min *
//            (param_.q2_acc_lat_min * acc_lat_real - 1.0) *
//            std::exp(param_.q2_acc_lat_min * (param_.acc_lat_min - acc_lat_real));
//  }

  // 12.0 get l_uu
  space_ptr->l_uu_space.resize(param_.action_size,param_.action_size);
  space_ptr->l_uu_space<< 0.0, 0.0,
                          0.0,0.0;

  // 12.1 ref omega
  if (space_ptr->l_condition.has_ref_omega) {
    space_ptr->l_uu_space(1, 1) += 2.0 * param_.w_ref_omega;
  }

  // 12.2 ref acc
  if (space_ptr->l_condition.has_ref_a) {
    space_ptr->l_uu_space(0, 0) += 2.0 * param_.w_ref_a;
  }

  // 12.3 ref acc_lat
  if (space_ptr->l_condition.has_ref_acc_lat) {
    space_ptr->l_uu_space(1, 1) += 2.0 * param_.w_ref_acc_lat * state[2] * state[2];
  }

  // 12.4 constraint acc_lat
//  if (space_ptr->l_condition.has_constraint_acc_lat) {
//    double acc_lat = state[2] * action[1];
//    space_ptr->l_uu_space(1, 1) +=
//        param_.q1_acc_lat_max * param_.q2_acc_lat_max * param_.q2_acc_lat_max *
//            state[2] * state[2] *
//            std::exp(param_.q2_acc_lat_max * (acc_lat - param_.acc_lat_max));
//    space_ptr->l_uu_space(1, 1) +=
//        param_.q1_acc_lat_min * param_.q2_acc_lat_min * param_.q2_acc_lat_min *
//            state[2] * state[2] *
//            std::exp(param_.q2_acc_lat_min * (param_.acc_lat_min - acc_lat));
//  }

  // 12.5 constraint omega
//  if (space_ptr->l_condition.has_constraint_omega) {
//    space_ptr->l_uu_space(1, 1) +=
//        param_.q1_omega_max * param_.q2_omega_max * param_.q2_omega_max *
//            std::exp(param_.q2_omega_max * (action[1] - param_.omega_max));
//    space_ptr->l_uu_space(1, 1) +=
//        param_.q1_omega_min * param_.q2_omega_min * param_.q2_omega_min *
//            std::exp(param_.q2_omega_min * (param_.omega_min - action[1]));
//  }
  // 13.0 obstacle constraint cost
  for (ILQRObstacleConstrain cost : cost_list) {
    cost.get_l_condition(frame_count, space_ptr->x_space, env_ptr, space_ptr->l_condition);
    cost.get_all_l_element(space_ptr->x_space,  space_ptr->l_condition, space_ptr->l_space, space_ptr->l_x_space,
                           space_ptr->l_xx_space);
    cost.get_l(space_ptr->l_condition, space_ptr->l_space);
  }

  // 14.0 last point part
  // todo: @jojo
}

// Get ILQRSpace f_x
// Args:
//    state: Current state [state_size].
//    action: Current control [action_size].
//    f_x: Output arg, df/dx [state_size, state_size].
void VehicleModelBicycle::get_f_x(const Eigen::VectorXd &state,
                                  const Eigen::VectorXd &action,
                                  Eigen::MatrixXd &f_x) {
  // x, y, v, theta
  double l = state[2] * param_.delta_t +
             0.5 * action[0] * param_.delta_t * param_.delta_t;
  double middle_sin = 0.5 * (std::sin(state[3]) +
                             std::sin(state[3] + action[1] * param_.delta_t));
  double middle_cos = 0.5 * (std::cos(state[3]) +
                             std::cos(state[3] + action[1] * param_.delta_t));

  f_x.resize(param_.state_size,param_.state_size);
  f_x << 1.0, 0.0, param_.delta_t * middle_cos,  -middle_sin * l,
      0.0, 1.0, param_.delta_t * middle_sin, middle_cos * l,
      0.0, 0.0, 1.0, 0.0,
      0.0, 0.0, 0.0, 1.0;
}

// Get ILQRSpace f_xx
//    state: Current state [state_size].
//    action: Current control [action_size].
//    f_xx: Output arg, d^2f/(dx_i * dx_j) [state_size, state_size].
void VehicleModelBicycle::get_f_xx(const Eigen::VectorXd& state,
              const Eigen::VectorXd& action,
              Eigen::MatrixXd& f_xx){
  double l = state[2] * param_.delta_t + 0.5 * action[0] * param_.delta_t* param_.delta_t;
  double middle_sin = 0.5 * (std::sin(state[3]) + std::sin(state[3] + action[1] * param_.delta_t));
  double middle_cos = 0.5 * (std::cos(state[3]) + std::cos(state[3] + action[1] * param_.delta_t));
  f_xx.resize(param_.state_size,param_.state_size);
  f_xx << 0.0, 0.0, 0.0, middle_cos * l,
          0.0, 0.0, 0.0, middle_sin * l,
          0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0;
}

// Get ILQRSpace f_ux
//    state: Current state [state_size].
//    action: Current control [action_size].
//    f_ux: Output arg, d^2f/(du * dx) [action_size, state_size].
void VehicleModelBicycle::get_f_ux(const Eigen::VectorXd& state,
              const Eigen::VectorXd& action,
              Eigen::MatrixXd& f_ux){
  f_ux.resize(param_.action_size,param_.state_size);
  f_ux << 0.0, 0.0,
          0.0, 0.0,
          0.0, 0.0,
          0.0, 0.0;
}

// Get ILQRSpace f_uu
//    state: Current state [state_size].
//    action: Current control [action_size].
//    f_uu: Output arg, d^2f/(du * du) [action_size, action_size].
void VehicleModelBicycle::get_f_uu(const Eigen::VectorXd& state,
              const Eigen::VectorXd& action,
              Eigen::MatrixXd& f_uu){
  double l = state[2] * param_.delta_t + 0.5 * action[0] * param_.delta_t* param_.delta_t;
  f_uu.resize(param_.action_size,param_.state_size);
  f_uu << 0.0, 0.5 * l * param_.delta_t* param_.delta_t * (-std::cos(state[3] + action[1] * param_.delta_t)),
          0.0, 0.5 * l * param_.delta_t* param_.delta_t * (-std::sin(state[3] + action[1] * param_.delta_t)),
          0.0, 0.0,
          0.0, 0.0;
}

// Get ILQRSpace u_x
// Args:
//    state: Current state [state_size].
//    action: Current control [action_size].
//    u_x: Output arg, df/du [state_size, action_size].
void VehicleModelBicycle::get_f_u(const Eigen::VectorXd &state,
                                  const Eigen::VectorXd &action,
                                  Eigen::MatrixXd &u_x) {
  // x, y, v, theta
  double l = state[2] * param_.delta_t + 0.5 * action[0] * param_.delta_t* param_.delta_t;
  double middle_sin = 0.5 * (std::sin(state[3]) + std::sin(state[3] + action[1] * param_.delta_t));
  double middle_cos = 0.5 * (std::cos(state[3]) + std::cos(state[3] + action[1] * param_.delta_t));
  u_x.resize(param_.state_size,param_.action_size);
  u_x << middle_cos * 0.5 * param_.delta_t * param_.delta_t, l * 0.5 * param_.delta_t * (-std::sin( param_.delta_t * action[1] + state[3])),
         middle_sin * 0.5 * param_.delta_t * param_.delta_t, l * 0.5 * param_.delta_t * std::cos(param_.delta_t * action[1] + state[3]),
         param_.delta_t, 0.0,
         0.0, param_.delta_t;
}

void VehicleModelBicycle::get_l_lon_condition(const int & frame_count,
                         const Eigen::VectorXd& pre_state,
                         const StepCondition& pre_l_condition,
                         const std::shared_ptr<EnvSim::EnvSimulator> &env_ptr,
                         StepCondition & l_condition) const{
  MathUtils::Point2D pos;
  pos.x = pre_state[0];
  pos.y = pre_state[1];
  double s_ego = env_ptr->get_lane_s(pos);
  double v_ego;
  if (pre_l_condition.has_ref_v){
    v_ego = pre_l_condition.v_ref;
  }else{
    v_ego = pre_state[2];
  }
  double  min_a = 0.0;
  for(auto& item : l_condition.obstacle_belief_state_map){
    if (item.second.belief < 1e-2){
      continue;
    }
    double v_obj = item.second.v.x * std::cos(pre_state[3]) + item.second.v.y * std::sin(pre_state[3]);
    double a_obj = item.second.a.x * std::cos(pre_state[3]) + item.second.a.y * std::sin(pre_state[3]);
    if (v_obj > v_ego){
      continue;
    }
    double delta_s =  item.second.s - 0.5 * item.second.length - s_ego + 0.5 * param_.vehicle_param.length;
    if (delta_s - 2.0 < 0){
      min_a = pre_l_condition.a_ref;
    }else{
      double a_desire = (v_ego - v_obj) * (v_ego - v_obj) * 0.5 / (delta_s- 2.0);
      if (-a_desire < min_a){
        min_a = -a_desire;
      }
    }
  }
  double curvature = env_ptr->get_next_point_curva(pos);
  l_condition.has_ref_a = true;
  l_condition.a_ref = min_a;
  l_condition.has_ref_v = true;
  l_condition.v_ref = v_ego + min_a * param_.delta_t;
}

void VehicleModelBicycle::get_l_condition(const int& frame_count, 
                                          const Eigen::VectorXd& state,
                                          const std::shared_ptr<EnvSim::EnvSimulator> &env_ptr,
                                          StepCondition & l_condition) const {
  // state : x, y, v, theta
  if (env_ptr == nullptr){
    std::cout<< " nullptr !"<< std::endl;
  }

  // 1.0 ref point
  MathUtils::Point2D pos;
  pos.x = state[0];
  pos.y = state[1];
  auto match_point = env_ptr->get_nearest_point(pos);
  l_condition.has_ref_point = true;
  l_condition.ref_point = match_point;

  // 2.0 ref omega
  double ref_curva = env_ptr->get_next_point_curva(pos);
  double acc_lat = 1.0;
  double ref_curva_v = 0.0;
  if (std::abs(ref_curva) > 1e-4) {
    ref_curva_v = std::sqrt(acc_lat / std::abs(ref_curva));
  }
  l_condition.has_ref_omega = true;
  l_condition.ref_curva = ref_curva;
  l_condition.ref_curva_v = ref_curva_v;
  l_condition.ref_omega = ref_curva * ref_curva_v;

  // 3.0 ref v

//  if (!v_ref_.empty()) {
//    double ref_v = v_ref_.at(frame_count);
//    l_condition.has_ref_v = true;
//    l_condition.v_ref = ref_v;
//  }

//   if l_condition
  
  // 4.0 ref a
//  if (!a_ref_.empty()) {
//    double ref_a = a_ref_.at(frame_count);
//    l_condition.has_ref_a = true;
//    l_condition.a_ref = ref_a;
//  }
  
  // 5.0 ref acc_lat
  l_condition.has_ref_acc_lat = false;
  l_condition.acc_lat_ref = 0.0;

  // 6.0 constraint acc_lat
  l_condition.has_constraint_acc_lat = true;

  // 7.0 constraint omega
  l_condition.has_constraint_omega = true;
};

// Get ILQRSpace l
// Args:
//    state: Current state [state_size].
//    action: Current control [action_size].
//    l: Output arg, cost .
void VehicleModelBicycle::get_l(const Eigen::VectorXd &state,
                                const Eigen::VectorXd &action,
                                const StepCondition &condition, double &l) const {
  l = 0.0;
  // 1.0 ref point
  if (condition.has_ref_point) {
    double delta_x = state[0] - condition.ref_point.x;
    double delta_y = state[1] - condition.ref_point.y;
    l += param_.w_ref_line * (delta_x * delta_x + delta_y * delta_y);
  }

  // // 2.0 ref v
  if (condition.has_ref_v) {
    double delta_v = state[2] - condition.v_ref;
    l += param_.w_ref_v * (delta_v * delta_v);
  }

  // // 3.0 ref a
  if (condition.has_ref_a) {
    double delta_acc = action[0] - condition.a_ref;
    l += param_.w_ref_a * (delta_acc * delta_acc);
  }else{

  }

  // 4.0 ref omega
  if (condition.has_ref_omega) {
    double delta_omega = action[1] - condition.ref_omega;
    l += param_.w_ref_omega * (delta_omega * delta_omega);
  }

  // 5.0 ref acc_lat
  if (condition.has_ref_acc_lat) {
    double acc_lat = state[2] * action[1];
    double delta_acc_lat = acc_lat - condition.acc_lat_ref;
    l += param_.w_ref_acc_lat * (delta_acc_lat * delta_acc_lat);
  }

  // 6.0 constraint acc_lat
//  if (condition.has_constraint_acc_lat) {
//    double acc_lat = state[2] * action[1];
//    l += param_.q1_acc_lat_max *
//         std::exp(param_.q2_acc_lat_max * (acc_lat - param_.acc_lat_max));
//    l += param_.q1_acc_lat_min *
//         std::exp(param_.q2_acc_lat_min * (param_.acc_lat_min - acc_lat));
//  }

  // 7.0 constraint omega
//  if (condition.has_constraint_omega) {
//    l += param_.q1_omega_max *
//         std::exp(param_.q2_omega_max * (action[1] - param_.omega_max));
//    l += param_.q1_omega_min *
//         std::exp(param_.q2_omega_min * (param_.omega_min - action[1]));
//  }
}

// Get ILQRSpace l_f
// Args:
//    state: Current state [state_size].
//    action: Current control [action_size].
//    l_f: Output arg, last point cost .
void VehicleModelBicycle::get_l_f(const Eigen::VectorXd &state,
                                  const StepCondition &condition, double &l_f) const {
  l_f = 0.0;
  // 1.0 ref point
  if (condition.has_ref_point) {
    double delta_x = state[0] - condition.ref_point.x;
    double delta_y = state[1] - condition.ref_point.y;
    l_f += param_.w_ref_line * (delta_x * delta_x + delta_y * delta_y);
  }

  //  2.0 ref v
  if (condition.has_ref_v) {
    double delta_v = state[2] - condition.v_ref;
    l_f += param_.w_ref_v * (delta_v * delta_v);
  }

  // 3.0 ref a

  // 4.0 ref omega

  // 5.0 ref acc_lat

  // 6.0 constraint acc_lat

  // 7.0 constraint omega

}

// Get ILQRSpace l_f_x
// Args:
//    state: Current state [state_size].
//    action: Current control [action_size].
//    l_f: Output arg, last point cost .
void VehicleModelBicycle::get_l_f_x(const Eigen::VectorXd &state,
                                    const StepCondition &condition,
                                    Eigen::VectorXd &l_f_x) {
  l_f_x.resize(param_.state_size);
  // 1.0 ref line
  l_f_x << param_.w_ref_line * 2.0 * (state[0] - condition.ref_point.x),
      param_.w_ref_line * 2.0 * (state[1] - condition.ref_point.y), 0.0, 0.0;

  // 2.0 ref v
  if (condition.has_ref_v) {
    l_f_x[2] += 2.0 * param_.w_ref_v * (state[2] - condition.v_ref);
  }
}

// Get ILQRSpace l_x
// Args:
//    state: Current state [state_size].
//    action: Current control [action_size].
//    l_x: Output arg, Jacobian of cost path w.r.t. x [N+1, state_size].
void VehicleModelBicycle::get_l_x(const Eigen::VectorXd &state,
                                  const Eigen::VectorXd &action,
                                  const StepCondition &condition,
                                  Eigen::VectorXd &l_x) {
  // 1.0 ref line
  l_x.resize(param_.state_size);
  l_x << param_.w_ref_line * 2.0 * (state[0] - condition.ref_point.x),
      param_.w_ref_line * 2.0 * (state[1] - condition.ref_point.y), 0.0, 0.0;

  // 2.0 ref v
  if (condition.has_ref_v) {
    l_x[2] += 2.0 * param_.w_ref_v * (state[2] - condition.v_ref);
  }

  // 3.0 ref acc_lat
  if (condition.has_ref_acc_lat) {
    l_x[2] += 2.0 * param_.w_ref_acc_lat * (state[2] * action[1] * action[1] -
                                            condition.acc_lat_ref * action[1]);
  }

  // 4.0 constraint acc_lat
//  if (condition.has_constraint_acc_lat) {
//    double acc_lat = state[2] * action[1];
//    l_x[2] += param_.q1_acc_lat_max * param_.q2_acc_lat_max * action[1] *
//              std::exp(param_.q2_acc_lat_max * (acc_lat - param_.acc_lat_max));
//    l_x[2] += -param_.q1_acc_lat_min * param_.q2_acc_lat_min * action[1] *
//              std::exp(param_.q2_acc_lat_min * (param_.acc_lat_min - acc_lat));
//  }
}

// Get ILQRSpace l_u
// Args:
//    state: Current state [state_size].
//    action: Current control [action_size].
//    l_u: Output arg, Jacobian of cost path w.r.t. x [N, action_size].
void VehicleModelBicycle::get_l_u(const Eigen::VectorXd &state,
                                  const Eigen::VectorXd &action,
                                  const StepCondition &condition,
                                  Eigen::VectorXd &l_u) {
  l_u.resize(param_.action_size);
  l_u << 0.0, 0.0;

  // 1.0 ref omega
  if (condition.has_ref_omega) {
    l_u[1] += 2.0 * param_.w_ref_omega * (action[1] - condition.ref_omega);
  }

  // 2.0 ref acc
  if (condition.has_ref_a) {
    l_u[0] += 2.0 * param_.w_ref_a * (action[0] - condition.a_ref);
  }

  // 3.0 ref acc_lat
  if (condition.has_ref_acc_lat) {
    l_u[1] += 2.0 * param_.w_ref_acc_lat * (action[1] * state[2] * state[2] -
                                            condition.acc_lat_ref * state[2]);
  }

  // 4.0 constraint acc_lat
//  if (condition.has_constraint_acc_lat) {
//    double acc_lat = state[2] * action[1];
//    l_u[1] += param_.q1_acc_lat_max * param_.q2_acc_lat_max * state[2] *
//              std::exp(param_.q2_acc_lat_max * (acc_lat - param_.acc_lat_max));
//    l_u[1] += -param_.q1_acc_lat_min * param_.q2_acc_lat_min * state[2] *
//              std::exp(param_.q2_acc_lat_min * (param_.acc_lat_min - acc_lat));
//  }

  // 5.0 constraint omega
//  if (condition.has_constraint_omega) {
//    l_u[1] += param_.q1_omega_max * param_.q2_omega_max *
//              std::exp(param_.q2_omega_max * (action[1] - param_.omega_max));
//    l_u[1] += -param_.q1_omega_min * param_.q2_omega_min *
//              std::exp(param_.q2_omega_min * (param_.omega_min - action[1]));
//  }
}

// Get ILQRSpace l_xx
// Args:
//    state: Current state [state_size].
//    action: Current control [action_size].
//    l_xx: Output arg, Jacobian of cost path w.r.t. x [N+1, state_size,
//    state_size].
void VehicleModelBicycle::get_l_xx(const Eigen::VectorXd &state,
                                   const Eigen::VectorXd &action,
                                   const StepCondition &condition,
                                   Eigen::MatrixXd &l_xx) {
  // 1.0 ref line
  l_xx.resize(param_.state_size,param_.state_size);
  l_xx<< param_.w_ref_line * 2.0, 0.0, 0.0 ,0.0,
        0.0, param_.w_ref_line * 2.0, 0.0 ,0.0,
        0.0, 0.0, 0.0 ,0.0,
        0.0, 0.0, 0.0 ,0.0;
  
  // 2.0 ref v
  if (condition.has_ref_v) {
    l_xx(2, 2) += 2.0 * param_.w_ref_v;
  }

  // 3.0 ref acc_lat
  if (condition.has_ref_acc_lat) {
    l_xx(2, 2) += 2.0 * param_.w_ref_acc_lat * action[1] * action[1];
  }

  // 4.0 constraint acc_lat
//  if (condition.has_constraint_acc_lat) {
//    double acc_lat = state[2] * action[1];
//    l_xx(2, 2) +=
//        param_.q1_acc_lat_max * param_.q2_acc_lat_max * param_.q2_acc_lat_max *
//        action[1] * action[1] *
//        std::exp(param_.q2_acc_lat_max * (acc_lat - param_.acc_lat_max));
//    l_xx(2, 2) +=
//        param_.q1_acc_lat_min * param_.q2_acc_lat_min * param_.q2_acc_lat_min *
//        action[1] * action[1] *
//        std::exp(param_.q2_acc_lat_min * (param_.acc_lat_min - acc_lat));
//  }
}

// Get ILQRSpace l_ux
// Args:
//    state: Current state [state_size].
//    action: Current control [action_size].
//    l_ux: Output arg, Jacobian of cost path w.r.t. x [N, action_size, state_size].
void VehicleModelBicycle::get_l_ux(const Eigen::VectorXd& state,
              const Eigen::VectorXd& action,
              const StepCondition& condition,
              Eigen::MatrixXd& l_ux){
  l_ux.resize(param_.action_size,param_.state_size);
  l_ux << 0.0, 0.0,
          0.0, 0.0,
          0.0, 0.0,
          0.0, 0.0;
  
  // 1.0 ref acc_lat
  if (condition.has_ref_acc_lat) {
    l_ux(1, 2) += 2.0 * param_.w_ref_acc_lat *
                  (2.0 * state[2] * action[1] - condition.acc_lat_ref);
  }

  // 2.0 constraint acc_lat
//  if (condition.has_constraint_acc_lat) {
//    double acc_lat = state[2] * action[1];
//    l_ux(1, 2) +=
//        param_.q1_acc_lat_max * param_.q2_acc_lat_max *
//        (1.0 + param_.q2_acc_lat_max * acc_lat) *
//        std::exp(param_.q2_acc_lat_max * (acc_lat - param_.acc_lat_max));
//    l_ux(1, 2) +=
//        param_.q1_acc_lat_min * param_.q2_acc_lat_min *
//        (param_.q2_acc_lat_min * acc_lat - 1.0) *
//        std::exp(param_.q2_acc_lat_min * (param_.acc_lat_min - acc_lat));
//  }
}

// Get ILQRSpace l_uu
// Args:
//    state: Current state [state_size].
//    action: Current control [action_size].
//    l_uu: Output arg, Jacobian of cost path w.r.t. x [N, action_size, action_size].
void VehicleModelBicycle::get_l_uu(const Eigen::VectorXd& state,
              const Eigen::VectorXd& action,
              const StepCondition& condition,
              Eigen::MatrixXd& l_uu){
  l_uu.resize(param_.action_size,param_.action_size);
  l_uu<< 0.0, 0.0,
         0.0,0.0;

  // 1.0 ref omega
  if (condition.has_ref_omega) {
    l_uu(1, 1) += 2.0 * param_.w_ref_omega;
  }

  // 2.0 ref acc
  if (condition.has_ref_a) {
    l_uu(0, 0) += 2.0 * param_.w_ref_a;
  }

  // 3.0 ref acc_lat
  if (condition.has_ref_acc_lat) {
    l_uu(1, 1) += 2.0 * param_.w_ref_acc_lat * state[2] * state[2];
  }

  // 4.0 constraint acc_lat
//  if (condition.has_constraint_acc_lat) {
//    double acc_lat = state[2] * action[1];
//    l_uu(1, 1) +=
//        param_.q1_acc_lat_max * param_.q2_acc_lat_max * param_.q2_acc_lat_max *
//        state[2] * state[2] *
//        std::exp(param_.q2_acc_lat_max * (acc_lat - param_.acc_lat_max));
//    l_uu(1, 1) +=
//        param_.q1_acc_lat_min * param_.q2_acc_lat_min * param_.q2_acc_lat_min *
//        state[2] * state[2] *
//        std::exp(param_.q2_acc_lat_min * (param_.acc_lat_min - acc_lat));
//  }

  // 5.0 constraint omega
//  if (condition.has_constraint_omega) {
//    l_uu(1, 1) +=
//        param_.q1_omega_max * param_.q2_omega_max * param_.q2_omega_max *
//        std::exp(param_.q2_omega_max * (action[1] - param_.omega_max));
//    l_uu(1, 1) +=
//        param_.q1_omega_min * param_.q2_omega_min * param_.q2_omega_min *
//        std::exp(param_.q2_omega_min * (param_.omega_min - action[1]));
//  }
}

// Get ILQRSpace l_f_xx
// Args:
//    state: Current state [state_size].
//    action: Current control [action_size].
//    l_f_xx: Output arg, last point dl/(dx * dx) [state_size, state_size].
void VehicleModelBicycle::get_l_f_xx(const Eigen::VectorXd &state,
                                     const StepCondition &condition,
                                     Eigen::MatrixXd &l_f_xx) {
  // 1.0 ref line
  l_f_xx.resize(param_.state_size,param_.state_size);

  // 1.0 ref line
  l_f_xx<< param_.w_ref_line * 2.0, 0.0, 0.0 ,0.0,
      0.0, param_.w_ref_line * 2.0, 0.0 ,0.0,
      0.0, 0.0, 0.0 ,0.0,
      0.0, 0.0, 0.0 ,0.0;

  // 2.0 ref v
  if (condition.has_ref_v) {
    l_f_xx(2, 2) += 2.0 * param_.w_ref_v;
  }
}

// Get ILQRSpace f_x
// Args:
//    state: Current state [state_size].
//    action: Current control [action_size].
//    f_x: Output arg, df/dx [state_size, state_size].
void VehicleModelBicycle::get_f_x_kappa(const Eigen::VectorXd &state,
                                        const Eigen::VectorXd &action,
                                        Eigen::MatrixXd &f_x) {
  // x, y, v, theta
  double l = state[2] * param_.delta_t + 0.5 * action[0] * param_.delta_t* param_.delta_t;
  f_x.resize(param_.state_size,param_.state_size);
  f_x << 1.0, 0.0, param_.delta_t * std::cos(state[3]), - std::sin(state[3]) * l,
         0.0, 1.0, param_.delta_t * std::sin(state[3]), std::cos(state[3]) * l,
      0.0, 0.0, 1.0, 0.0, 0.0, 0.0, action[1] * param_.delta_t, 1.0;
}

void ILQRSpace::init() {
  param_.delta_t = 0.2;
  param_.horizon = 30;
  param_.max_iter_num = 1;
  param_.tol = 1e-2;
  param_.kappa_thr = 1e-5;
  param_.use_hessians = false;
  param_.mu = 1e-6;
  param_.vehicle_param.length = 4.9;
  param_.vehicle_param.width = 1.9;
  param_.vehicle_param.r_4 = std::sqrt(
      (param_.vehicle_param.length * 0.5 * 0.25) *
          (param_.vehicle_param.length * 0.5 * 0.25) +
      (param_.vehicle_param.width * 0.5) * (param_.vehicle_param.width * 0.5));
  std::cout << " r_4 is : " << param_.vehicle_param.r_4 << std::endl;
  param_.exp_model_param.q1 = 1.0;
  param_.exp_model_param.q2 = 5.0;
  param_.exp_model_param.q3 = 4.0;
  param_.enable_log_model = true;
  param_.log_model_param.t = 2;
  vehicle_model_.update_parameter(param_);

  x_space_.clear();
  x_space_.resize(param_.horizon + 1);
  for (auto &x : x_space_) {
    x.resize(param_.state_size);
  }

  x_space_init_.clear();
  x_space_init_.resize(param_.horizon + 1);
  for (auto &x : x_space_init_) {
    x.resize(param_.state_size);
  }

  u_space_.clear();
  u_space_.resize(param_.horizon);
  for (auto &u : u_space_) {
    u.resize(param_.action_size);
  }

  u_space_init_.clear();
  u_space_init_.resize(param_.horizon);
  for (auto &u : u_space_init_) {
    u.resize(param_.action_size);
  }

  f_x_space_.clear();
  f_x_space_.resize(param_.horizon);
  for (auto &f_x : f_x_space_) {
    f_x.resize(param_.state_size, param_.state_size);
  }

  f_u_space_.clear();
  f_u_space_.resize(param_.horizon);
  for (auto &f_u : f_u_space_) {
    f_u.resize(param_.state_size, param_.action_size);
  }

  f_xx_space_.clear();
  f_xx_space_.resize(param_.horizon);
  for (auto &f_xx : f_xx_space_) {
    f_xx.resize(param_.state_size, param_.state_size);
  }

  f_ux_space_.clear();
  f_ux_space_.resize(param_.horizon);
  for (auto &f_ux : f_ux_space_) {
    f_ux.resize(param_.action_size, param_.state_size);
  }

  f_uu_space_.clear();
  f_uu_space_.resize(param_.horizon);
  for (auto &f_uu : f_uu_space_) {
    f_uu.resize(param_.action_size, param_.action_size);
  }

  l_condition_.clear();
  l_condition_.resize(param_.horizon + 1);

  l_new_condition_.clear();
  l_new_condition_.resize(param_.horizon + 1);

  l_space_.clear();
  l_space_.resize(param_.horizon + 1, 0.0);

  l_x_space_.clear();
  l_x_space_.resize(param_.horizon + 1);
  for (auto &l_x : l_x_space_) {
    l_x.resize(param_.state_size);
  }

  l_u_space_.clear();
  l_u_space_.resize(param_.horizon);
  for (auto &l_u : l_u_space_) {
    l_u.resize(param_.action_size);
  }

  l_xx_space_.clear();
  l_xx_space_.resize(param_.horizon + 1);
  for (auto &l_xx : l_xx_space_) {
    l_xx.resize(param_.state_size, param_.state_size);
  }

  l_ux_space_.clear();
  l_ux_space_.resize(param_.horizon);
  for (auto &l_ux : l_ux_space_) {
    l_ux.resize(param_.action_size, param_.state_size);
  }

  l_uu_space_.clear();
  l_uu_space_.resize(param_.horizon);
  for (auto &l_uu : l_uu_space_) {
    l_uu.resize(param_.action_size, param_.action_size);
  }
  x_new_space_.resize(param_.horizon + 1);
  u_new_space_.resize(param_.horizon);
}

void ILQRSpace::set_obstacle_cost() {
  auto obs_mgr = env_ptr_->get_obstacle_mgr();
  obstacle_cost_.clear();
  for (auto obj : obs_mgr.obstacle_map) {
    ILQRObstacleConstrain new_obj_cost;
    new_obj_cost.set_ilqr_param(param_);
    new_obj_cost.init(obj.second);
    obstacle_cost_.emplace_back(new_obj_cost);
  }
}
//  Update ILQRSpace firstly
//  xs, F_x, F_u, L, L_x, L_u, L_xx, L_ux, L_uu, F_xx, F_ux, F_uu
//  Args:
//    init_state: Initial state [state_size].
//    default_action: Control path [N, action_size].
FuncStatus
ILQRSpace::forward_rollout(std::vector<Eigen::VectorXd> &default_actions) {
  // 0. check data valid
  if (init_state_.size() != param_.state_size ||
      default_actions.size() != param_.horizon) {
    std::cout << " forward_rollout input size wrong! " << std::endl;
    return FuncStatus::FuncFailed;
  }
  for (const auto &action : default_actions) {
    if (action.size() != param_.action_size) {
      std::cout << " forward_rollout input size wrong! " << std::endl;
      return FuncStatus::FuncFailed;
    }
  }

  // 1. init space
  // x [N+1, state_size]
  x_space_.resize(param_.horizon + 1);
  for (auto &x : x_space_) {
    x.resize(param_.state_size);
  }

  u_space_.resize(param_.horizon);
  u_space_ = default_actions;

  // f_x [N, state_size, state_size]
  f_x_space_.resize(param_.horizon);
  for (auto &f_x : f_x_space_) {
    f_x.resize(param_.state_size, param_.state_size);
  }

  // 2. forward
  x_space_[0] = init_state_;
  l_condition_[0].v_ref = init_state_[2];

  Eigen::VectorXd next_x;
  Eigen::MatrixXd cur_f_x;
  Eigen::MatrixXd cur_f_u;
  Eigen::MatrixXd cur_f_xx;
  StepCondition cur_l_condition;
  Eigen::VectorXd cur_l_x;
  Eigen::VectorXd cur_l_u;
  Eigen::MatrixXd cur_l_xx;
  Eigen::MatrixXd cur_l_ux;
  Eigen::MatrixXd cur_l_uu;
  double cur_l;

  next_x.resize(param_.state_size);
  for(int i = 0; i< param_.horizon; i++){
    cur_l_condition = l_condition_[i];
    vehicle_model_.step(x_space_[i],u_space_[i],next_x);
    vehicle_model_.get_f_x(x_space_[i],u_space_[i],cur_f_x);
    vehicle_model_.get_f_u(x_space_[i],u_space_[i],cur_f_u);
    vehicle_model_.get_l_condition(i, x_space_[i],env_ptr_,cur_l_condition);
    if (i>0){
      vehicle_model_.get_l_lon_condition(i, x_space_[i-1],l_condition_[i-1],env_ptr_,cur_l_condition);
    }
    vehicle_model_.get_l(x_space_[i],u_space_[i],cur_l_condition,cur_l);
    vehicle_model_.get_l_x(x_space_[i],u_space_[i],cur_l_condition,cur_l_x);
    vehicle_model_.get_l_u(x_space_[i],u_space_[i],cur_l_condition,cur_l_u);
    vehicle_model_.get_l_xx(x_space_[i],u_space_[i],cur_l_condition,cur_l_xx);
    vehicle_model_.get_l_ux(x_space_[i],u_space_[i],cur_l_condition,cur_l_ux);
    vehicle_model_.get_l_uu(x_space_[i],u_space_[i],cur_l_condition,cur_l_uu);
    // constraint condition
    for (auto &cost : obstacle_cost_) {
      cost.get_l_condition(i, x_space_[i], env_ptr_, cur_l_condition);
      cost.get_all_l_element(x_space_[i], cur_l_condition, cur_l, cur_l_x,
                             cur_l_xx);
    }

    x_space_[i + 1] = next_x;
    f_x_space_[i] = cur_f_x;
    f_u_space_[i] = cur_f_u;
    l_condition_[i] = cur_l_condition;
    l_space_[i] = cur_l;
    l_x_space_[i] = cur_l_x;
    l_u_space_[i] = cur_l_u;
    l_xx_space_[i] = cur_l_xx;
    l_ux_space_[i] = cur_l_ux;
    l_uu_space_[i] = cur_l_uu;
    if (param_.use_hessians) {
      vehicle_model_.get_f_xx(x_space_[i], u_space_[i], cur_f_xx);
      f_xx_space_[i] = cur_f_xx;
    }
  }

  // last point
  Eigen::VectorXd cur_lf_x;
  Eigen::MatrixXd cur_lf_xx;
  double cur_lf;
  vehicle_model_.get_l_condition(param_.horizon, x_space_.back(),env_ptr_,cur_l_condition);
  vehicle_model_.get_l_f(x_space_.back(),cur_l_condition,cur_lf);
  vehicle_model_.get_l_f_x(x_space_.back(),cur_l_condition,cur_lf_x);
  vehicle_model_.get_l_f_xx(x_space_.back(),cur_l_condition,cur_lf_xx);
  for(auto& cost :obstacle_cost_){
    cost.get_l_condition(param_.horizon,x_space_.back(),env_ptr_,cur_l_condition);
    cost.get_all_l_element(x_space_.back(),cur_l_condition,cur_lf,cur_lf_x,cur_lf_xx);
  }
  l_space_[param_.horizon] =  cur_lf;
  l_x_space_[param_.horizon]= cur_lf_x;
  l_xx_space_[param_.horizon]= cur_lf_xx;
  l_condition_[param_.horizon] = cur_l_condition;
  return FuncStatus::FuncSucceed;
}

// Computes the feedforward and feedback gains k and K.
//  Args:
//      F_x: Jacobian of state path w.r.t. x [N, state_size, state_size].
//      F_u: Jacobian of state path w.r.t. u [N, state_size, action_size].
//      L_x: Jacobian of cost path w.r.t. x [N+1, state_size].
//      L_u: Jacobian of cost path w.r.t. u [N, action_size].
//      L_xx: Hessian of cost path w.r.t. x, x [N+1, state_size, state_size].
//      L_ux: Hessian of cost path w.r.t. u, x [N, action_size, state_size].
//      L_uu: Hessian of cost path w.r.t. u, u [N, action_size, action_size].
//      F_xx: Hessian of state path w.r.t. x, x if Hessians are used [N,
//      state_size, state_size, state_size].
//      F_ux: Hessian of state path w.r.t. u, x if Hessians are used [N,
//      state_size, action_size, state_size].
//      F_uu: Hessian of state path w.r.t. u, u if Hessians are used [N,
//      state_size, action_size, action_size].
//      Returns:
//  Tuple of
//      k: feedforward gains [N, action_size].
//      K: feedback gains [N, action_size, state_size].
FuncStatus ILQRSpace::backward_pass() {
  k_space_.resize(param_.horizon);
  for (auto &k : k_space_) {
    k.resize(param_.action_size);
  }

  k_matrix_space_.resize(param_.horizon);
  for (auto &k_m : k_matrix_space_) {
    k_m.resize(param_.action_size, param_.state_size);
  }
  // why not 另设一个变量, 而使用第31个l_x? 不影响
  auto &v_x = l_x_space_.back();
  auto &v_xx = l_xx_space_.back();

  //  Q_x: [state_size].
  //  Q_u: [action_size].
  //  Q_xx: [state_size, state_size].
  //  Q_ux: [action_size, state_size].
  //  Q_uu: [action_size, action_size].
  Eigen::VectorXd q_x;
  Eigen::VectorXd q_u;
  Eigen::MatrixXd q_xx;
  Eigen::MatrixXd q_ux;
  Eigen::MatrixXd q_uu;
  q_x.resize(param_.state_size);
  q_u.resize(param_.action_size);
  q_xx.resize(param_.state_size, param_.state_size);
  q_uu.resize(param_.action_size, param_.action_size);
  q_ux.resize(param_.action_size, param_.state_size);
  Eigen::MatrixXd reg = param_.mu * Eigen::MatrixXd::Identity(
                                        param_.state_size, param_.state_size);

  Eigen::VectorXd cur_k;
  cur_k.resize(param_.state_size);
  Eigen::MatrixXd cur_k_m;
  cur_k_m.resize(param_.action_size, param_.state_size);
  for (int i = param_.horizon - 1; i >= 0; i--) {
    auto &l_x = l_x_space_[i];
    auto &l_u = l_u_space_[i];
    auto &l_xx = l_xx_space_[i];
    auto &l_ux = l_ux_space_[i];
    auto &l_uu = l_uu_space_[i];
    auto &f_x = f_x_space_[i];
    auto &f_u = f_u_space_[i];
    if (param_.use_hessians) {
      // todo: @jojo
    } else {
      q_x = l_x + f_x.transpose() * v_x;
      q_u = l_u + f_u.transpose() * v_x;
      q_xx = l_xx + f_x.transpose() * v_xx * f_x;
      q_ux = l_ux + f_u.transpose() * (v_xx + reg) * f_x;
      q_uu = l_uu + f_u.transpose() * (v_xx + reg) * f_u;
    }
    std::cout << " i : "<< i << " q_x : "<< q_x[0] << " "<< q_x[1]<< " " << q_x[2]<< " " << q_x[3] << std::endl;
    Eigen::FullPivLU<Eigen::MatrixXd> lu(q_uu);

    if (!lu.isInvertible()) {
      std::cout << "Matrix is not invertible.\n";
      return FuncStatus::FuncFailed;
    }
    cur_k = -lu.inverse() * q_u;
    cur_k_m = -lu.inverse() * q_ux;

    v_x = q_x + cur_k_m.transpose() * q_uu * cur_k;
    v_x += cur_k_m.transpose() * q_u + q_ux.transpose() * cur_k;

    v_xx = q_xx + cur_k_m.transpose() * (q_uu)*cur_k_m;
    v_xx += cur_k_m.transpose() * (q_ux) + q_ux.transpose() * cur_k_m;
    v_xx = 0.5 * (v_xx + v_xx.transpose());

    k_space_[i] = cur_k;
    k_matrix_space_[i] = cur_k_m;
  }
  return FuncStatus::FuncSucceed;
}

//  Applies the controls for a given trajectory.
// xs: Nominal state path [N+1, state_size].
// us: Nominal control path [N, action_size].
// k: Feedforward gains [N, action_size].
// K: Feedback gains [N, action_size, state_size].
//  Returns:
//    xs: state path [N+1, state_size].
//    us: control path [N, action_size].
FuncStatus ILQRSpace::control(const double &alpha) {
  x_new_space_.resize(param_.horizon + 1);
  u_new_space_.resize(param_.horizon);
  x_new_space_[0] = x_space_[0];

  for (int i = 0; i < param_.horizon; i++) {
    u_new_space_[i] = u_space_[i] + alpha * k_space_[i] +
                      k_matrix_space_[i] * (x_new_space_[i] - x_space_[i]);
    vehicle_model_.step(x_new_space_[i], u_new_space_[i], x_new_space_[i + 1]);
    vehicle_model_.get_l_condition(i, x_new_space_[i],env_ptr_,l_new_condition_[i]);
    if (i>0){
      vehicle_model_.get_l_lon_condition(i, x_new_space_[i-1],l_new_condition_[i-1],env_ptr_,l_new_condition_[i]);
    }
  }
  return FuncStatus::FuncSucceed;
}

//  Applies the controls for a given trajectory.
// xs: Nominal state path [N+1, state_size].
// us: Nominal control path [N, action_size].
// k: Feedforward gains [N, action_size].
// K: Feedback gains [N, action_size, state_size].
//  Returns:
//    xs: state path [N+1, state_size].
//    us: control path [N, action_size].
FuncStatus ILQRSpace::forward_pass() {
  Eigen::VectorXd next_x;
  Eigen::MatrixXd cur_f_x;
  Eigen::MatrixXd cur_f_u;
  Eigen::MatrixXd cur_f_xx;
  StepCondition cur_l_condition;
  Eigen::VectorXd cur_l_x;
  Eigen::VectorXd cur_l_u;
  Eigen::MatrixXd cur_l_xx;
  Eigen::MatrixXd cur_l_ux;
  Eigen::MatrixXd cur_l_uu;
  double cur_l;
  for(int i= 0; i< param_.horizon; i++){
    cur_l_condition = l_condition_[i];
    vehicle_model_.get_f_x(x_new_space_[i],u_new_space_[i],cur_f_x);
    vehicle_model_.get_f_u(x_new_space_[i],u_new_space_[i],cur_f_u);
    vehicle_model_.get_l_condition(i, x_new_space_[i],env_ptr_,cur_l_condition);
    if (i>0){
      vehicle_model_.get_l_lon_condition(i, x_new_space_[i-1],l_condition_[i-1],env_ptr_,cur_l_condition);
    }
    vehicle_model_.get_l(x_new_space_[i],u_new_space_[i],cur_l_condition,cur_l);
    vehicle_model_.get_l_x(x_new_space_[i],u_new_space_[i],cur_l_condition,cur_l_x);
    vehicle_model_.get_l_u(x_new_space_[i],u_new_space_[i],cur_l_condition,cur_l_u);
    vehicle_model_.get_l_xx(x_new_space_[i],u_new_space_[i],cur_l_condition,cur_l_xx);
    vehicle_model_.get_l_ux(x_new_space_[i],u_new_space_[i],cur_l_condition,cur_l_ux);
    vehicle_model_.get_l_uu(x_new_space_[i],u_new_space_[i],cur_l_condition,cur_l_uu);
    // constraint condition
    for (auto &cost : obstacle_cost_) {
      cost.get_l_condition(i, x_new_space_[i], env_ptr_, cur_l_condition);
      cost.get_all_l_element(x_new_space_[i], cur_l_condition, cur_l, cur_l_x,
                             cur_l_xx);
    }
    f_x_space_[i] = cur_f_x;
    f_u_space_[i] = cur_f_u;
    l_condition_[i] = cur_l_condition;
    l_space_[i] = cur_l;
    l_x_space_[i] = cur_l_x;
    l_u_space_[i] = cur_l_u;
    l_xx_space_[i] = cur_l_xx;
    l_ux_space_[i] = cur_l_ux;
    l_uu_space_[i] = cur_l_uu;
    if (param_.use_hessians) {
      vehicle_model_.get_f_xx(x_new_space_[i], u_new_space_[i], cur_f_xx);
      f_xx_space_[i] = cur_f_xx;
    }
  }

  Eigen::VectorXd cur_lf_x;
  Eigen::MatrixXd cur_lf_xx;
  double cur_lf;
  vehicle_model_.get_l_condition(param_.horizon, x_new_space_.back(), env_ptr_,
                                 cur_l_condition);
  vehicle_model_.get_l_f(x_new_space_.back(), cur_l_condition, cur_lf);
  vehicle_model_.get_l_f_x(x_new_space_.back(), cur_l_condition, cur_lf_x);
  vehicle_model_.get_l_f_xx(x_new_space_.back(), cur_l_condition, cur_lf_xx);
  for (auto &cost : obstacle_cost_) {
    cost.get_l_condition(param_.horizon, x_new_space_.back(), env_ptr_,
                         cur_l_condition);
    cost.get_all_l_element(x_new_space_.back(), cur_l_condition, cur_lf,
                           cur_lf_x, cur_lf_xx);
  }
  l_space_[param_.horizon] = cur_lf;
  l_x_space_[param_.horizon] = cur_lf_x;
  l_xx_space_[param_.horizon] = cur_lf_xx;
  l_condition_[param_.horizon] = cur_l_condition;

  for (int j = 0; j < param_.horizon + 1; j++) {
    x_space_[j] = x_new_space_[j];
  }
  for (int j = 0; j < param_.horizon; j++) {
    u_space_[j] = u_new_space_[j];
  }
  return FuncStatus::FuncSucceed;
}

double ILQRSpace::get_l_sum(const std::vector<Eigen::VectorXd> &x_space,
                            const std::vector<Eigen::VectorXd> &u_space,
                            const std::vector<StepCondition>& l_condition) {
  double sum = 0.0;
  double cur_l;
  for (int i = 0; i < param_.horizon; i++) {
    vehicle_model_.get_l(x_space[i], u_space[i], l_condition[i], cur_l);
    for (auto &cost : obstacle_cost_) {
      cost.get_l(l_condition[i], cur_l);
    }
    sum += cur_l;
  }
  vehicle_model_.get_l_f(x_space.back(), l_condition[param_.horizon], cur_l);
  sum += cur_l;
  return sum;
}

void ILQRSpace::condition_init(){
   auto obs = env_ptr_->get_all_obstacle();
  for (int i =0 ; i < param_.horizon;i++){
    StepCondition cur_condition;
    cur_condition.obstacle_belief_state_map.clear();
    for(auto ob :obs){
      double x = ob.trajectory_points[i].position.x;
      double y = ob.trajectory_points[i].position.y;
      auto nearest_point = env_ptr_->get_nearest_point(MathUtils::Point2D{x,y});
      double dis = std::hypot(x-nearest_point.x,y-nearest_point.y);

      LonObstacleInfo info;
      if (dis < 3.0 ){
        info.belief = 1.0;
      }
      info.position.x = x;
      info.position.y = y;
      info.length = ob.get_length();
      info.s = env_ptr_->get_lane_s(MathUtils::Point2D{x,y});
      info.v.x = ob.trajectory_points[i].v * std::cos(ob.trajectory_points[i].theta);
      info.v.y = ob.trajectory_points[i].v * std::sin(ob.trajectory_points[i].theta);
      double tmp_a = 0.0;
      if (i<param_.horizon -1){
        tmp_a = (ob.trajectory_points[i+1].v - ob.trajectory_points[i].v) / param_.delta_t;
      }
      info.a.x = tmp_a * std::cos(ob.trajectory_points[i].theta);
      info.a.y = tmp_a * std::sin(ob.trajectory_points[i].theta);
      cur_condition.obstacle_belief_state_map.insert({ob.get_id(),info});

    }
    l_condition_[i] = cur_condition;
  }

}

//  Process ilqr solver
FuncStatus ILQRSpace::solve(const Eigen::VectorXd &init_state,
                            std::vector<Eigen::VectorXd> &default_actions) {
  auto start = std::chrono::high_resolution_clock::now();
  // 1.0 set input state
  if (init_state.size() != param_.state_size) {
    std::cout << " forward_rollout input size wrong! " << std::endl;
    return FuncStatus::FuncFailed;
  } else {
    init_state_ = init_state;
  }

  auto end1 = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> duration = end1 - start;
  std::cout << " **************** ilqr 1.0 time:：" << duration.count() * 1000
            << " ms " << std::endl;

  // 2.0 forward_rollout
  if (forward_rollout(default_actions) == FuncStatus::FuncFailed) {
    std::cout << " forward_rollout failed! " << std::endl;
    return FuncStatus::FuncFailed;
  }
  for (int i = 0; i < param_.horizon + 1; i++) {
    x_space_init_[i] = x_space_[i];
  }
  for (int i = 0; i < param_.horizon; i++) {
    u_space_init_[i] = u_space_[i];
  }
  double j_opt = get_l_sum(x_space_, u_space_,l_condition_);
  double j_new = j_opt;

  iter_stat_list_.clear();
  IterationStatistic iter_stat(x_space_, u_space_, j_opt);
  iter_stat_list_.emplace_back(iter_stat);

  auto end2 = std::chrono::high_resolution_clock::now();
  duration = end2 - end1;
  std::cout << " **************** ilqr 2.0 time:：" << duration.count() * 1000
            << " ms " << std::endl;
  // 3.0 loop dp until finish
  for (int i = 0; i < param_.max_iter_num; i++) {
    // 3.1 backward to get k & k_matrix
    if (backward_pass() != FuncStatus::FuncSucceed) {
      // increase regularization term
      param_.delta = std::max(1.0, param_.delta) * param_.delta_0;
      param_.mu *= param_.delta;
      if (param_.mu >= param_.mu_max) {
        param_.mu = param_.mu_max;
        std::cout << "[ERROR]: exceeded max regularization term " << param_.mu_max << std::endl;
      }
      continue;
    }
    double j_alpha_best = j_opt;
    std::cout << " j_opt: " << j_opt << std::endl;
//    int j =0;
//    for(auto& f_x : f_u_space_){
//      std::cout << j << "f_u " <<f_x<< std::endl;
//      j++;
//    }
    double best_alpha = 1.0;
    // 3.2 get best new state
    for (auto alpha : alpha_) {
      control(alpha);
      j_new = get_l_sum(x_new_space_, u_new_space_,l_new_condition_);
      if (j_new < j_alpha_best) {
        best_alpha = alpha;
        j_alpha_best = j_new;
      }
      std::cout << " alpha: " << alpha << " j new: "<< j_new << std::endl;
    }
    // 3.3 update best new state & space
    std::cout << "j_opt: " << j_opt << " j_best_alpha: " << j_alpha_best
              << std::endl;
    if (j_alpha_best > j_opt) {
      not_converged_counter_++;
      std::cout << " found not converged" << std::endl;
    }
    if (j_opt > j_alpha_best + param_.tol &&
        not_converged_counter_ < param_.max_not_converged_time) {
      control(best_alpha);
      j_new = get_l_sum(x_new_space_, u_new_space_,l_new_condition_);
      forward_pass();

      // decrease regularization term
      param_.delta = std::min(1.0, param_.delta) / param_.delta_0;
      param_.mu *= param_.delta;
      if (param_.mu <= param_.mu_min) {
        param_.mu = param_.mu_min;
      }
      j_opt = j_new;
    } else {
      std::cout << " stop search " << std::endl;
      break;
    }

    // 3.4 record log
    IterationStatistic iter_stat(x_space_, u_space_, j_opt);
    iter_stat_list_.emplace_back(iter_stat);
  }
  auto end3 = std::chrono::high_resolution_clock::now();
  duration = end3 - end2;
  std::cout << " **************** ilqr 3.0 time:：" << duration.count() * 1000
            << " ms " << std::endl;
  // 4.0 check trajectory

  return FuncStatus::FuncSucceed;
}
}