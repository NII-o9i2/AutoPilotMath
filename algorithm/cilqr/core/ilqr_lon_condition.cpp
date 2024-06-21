//
// Created by SENSETIME\fengxiaotong on 24-1-11.
//
#include "iostream"
#include "math.h"
#include "ilqr_lon_condition.h"
#include "ilqr_tree_solver.h"

namespace ILQR {
void IDMCarInfo::print_car_info() const {
  std::string log_msg = "CarInfo, length:" + std::to_string(car_length) +
                        ",(s,v,a) is: (" + std::to_string(car_s) + "," +
                        std::to_string(car_v) + "," + std::to_string(car_a) +
                        ")";
  SolverInfoLog::Instance()->log(log_msg);
}

IDMLongiProfile LongitudinalOperator::calc_longi_profile(
    const ILQR::IDMParam &idm_params,
    const IDMCarInfo &ego_state,
    const IDMCarInfo &obs_state,
    const double &delta_t,
    const int &exist_leader) {
  double delta_s = obs_state.car_s - 0.5 * obs_state.car_length -
                   0.5 * ego_state.car_length - ego_state.car_s;

  auto res = calc_idm_acc(idm_params, delta_s, ego_state.car_v, obs_state.car_v,
                          exist_leader);
  double idm_acc = res.first;
  IDMLongiProfile longi_profile;
  const double factor = 0.5 * std::max(1e-3, delta_t);
  longi_profile.a = idm_acc;
  longi_profile.v = std::max(0.0, ego_state.car_v + idm_acc * delta_t);
  longi_profile.s =
      ego_state.car_s + (ego_state.car_v + longi_profile.v) * factor;
  return longi_profile;
}

std::pair<double, double> LongitudinalOperator::calc_idm_acc(
    const IDMParam &idm_params,
    double delta_s,
    double curr_v,
    double leader_v,
    int exist_leader) {
  double delta_v = curr_v - leader_v;
  double change_rate = ILQR::LongitudinalOperator::linear_interpolate(
      delta_v, idm_params.delta_v, idm_params.s_change_rate);
  double min_spacing = ILQR::LongitudinalOperator::linear_interpolate(
      curr_v, idm_params.idm_delta_v, idm_params.idm_min_spacing_list);
  double idm_acc = 0.0;
  double const_max_dec = -10;
  int delta_v_pow = 0;
  if (curr_v < idm_params.desired_spd) {
    delta_v_pow = idm_params.delta;
  } else {
    delta_v_pow = idm_params.acc_delta;
  }
  double s_desire = 0.0;
  // delta_v = ego - leader
  // delta_s = leader - ego
  if (exist_leader == 1) {
    double sqrt_item =
        1 / (std::sqrt(idm_params.desired_deceleration * idm_params.max_acc));
    double headway_s_item = curr_v * idm_params.time_headway;
    double dealta_v_item = 0.5 * curr_v * delta_v * sqrt_item;
    double s_desire_raw =
        min_spacing + std::max(0.0, headway_s_item + dealta_v_item);

    s_desire = s_desire_raw;
    if (delta_v < 0 && s_desire > delta_s * 1.15) {
      s_desire = std::min(s_desire, delta_s * (1 + change_rate));
    }

    // consider acc change smoothly
    double acc_to_desire_vel = 0.0;
    double acc_to_leader_car = 0.0;
    double delta_vel = idm_params.desired_spd - curr_v;
    double default_error_vel =
        linear_interpolate(curr_v, idm_params.v_std, idm_params.vel_error_std);
    if (delta_vel > 0) {
      delta_vel = std::min(delta_vel, default_error_vel);
    } else {
      delta_vel = std::max(delta_vel, -1 * default_error_vel);
    }
    acc_to_desire_vel =
        idm_params.max_acc *
        (1.0 - std::pow(curr_v / (curr_v + delta_vel), delta_v_pow));
    acc_to_leader_car =
        delta_s > 0
            ? idm_params.max_acc * (1 - std::pow(s_desire / delta_s,
                                                 idm_params.follow_car_delta))
            : const_max_dec;
    idm_acc = std::min(acc_to_desire_vel, acc_to_leader_car);
    // std::cout << " s spacing is : " << min_spacing << ", headway s is: " <<
    // headway_s_item
    //           << ", delat_v_item is: " << dealta_v_item << std::endl;

    // std::cout << " s_desire_raw is : " << s_desire_raw << ",change rate
    // is:"
    // << change_rate
    //           << " s_desire is : " << s_desire
    //           <<  ",acc to desired spd is:" << acc_to_desire_vel
    //           << ", acc to leader car is: " << acc_to_leader_car <<
    //           std::endl;
  } else {
    double delta_vel = idm_params.desired_spd - curr_v;
    double default_error_vel =
        linear_interpolate(curr_v, idm_params.v_std, idm_params.vel_error_std);
    if (delta_vel > 0) {
      delta_vel = std::min(delta_vel, default_error_vel);
    } else {
      delta_vel = std::max(delta_vel, -1 * default_error_vel);
    }
    idm_acc = idm_params.max_acc *
              (1.0 - std::pow(curr_v / (curr_v + delta_vel), delta_v_pow));
  }
  idm_acc = std::max(-1.0 * idm_params.min_deceleration,
                     std::min(idm_acc, idm_params.max_acc));
  return std::make_pair(idm_acc, s_desire);
}

std::pair<double, double> LongitudinalOperator::calc_idm_acc(
    const IDMParam &idm_params,
    double delta_s,
    double curr_v,
    double leader_v,
    double speed_limit,
    int exist_leader) {
  // SolverInfoLog::Instance()->error(
  //         "speed limit is:" + std::to_string(speed_limit));
  double delta_v = curr_v - leader_v;
  double change_rate = ILQR::LongitudinalOperator::linear_interpolate(
      delta_v, idm_params.delta_v, idm_params.s_change_rate);
  double min_spacing = ILQR::LongitudinalOperator::linear_interpolate(
      curr_v, idm_params.idm_delta_v, idm_params.idm_min_spacing_list);
  double idm_acc = 0.0;
  double const_max_dec = -10;
  int delta_v_pow = 0;
  if (curr_v < speed_limit) {
    delta_v_pow = idm_params.delta;
  } else {
    delta_v_pow = idm_params.acc_delta;
  }
  double s_desire = 0.0;
  // delta_v = ego - leader
  // delta_s = leader - ego
  if (exist_leader == 1) {
    double sqrt_item =
        1 / (std::sqrt(idm_params.desired_deceleration * idm_params.max_acc));
    double headway_s_item = curr_v * idm_params.time_headway;
    double dealta_v_item = 0.5 * curr_v * delta_v * sqrt_item;
    double s_desire_raw =
        min_spacing + std::max(0.0, headway_s_item + dealta_v_item);

    s_desire = s_desire_raw;
    if (delta_v < 0 && s_desire > delta_s * 1.15) {
      s_desire = std::min(s_desire, delta_s * (1 + change_rate));
    }

    // consider acc change smoothly
    double acc_to_desire_vel = 0.0;
    double acc_to_leader_car = 0.0;
    double delta_vel = speed_limit - curr_v;
    double default_error_vel =
        linear_interpolate(curr_v, idm_params.v_std, idm_params.vel_error_std);
    if (delta_vel > 0) {
      delta_vel = std::min(delta_vel, default_error_vel);
    } else {
      delta_vel = std::max(delta_vel, -1 * default_error_vel);
    }
    acc_to_desire_vel =
        idm_params.max_acc *
        (1.0 - std::pow(curr_v / (curr_v + delta_vel), delta_v_pow));
    acc_to_leader_car =
        delta_s > 0
            ? idm_params.max_acc * (1 - std::pow(s_desire / delta_s,
                                                 idm_params.follow_car_delta))
            : const_max_dec;
    idm_acc = std::min(acc_to_desire_vel, acc_to_leader_car);
    // std::cout << " s spacing is : " << min_spacing << ", headway s is: " <<
    // headway_s_item
    //           << ", delat_v_item is: " << dealta_v_item << std::endl;

    // std::cout << " s_desire_raw is : " << s_desire_raw << ",change rate
    // is:"
    // << change_rate
    //           << " s_desire is : " << s_desire
    //           <<  ",acc to desired spd is:" << acc_to_desire_vel
    //           << ", acc to leader car is: " << acc_to_leader_car <<
    //           std::endl;
  } else {
    double delta_vel = speed_limit - curr_v;
    double default_error_vel =
        linear_interpolate(curr_v, idm_params.v_std, idm_params.vel_error_std);
    if (delta_vel > 0) {
      delta_vel = std::min(delta_vel, default_error_vel);
    } else {
      delta_vel = std::max(delta_vel, -1 * default_error_vel);
    }
    idm_acc = idm_params.max_acc *
              (1.0 - std::pow(curr_v / (curr_v + delta_vel), delta_v_pow));
  }
  idm_acc = std::max(-1.0 * idm_params.min_deceleration,
                     std::min(idm_acc, idm_params.max_acc));
  return std::make_pair(idm_acc, s_desire);
}

double LongitudinalOperator::calc_cah_acc(const double actual_s,
                                          const double leader_v,
                                          const double curr_v,
                                          const double leader_a) {
  double res_acc = 0.0;
  bool flag = leader_v * (curr_v - leader_v) <= (-2 * actual_s * leader_a);
  double tag;
  if (flag) {
    res_acc = (curr_v * curr_v * leader_a) /
              (leader_v * leader_v - 2 * actual_s * leader_a);
  } else {
    tag = curr_v - leader_v >= 0 ? 1 : 0;
    res_acc =
        leader_a -
        (curr_v - leader_v) * (curr_v - leader_v) * (tag) / (2 * actual_s);
    // AD_LINFO(idm) << "enter EM status!!!";
  }
  std::cout << " leader s,v,a is : " << actual_s << ",  " << leader_v
            << ", leader_a" << leader_a << ", ego v: " << curr_v << std::endl;

  std::cout << " flag is : " << flag << ",tag is" << tag
            << " ,cah_acc is : " << res_acc << std::endl;

  return res_acc;
}

double LongitudinalOperator::calc_acc(IDMParam &idm_params,
                                      const double actual_s,
                                      const double leader_v,
                                      const double curr_v,
                                      const double leader_a) {
  double res_acc = 0.0;
  auto res = LongitudinalOperator::calc_idm_acc(idm_params, actual_s, curr_v,
                                                curr_v - leader_v, 1);
  double acc_idm = res.first;
  double acc_CAH =
      LongitudinalOperator::calc_cah_acc(actual_s, leader_v, curr_v, leader_a);
  double ratio = 0.99;
  double b = 3.5;
  // AD_LINFO(idm) << "acc_idm: " << acc_idm;
  // AD_LINFO(idm) << "acc_CAH: " << acc_CAH;
  if (acc_idm >= acc_CAH) {
    res_acc = acc_idm;
  } else {
    // AD_LINFO(idm) << "ratio: " << std::tan((acc_idm - acc_CAH) / b);
    res_acc = (1 - ratio) * acc_idm +
              ratio * (acc_CAH + b * std::tan((acc_idm - acc_CAH) / b));
  }
  return std::max(-1.0 * 3.5, std::min(res_acc, 2.5));
}

double LongitudinalOperator::linear_interpolate(
    const double &x,
    const std::vector<double> &a_std,
    const std::vector<double> &b_std) {
  if (a_std.size() != b_std.size() || a_std.empty()) {
    return 0.0;
  }

  std::vector<double>::const_iterator upper =
      std::upper_bound(a_std.begin(), a_std.end(), x);
  std::vector<double>::const_iterator lower = upper - 1;

  if (upper == a_std.end()) {
    return b_std.back();
  } else if (lower < a_std.begin()) {
    return b_std.front();
  }

  double weight = static_cast<double>((x - *lower) / (*upper - *lower));

  int low_index = lower - a_std.begin();
  int upper_index = upper - a_std.begin();

  return (b_std[low_index]) * (1 - weight) + (b_std[upper_index]) * weight;
}
}  // namespace ILQR