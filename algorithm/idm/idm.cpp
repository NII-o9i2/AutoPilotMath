#include "idm.h"
#include <cmath>
#include <iostream>
#include <vector>

namespace IDM {

double get_target_s(double desired_s,
                    double actual_s,
                    double delta_v,
                    double change_rate) {
  if (delta_v < 0 && desired_s > actual_s * 1.15) {
    return std::min(desired_s, actual_s * (1 + change_rate));
  } else {
    return desired_s;
  }
};

double linear_interpolate(const double &x,
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

double calc_idm_acc(IDMParam &idm_params,
                    double &delta_s,
                    double &curr_v,
                    double &delta_v,
                    int &exist_leader,
                    std::size_t &count) {
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
        1 / (std::sqrt(idm_params.desired_deacc * idm_params.max_acc));
    double headway_s_item = curr_v * idm_params.time_headway;
    double dealta_v_item = 0.5 * curr_v * delta_v * sqrt_item;
    double s_desire_raw =
        idm_params.min_spacing + std::max(0.0, headway_s_item + dealta_v_item);
    // double min_spacing_item = headway_s_item + dealta_v_item;
    // if (min_spacing_item > 0.0) {
    //   double min_spacing_add_headway = idm_params.min_spacing + headway_s_item;

    //   double delta_s_desire_add_headway = min_spacing_add_headway - delta_s;
    //   delta_s_desire_add_headway = std::min(delta_s_desire_add_headway, 0.5);
    //   s_desire = delta_s + delta_s_desire_add_headway + dealta_v_item;
    // } else {
    //   double s_desire_spacing = idm_params.min_spacing;
    //   s_desire = s_desire_spacing;
    // }
    s_desire = s_desire_raw;
    s_desire =
        IDM::get_target_s(s_desire, delta_s, delta_v, idm_params.change_rate);
    // consider acc change smothly
    double acc_to_desire_vel = 0.0;
    double acc_to_leader_car = 0.0;
    acc_to_desire_vel =
        idm_params.max_acc *
        (1 - std::pow(curr_v / idm_params.desired_spd, delta_v_pow));
    acc_to_leader_car =
        delta_s > 0
            ? idm_params.max_acc * (1 - std::pow(s_desire / delta_s,
                                                 idm_params.follow_car_delta))
            : const_max_dec;
    idm_acc = std::min(acc_to_desire_vel, acc_to_leader_car);
    std::cout << "--------------index is:" << count << "--------------"
              << std::endl;
    std::cout << "s_desire is : " << s_desire << ",cur v is :" << curr_v
              << ",delat v is: " << delta_v
              << ",s0 is :" << idm_params.min_spacing
              << ",delta s is:" << delta_s << std::endl;
    std::cout << "has leader car, raw acc to desire vel:" << acc_to_desire_vel
              << ",raw acc to leader car::" << acc_to_leader_car
              << ",raw idm acc is:" << idm_acc << std::endl;
  } else {
    double delta_vel = idm_params.desired_spd - curr_v;
    std::cout << delta_vel << ",raw vel_error " << std::endl;
    double default_error_vel = IDM::linear_interpolate(
        curr_v, idm_params.v_std, idm_params.vel_error_std);
    if (delta_vel > 0) {
      delta_vel = std::min(delta_vel, default_error_vel);
    } else {
      delta_vel = std::max(delta_vel, -1 * default_error_vel);
    }
    idm_acc = idm_params.max_acc *
              (1.0 - std::pow(curr_v / (curr_v + delta_vel), delta_v_pow));
    std::cout << "--------------index is:" << count << "--------------"
              << std::endl;
    std::cout << "no leader car,acc to desire vel:" << idm_acc
              << ",curr vel:" << curr_v << ",desired spd" << default_error_vel
              << ",clamp_vel_error is:" << delta_vel << std::endl;
  }
  return std::max(-1.0 * idm_params.min_dacc,
                  std::min(idm_acc, idm_params.max_acc));
}

std::vector<IDMOutput> get_idm_output(
    EgoInfo &ego_info,
    std::vector<SpeedPlannerProfile> &speed_planner_profile,
    IDMParam &idm_params,
    std::vector<double> v_refs) {
  std::vector<IDMOutput> idm_output;
  std::vector<double> raw_s_ref_follow;
  std::vector<double> raw_v_ref_follow;
  std::vector<double> raw_a_ref_follow;
  std::vector<double> after_s_ref_follow;
  std::vector<double> after_v_ref_follow;
  std::vector<double> after_a_ref_follow;
  raw_s_ref_follow.resize(31);
  raw_v_ref_follow.resize(31);
  raw_a_ref_follow.resize(31);
  after_s_ref_follow.resize(31);
  after_v_ref_follow.resize(31);
  after_a_ref_follow.resize(31);
  double delta_t = 0.2;
  double kMaxJerk = 4.0;
  bool maybe_collision = false;
  // 40 kph
  double kMinEnableJerkConstrantSpd = 11.11;
  const double factor = 0.5 * std::max(1e-3, delta_t);
  // consider ego_v is large than max(v_refs&svlimit)
  std::vector<double> preview_desired_vel;
  preview_desired_vel.clear();
  int v_refs_size = v_refs.size();
  int preview_item = 10;
  double item_new_preview_vel = 0.0;
  for (size_t i = 0; i < v_refs.size(); i++) {
    if (i + preview_item >= v_refs_size) {
      item_new_preview_vel = v_refs[v_refs_size - 1];
    } else {
      item_new_preview_vel = v_refs[i + preview_item];
    }
    preview_desired_vel.push_back(item_new_preview_vel);
  }

  // 1. idm construct reference
  for (std::size_t index = 0; index < speed_planner_profile.size(); ++index) {
    if (index == 0) {
      raw_s_ref_follow.at(index) = ego_info.ego_s;
      raw_v_ref_follow.at(index) = ego_info.ego_v;
      raw_a_ref_follow.at(index) = ego_info.ego_a;
    } else {
      idm_params.desired_spd = preview_desired_vel.at(index);
      double acc = 0.0;
      // idm_params.min_spacing =
      //     IDM::linear_interpolate(v_ref_follow.at(index - 1),
      //     idm_params.v_std,
      //                             idm_params.vel_error_std);
      idm_params.min_spacing = IDM::linear_interpolate(
          raw_v_ref_follow.at(index - 1), idm_params.idm_delta_v,
          idm_params.idm_min_spacing_list);
      // bumper to bumper dis
      double delta_s = speed_planner_profile.at(index).s_leader - 5 -
                       0.5 * speed_planner_profile.at(index).leadercarlength -
                       raw_s_ref_follow.at(index - 1);
      // std::cout << "s_leader: " << speed_planner_profile.at(index).s_leader
      //           << ",s_ego: " << raw_s_ref_follow.at(index - 1) << std::endl;
      double delta_v = raw_v_ref_follow.at(index - 1) -
                       speed_planner_profile.at(index).v_leader;
      // idm_params.change_rate = IDM::linear_interpolate(
      //     delta_v, idm_params.idm_delta_v, idm_params.idm_min_spacing_list);
      idm_params.change_rate = IDM::linear_interpolate(
          delta_v, idm_params.delta_v, idm_params.s_change_rate);
      acc = IDM::calc_idm_acc(
          idm_params, delta_s, raw_v_ref_follow.at(index - 1), delta_v,
          speed_planner_profile.at(index).exist_leader, index);
      raw_a_ref_follow.at(index) = acc;
      raw_v_ref_follow.at(index) =
          std::max(0.0, raw_v_ref_follow.at(index - 1) + acc * delta_t);
      raw_s_ref_follow.at(index) =
          raw_s_ref_follow.at(index - 1) +
          (raw_v_ref_follow.at(index - 1) + raw_v_ref_follow.at(index)) *
              factor;
      // consider avoid to collision
      if (delta_s < 7) {
        maybe_collision = true;
      }
    }
  }
  for (std::size_t index = 0; index < speed_planner_profile.size(); ++index) {
    if (index == 0) {
      after_s_ref_follow[index] = raw_s_ref_follow.at(0);
      after_v_ref_follow[index] = raw_v_ref_follow.at(0);
      after_a_ref_follow[index] = raw_a_ref_follow.at(0);
    } else {
      double acc_delta = raw_a_ref_follow[index] - raw_a_ref_follow[index - 1];
      std::cout << "index : " << index << ",before acc delta :" << acc_delta
                << ",raw acc index:" << raw_a_ref_follow[index]
                << ",raw acc index-1:" << raw_a_ref_follow[index - 1]
                << std::endl;
      if (acc_delta > 0) {
        acc_delta = std::min(acc_delta, kMaxJerk * delta_t);
      } else {
        acc_delta = std::max(acc_delta, -kMaxJerk * delta_t);
      }
      after_a_ref_follow.at(index) = raw_a_ref_follow[index - 1] + acc_delta;
      std::cout << ",after acc delta :" << acc_delta
                << ",after acc index:" << after_a_ref_follow.at(index)
                << std::endl;
      after_v_ref_follow.at(index) =
          std::max(0.0, after_v_ref_follow.at(index - 1) +
                            after_a_ref_follow.at(index) * delta_t);
      after_s_ref_follow.at(index) =
          after_s_ref_follow.at(index - 1) +
          (after_v_ref_follow.at(index - 1) + after_v_ref_follow.at(index)) *
              factor;
    }
  }
  // for (size_t i = 0; i < speed_planner_profile.size(); i++) {
  //   std::cout << "raw a:" << raw_a_ref_follow.at(i)
  //             << ",after a is:" << after_a_ref_follow.at(i)
  //             << ",raw v:" << raw_v_ref_follow.at(i)
  //             << ",after v is:" << after_v_ref_follow.at(i)
  //             << ",raw s:" << raw_s_ref_follow.at(i)
  //             << ",after s is:" << after_s_ref_follow.at(i) << std::endl;
  // }
  // if (1) {
  if (!maybe_collision && ego_info.ego_v > kMinEnableJerkConstrantSpd) {
    std::cout << "use const jerk!" << std::endl;
    for (size_t i = 0; i < speed_planner_profile.size(); i++) {
      IDM::IDMOutput output;
      output.t = i * 0.2;
      output.s_ref_out = after_s_ref_follow.at(i);
      output.v_ref_out = after_v_ref_follow.at(i);
      output.a_ref_out = after_a_ref_follow.at(i);
      idm_output.push_back(output);
      std::cout << "infex :" << i
                << ",after acc is:" << after_a_ref_follow.at(i) << std::endl;
    }
  } else {
    std::cout << "use raw idm output!" << std::endl;
    for (size_t i = 0; i < speed_planner_profile.size(); i++) {
      IDM::IDMOutput output;
      output.t = i * 0.2;
      output.s_ref_out = raw_s_ref_follow.at(i);
      output.v_ref_out = raw_v_ref_follow.at(i);
      output.a_ref_out = raw_a_ref_follow.at(i);
      idm_output.push_back(output);
        std::cout << "infex :" << i
            << ",after acc is:" << raw_a_ref_follow.at(i) << std::endl;
    }
  }
  return idm_output;
}
}