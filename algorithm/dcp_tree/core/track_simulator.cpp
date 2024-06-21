#include "track_simulator.h"

namespace DCP_TREE {

void IDMLongiSimulator::init(
    const std::unordered_map<DCPLaneDir,
                             std::vector<ILQR::IDMCarInfo>,
                             DCPLaneDirHash> &_lanes_objs,
    const ILQR::IDMCarInfo &_ego_info,
    const DCPLaneDir &_ego_lane_dir,
    const DCPActionDir &_lc_type,
    const Params &_params) {
  raw_lanes_objs_ = _lanes_objs;
  raw_ego_info_ = _ego_info;
  ego_raw_lane_dir_ = _ego_lane_dir;
  next_action_ = _lc_type;
  params_ = _params;
  update_lanes_objs_.clear();
}

void IDMLongiSimulator::sort_lane_objs_with_s(
    std::vector<ILQR::IDMCarInfo> &cars) {
  std::sort(cars.begin(), cars.end(),
            [](const ILQR::IDMCarInfo &car_a, const ILQR::IDMCarInfo &car_b) {
              return car_a.car_s > car_b.car_s;
            });
}

double IDMLongiSimulator::update_lc_belief() {
  double lc_belief = 0.0;
  if (update_ego_info_.car_a < min_acc_threshold_ ||
      update_ego_info_.car_a > max_acc_threshold_) {
    return lc_belief;
  } else {
    lc_belief = 1.0;
  }
  for (auto &single_lane_objs : update_lanes_objs_) {
    for (auto &obj : single_lane_objs.second) {
      if (obj.car_a < min_acc_threshold_ || obj.car_a > max_acc_threshold_) {
        return 0.0;
      } else {
        lc_belief = 1.0;
      }
    }
  }
  return lc_belief;
}

double IDMLongiSimulator::forward_simu(const double &dur_time) {
  double lc_belief = 0.0;
  // 1.0 update lane_obs_list
  update_lanes_objs_ = raw_lanes_objs_;
  // DCP_TREE::SolverInfoLog::Instance()->log(
  //     ",obs lane size is:" + std::to_string(update_lanes_objs_.size()));
  auto next_lane = ego_raw_lane_dir_;
  if (next_action_ == DCPActionDir::LK) {
    update_lanes_objs_[ego_raw_lane_dir_].emplace_back(raw_ego_info_);
  } else if (next_action_ == DCPActionDir::LC_LEFT) {
    if (ego_raw_lane_dir_ == DCPLaneDir::ABSOLUTE_LEFT) {
      return 0.0;
    } else {
      lc_belief = 1.0;
    }
    next_lane = static_cast<DCPLaneDir>(ego_raw_lane_dir_ - 1);
    if (update_lanes_objs_.find(next_lane) == update_lanes_objs_.end()) {
      return 0.0;
    } else {
      lc_belief = 1.0;
    }
    update_lanes_objs_[next_lane].emplace_back(raw_ego_info_);
  } else if (next_action_ == DCPActionDir::LC_RIGHT) {
    if (ego_raw_lane_dir_ == DCPLaneDir::ABSOLUTE_RIGHT) {
      return 0.0;
    } else {
      lc_belief = 1.0;
    }
    next_lane = static_cast<DCPLaneDir>(ego_raw_lane_dir_ + 1);
    if (update_lanes_objs_.find(next_lane) == update_lanes_objs_.end()) {
      return 0.0;
    } else {
      lc_belief = 1.0;
    }
    update_lanes_objs_[next_lane].emplace_back(raw_ego_info_);
  }
  // DCP_TREE::SolverInfoLog::Instance()->log(",next lane is:" +
  //                                          std::to_string(next_lane));
  // for (const auto &pair : update_lanes_objs_) {
  //   DCP_TREE::SolverInfoLog::Instance()->log(
  //       "Key: " + std::to_string(static_cast<int>(pair.first)));
  // }

  // 2.0 check crash or too close
  for (auto &item : update_lanes_objs_) {
    sort_lane_objs_with_s(item.second);
    // TODO: idm must will not crash
    // for (int i = 1; i < item.second.size(); i++) {
    //   auto &front = item.second[i - 1];
    //   auto &back = item.second[i];
    //   if (front.car_s - back.car_s -
    //           0.5 * (front.car_length + back.car_length) <
    //       0.2) {
    //     // crash then return
    //     lc_belief_ = 0.0;
    //     return;
    //   }
    // }
  }
  // for (auto &lane : update_lanes_objs_) {
  //   std::cout << " id is:" << lane.first << std::endl;
  //   for (auto &car : lane.second) {
  //     std::cout << "car is:" << std::endl;
  //     car.print_car_info();
  //   }
  // }

  // 3.0 update lane obs list
  for (auto &lane : update_lanes_objs_) {
    // 3.1 update lead-one
    // 3.2 update back-car && if car == ego car update info
    // 3.3 update belief
    auto &single_lane_objs = lane.second;
    if (single_lane_objs.empty()) continue;

    bool is_ego_next_lane = lane.first == next_lane;
    lane.second =
        forward_simu_single_lane(single_lane_objs, dur_time, is_ego_next_lane);
  }
  // std::cout << "update ego info is:" << std::endl;
  // update_ego_info_.print_car_info();
  lc_belief = update_lc_belief();
  return lc_belief;
}

std::vector<ILQR::IDMCarInfo> IDMLongiSimulator::forward_simu_single_lane(
    const std::vector<ILQR::IDMCarInfo> &single_lane_objs,
    const double &dur_time,
    const bool &is_ego_next_lane) {
  if (dur_time > params_.delta_t) {
    std::vector<ILQR::IDMCarInfo> init_lane_obs = single_lane_objs;
    ILQR::IDMCarInfo init_ego_info = raw_ego_info_;
    double accum_time = 0.0;
    while (accum_time < (dur_time - 1e-3)) {
      double cur_delta_t = params_.delta_t;
      if ((accum_time + cur_delta_t) > dur_time) {
        cur_delta_t = dur_time - accum_time;
      }

      ILQR::IDMCarInfo updated_ego_info;
      bool ego_updated = false;
      auto new_lane_obs = forward_simu_single_lane_step(
          init_lane_obs, cur_delta_t, init_ego_info, is_ego_next_lane,
          updated_ego_info, ego_updated);
      if (ego_updated) {
        init_ego_info = updated_ego_info;
      }
      init_lane_obs = std::move(new_lane_obs);

      accum_time += cur_delta_t;
    }
  }

  ILQR::IDMCarInfo updated_ego_info;
  bool ego_updated = false;
  auto updated_lane_obs = forward_simu_single_lane_step(
      single_lane_objs, dur_time, raw_ego_info_, is_ego_next_lane,
      updated_ego_info, ego_updated);
  if (ego_updated) {
    update_ego_info_ = updated_ego_info;
  }
  return updated_lane_obs;
}

std::vector<ILQR::IDMCarInfo> IDMLongiSimulator::forward_simu_single_lane_step(
    const std::vector<ILQR::IDMCarInfo> &single_lane_objs,
    const double &dur_time,
    const ILQR::IDMCarInfo &init_ego_info,
    const bool &is_ego_next_lane,
    ILQR::IDMCarInfo &updated_ego_info,
    bool &ego_updated) {
  ego_updated = false;
  std::vector<ILQR::IDMCarInfo> updated_lane_obs;
  for (size_t i = 0; i < single_lane_objs.size(); i++) {
    const double tolerance = 0.01;
    auto &ego_car = single_lane_objs[i];
    bool is_s_close = std::abs(ego_car.car_s - init_ego_info.car_s) < tolerance;
    bool is_v_close = std::abs(ego_car.car_v - init_ego_info.car_v) < tolerance;
    bool is_a_close = std::abs(ego_car.car_a - init_ego_info.car_a) < tolerance;
    bool car_is_ego = is_s_close && is_v_close && is_a_close;
    ILQR::IDMCarInfo updated_car_info;
    if (i == 0) {
      if (car_is_ego) {
        ILQR::IDMCarInfo leader_car(0.0, 0.0, 0.0, 0.0);
        auto longi_profile = ILQR::LongitudinalOperator::calc_longi_profile(
            params_.idm_params, ego_car, leader_car, dur_time, 0);
        updated_car_info.car_s = longi_profile.s;
        updated_car_info.car_v = longi_profile.v;
        updated_car_info.car_a = longi_profile.a;
        updated_car_info.car_length = ego_car.car_length;
      } else {
        updated_car_info.car_s = ego_car.car_s + ego_car.car_v * dur_time;
        updated_car_info.car_v = ego_car.car_v;
        updated_car_info.car_a = ego_car.car_a;
        updated_car_info.car_length = ego_car.car_length;
      }
    } else {
      auto leader_car = single_lane_objs[i - 1];
      auto longi_profile = ILQR::LongitudinalOperator::calc_longi_profile(
          params_.idm_params, ego_car, leader_car, dur_time, 1);
      updated_car_info.car_s = longi_profile.s;
      updated_car_info.car_v = longi_profile.v;
      updated_car_info.car_a = longi_profile.a;
      updated_car_info.car_length = ego_car.car_length;
    }
    if (car_is_ego && is_ego_next_lane) {
      ego_updated = true;
      updated_ego_info = updated_car_info;
    } else {
      updated_lane_obs.push_back(updated_car_info);
    }
  }
  return updated_lane_obs;
}

}  // namespace DCP_TREE
