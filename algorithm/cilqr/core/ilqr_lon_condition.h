//
// Created by SENSETIME\fengxiaotong on 24-1-11.
//

#ifndef BENCH_TEST_ALGORITHM_CILQR_ILQR_LON_CONDITION_H_
#define BENCH_TEST_ALGORITHM_CILQR_ILQR_LON_CONDITION_H_

#include <algorithm>
#include <vector>
#include <cmath>
namespace ILQR {
class SolverInfoLog;
struct IDMLongiProfile {
  double s = 0.0;
  double v = 0.0;
  double a = 0.0;
};

struct IDMCarInfo {
  double car_s;
  double car_v;
  double car_a;
  double car_length;

  IDMCarInfo() = default;
  explicit IDMCarInfo(const double &_car_s,
                      const double &_car_v,
                      const double &_car_a,
                      const double &_car_length)
      : car_s(_car_s), car_v(_car_v), car_a(_car_a), car_length(_car_length){};

  void print_car_info() const;
};

struct IDMParam {
  double delta = 3.0;
  double acc_delta = 2.0;
  double desired_deceleration = 3.0;
  double max_acc = 1.2;
  double time_headway = 1.0;
  double follow_car_delta = 1.6;
  double min_deceleration = 4.0;
  std::vector<double> v_std = {9.49, 18.98, 28.48};       // 30kph, 60kph, 90kph
  std::vector<double> vel_error_std = {1.88, 2.70, 3.7};  // 3kph, 6kph, 9kph
  std::vector<double> idm_delta_v = {0.0, 2.78, 11.11, 22.22,
                                     90.0};  // 0kph 10kph 40kph 80kph
  std::vector<double> idm_min_spacing_list = {2.0, 2.5, 3.0, 3.5,
                                              3.5};  // 3m 4m 5m 6m 8m
  std::vector<double> delta_v = {-3.0, -2.0, -1.0, 0.0};
  std::vector<double> s_change_rate = {0.3, 0.5, 0.7, 0.9};
  double desired_spd = 80 / 3.6;

  IDMParam() = default;
};

class LongitudinalOperator {
 public:
  static IDMLongiProfile calc_longi_profile(const ILQR::IDMParam &idm_params,
                                            const IDMCarInfo &ego_state,
                                            const IDMCarInfo &obs_state,
                                            const double &delta_t,
                                            const int &exist_leader);

  static std::pair<double, double> calc_idm_acc(const IDMParam &idm_params,
                                                double delta_s,
                                                double curr_v,
                                                double leader_v,
                                                int exist_leader);

  // overload func, take desired sped as input param
  static std::pair<double, double> calc_idm_acc(const IDMParam &idm_params,
                                                double delta_s,
                                                double curr_v,
                                                double leader_v,
                                                double speed_limit,
                                                int exist_leader);

  static double linear_interpolate(const double &x,
                                   const std::vector<double> &a_std,
                                   const std::vector<double> &b_std);
  static double calc_cah_acc(const double actual_s,
                             const double leader_v,
                             const double curr_v,
                             const double leader_a);
  static double calc_acc(IDMParam &idm_params,
                         const double actual_s,
                         const double leader_v,
                         const double curr_v,
                         const double leader_a);
};
}  // namespace ILQR

#endif  // BENCH_TEST_ALGORITHM_CILQR_ILQR_LON_CONDITION_H_