#pragma once

#include <algorithm>
#include <vector>
#include <cmath>
#include <iostream>

namespace IDM {

struct ReturnTestOutput {
  double a = 1;
};
struct IDMOutput {
  double t;
  double s_ref_out;
  double v_ref_out;
  double a_ref_out;
};
struct SpeedPlannerProfile {
  double leadercarlength;
  double s_leader;
  double v_leader;
  int exist_leader;

  SpeedPlannerProfile(double _leadercarlength,
                      double _s_leader,
                      double _v_leader,
                      int _exist_leader)
      : leadercarlength(_leadercarlength),
        s_leader(_s_leader),
        v_leader(_v_leader),
        exist_leader(_exist_leader) {}
};

struct Test {
  double a;
  Test(double _a) : a(_a){};
};
struct IDMParam {
  double delta_t;
  double idm_cosnt_jerk;
  double delta;
  double acc_delta;
  double desired_deacc;
  double max_acc;
  double time_headway;
  double follow_car_delta;
  double min_spacing;
  double min_dacc;
  std::vector<double> v_std;
  std::vector<double> vel_error_std;
  std::vector<double> idm_delta_v;
  std::vector<double> idm_min_spacing_list;
  std::vector<double> delta_v;
  std::vector<double> s_change_rate;
  double desired_spd;
  double change_rate;

  IDMParam() = default;

  IDMParam(const double &dt,
           const double &jerk,
           const double &d,
           const double &ad,
           const double &dd,
           const double &maxa,
           const double &th,
           const double &fcd,
           const double &minS,
           const double &mind,
           const std::vector<double> &vstd,
           const std::vector<double> &vels,
           const std::vector<double> &idv,
           const std::vector<double> &idmsl,
           const std::vector<double> &dv,
           const std::vector<double> &schra,
           const double &dspd,
           const double &crate)
      : delta_t(dt),
        idm_cosnt_jerk(jerk),
        delta(d),
        acc_delta(ad),
        desired_deacc(dd),
        max_acc(maxa),
        time_headway(th),
        follow_car_delta(fcd),
        min_spacing(minS),
        min_dacc(mind),
        v_std(vstd),
        vel_error_std(vels),
        idm_delta_v(idv),
        idm_min_spacing_list(idmsl),
        delta_v(dv),
        s_change_rate(schra),
        desired_spd(dspd),
        change_rate(crate){};
};

struct EgoInfo {
  double ego_s;
  double ego_v;
  double ego_a;
  EgoInfo(const double &s, const double &v, const double &a)
      : ego_s(s), ego_v(v), ego_a(a) {}
  EgoInfo() = default;
};

double calc_idm_acc(IDMParam &idm_params,
                    double &delta_s,
                    double &curr_v,
                    double &delta_v,
                    int &exist_leader,
                    std::size_t &count);

double get_target_s(double desired_s,
                    double actual_s,
                    double delta_v,
                    double change_rate);

double linear_interpolate(const double &x,
                          const std::vector<double> &a_std,
                          const std::vector<double> &b_std);

std::vector<IDMOutput> get_idm_output(
    EgoInfo &ego_info,
    std::vector<SpeedPlannerProfile> &speed_planner_profile,
    IDMParam &idm_params,
    std::vector<double> v_refs);
}