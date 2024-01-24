#include <iostream>
#include <vector>
#include <vector>
#include "idm.h"

int main() {
  IDM::IDMParam idm_params;
  idm_params.delta_t = 0.2;
  idm_params.idm_cosnt_jerk = 4.0;
  idm_params.delta = 3.0;
  idm_params.acc_delta = 2.0;
  idm_params.desired_deacc = 3.5;
  idm_params.max_acc = 2.5;
  idm_params.time_headway = 1.0;
  idm_params.follow_car_delta = 1.5;
  idm_params.min_spacing = 0.0;
  idm_params.min_dacc = 3.5;
  idm_params.v_std.push_back(9.49);
  idm_params.v_std.push_back(18.98);
  idm_params.v_std.push_back(28.48);
  //   idm_params.v_std = {9.49, 18.98, 28.48};       // 30kph, 60kph, 90kph
  idm_params.vel_error_std.push_back(0.88);
  idm_params.vel_error_std.push_back(1.70);
  idm_params.vel_error_std.push_back(2.7);
  //   idm_params.vel_error_std = {0.88, 1.70, 2.7};  // 3kph, 6kph, 9kph
  //   idm_params.idm_delta_v = {0.0, 2.78, 11.11, 22.22,
  // 90.0};  // 0kph 10kph 40kph 80kph
  idm_params.idm_delta_v.push_back(0.0);
  idm_params.idm_delta_v.push_back(2.78);
  idm_params.idm_delta_v.push_back(11.11);
  idm_params.idm_delta_v.push_back(22.22);
  idm_params.idm_delta_v.push_back(90.0);
  //   idm_params.idm_min_spacing_list = {3.0, 4.0, 5.0, 6.0,
  //  8.0};  // 3m 4m 5m 6m 8m
  idm_params.idm_min_spacing_list.push_back(3.0);
  idm_params.idm_min_spacing_list.push_back(4.0);
  idm_params.idm_min_spacing_list.push_back(5.0);
  idm_params.idm_min_spacing_list.push_back(6.0);
  idm_params.idm_min_spacing_list.push_back(8.0);
  idm_params.desired_spd = 0.0;
  idm_params.change_rate = 0.0;

  IDM::EgoInfo ego_info;
  ego_info.ego_s = -1.71109;
  ego_info.ego_v = 14.7982;
  ego_info.ego_a = 0.0606892;

  std::vector<IDM::SpeedPlannerProfile> tmp_test;
  std::vector<double> refs;
  // tmp_test.resize(31);

  for (size_t i = 0; i < 31; i++)
  {
    IDM::SpeedPlannerProfile item(40,1,30,0);
    // item.leadercarlength = 4;
    // item.exist_leader = 1;
    // item.s_leader = 30;
    // item.v_leader = 0;
    tmp_test.push_back(item);
  }
  

  // tmp_test.at(0).s_leader = 21.6249;
  // tmp_test.at(1).s_leader = 24.6555;
  // tmp_test.at(2).s_leader = 27.6862;
  // tmp_test.at(3).s_leader = 30.7096;
  // tmp_test.at(4).s_leader = 33.7257;
  // tmp_test.at(5).s_leader = 36.7417;
  // tmp_test.at(6).s_leader = 39.777;
  // tmp_test.at(7).s_leader = 42.8122;
  // tmp_test.at(8).s_leader = 45.8485;
  // tmp_test.at(9).s_leader = 48.8857;
  // tmp_test.at(10).s_leader = 51.923;
  // tmp_test.at(11).s_leader = 54.952;
  // tmp_test.at(12).s_leader = 57.981;
  // tmp_test.at(13).s_leader = 61.0104;
  // tmp_test.at(14).s_leader = 64.0402;
  // tmp_test.at(15).s_leader = 67.0701;
  // tmp_test.at(16).s_leader = 70.0994;
  // tmp_test.at(17).s_leader = 73.1288;
  // tmp_test.at(18).s_leader = 76.1582;
  // tmp_test.at(19).s_leader = 79.1877;
  // tmp_test.at(20).s_leader = 82.2172;
  // tmp_test.at(21).s_leader = 85.2466;
  // tmp_test.at(22).s_leader = 88.276;
  // tmp_test.at(23).s_leader = 91.3052;
  // tmp_test.at(24).s_leader = 94.3342;
  // tmp_test.at(25).s_leader = 97.3632;
  // tmp_test.at(26).s_leader = 100.393;
  // tmp_test.at(27).s_leader = 103.422;
  // tmp_test.at(28).s_leader = 106.452;
  // tmp_test.at(29).s_leader = 109.481;
  // tmp_test.at(30).s_leader = 500;

  // tmp_test.at(0).v_leader = 15.1468;
  // tmp_test.at(1).v_leader = 15.1457;
  // tmp_test.at(2).v_leader = 15.1446;
  // tmp_test.at(3).v_leader = 15.1402;
  // tmp_test.at(4).v_leader = 15.1324;
  // tmp_test.at(5).v_leader = 15.1246;
  // tmp_test.at(6).v_leader = 15.131;
  // tmp_test.at(7).v_leader = 15.1373;
  // tmp_test.at(8).v_leader = 15.1417;
  // tmp_test.at(9).v_leader = 15.1442;
  // tmp_test.at(10).v_leader = 15.1467;
  // tmp_test.at(11).v_leader = 15.1468;
  // tmp_test.at(12).v_leader = 15.1468;
  // tmp_test.at(13).v_leader = 15.1468;
  // tmp_test.at(14).v_leader = 15.1467;
  // tmp_test.at(15).v_leader = 15.1467;
  // tmp_test.at(16).v_leader = 15.1467;
  // tmp_test.at(17).v_leader = 15.1467;
  // tmp_test.at(18).v_leader = 15.1467;
  // tmp_test.at(19).v_leader = 15.1467;
  // tmp_test.at(20).v_leader = 15.1467;
  // tmp_test.at(21).v_leader = 15.1467;
  // tmp_test.at(22).v_leader = 15.1467;
  // tmp_test.at(23).v_leader = 15.1467;
  // tmp_test.at(24).v_leader = 15.1467;
  // tmp_test.at(25).v_leader = 15.1467;
  // tmp_test.at(26).v_leader = 15.1467;
  // tmp_test.at(27).v_leader = 15.1467;
  // tmp_test.at(28).v_leader = 15.1467;
  // tmp_test.at(29).v_leader = 15.1467;
  // tmp_test.at(30).v_leader = 1.79769e+308;

  // tmp_test.at(0).leadercarlength = 17.3299;
  // tmp_test.at(1).leadercarlength = 17.3299;
  // tmp_test.at(2).leadercarlength = 17.3299;
  // tmp_test.at(3).leadercarlength = 17.3299;
  // tmp_test.at(4).leadercarlength = 17.3299;
  // tmp_test.at(5).leadercarlength = 17.3299;
  // tmp_test.at(6).leadercarlength = 17.3299;
  // tmp_test.at(7).leadercarlength = 17.3299;
  // tmp_test.at(8).leadercarlength = 17.3299;
  // tmp_test.at(9).leadercarlength = 17.3299;
  // tmp_test.at(10).leadercarlength = 17.3299;
  // tmp_test.at(11).leadercarlength = 17.3299;
  // tmp_test.at(12).leadercarlength = 17.3299;
  // tmp_test.at(13).leadercarlength = 17.3299;
  // tmp_test.at(14).leadercarlength = 17.3299;
  // tmp_test.at(15).leadercarlength = 17.3299;
  // tmp_test.at(16).leadercarlength = 17.3299;
  // tmp_test.at(17).leadercarlength = 17.3299;
  // tmp_test.at(18).leadercarlength = 17.3299;
  // tmp_test.at(19).leadercarlength = 17.3299;
  // tmp_test.at(20).leadercarlength = 17.3299;
  // tmp_test.at(21).leadercarlength = 17.3299;
  // tmp_test.at(22).leadercarlength = 17.3299;
  // tmp_test.at(23).leadercarlength = 17.3299;
  // tmp_test.at(24).leadercarlength = 17.3299;
  // tmp_test.at(25).leadercarlength = 17.3299;
  // tmp_test.at(26).leadercarlength = 17.3299;
  // tmp_test.at(27).leadercarlength = 17.3299;
  // tmp_test.at(28).leadercarlength = 17.3299;
  // tmp_test.at(29).leadercarlength = 17.3299;
  // tmp_test.at(30).leadercarlength = 5;

  // tmp_test.at(0).exist_leader = 1;
  // tmp_test.at(1).exist_leader = 1;
  // tmp_test.at(2).exist_leader = 1;
  // tmp_test.at(3).exist_leader = 1;
  // tmp_test.at(4).exist_leader = 1;
  // tmp_test.at(5).exist_leader = 1;
  // tmp_test.at(6).exist_leader = 1;
  // tmp_test.at(7).exist_leader = 1;
  // tmp_test.at(8).exist_leader = 1;
  // tmp_test.at(9).exist_leader = 1;
  // tmp_test.at(10).exist_leader = 1;
  // tmp_test.at(11).exist_leader = 1;
  // tmp_test.at(12).exist_leader = 1;
  // tmp_test.at(13).exist_leader = 1;
  // tmp_test.at(14).exist_leader = 1;
  // tmp_test.at(15).exist_leader = 1;
  // tmp_test.at(16).exist_leader = 1;
  // tmp_test.at(17).exist_leader = 1;
  // tmp_test.at(18).exist_leader = 1;
  // tmp_test.at(19).exist_leader = 1;
  // tmp_test.at(20).exist_leader = 1;
  // tmp_test.at(21).exist_leader = 1;
  // tmp_test.at(22).exist_leader = 1;
  // tmp_test.at(23).exist_leader = 1;
  // tmp_test.at(24).exist_leader = 1;
  // tmp_test.at(25).exist_leader = 1;
  // tmp_test.at(26).exist_leader = 1;
  // tmp_test.at(27).exist_leader = 1;
  // tmp_test.at(28).exist_leader = 1;
  // tmp_test.at(29).exist_leader = 1;
  // tmp_test.at(30).exist_leader = 0;

  for (size_t i = 0; i < 31; i++) {
    // IDM::SpeedPlannerProfile profile;
    // profile.leadercarlength = 5;
    // profile.s_leader = 500;
    // profile.v_leader = 1.79769e+308;
    // profile.exist_leader = 0;

    // tmp_test.push_back(profile);
    refs.push_back(22.22);
    // std::cout << "index:" << i << "length is:" <<
    // tmp_test.at(i).leadercarlength
    //           << ",s leader:" << tmp_test.at(i).s_leader
    //           << ",v leader:" << tmp_test.at(i).v_leader
    //           << ",exist :" << tmp_test.at(i).exist_leader
    //           << ",v refs:" << refs.at(i) << std::endl;
  }

  std::vector<IDM::IDMOutput> idm_output_test;
  // IDM::IDMPLANNER idm_planner;

  idm_output_test =
      IDM::get_idm_output(ego_info, tmp_test, idm_params, refs);

  for (size_t i = 0; i < idm_output_test.size(); i++) {
    std::cout << "idm_output_s:" << idm_output_test.at(i).s_ref_out
              << ",idm_output_v:" << idm_output_test.at(i).v_ref_out
              << ",idm_output_a:" << idm_output_test.at(i).a_ref_out
              << ",t is:" << idm_output_test.at(i).t << ",index:" << i
              << std::endl;
  }

  return 0;
}