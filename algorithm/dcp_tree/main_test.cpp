//
// Created by SENSETIME\fengxiaotong on 23-11-27.
//
#include "iostream"
#include "core/track_simulator.h"
#include "env_simulator.h"
#include "obstacle.h"
#include "chrono"
#include "core/dcp_tree.h"
#include <queue>
#include "3rd/json.hpp"
#include <fstream>
#include "dcp_motion_tree_dev.h"

using json = nlohmann::json;

int main(int argc, char* argv[]) {
  // uint32_t temp = 0x03333222;
  // DCPTree test_tree1;
  // std::cout << test_tree1.check_available(temp,4) << std::endl;
  // return 0;
  auto start = std::chrono::high_resolution_clock::now();
  std::cout << "-------------*******-------------" << std::endl;
  DCP_TREE::IDMLongiSimulator idm_simulator;
  if (argc < 2) {
    // std::cout << "please input simulator case path" << std::endl;
    return 1;
  }
  std::ifstream file(argv[1]);
  if (!file.is_open()) {
    std::cerr << "Could not open the file" << std::endl;
    return 1;
  }
  // read JSON
  json j;
  file >> j;
  std::string scenarioName = j["name"];
  std::cout << "test case name is:" << scenarioName << std::endl;
  // std::unordered_map<DCP_TREE::DCPLaneDir, std::vector<ILQR::IDMCarInfo>>
  //     input_test;
  std::unordered_map<DCP_TREE::DCPLaneDir, std::vector<ILQR::IDMCarInfo>,
                     DCP_TREE::DCPLaneDirHash>
      input_test;
  for (const auto& lane : j["lanes_objs"]) {
    std::cout << "lane id is:" << lane["lane_id"] << std::endl;
    DCP_TREE::DCPLaneDir lane_dir;
    lane_dir = static_cast<DCP_TREE::DCPLaneDir>(lane["lane_id"]);
    std::vector<ILQR::IDMCarInfo> lane_cars;
    lane_cars.clear();
    for (const auto& car : lane["cars"]) {
      ILQR::IDMCarInfo car_info(
          car["car_s"].get<double>(), car["car_v"].get<double>(),
          car["car_a"].get<double>(), car["car_length"].get<double>());
      lane_cars.push_back(car_info);
      car_info.print_car_info();
    }
    input_test.insert(
        std::pair<DCP_TREE::DCPLaneDir, std::vector<ILQR::IDMCarInfo>>(
            lane_dir, lane_cars));
  }
  std::cout << "input size:" << input_test.size() << std::endl;
  for (auto item : input_test) {
    std::cout << "lane id is:" << item.first
              << ",obj size is:" << item.second.size() << std::endl;
    for (auto obj : item.second) {
      obj.print_car_info();
    }
  }
  DCP_TREE::DCPActionDir lc_type =
      static_cast<DCP_TREE::DCPActionDir>(j["change_direction"]);
  std::cout << "lc_type is: " << lc_type << std::endl;

  file.close();

  ILQR::IDMCarInfo ego_car_info{};
  ego_car_info.car_s = 0.2;
  ego_car_info.car_v = 6.6;
  ego_car_info.car_a = 1.0;
  ego_car_info.car_length = 5.0;
  std::cout << "raw ego info  is: " << std::endl;
  ego_car_info.print_car_info();

  DCP_TREE::DCPTree test_tree;
  DCP_TREE::DCPTreeNodeData root_node;
  root_node.obs_pos = input_test;
  root_node.ego_pos = ego_car_info;
  test_tree.set_IDM_params(1.0, 3, ego_car_info.car_v, 60);
  root_node.tree_trace.clear();
  test_tree.update_tree(root_node);
  test_tree.show();
  auto all_path = test_tree.get_all_path();
  for (size_t i = 0; i < all_path.size(); i++) {
    auto path = all_path.at(i);
    std::cout << "path number:" << i + 1 << std::endl;
    for (const auto& step : path) {
      std::cout << "Time: " << std::get<1>(step)
                << ", Action: " << std::get<2>(step) << "; "
                << ",Cost: " << std::get<0>(step) << std::endl;
    }
    std::cout << std::endl;
  }
  std::cout << "all paths size is:" << all_path.size() << std::endl;

  size_t min_cost_path_index = 0;
  double min_cost = std::get<0>(all_path[0].back());

  for (size_t i = 1; i < all_path.size(); i++) {
    double current_path_last_cost = std::get<0>(all_path[i].back());
    if (current_path_last_cost < min_cost) {
      min_cost = current_path_last_cost;
      min_cost_path_index = i;
    }
  }
  std::cout << "Path with minimum last step cost (Path number "
            << min_cost_path_index + 1 << "):" << std::endl;
  for (const auto& step : all_path[min_cost_path_index]) {
    std::cout << "Time: " << std::get<1>(step)
              << ", Action: " << std::get<2>(step)
              << ", Cost: " << std::get<0>(step) << std::endl;
  }
  std::cout << "Minimum last step cost: " << min_cost << std::endl;
  DCP_TREE::DCPTreeOutput output;
  if (all_path[min_cost_path_index].size() > 1) {
    auto& min_path_second_elem = all_path[min_cost_path_index][1];
    output.suggest_action_confidence = std::get<0>(min_path_second_elem);
    output.suggest_action_relative_time = std::get<1>(min_path_second_elem);
    output.suggest_action = std::get<2>(min_path_second_elem);
  } else {
    DCP_TREE::SolverInfoLog::Instance()->error(
        "The minimum cost path does not have enough elements.");
  }
  std::cout << "dcp_tree output action is:" << output.suggest_action
            << ",cost is:" << output.suggest_action_confidence
            << ",relative time is:" << output.suggest_action_relative_time
            << std::endl;
  std::cout << "***********debug info**********" << std::endl;
  // DCP_TREE::DCPTreeDebug debug;
  // for (auto& item : all_path) {
  //   auto path = item.second;
  //   std::vector<int> node_id_list;
  //   int node_index = 0;
  //   for (auto& node : path) {
  //     DCP_TREE::DebugNode debug_node;
  //     debug_node.cost = std::get<0>(node);
  //     debug_node.time = std::get<1>(node);
  //     debug_node.action = std::get<2>(node);
  //     debug_node.id = node_index++;
  //     node_id_list.push_back(node_index);
  //     node_index++;
  //     debug.node_debug_list[debug_node.id] = debug_node;
  //   }
  //   debug.node_path_debug_list.push_back(node_id_list);
  // }

  // for (const auto& nodePair : debug.node_debug_list) {
  //   const auto& node = nodePair.second;
  //   std::cout << "Node ID: " << node.id << ", Time: " << node.time
  //             << ", Cost: " << node.cost
  //             << ", Action: " << static_cast<int>(node.action) << std::endl;
  // }
  // std::cout << "Paths:" << std::endl;
  // for (const auto& path : debug.node_path_debug_list) {
  //   std::cout << "Path:";
  //   for (int nodeId : path) {
  //     std::cout << " " << nodeId;
  //   }
  //   std::cout << std::endl;
  // }

  std::cout << "***********test idm simulator**********" << std::endl;
  bool test_idm = false;
  if (false) {
    // prepare data
    DCP_TREE::Params parsms;
    ILQR::IDMParam idm_params;
    parsms.idm_params = idm_params;
    parsms.delta_t = 1;
    parsms.consider_lanes_size = 3;
    auto ego_raw_lane_dir_ = DCP_TREE::DCPLaneDir::ABSOLUTE_MIDDLE;

    // init and run
    idm_simulator.init(input_test, ego_car_info, ego_raw_lane_dir_, lc_type,
                       parsms);
    double belief = idm_simulator.forward_simu(parsms.delta_t);

    // output test
    auto res_ego = idm_simulator.get_update_ego_info();
    std::cout << "after ego info is:" << std::endl;
    res_ego.print_car_info();
    std::cout << "after lanes objs is:" << std::endl;
    auto res = idm_simulator.get_update_lanes_obs();

    std::cout << "output size:" << res.size() << std::endl;
    for (auto item : res) {
      std::cout << "lane id is:" << item.first
                << ",obj size is:" << item.second.size() << std::endl;
      for (auto obj : item.second) {
        std::cout << "car is:";
        obj.print_car_info();
      }
    }
    std::cout << "belief is:" << belief << std::endl;
  }

  auto end_idm = std::chrono::high_resolution_clock::now();
  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> duration = end - start;
  std::chrono::duration<double> duration_idm = end_idm - start;
  std::cout << "all time:" << duration.count() * 1000 << " ms"
            << ",idm_simulator time is:" << duration_idm.count() * 1000 << " ms"
            << std::endl;

  // dcp motion
  DCP_TREE::DCPMotionTreeDev motion_tree;
  motion_tree.execute(0.0,0.0);

//  double x1, double y1, double x2, double y2, // 线段端点
//  double h, double k, double a, double b,     // 椭圆参数
//  double theta                                // 椭圆旋转角度
  MathUtils::Point2D pa;
  MathUtils::Point2D pb;
  MathUtils::Point2D po;

  pa.x = 0.0;
  pa.y = 6.0;
  pb.x = 4.0;
  pb.y = 10;
  po.x = 4;
  po.y = 3.5;

  double ell_a = 2.85;
  double ell_b = 2.2;
  double theta = 2.1589;

  auto res = DCP_TREE::DCPMotionTree::is_line_segment_intersecting_tilted_ellipse(pa.x,pa.y,pb.x,pb.y,po.x,po.y,
                                                                       ell_a,ell_b,theta);
  std::cout<< " F right is : "<< res <<  std::endl;

  pa.x = 0.0;
  pa.y = -3.0;
  pb.x = 4.0;
  pb.y = 10;
  res = DCP_TREE::DCPMotionTree::is_line_segment_intersecting_tilted_ellipse(pa.x,pa.y,pb.x,pb.y,po.x,po.y,
                                                                                  ell_a,ell_b,theta);
  std::cout<< " T right is : "<< res <<  std::endl;

  pa.x = -2.0;
  pa.y = 8.0;
  pb.x = 8.0;
  pb.y = -2;
  res = DCP_TREE::DCPMotionTree::is_line_segment_intersecting_tilted_ellipse(pa.x,pa.y,pb.x,pb.y,po.x,po.y,
                                                                             ell_a,ell_b,theta);
  std::cout<< " T right is : "<< res <<  std::endl;

  pa.x = -6.0;
  pa.y = 8.0;
  pb.x = -2.0;
  pb.y = 4;
  res = DCP_TREE::DCPMotionTree::is_line_segment_intersecting_tilted_ellipse(pa.x,pa.y,pb.x,pb.y,po.x,po.y,
                                                                             ell_a,ell_b,theta);
  std::cout<< " F right is : "<< res <<  std::endl;

  pa.x = -6.0;
  pa.y = 8.0;
  pb.x = 11.0;
  pb.y = 6.0;
  res = DCP_TREE::DCPMotionTree::is_line_segment_intersecting_tilted_ellipse(pa.x,pa.y,pb.x,pb.y,po.x,po.y,
                                                                             ell_a,ell_b,theta);
  std::cout<< " F right is : "<< res <<  std::endl;

  pa.x = -12.0;
  pa.y = 8.0;
  pb.x = 20.0;
  pb.y = 4.0;
  res = DCP_TREE::DCPMotionTree::is_line_segment_intersecting_tilted_ellipse(pa.x,pa.y,pb.x,pb.y,po.x,po.y,
                                                                             ell_a,ell_b,theta);
  std::cout<< " T right is : "<< res <<  std::endl;
}