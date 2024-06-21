//
// Created by SENSETIME\fengxiaotong on 24-5-17.
//
#include "iostream"
#include "dcp_motion_tree_dev.h"

namespace DCP_TREE{

void DCPMotionTreeDev::execute(double init_theta,double init_omega) {
  int step_size = 9;
  double delta_t = 1.0;
  DCPMotionTreePoint state;
  DCPMotionTreePoint next_state;
  state.index = 1;
  state.theta = init_theta;
  state.omega = init_omega;
  state.speed_limit = 40;
  state.v = 5;
  state.point.x = 0.0;
  state.point.y = 0.0;
  state.length = 4.9;
  state.width = 2.0;

  for (int i = 0; i< step_size; i++){
    state.update_polygon();
    init_points.emplace_back(state);
    DCPMotionTree::step_a_omega_dcp(state,next_state,delta_t);
    state = next_state;
  }
  DCPObjectInfo obj1;
  obj1.center_point.x = 25.0;
  obj1.center_point.y = 0.0;
  obj1.theta = 0.523;
  obj1.ellipse_a = 3.0;
  obj1.ellipse_b = 1.8;
  std::vector<DCPObjectInfo> obj_list;
  obj_list.emplace_back(obj1);

  DCPMotionTree motion_tree;
  DCPMotionTreeParameter parameter;
  auto an_interface_ptr = std::make_shared<DCPMotionTreeDevelopInterface>();
  auto dcp_interface = std::dynamic_pointer_cast<DCPMotionTreeInterface>(an_interface_ptr);
  motion_tree.init(init_points,obj_list, delta_t, parameter,dcp_interface);
  motion_tree.process();

//  obj1.polygon
  std::cout<< " finish execute"<< std::endl;
}
}