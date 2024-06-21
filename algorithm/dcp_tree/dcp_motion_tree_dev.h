//
// Created by SENSETIME\fengxiaotong on 24-5-17.
//

#pragma once
#include "core/dcp_motion_tree.h"
namespace DCP_TREE{

class DCPMotionTreeDevelopInterface : public DCPMotionTreeInterface{
 public:
  bool is_collision_in_free_space(const MathUtils::Point2D& start_pos,const MathUtils::Point2D& end_pos) override {
    return false;
  };
  ~DCPMotionTreeDevelopInterface() = default;
};

class DCPMotionTreeDev{
 public:
  DCPMotionTreeDev() =default;
  ~DCPMotionTreeDev() = default;

  void execute(double init_theta,double init_omega);
  std::vector<DCPMotionTreePoint> init_points;

};

}