#pragma once

#include <vector>
#include <iostream>
#include <string>

#include "utils.h"

namespace MathUtils {

class KDTreeNode {
 public:
  int data;
  KDTreeNode* left_child_;
  KDTreeNode* right_child_;
};

class NearestNeighborTool {
 public:
  NearestNeighborTool();

  ~NearestNeighborTool();

  void reset();

  bool update_data(const std::vector<Point2D>& tj_pts);

  int nearest_neighbor(const Point2D& tjp) const;

  void set_nn_tool_mode(int mode) { nn_tool_mode_ = mode; }

  void set_down_sample_rate(int rate) { nn_down_sample_rate_ = rate; }

 private:
  KDTreeNode* build_tree(std::vector<int>* tjps,
                         uint32_t begin,
                         uint32_t end,
                         uint32_t depth);

  KDTreeNode* delete_tree(KDTreeNode* tree);

  void print_tree(KDTreeNode* tree);

  int nn(KDTreeNode* root, Point2D point, int depth) const;

  int closer_one(const Point2D& point, const int& a, const int& b) const;

 private:
  int nn_tool_mode_ = 1;
  int nn_down_sample_rate_ = 5;

  KDTreeNode* kd_tree_ = NULL;
  std::vector<Point2D> pts_;
  std::vector<int> ids_;
};

}  // namespace MathUtils
