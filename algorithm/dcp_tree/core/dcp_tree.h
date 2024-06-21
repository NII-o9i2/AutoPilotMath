#pragma once
#include <iomanip>
#include <queue>
#include <algorithm>
#include <utility>
#include "memory"
#include "utils.h"
#include "core/track_simulator.h"
#include "core/ilqr_lon_condition.h"
namespace DCP_TREE {

struct DCPTreeNodeData {
  std::unordered_map<DCPLaneDir, std::vector<ILQR::IDMCarInfo>, DCPLaneDirHash>
      obs_pos;
  //  std::vector<std::vector<ILQR::IDMCarInfo>> obs_pos;  // left-middle-right
  ILQR::IDMCarInfo ego_pos;
  DCPLaneDir ego_lane_id = ABSOLUTE_MIDDLE;
  std::vector<DCPActionDir> tree_trace;
  double accumulated_s = 0.0;
  double cost = 0.0;
  double accumulated_cost = 0.0;
  DCPTreeNodeDebugInfo debug_info;
  int test_data = 0;
};

class DCPTreeNode {
 public:
  DCPTreeNode() = default;
  ~DCPTreeNode() = default;
  DCPTreeNode(DCPTreeNodeData value,
              const std::shared_ptr<DCPTreeNode> &parentNode)
      : data(std::move(value)), parent(parentNode) {
    child_list.clear();
  }
  DCPTreeNode(const DCPTreeNode &src) : data(src.data) { child_list.clear(); }
  explicit DCPTreeNode(DCPTreeNodeData data) : data(std::move(data)) {
    child_list.clear();
  }

  void show_tree_list(std::string trace,
                      Path &path,
                      std::vector<Path> &all_paths);

  void calc_node_cost(double pre_cost, const double &init_ego_vel);

  std::vector<std::shared_ptr<DCPTreeNode>> child_list;
  std::shared_ptr<DCPTreeNode> parent = nullptr;
  DCPTreeNodeData data;

  DCPActionDir ongoing_action = LK;
  double current_time = 0.0;
  MathUtils::Point2D ego_position;
};

class DCPTree {
 public:
  DCPTree() : root_(nullptr) {}
  ~DCPTree() = default;

  void set_root(std::shared_ptr<DCPTreeNode> &new_root) { root_ = new_root; }

  void update_tree(const DCPTreeNodeData &src);

  static std::shared_ptr<DCPTreeNode> generate_next_node(
      DCPActionDir action,
      double next_t,
      std::shared_ptr<DCPTreeNode> &parent,
      DCP_TREE::IDMLongiSimulator &simulator,
      const DCP_TREE::Params &params);

  void set_IDM_params(const double &delta_t,
                      const int &lanes_size,
                      const double &init_ego_vel,
                      const double &desired_speed);

  void show() { root_->show_tree_list("", path_, all_paths_); }

  std::vector<Path> get_all_path() { return all_paths_; };

 private:
  std::shared_ptr<DCPTreeNode> root_ = nullptr;

  DCP_TREE::Params params_;

  DCP_TREE::Path path_;

  std::vector<Path> all_paths_;
};
}  // namespace DCP_TREE