#include <limits>
#include <algorithm>

#include "nn_tool.h"

namespace MathUtils {

NearestNeighborTool::NearestNeighborTool() {}
NearestNeighborTool::~NearestNeighborTool() {
  if (kd_tree_ != NULL) {
    kd_tree_ = delete_tree(kd_tree_);
  }
}

void NearestNeighborTool::reset() {
  nn_tool_mode_ = 1;
  nn_down_sample_rate_ = 5;

  ids_.clear();
  pts_.clear();

  if (kd_tree_ != NULL) {
    kd_tree_ = delete_tree(kd_tree_);
  }
  return;
}

bool NearestNeighborTool::update_data(const std::vector<Point2D>& tj_pts) {
  ids_.clear();
  pts_.clear();
  for (uint32_t i = 0; i < tj_pts.size(); i += nn_down_sample_rate_) {
    ids_.emplace_back(i / nn_down_sample_rate_);
    pts_.emplace_back(tj_pts[i]);
  }

  if (kd_tree_ != NULL) {
    kd_tree_ = delete_tree(kd_tree_);
  }
  this->kd_tree_ = build_tree(&ids_, 0, ids_.size(), 0);

  return true;
}

int NearestNeighborTool::nearest_neighbor(const Point2D& tjp) const {
  if (pts_.size() == 0) {
    return -1;
  }

  // Using brutal
  if (nn_tool_mode_ == 0) {
    double min_dist = std::numeric_limits<double>::max();
    int min_index = -1;
    for (uint32_t i = 0; i < pts_.size(); i++) {
      auto ref_pt = pts_[i];
      if ((ref_pt - tjp).norm() < min_dist) {
        min_index = i;
        min_dist = (ref_pt - tjp).norm();
      }
    }
    return min_index * nn_down_sample_rate_;
  }

  if (nn_tool_mode_ == 1) {
    auto result = nn(kd_tree_, tjp, 0);
    return result * nn_down_sample_rate_;
  }
  return -1;
}

KDTreeNode* NearestNeighborTool::build_tree(std::vector<int>* pt_ids,
                                            uint32_t begin,
                                            uint32_t end,
                                            uint32_t depth) {
  if (begin == end) return NULL;
  // TODO(congq): is static faster?
  if (depth % 2 == 1) {
    std::sort(pt_ids->begin() + begin, pt_ids->begin() + end,
              [this](int& a, int& b) -> bool {
                return this->pts_[a].x < this->pts_[b].x;
              });
  } else {
    std::sort(pt_ids->begin() + begin, pt_ids->begin() + end,
              [this](int& a, int& b) -> bool {
                return this->pts_[a].y < this->pts_[b].y;
              });
  }
  KDTreeNode* root = new KDTreeNode();

  uint32_t mid = (begin + end) / 2;
  root->data = pt_ids->at(mid);
  root->left_child_ = build_tree(pt_ids, begin, mid, depth + 1);
  root->right_child_ = build_tree(pt_ids, mid + 1, end, depth + 1);
  return root;
}

int NearestNeighborTool::nn(KDTreeNode* root, Point2D point, int depth) const {
  KDTreeNode *next_branch, *opp_branch;
  if ((depth % 2 == 1 && point.x < pts_[root->data].x) ||
      (depth % 2 != 1 && point.y < pts_[root->data].y)) {
    next_branch = root->left_child_;
    opp_branch = root->right_child_;

  } else {
    next_branch = root->right_child_;
    opp_branch = root->left_child_;
  }

  int tmp_result = root->data;
  if (next_branch != NULL) {
    int nn_next = nn(next_branch, point, depth + 1);
    tmp_result = closer_one(point, nn_next, root->data);
  }
  if (opp_branch != NULL &&
      (point - pts_[tmp_result]).norm() >
          std::abs(depth % 2 == 1 ? point.x - pts_[root->data].x
                                  : point.y - pts_[root->data].y)) {
    int nn_opt = nn(opp_branch, point, depth + 1);
    tmp_result = closer_one(point, nn_opt, tmp_result);
  }
  return tmp_result;
}

int NearestNeighborTool::closer_one(const Point2D& point,
                                    const int& a,
                                    const int& b) const {
  auto d_a = (point - pts_[a]).norm();
  auto d_b = (point - pts_[b]).norm();
  if (d_a < d_b) {
    return a;
  } else {
    return b;
  }
}

KDTreeNode* NearestNeighborTool::delete_tree(KDTreeNode* root) {
  if (root == NULL) {
    return NULL;
  }

  root->left_child_ = delete_tree(root->left_child_);
  root->right_child_ = delete_tree(root->right_child_);
  delete root;

  return NULL;
}

void NearestNeighborTool::print_tree(KDTreeNode* tree) {
  if (tree == NULL) {
    return;
  }
  print_tree(tree->left_child_);
  print_tree(tree->right_child_);
  return;
}

}  // namespace MathUtils
