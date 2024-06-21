//
// Created by SENSETIME\fengxiaotong on 24-5-17.
//
#pragma once
#include "memory"
#include "unordered_map"
#include "utils.h"
#include "core/dcp_tree_interface.h"
namespace DCP_TREE {

struct SearchInfo {
  int id = 0;
  int inspire_id = -1;
  int back_id = -1;
  int index = -1;
  bool is_collision = false;
  double omega_dot = 0.0;
  double a_dot = 0.0;
  double h_cost = 0.0;
};

struct DCPMotionTreePointPOIOutput {
  bool is_valid = false;
  double expect_delta = 0.0;
  MathUtils::Point2D point;
};
struct DCPMotionTreePoint {
  DCPMotionTreePoint() = default;

  void update_polygon();
  MathUtils::Point2D rotate_point(const MathUtils::Point2D &point, double angle);

  MathUtils::Point2D point;
  double theta = 0.0;
  double v = 0.0;
  double omega = 0.0;
  double speed_limit = 0.0;
  int index = 0;
  double length = 0.0;
  double width = 0.0;
  std::vector<MathUtils::Point2D> polygon;
  SearchInfo search_info;

  DCPMotionTreePoint(const DCPMotionTreePoint &other) {
    point = other.point;
    theta = other.theta;
    v = other.v;
    omega = other.omega;
    speed_limit = other.speed_limit;
    index = other.index;
    length = other.length;
    width = other.width;
    polygon = other.polygon;
    search_info = other.search_info;
  }
};

class DCPObjectInfo {
 public:
  DCPObjectInfo() {
    polygon.clear();
    theta = 0.0;
    ellipse_a = 0.0;
    ellipse_b = 0.0;
  }
  ~DCPObjectInfo() = default;
  int id = 0;
  double theta = 0.0;
  double ellipse_a = 0.0;
  double ellipse_b = 0.0;
  // polygon must have same sequence: left front/ right front/ right back/ left back
  std::vector<MathUtils::Point2D> polygon;
  MathUtils::Point2D center_point;
};

class PurePursuit {
 public:
  static double pp_control(const MathUtils::Point2D &target_pose,
                           const MathUtils::Point2D &current_pose,
                           const double &current_theta) {
    const double wheel_base = 2.92;
    double ld = (current_pose - target_pose).norm();
    double alfa = std::atan2(target_pose.y - current_pose.y,
                             target_pose.x - current_pose.x) -
        current_theta;

    return std::atan(2 * wheel_base * std::sin(alfa) / ld);
  }
};

struct DCPMotionTreeParameter{
  double wheel_base = 2.92;
  double preview_collision_time = 4.1;
  double kMaxOmegaDot = 0.174532925 * 2;
  double kMaxLatAcc = 1.2;
  double kConsiderDecLatAcc = 0.8;
  double kMaxDecAcc = -1.0;
  int max_iter = 20000;
};

class DCPMotionTreeInterface{
 public:
  virtual bool is_collision_in_free_space(const MathUtils::Point2D& start_pos,const MathUtils::Point2D& end_pos) = 0;
  virtual ~DCPMotionTreeInterface() = default;
};

class DCPMotionTree {
 public:
  DCPMotionTree() = default;
  ~DCPMotionTree() = default;
  void init(const std::vector<DCPMotionTreePoint> &init_points,
            const std::vector<DCPObjectInfo> &obj_info,
            double delta_t,DCPMotionTreeParameter& dcp_motion_tree_parameter,
            std::shared_ptr<DCPMotionTreeInterface>& dcp_motion_tree_interface) {
    init_points_ = init_points;
    obj_info_ = obj_info;
    delta_t_ = delta_t;
    parameter_ = dcp_motion_tree_parameter;
    max_id_ = 0;
    dcp_motion_tree_interface_ = dcp_motion_tree_interface;
  }
  void process();

  static double vec_dot_product(const MathUtils::Point2D &vec_a, const MathUtils::Point2D &vec_b) {
    return vec_a.x * vec_b.x + vec_a.y * vec_b.y;
  }

  static DCPMotionTreePointPOIOutput search_poi_from_source_hard(
      const std::vector<DCPMotionTreePoint> &source_pts,
      const DCPMotionTreePoint &query_point);

  static void step_a_omega_dcp(DCPMotionTreePoint &state,
                               DCPMotionTreePoint &next_state,
                               double delta_t);
  static bool check_collision(DCPMotionTreePoint &state,
                              DCPMotionTreePoint &next_state,
                              DCPObjectInfo &obj_info);

  static bool solve_quadratic(double a, double b, double c, double &t1, double &t2);
  // from gpt
  static bool is_line_segment_intersecting_tilted_ellipse(
      double x1, double y1, double x2, double y2, // 线段端点
      double h, double k, double a, double b,     // 椭圆参数
      double theta                                // 椭圆旋转角度
  );

  std::vector<DCPMotionTreePoint> &get_init_points() {
    return init_points_;
  }

  int get_a_new_id() {
    return max_id_++;
  }

  std::vector<DCPMotionTreePoint> get_open_points() {
    std::vector<DCPMotionTreePoint> res;
    for (auto &pt : point_map_) {
      res.emplace_back(*pt.second);
    }
    return res;
  }

 const std::vector<DCPMotionTreePoint>& get_result_points() {
    return result_points_;
  }

 private:
  int max_id_ = 0;
  int max_target_size_ = 1;
  double delta_t_ = 0.5;
  // a star 0; dcp motion 1
  int method_option_ = 1;
  std::vector<DCPMotionTreePoint> init_points_;
  std::vector<DCPMotionTreePoint> result_points_;
  std::vector<DCPObjectInfo> obj_info_;
  std::unordered_map<int, std::shared_ptr<DCPMotionTreePoint>> point_map_;
  std::vector<int> open_ids_;
  std::vector<int> close_ids_;
  std::vector<DCPMotionTreePoint> target_source_;
  DCPMotionTreeParameter parameter_;
  std::shared_ptr<DCPMotionTreeInterface> dcp_motion_tree_interface_;
};
}