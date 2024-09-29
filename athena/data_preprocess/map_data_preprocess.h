//
// Created by SENSETIME\fengxiaotong on 24-8-22.
//
#pragma once
#include "cmath"
#include "data_preprocess/train_data_type.h"
namespace DLP {

struct Point3D {
  double x;
  double y;
  double z;

  double norm() const { return std::sqrt(x * x + y * y + z * z); }

  Point3D operator+(const Point3D &b) const {
    Point3D c{};
    c.x = this->x + b.x;
    c.y = this->y + b.y;
    c.z = this->z + b.z;
    return c;
  }

  Point3D operator-(const Point3D &b) const {
    Point3D c{};
    c.x = this->x - b.x;
    c.y = this->y - b.y;
    c.z = this->z - b.z;
    return c;
  }

  template<typename N, std::enable_if_t<std::is_arithmetic<N>::value, int> = 0>
  Point3D operator*(const N scalar) const {
    Point3D q{};
    q.x = this->x * scalar;
    q.y = this->y * scalar;
    q.z = this->z * scalar;
    return q;
  }

  template<typename N, std::enable_if_t<std::is_arithmetic<N>::value, int> = 0>
  Point3D operator/(const N scalar) {
    Point3D c{};
    c.x = this->x / scalar;
    c.y = this->y / scalar;
    c.z = this->z / scalar;
    return c;
  }
};
enum MapDataMode {
  M_TRAIN = 0,
  M_PREDICT = 1
};

struct MapDataParam {
  int max_polygon_num = 30;          // Number of Max polygon num (default = 30)
  int max_point_num = 500;           // Number of Max points num (default = 500)
  double pl2pl_radius = 150;
  MapDataMode mode = M_TRAIN;
};

class MapTensorParam {
 public:
  explicit MapTensorParam(MapDataParam &param) {
    N_pl = param.max_polygon_num;
    N_pt = param.max_point_num;
    N_pt2pl = param.max_point_num;
    N_pl2pl = param.max_polygon_num * param.max_polygon_num;
  };

  int N_pl = 0;               // Number of polygon
  int N_pt = 0;               // Number of points
  int N_pt2pl = 0;            // Number of points to polygon
  int N_pl2pl = 0;            // Number of polygon to polygon
};

enum PL2PLType{
  PL2PL_NONE = 0,
  PL2PL_PRED,
  PL2PL_SUCC,
  PL2PL_LEFT,
  PL2PL_RIGHT
};

enum PolygonType {
  VEHICLE_LANE = 0,
  BIKE_LANE,
  BUS_LANE,
  PEDESTRIAN_LANE,
  STOP_LANE,
  OCC
};

enum PointType {
  DASH = 0,
  SOLID,
  NONE,
  UNKNOWN,
  CROSSWALK,
  CENTER_LINE,
  CENTER_LINE_YELLOW_RED,
  CENTER_LINE_GREEN,
  STOP_LINE_VALID,
  STOP_LINE_INVALID,
  OCC_LINE,
};

enum PointSide {
  LEFT = 0,
  RIGHT,
  CENTER
};

enum PolygonBaseType{
  P_MAP_LANE_SEGMENT = 0,
  P_CROSSWALK,
  P_OCC,
  P_NONE
};

class PolygonBase {
 public:
  explicit PolygonBase(int id) : raw_id_(id) {

  };
  ~PolygonBase() = default;

  void reshape(int left_sum, int right_sum, int center_sum) {
    left_sum_ = left_sum;
    right_sum_ = right_sum;
    center_sum_ = center_sum;
    points_sum_ = std::max(left_sum - 1,0) + std::max(right_sum -1 ,0) + std::max(center_sum - 1,0);
    if (points_sum_ < 0) {
      points_sum_ = 0;
    }
    base_type_ = P_NONE;
    polygon_position_.create(1, 3);
    polygon_orientation_.create(1);
    polygon_height_.create(1);
    polygon_type_.create(1);
    polygon_is_intersection_.create(1);
    point_position_.create(points_sum_, 3);
    point_orientation_.create(points_sum_);
    point_magnitude_.create(points_sum_);
    point_height_.create(points_sum_);
    point_type_.create(points_sum_);
    point_side_.create(points_sum_);
  };


  void compress(std::vector<std::pair<PointType, Point3D>>& points,
                const float compress_rate,
                const int min_size);

  bool update_tensor(PolygonBaseType type,  PolygonType polygon_type,
                     std::vector<std::pair<PointType, Point3D>>& left_pts,
                     std::vector<std::pair<PointType, Point3D>>& right_pts,
                     std::vector<std::pair<PointType, Point3D>>& center_pts,
                     float compress_rate);
  int len() const{
    return points_sum_;
  }

  TensorD2<float> &polygon_position(){
    return polygon_position_;
  }

  TensorD1<float> &polygon_orientation(){
    return polygon_orientation_;
  }

  TensorD1<int> &polygon_type(){
    return polygon_type_;
  }
  TensorD1<int> &polygon_is_intersection(){
    return polygon_is_intersection_;
  }

  TensorD2<float>& point_position(){
    return point_position_;
  };

  TensorD1<float>& point_orientation(){
    return point_orientation_;
  };

  TensorD1<float>& point_magnitude(){
    return point_magnitude_;
  };

  TensorD1<int> &point_type(){
    return point_type_;
  };
  TensorD1<int>& point_side(){
    return point_side_;
  };

  int raw_id(){
    return raw_id_;
  }

 private:
  // tensor info
  TensorD2<float> polygon_position_;
  TensorD1<float> polygon_orientation_;
  TensorD1<float> polygon_height_;
  TensorD1<int> polygon_type_;
  TensorD1<int> polygon_is_intersection_;
  TensorD2<float> point_position_;
  TensorD1<float> point_orientation_;
  TensorD1<float> point_magnitude_;
  TensorD1<float> point_height_;
  TensorD1<int> point_type_;
  TensorD1<int> point_side_;
  // polygon base param
  int points_sum_ = 0;
  int left_sum_ = 0;
  int right_sum_ = 0;
  int center_sum_ = 0;
  int raw_id_ = 0;
  PolygonBaseType base_type_ = P_NONE;
};

class MapTensorOutput {
 public:
  explicit MapTensorOutput(const MapTensorParam &params) {
    map_polygon_position_.create(params.N_pl, 3);
    map_polygon_orientation_.create(params.N_pl);
    map_polygon_type_.create(params.N_pl);
    map_polygon_is_intersection_.create(params.N_pl);
    map_polygon_num_.create(1);

    map_point_num_nodes_.create(1);
    map_point_position_.create(params.N_pt, 3);
    map_point_orientation_.create(params.N_pt);
    map_point_magnitude_.create(params.N_pt);
    map_point_type_.create(params.N_pt);
    map_point_side_.create(params.N_pt);

    map_enc_num_pt2pl_.create(1);
    map_enc_edge_index_pt2pl_.create(2, params.N_pt2pl);
    map_enc_r_pt2pl_.create(params.N_pt2pl, 3);

    map_enc_num_pl2pl_.create(1);
    map_enc_edge_index_pl2pl_.create(2, params.N_pl2pl);
    map_enc_type_pl2pl_.create(params.N_pl2pl);
    map_enc_r_pl2pl_.create(params.N_pl2pl, 3);
  }
  TensorD2<float> &get_map_polygon_position() {
    return map_polygon_position_;
  }
  const TensorD2<float> &map_polygon_position() const {
    return map_polygon_position_;
  }
  TensorD1<float> &get_map_polygon_orientation() {
    return map_polygon_orientation_;
  }
  const TensorD1<float> &map_polygon_orientation() const {
    return map_polygon_orientation_;
  }
  TensorD1<int32_t> &get_map_polygon_type() {
    return map_polygon_type_;
  }
  TensorD1<int32_t> &get_map_polygon_is_intersection() {
    return map_polygon_is_intersection_;
  }
  TensorD1<int> &get_map_polygon_num() {
    return map_polygon_num_;
  }

  TensorD1<int> &get_map_point_num_nodes() {
    return map_point_num_nodes_;
  }
  TensorD2<float> &get_map_point_position() {
    return map_point_position_;
  }
  TensorD1<float> &get_map_point_orientation() {
    return map_point_orientation_;
  }
  TensorD1<float> &get_map_point_magnitude() {
    return map_point_magnitude_;
  }
  TensorD1<int32_t> &get_map_point_type() {
    return map_point_type_;
  }
  TensorD1<int32_t> &get_map_point_side() {
    return map_point_side_;
  }
  TensorD1<int> &get_map_enc_num_pt2pl() {
    return map_enc_num_pt2pl_;
  }
  TensorD2<int64_t> &get_map_enc_edge_index_pt2pl() {
    return map_enc_edge_index_pt2pl_;
  }
  TensorD2<float> &get_map_enc_r_pt2pl() {
    return map_enc_r_pt2pl_;
  }

  TensorD1<int> &get_map_enc_num_pl2pl() {
    return map_enc_num_pl2pl_;
  }
  TensorD2<int64_t> &get_map_enc_edge_index_pl2pl() {
    return map_enc_edge_index_pl2pl_;
  }
  TensorD1<int32_t> &get_map_enc_type_pl2pl() {
    return map_enc_type_pl2pl_;
  }
  TensorD2<float> &get_map_enc_r_pl2pl() {
    return map_enc_r_pl2pl_;
  }

 private:
  TensorD2<float> map_polygon_position_;
  TensorD1<float> map_polygon_orientation_;
  TensorD1<int32_t> map_polygon_type_;
  TensorD1<int32_t> map_polygon_is_intersection_;
  TensorD1<int> map_polygon_num_;

  TensorD1<int> map_point_num_nodes_;
  TensorD2<float> map_point_position_;
  TensorD1<float> map_point_orientation_;
  TensorD1<float> map_point_magnitude_;
  TensorD1<int32_t> map_point_type_;
  TensorD1<int32_t> map_point_side_;

  TensorD1<int> map_enc_num_pt2pl_;
  TensorD2<int64_t> map_enc_edge_index_pt2pl_;
  TensorD2<float> map_enc_r_pt2pl_;

  TensorD1<int> map_enc_num_pl2pl_;
  TensorD2<int64_t> map_enc_edge_index_pl2pl_;
  TensorD1<int32_t> map_enc_type_pl2pl_;
  TensorD2<float> map_enc_r_pl2pl_;
};

class MapInfoManager {
 public:
  explicit MapInfoManager(MapDataParam &_param)
      : param_(_param), tensor_param_(_param), out_map_tensor_(tensor_param_) {
    polygon_list_.clear();
  };

  ~MapInfoManager() = default;

  void reset();

  void push_polygon(PolygonBase & new_polygon){
    if (new_polygon.len() < 1){
      return;
    }
    polygon_list_.emplace_back(new_polygon);
  }

  void push_pl2pl_edge(std::pair<int,int> edge, PL2PLType type){
    pl2pl_list_.emplace_back(edge,type);
  }

  void update_tensor();

  MapTensorOutput &get_map_tensor_out() {
    return out_map_tensor_;
  }

  std::vector<PolygonBase> &get_polygon_list() {
    return polygon_list_;
  }
 private:
  std::vector<std::pair<std::pair<int,int>, PL2PLType>> pl2pl_list_;
  MapDataParam param_;
  MapTensorParam tensor_param_;
  MapTensorOutput out_map_tensor_;
  std::vector<PolygonBase> polygon_list_;
};

}