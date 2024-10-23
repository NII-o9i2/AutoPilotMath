//
// Created by SENSETIME\fengxiaotong on 24-8-22.
//

#include "data_preprocess/map_data_preprocess.h"
namespace DLP {

void PolygonBase::compress(std::vector<std::pair<PointType, Point3D>> &points,
                            const float compress_rate,
                            const int min_size) {
    if (compress_rate < 0.0 || compress_rate > 1.0) {
      return;
    }
    if (points.size() <= min_size || points.empty()) {
      return;
    }
    int original_size = static_cast<int>(points.size());
    int compressed_size = static_cast<int>(std::ceil(original_size * compress_rate));
    if (compressed_size < min_size) {
        compressed_size = min_size;
    }
    std::vector<std::pair<PointType, Point3D>> compressed_points;
    compressed_points.reserve(compressed_size);
    compressed_points.emplace_back(points.front());
    double step = static_cast<double>(original_size) / compressed_size;
    for (int i = 1; i < compressed_size - 1; ++i) {
        int index = static_cast<int>(std::ceil(i * step));
        index = std::min(index, original_size - 1);
        if (index >= original_size) {
            index = original_size - 1;
        }
        compressed_points.emplace_back(points[index]);
    }
    compressed_points.emplace_back(points.back());
    points = std::move(compressed_points);
}

bool PolygonBase::update_tensor(DLP::PolygonBaseType type,
                                DLP::PolygonType polygon_type,
                                std::vector<std::pair<PointType, Point3D>> &left_pts,
                                std::vector<std::pair<PointType, Point3D>> &right_pts,
                                std::vector<std::pair<PointType, Point3D>> &center_pts,
                                float compress_rate) {
  if (type == P_NONE) {
    return false;
  }
  base_type_ = type;
  // compress the points
  if (compress_rate < 1.0) {
    compress(left_pts, compress_rate, 2);
    compress(right_pts, compress_rate, 2);
    compress(center_pts, compress_rate, 2);
  }
  reshape(left_pts.size(), right_pts.size(), center_pts.size());

  if (points_sum_ < 1){
    return false;
  }
  float vec_fist_x = 0;
  float vec_fist_y = 0;
  // fill tensor
  if (!center_pts.empty()){
    polygon_position_.data()[0][0] = static_cast<float>(center_pts.front().second.x);
    polygon_position_.data()[0][1] = static_cast<float>(center_pts.front().second.y);
    polygon_position_.data()[0][2] = 0.0;
    if (center_pts.size() > 1){
      vec_fist_x = static_cast<float>(center_pts[1].second.x - center_pts[0].second.x);
      vec_fist_y = static_cast<float>(center_pts[1].second.y - center_pts[0].second.y);
    }
  }else if (!left_pts.empty()){
      polygon_position_.data()[0][0] = static_cast<float>(left_pts.front().second.x);
      polygon_position_.data()[0][1] = static_cast<float>(left_pts.front().second.y);
      polygon_position_.data()[0][2] = 0.0;
      if (left_pts.size() > 1){
        vec_fist_x = static_cast<float>(left_pts[1].second.x - left_pts[0].second.x);
        vec_fist_y = static_cast<float>(left_pts[1].second.y - left_pts[0].second.y);
      }
    }else if (!right_pts.empty()){
    polygon_position_.data()[0][0] = static_cast<float>(right_pts.front().second.x);
    polygon_position_.data()[0][1] = static_cast<float>(right_pts.front().second.y);
    polygon_position_.data()[0][2] = 0.0;
    if (right_pts.size() > 1){
      vec_fist_x = static_cast<float>(right_pts[1].second.x - right_pts[0].second.x);
      vec_fist_y = static_cast<float>(right_pts[1].second.y - right_pts[0].second.y);
    }
  }

  polygon_orientation_.data()[0] = std::atan2(vec_fist_y,vec_fist_x);

  polygon_height_.data()[0] = 0;

  polygon_type_.data()[0] = polygon_type;

  if (type == P_CROSSWALK){
    polygon_is_intersection_.data()[0] = 1;
  }else{
    polygon_is_intersection_.data()[0] = 0;
  }

  for (int i = 1; i < left_pts.size();i++){
    point_position_.data()[i-1][0] = static_cast<float>(left_pts[i-1].second.x);
    point_position_.data()[i-1][1] = static_cast<float>(left_pts[i-1].second.y);
    point_position_.data()[i-1][2] = 0.0;
    float vec_x = left_pts[i].second.x - left_pts[i-1].second.x;
    float vec_y = left_pts[i].second.y - left_pts[i-1].second.y;
    point_orientation_.data()[i-1] = std::atan2(vec_y,vec_x);
    point_magnitude_.data()[i-1] = std::hypot(vec_x,vec_y);
    point_height_.data()[i-1] = 0.0;
    point_type_.data()[i-1] = left_pts[i-1].first;
    point_side_.data()[i-1] = static_cast<int>(PointSide::LEFT);
  }
  int sum_num = std::max(static_cast<int>(left_pts.size()) - 1,0);
  for (int i = 1; i < right_pts.size();i++){
    point_position_.data()[i-1+sum_num][0] = right_pts[i-1].second.x;
    point_position_.data()[i-1+sum_num][1] = right_pts[i-1].second.y;
    point_position_.data()[i-1+sum_num][2] = 0.0;
    float vec_x = right_pts[i].second.x - right_pts[i-1].second.x;
    float vec_y = right_pts[i].second.y - right_pts[i-1].second.y;
    point_orientation_.data()[i-1+sum_num] = std::atan2(vec_y,vec_x);
    point_magnitude_.data()[i-1+sum_num] = std::hypot(vec_x,vec_y);
    point_height_.data()[i-1+sum_num] = 0.0;
    point_type_.data()[i-1+sum_num] = right_pts[i-1].first;
    point_side_.data()[i-1+sum_num] = static_cast<int>(PointSide::RIGHT);
  }
  sum_num += std::max(static_cast<int>(right_pts.size()) - 1,0);
  for (int i = 1; i < center_pts.size();i++){
    point_position_.data()[i-1+sum_num][0] = center_pts[i-1].second.x;
    point_position_.data()[i-1+sum_num][1] = center_pts[i-1].second.y;
    point_position_.data()[i-1+sum_num][2] = 0.0;
    float vec_x = center_pts[i].second.x - center_pts[i-1].second.x;
    float vec_y = center_pts[i].second.y - center_pts[i-1].second.y;
    point_orientation_.data()[i-1+sum_num] = std::atan2(vec_y,vec_x);
    point_magnitude_.data()[i-1+sum_num] = std::hypot(vec_x,vec_y);
    point_height_.data()[i-1+sum_num] = 0.0;
    point_type_.data()[i-1+sum_num] = center_pts[i-1].first;
    point_side_.data()[i-1+sum_num] = static_cast<int>(PointSide::CENTER);
  }
  return true;
}

void MapInfoManager::reset() {
  polygon_list_.clear();
  pl2pl_list_.clear();
  auto& tensor_polygon_position = out_map_tensor_.get_map_polygon_position();
  auto& tensor_polygon_position_data = tensor_polygon_position.data();
  for (int i = 0;i < tensor_param_.N_pl;i++){
    tensor_polygon_position_data[i].resize(3, 0.0);
    std::fill(tensor_polygon_position_data[i].begin(), tensor_polygon_position_data[i].end(), 0.0);
  }
  auto& tensor_polygon_orient = out_map_tensor_.get_map_polygon_orientation();
  auto& tensor_polygon_orient_data = tensor_polygon_orient.data();
  tensor_polygon_orient_data.resize(tensor_param_.N_pl, 0.0);
  std::fill(tensor_polygon_orient_data.begin(), tensor_polygon_orient_data.end(), 0.0);

  auto& tensor_polygon_type = out_map_tensor_.get_map_polygon_type();
  auto& tensor_polygon_type_data = tensor_polygon_type.data();
  tensor_polygon_type_data.resize(tensor_param_.N_pl,0);
  std::fill(tensor_polygon_type_data.begin(), tensor_polygon_type_data.end(), 0);

  auto& tensor_polygon_is_intersection = out_map_tensor_.get_map_polygon_is_intersection();
  auto& tensor_polygon_is_intersection_data = tensor_polygon_is_intersection.data();
  tensor_polygon_is_intersection_data.resize(tensor_param_.N_pl,0);
  std::fill(tensor_polygon_is_intersection_data.begin(), tensor_polygon_is_intersection_data.end(), 0);

  auto& tensor_points_num = out_map_tensor_.get_map_point_num_nodes();
  auto& tensor_points_num_data = tensor_points_num.data();
  tensor_points_num_data[0] = 0;

  auto& tensor_points_position = out_map_tensor_.get_map_point_position();
  auto& tensor_points_position_data = tensor_points_position.data();
  for (int i =0; i< tensor_param_.N_pt;i++){
    tensor_points_position_data[i].resize(3,0.0);
    std::fill(tensor_points_position_data[i].begin(), tensor_points_position_data[i].end(), 0.0);
  }

  auto& tensor_points_orients = out_map_tensor_.get_map_point_orientation();
  auto& tensor_points_orients_data = tensor_points_orients.data();
  tensor_points_orients_data.resize(tensor_param_.N_pt,0.0);
  std::fill(tensor_points_orients_data.begin(), tensor_points_orients_data.end(), 0.0);

  auto& tensor_points_magnitude = out_map_tensor_.get_map_point_magnitude();
  auto& tensor_points_magnitude_data = tensor_points_magnitude.data();
  tensor_points_magnitude_data.resize(tensor_param_.N_pt,0.0);
  std::fill(tensor_points_magnitude_data.begin(), tensor_points_magnitude_data.end(), 0.0);

  auto& tensor_points_type= out_map_tensor_.get_map_point_type();
  auto& tensor_points_type_data = tensor_points_type.data();
  tensor_points_type_data.resize(tensor_param_.N_pt,0);
  std::fill(tensor_points_type_data.begin(), tensor_points_type_data.end(), 0);

  auto& tensor_points_side= out_map_tensor_.get_map_point_side();
  auto& tensor_points_side_data = tensor_points_side.data();
  tensor_points_side_data.resize(tensor_param_.N_pt,0);
  std::fill(tensor_points_side_data.begin(), tensor_points_side_data.end(), 0);

}

void MapInfoManager::update_tensor() {
  int points_raw_sum  = 0;
  int valid_polygon_size = polygon_list_.size() > param_.max_polygon_num ? param_.max_polygon_num: polygon_list_.size();

  auto& tensor_get_map_polygon_num = out_map_tensor_.get_map_polygon_num();
  auto& tensor_get_map_polygon_num_data = tensor_get_map_polygon_num.data();
  tensor_get_map_polygon_num_data[0] = 0;

  auto& tensor_polygon_position = out_map_tensor_.get_map_polygon_position();
  auto& tensor_polygon_position_data = tensor_polygon_position.data();

  auto& tensor_polygon_orient = out_map_tensor_.get_map_polygon_orientation();
  auto& tensor_polygon_orient_data = tensor_polygon_orient.data();

  auto& tensor_polygon_type = out_map_tensor_.get_map_polygon_type();

  auto& tensor_polygon_is_intersection = out_map_tensor_.get_map_polygon_is_intersection();

  auto& tensor_points_num = out_map_tensor_.get_map_point_num_nodes();
  auto& tensor_points_num_data = tensor_points_num.data();
  tensor_points_num_data[0] = 0;

  auto& tensor_points_position = out_map_tensor_.get_map_point_position();
  auto& tensor_points_position_data = tensor_points_position.data();

  auto& tensor_points_orients = out_map_tensor_.get_map_point_orientation();
  auto& tensor_points_orients_data = tensor_points_orients.data();

  auto& tensor_points_magnitude = out_map_tensor_.get_map_point_magnitude();

  auto& tensor_points_type= out_map_tensor_.get_map_point_type();
  auto& tensor_points_type_data = tensor_points_type.data();

  auto& tensor_points_side= out_map_tensor_.get_map_point_side();
  auto& tensor_points_side_data = tensor_points_side.data();

  auto& tensor_map_enc_num_pt2pl = out_map_tensor_.get_map_enc_num_pt2pl();
  auto& tensor_map_enc_num_pt2pl_data = tensor_map_enc_num_pt2pl.data();
  tensor_map_enc_num_pt2pl_data[0] = 0;

  auto& tensor_map_enc_edge_index_pt2pl = out_map_tensor_.get_map_enc_edge_index_pt2pl();
  auto& tensor_map_enc_edge_index_pt2pl_data = tensor_map_enc_edge_index_pt2pl.data();

  auto& tensor_map_enc_r_pt2pl = out_map_tensor_.get_map_enc_r_pt2pl();
  auto& tensor_map_enc_r_pt2pl_data = tensor_map_enc_r_pt2pl.data();

  auto& tensor_map_enc_mask_r2src_pt2pl = out_map_tensor_.get_map_enc_mask_r2src_pt2pl();
  auto& tensor_map_enc_mask_r2src_pt2pl_data = tensor_map_enc_mask_r2src_pt2pl.data();
  for (int i = 0; i < tensor_param_.N_pt2pl; i++) {
    tensor_map_enc_mask_r2src_pt2pl_data[i].resize(tensor_param_.N_pt, 0.0);
    std::fill(tensor_map_enc_mask_r2src_pt2pl_data[i].begin(), tensor_map_enc_mask_r2src_pt2pl_data[i].end(), 0.0);
  }

  auto& tensor_map_enc_mask_r2dst_pt2pl = out_map_tensor_.get_map_enc_mask_r2dst_pt2pl();
  auto& tensor_map_enc_mask_r2dst_pt2pl_data = tensor_map_enc_mask_r2dst_pt2pl.data();
  for (int i = 0; i < tensor_param_.N_pt2pl; i++) {
    tensor_map_enc_mask_r2dst_pt2pl_data[i].resize(tensor_param_.N_pl, 0.0);
    std::fill(tensor_map_enc_mask_r2dst_pt2pl_data[i].begin(), tensor_map_enc_mask_r2dst_pt2pl_data[i].end(), 0.0);
  }


  for (int i = 0; i< valid_polygon_size; i++){
    tensor_polygon_position.cover(tensor_get_map_polygon_num_data[0],0,polygon_list_[i].polygon_position());
    tensor_polygon_orient.cover(tensor_get_map_polygon_num_data[0],polygon_list_[i].polygon_orientation());
    tensor_polygon_type.cover(tensor_get_map_polygon_num_data[0],polygon_list_[i].polygon_type());
    tensor_polygon_is_intersection.cover(tensor_get_map_polygon_num_data[0],polygon_list_[i].polygon_is_intersection());
    tensor_points_position.cover(tensor_points_num_data[0],0,polygon_list_[i].point_position());
    tensor_points_orients.cover(tensor_points_num_data[0],polygon_list_[i].point_orientation());
    tensor_points_magnitude.cover(tensor_points_num_data[0],polygon_list_[i].point_magnitude());
    tensor_points_type.cover(tensor_points_num_data[0],polygon_list_[i].point_type());
    tensor_points_side.cover(tensor_points_num_data[0],polygon_list_[i].point_side());
    for (int j = 0; j < polygon_list_[i].len(); j++){
      tensor_map_enc_edge_index_pt2pl_data[0][tensor_map_enc_num_pt2pl_data[0]] = tensor_map_enc_num_pt2pl_data[0];
      tensor_map_enc_edge_index_pt2pl_data[1][tensor_map_enc_num_pt2pl_data[0]] = i;
      tensor_map_enc_mask_r2src_pt2pl_data[tensor_map_enc_num_pt2pl_data[0]][tensor_map_enc_num_pt2pl_data[0]] = 1.0;
      tensor_map_enc_mask_r2dst_pt2pl_data[tensor_map_enc_num_pt2pl_data[0]][i] = 1.0;
      float vec_x = tensor_points_position_data[tensor_map_enc_num_pt2pl_data[0]][0] - tensor_polygon_position_data[i][0];
      float vec_y = tensor_points_position_data[tensor_map_enc_num_pt2pl_data[0]][1] - tensor_polygon_position_data[i][1];
      float cos_pl = std::cos(tensor_polygon_orient_data[i]);
      float sin_pl = std::sin(tensor_polygon_orient_data[i]);
      tensor_map_enc_r_pt2pl_data[tensor_map_enc_num_pt2pl_data[0]][0] = std::hypot(vec_x,vec_y);
      tensor_map_enc_r_pt2pl_data[tensor_map_enc_num_pt2pl_data[0]][1] = angle_between_2d_vectors(cos_pl, sin_pl, vec_x,vec_y);
      tensor_map_enc_r_pt2pl_data[tensor_map_enc_num_pt2pl_data[0]][2] = normalize_angle(tensor_points_orients_data[tensor_map_enc_num_pt2pl_data[0]] - tensor_polygon_orient_data[i]);
      tensor_map_enc_num_pt2pl_data[0]++;
    }
    tensor_points_num_data[0]+=polygon_list_[i].len();
    tensor_get_map_polygon_num_data[0]++;
  }
  // fill pt2pl edge index & r
  if (tensor_map_enc_num_pt2pl_data[0] > 0){
    for (int i = tensor_map_enc_num_pt2pl_data[0]; i < tensor_param_.N_pt2pl; i++){
      tensor_map_enc_edge_index_pt2pl_data[0][i] = tensor_map_enc_edge_index_pt2pl_data[0][0];
      tensor_map_enc_edge_index_pt2pl_data[1][i] = tensor_map_enc_edge_index_pt2pl_data[1][0];
      tensor_map_enc_r_pt2pl_data[i][0] = tensor_map_enc_r_pt2pl_data[0][0];
      tensor_map_enc_r_pt2pl_data[i][1] = tensor_map_enc_r_pt2pl_data[0][1];
      tensor_map_enc_r_pt2pl_data[i][2] = tensor_map_enc_r_pt2pl_data[0][2];
    }
  }

  auto& tensor_map_enc_num_pl2pl = out_map_tensor_.get_map_enc_num_pl2pl();
  auto& tensor_map_enc_num_pl2pl_data = tensor_map_enc_num_pl2pl.data();
  tensor_map_enc_num_pl2pl_data[0] = 0;

  auto& tensor_map_enc_edge_index_pl2pl = out_map_tensor_.get_map_enc_edge_index_pl2pl();
  auto& tensor_map_enc_edge_index_pl2pl_data = tensor_map_enc_edge_index_pl2pl.data();

  auto& tensor_map_enc_type_pl2pl = out_map_tensor_.get_map_enc_type_pl2pl();
  auto& tensor_map_enc_type_pl2pl_data = tensor_map_enc_type_pl2pl.data();

  auto& tensor_map_enc_r_pl2pl = out_map_tensor_.get_map_enc_r_pl2pl();
  auto& tensor_map_enc_r_pl2pl_data = tensor_map_enc_r_pl2pl.data();

  auto& tensor_map_enc_mask_r2src_pl2pl = out_map_tensor_.get_map_enc_mask_r2src_pl2pl();
  auto& tensor_map_enc_mask_r2src_pl2pl_data = tensor_map_enc_mask_r2src_pl2pl.data();
  for (int i = 0; i < tensor_param_.N_pl2pl; i++) {
    tensor_map_enc_mask_r2src_pl2pl_data[i].resize(tensor_param_.N_pl, 0.0);
    std::fill(tensor_map_enc_mask_r2src_pl2pl_data[i].begin(), tensor_map_enc_mask_r2src_pl2pl_data[i].end(), 0.0);
  }

  auto& tensor_map_enc_mask_r2dst_pl2pl = out_map_tensor_.get_map_enc_mask_r2dst_pl2pl();
  auto& tensor_map_enc_mask_r2dst_pl2pl_data = tensor_map_enc_mask_r2dst_pl2pl.data();
  for (int i = 0; i < tensor_param_.N_pl2pl; i++) {
    tensor_map_enc_mask_r2dst_pl2pl_data[i].resize(tensor_param_.N_pl, 0.0);
    std::fill(tensor_map_enc_mask_r2dst_pl2pl_data[i].begin(), tensor_map_enc_mask_r2dst_pl2pl_data[i].end(), 0.0);
  }

  // map pl2pl edge
  for (int i = 0; i <valid_polygon_size;i++){
    for (int j = i+1; j <valid_polygon_size;j++) {
      // map connect
      std::vector<int> first_ids;
      std::vector<int> second_ids;
      first_ids.clear();
      second_ids.clear();
      std::vector<PL2PLType> types;
      types.clear();
//      PL2PLType pl_2_pl_type = PL2PL_NONE;
      int raw_i = polygon_list_[i].raw_id();
      int raw_j = polygon_list_[j].raw_id();
      for (auto &item : pl2pl_list_){
        if (item.first.first == raw_i && item.first.second ==raw_j){
          first_ids.emplace_back(i);
          second_ids.emplace_back(j);
          types.emplace_back(item.second);
        }
        if (item.first.first == raw_j && item.first.second ==raw_i){
          first_ids.emplace_back(j);
          second_ids.emplace_back(i);
          types.emplace_back(item.second);
        }
      }
      // dis connect
      if (first_ids.empty() || second_ids.empty()){
        float vec_x = polygon_list_[i].polygon_position().data()[0][0] - polygon_list_[j].polygon_position().data()[0][0];
        float vec_y = polygon_list_[i].polygon_position().data()[0][1] - polygon_list_[j].polygon_position().data()[0][1];
        if (std::hypot(vec_x,vec_y) > param_.pl2pl_radius){
          continue;
        }else{
          first_ids.emplace_back(i);
          second_ids.emplace_back(j);
          types.emplace_back(PL2PL_NONE);;
        }
      }
      // dis < pl2pl_radius or map connect
      for (int d = 0 ; d < first_ids.size(); d++){
        auto first_id = first_ids[d];
        auto second_id = second_ids[d];
        auto pl_2_pl_type = types[d];
        tensor_map_enc_edge_index_pl2pl_data[0][tensor_map_enc_num_pl2pl_data[0]] = first_id;
        tensor_map_enc_edge_index_pl2pl_data[1][tensor_map_enc_num_pl2pl_data[0]] = second_id;
        tensor_map_enc_mask_r2src_pl2pl_data[tensor_map_enc_num_pl2pl_data[0]][first_id] = 1.0;
        tensor_map_enc_mask_r2dst_pl2pl_data[tensor_map_enc_num_pl2pl_data[0]][second_id] = 1.0;
        tensor_map_enc_type_pl2pl_data[tensor_map_enc_num_pl2pl_data[0]] = pl_2_pl_type;
        float vec_x = polygon_list_[first_id].polygon_position().data()[0][0] - polygon_list_[second_id].polygon_position().data()[0][0];
        float vec_y = polygon_list_[first_id].polygon_position().data()[0][1] - polygon_list_[second_id].polygon_position().data()[0][1];
        float cos_heading = std::cos(polygon_list_[second_id].polygon_orientation().data()[0]);
        float sin_heading = std::sin(polygon_list_[second_id].polygon_orientation().data()[0]);
        tensor_map_enc_r_pl2pl_data[tensor_map_enc_num_pl2pl_data[0]][0] = std::hypot(vec_x,vec_y);
        tensor_map_enc_r_pl2pl_data[tensor_map_enc_num_pl2pl_data[0]][1] = angle_between_2d_vectors(cos_heading,sin_heading,vec_x,vec_y);
        tensor_map_enc_r_pl2pl_data[tensor_map_enc_num_pl2pl_data[0]][2] = normalize_angle(polygon_list_[first_id].polygon_orientation().data()[0] - polygon_list_[second_id].polygon_orientation().data()[0]);
        tensor_map_enc_num_pl2pl_data[0]++;
      }
    }
  }
  // fill pl2pl edge
  if (tensor_map_enc_num_pl2pl_data[0] > 0){
    for (int i = tensor_map_enc_num_pl2pl_data[0]; i < tensor_param_.N_pl2pl; i++){
      tensor_map_enc_edge_index_pl2pl_data[0][i] = tensor_map_enc_edge_index_pl2pl_data[0][0];
      tensor_map_enc_edge_index_pl2pl_data[1][i] = tensor_map_enc_edge_index_pl2pl_data[1][0];
      tensor_map_enc_type_pl2pl_data[i] = tensor_map_enc_type_pl2pl_data[0];
      tensor_map_enc_r_pl2pl_data[i][0] = tensor_map_enc_r_pl2pl_data[0][0];
      tensor_map_enc_r_pl2pl_data[i][1] = tensor_map_enc_r_pl2pl_data[0][1];
      tensor_map_enc_r_pl2pl_data[i][2] = tensor_map_enc_r_pl2pl_data[0][2];
    }
  }

}
}
