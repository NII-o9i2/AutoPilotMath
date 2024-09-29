//
// Created by SENSETIME\fengxiaotong on 24-8-7.
//
#pragma once
#include "data_preprocess/train_data_type.h"

namespace DLP{

// map_polygon_position : [30,3] @float32
// map_polygon_orientation : [30] @float32
// map_polygon_type : [30] @uint8
// map_polygon_is_intersection : [30] @uint8
// map_point_num_nodes : [1] @int32
// map_point_position : [500,3] @float32
// map_point_orientation : [500] @float32
// map_point_magnitude : [500] @float32
// map_point_type : [500] @uint8
// map_point_side : [500] @uint8
// map_point_to_map_polygon_edge_index : [2,500] @int64
// map_enc_num_pl2pl : [1] @int64
// map_enc_edge_index_pl2pl : [2,500] @int64
// map_enc_type_pl2pl : [500] @uint5
// agent_valid_mask : [15,110] @bool
// agent_type : [15] @uint8
// agent_position : [15,110,3] @float32
// agent_heading : [15,110] @float32
// agent_velocity : [15,110,3] @float32
// agent_enc_num_pl2a : [1] @int64
// agent_enc_num_a2a : [1] @int64
// agent_enc_edge_index_pl2a : [2,5000] @int64
// agent_enc_edge_index_a2a : [2,500] @int64
// agent_predict_mask : [15,110] @bool
// dec_num_pl2m : [1] @int64
// dec_num_a2m : [1] @int64
// dec_num_pl2m_multi : [1] @int64
// dec_num_a2m_multi : [1] @int64
// dec_edge_index_pl2m : [2,500] @int64
// dec_edge_index_pl2m_multi : [2,3000] @int64
// dec_edge_index_a2m : [2,500] @int64
// dec_edge_index_a2m_multi : [2,3000] @int64

struct TrainDataSetParam{
  size_t time_num = 110;

};

class TrainDataFeature{
 public:
  TrainDataFeature() = default;
  ~TrainDataFeature() = default;

  // load agent data

  // load map data

  // get feature

 private:
  size_t feature_num = 110;


};

}