//
// Created by SENSETIME\fengxiaotong on 24-8-9.
//

#pragma once
#include "vector"
#include "algorithm"
#include "unordered_map"
#include "memory"
#include "data_preprocess/train_data_type.h"
#include "data_preprocess/map_data_preprocess.h"

namespace DLP {

enum AgentInfoMode {
  Train = 0,
  Predict = 1
};

struct AgentDataParam {
  int history_num = 50;          // Number of historical frames per agent (default = 50)
  int predict_num = 60;          // Number of prediction frames per agent (default = 60)
  int multi_predict_num = 2;     // Number of predicted trajectories per agent (default = 2)
  int agent_max_num = 15;        // Maximum number of agents including ego (default = 15)
  int map_polygon_num = 30;      // Number of map polygons (default = 30)
  int time_span = 10;            // Number of agent encoder span (default = 10)
  double a2a_radius = 50;        // distance of agent to agent (default = 50)
  int num_t2m_steps = 30;        // Number of t2m steps (default = 30)
  AgentInfoMode mode = Predict;  // Mode of agent information (default = Predict)
  double a2m_radius = 150;       // distance of agent to agent (default = 150)
  double pl2a_radius = 50;       // distance of agent to polygon (default = 50)
  double pl2m_radius = 150;      // distance of current agent to polygon (default = 150)

  double lat_dis_consider = 4.0;
  double lon_dis_consider = 16.0;
};

class AgentTensorParam {
 public:
  AgentTensorParam(AgentDataParam &param) {
    // N_pl: Number of map polygons (default = 30)
    N_pl = param.map_polygon_num;
    // N_a: Number of agents including ego (default = 15)
    N_a = param.agent_max_num;
    // N_h: Number of historical frames per agent (default = 50)
    N_h = param.history_num;
    // N_p: Number of prediction frames per agent (default = 60)
    N_p = param.predict_num;
    // N_hp: Total number of historical and prediction frames = N_h + N_p (default = 110)
    N_hp = N_h + N_p;
    // N_m: Number of predicted trajectories per agent (default = 2)
    N_m = param.multi_predict_num;
    // N_t: Number of interactions within the agent's history = N_a * N_h * N_h (default = 15 * 50 * 50 = 37500)
    N_t = N_a * N_h * (N_h + 1) / 2;
    // N_pl2a: Number of interactions between the agent and the map during history = N_a * N_h * N_pl (default = 15 * 50 * 30 = 22500)
    N_pl2a = N_a * N_h * N_pl;
    // N_a2a: Number of interactions between agents = N_a * N_h * N_a (default = 15 * 50 * 15 = 11250)
    N_a2a = N_a * N_h * N_a;
    // N_t2m: Number of agent history trajectories = N_a * N_h (default = 15 * 50 = 750)
    N_t2m = N_a * N_h;
    // N_t2m_multi: Number of historical trajectories for multiple predictions = N_t2m * N_m (default = 750 * 2 = 1500)
    N_t2m_multi = N_t2m * N_m;
    // N_pl2m: Number of predicted future frames per agent = N_a * N_pl (default = 15 * 30 = 450)
    N_pl2m = N_a * N_pl;
    // N_pl2m_multi: Number of predicted future frames for multiple predictions = N_pl2m * N_m (default = 450 * 2 = 900)
    N_pl2m_multi = N_pl2m * N_m;
    // N_a2m: Number of interactions between agents and future frames = N_a * N_a (default = 15 * 15 = 225)
    N_a2m = N_a * N_a;
    // N_a2m_multi: Number of interactions between agents for multiple predictions = N_a2m * N_m (default = 225 * 2 = 450)
    N_a2m_multi = N_a2m * N_m;
    // N_m2m: Number of mode interactions = N_a * N_m * N_m (default = 15 * 2 * 2 = 60)
    N_m2m = N_a * N_m * N_m;
  };

  int N_a = 0;           // Number of agents including ego
  int N_h = 0;           // Number of historical frames per agent
  int N_p = 0;           // Number of prediction frames per agent
  int N_pl = 0;          // Number of map polygons
  int N_hp = 0;          // Total number of historical and prediction frames
  int N_m = 0;           // Number of predicted trajectories per agent
  int N_t = 0;           // Number of interactions within the agent's history
  int N_pl2a = 0;        // Number of interactions between the agent and the map during history
  int N_a2a = 0;         // Number of interactions between agents
  int N_t2m = 0;         // Number of agent history trajectories
  int N_t2m_multi = 0;   // Number of historical trajectories for multiple predictions
  int N_pl2m = 0;        // Number of predicted future frames per agent
  int N_pl2m_multi = 0;  // Number of predicted future frames for multiple predictions
  int N_a2m = 0;         // Number of interactions between agents and future frames
  int N_a2m_multi = 0;   // Number of interactions between agents for multiple predictions
  int N_m2m = 0;         // Number of mode interactions
};

enum AgentType {
  VEHICLE = 0,
  PEDESTRIAN = 1,
  MOTORCYCLIST = 2,
  CYCLIST = 3,
  BUS = 4,
  AV_STRAIGHT = 5,
  AV_LEFT = 6,
  AV_RIGHT = 7,
  OTHERS = 8
};

struct AgentFrameInfo {
  int rel_time_step = 0;
  double position_x = 0;
  double position_y = 0;
  double position_z = 0;
  double heading = 0;
  double velocity = 0;
  double acceleration = 0;
  int id;
  AgentType type;
  bool is_static;
};

class AgentInfo {
 public:
  AgentInfo(AgentDataParam &_param) : param_(_param) {
    historical_info_.clear();
  };
  ~AgentInfo() = default;

  void reset() {
    historical_info_.clear();
  }

  bool update_and_check_exist(std::vector<AgentFrameInfo> &_input);

  const std::vector<AgentFrameInfo> &get_historical_info() const {
    return historical_info_;
  }

  const AgentFrameInfo *now_ptr() const {
    for (const auto &info : historical_info_) {
      if (info.rel_time_step == 0) {
        return &info;
      }
    }
    return nullptr;
  }

  const AgentFrameInfo *rel_time_ptr(int time) const {
    for (const auto &info : historical_info_) {
      if (info.rel_time_step == time) {
        return &info;
      }
    }
    return nullptr;
  }

  void create(AgentType &agent_type, int id);

  AgentType agent_type() {
    return type_;
  }

 private:
  std::vector<AgentFrameInfo> historical_info_;
  AgentType type_;
  int id_;
  bool should_preform_predict_;
  AgentDataParam param_;

};

class AgentOutputTensor {
 public:
  explicit AgentOutputTensor(const AgentTensorParam &param_) {
    agent_valid_mask_.create(param_.N_a, param_.N_hp);       // [N_a, N_hp] @bool
    agent_type_.create(param_.N_a);                          // [N_a] @uint8_t
    agent_position_.create(param_.N_a, param_.N_hp, 3);      // [N_a, N_hp, 3] @float32
    agent_heading_.create(param_.N_a, param_.N_hp);          // [N_a, N_hp] @float32
    agent_velocity_.create(param_.N_a, param_.N_hp, 3);      // [N_a, N_hp, 3] @float32
    agent_enc_x_a_.create(param_.N_a, param_.N_h, 4);        // [N_a, N_h, 4] @float32
    agent_enc_num_t_.create(1);                              // [1] @int64
    agent_enc_edge_index_t_.create(2, param_.N_t);           // [2, N_t] @int64
    agent_enc_r_t_.create(param_.N_t, 4);                     // [N_t, 4] @float32
    agent_enc_num_a2a_.create(1);                             // [1] @int64
    agent_enc_edge_index_a2a_.create(2,param_.N_a2a);        // [2, N_a2a] @int64
    agent_enc_r_a2a_.create(param_.N_a2a,3);                  // [N_a2a, 3] @float32
    agent_predict_mask_.create(param_.N_a, param_.N_hp);     // [N_a, N_hp] @bool
    dec_num_t2m_.create(1);                                 // [1] @int64
    dec_edge_index_t2m_.create(2,param_.N_t2m);           // [2, N_t2m] @int64
    dec_r_t2m_.create(param_.N_t2m,4);                   // [N_t2m, 4] @float32
    dec_num_t2m_multi_.create(1);                           // [1] @int64
    dec_edge_index_t2m_multi_.create(2, param_.N_t2m * param_.N_m);    // [2, N_t2m] @int64
    dec_num_a2m_.create(1);                                      // [1] @int64
    dec_edge_index_a2m_.create(2,param_.N_a2m);               // [2, N_a2m] @int64
    dec_r_a2m_.create(param_.N_a2m,3);                        // [N_a2m, 3] @float32
    dec_num_a2m_multi_.create(1);                           // [1] @int64
    dec_edge_index_a2m_multi_.create(2, param_.N_a2m_multi);   // [2, N_a2m_multi]
    dec_num_m2m_.create(1);  // [1] @int64
    dec_edge_index_m2m_.create(2,param_.N_m2m);             // [2, N_m2m] @int64
    agent_enc_num_pl2a_.create(1);                            // [1] @int64
    agent_enc_edge_index_pl2a_.create(2,param_.N_pl2a);     // [2, N_pl2a] @int64
    agent_enc_r_pl2a_.create(param_.N_pl2a,3);                // [N_pl2a, 3] @float32
    dec_num_pl2m_multi_.create(1);                               // [1] @int64
    dec_edge_index_pl2m_multi_.create(2, param_.N_pl2m_multi);      // [2, N_pl2m_multi] @int64
    dec_r_pl2m_.create(param_.N_pl2m,3);                // [N_pl2m, 3] @float32
  }


  TensorD2<bool> &get_agent_valid_mask() {
    return agent_valid_mask_;
  }

  TensorD1<int32_t> &get_agent_type() {
    return agent_type_;
  }

  TensorD3<float> &get_agent_position() {
    return agent_position_;
  }

  TensorD2<float> &get_agent_heading() {
    return agent_heading_;
  }

  TensorD3<float> &get_agent_velocity() {
    return agent_velocity_;
  }

  TensorD3<float> &get_agent_enc_x_a() {
    return agent_enc_x_a_;
  }

  TensorD1<int> &get_agent_enc_num_t() {
    return agent_enc_num_t_;
  }

  TensorD2<int64_t> &get_agent_enc_edge_index_t() {
    return agent_enc_edge_index_t_;
  }

  TensorD2<float> &get_agent_enc_r_t() {
    return  agent_enc_r_t_;
  }

  TensorD1<int> &get_agent_enc_num_a2a() {
    return agent_enc_num_a2a_;
  }

  TensorD2<int64_t> &get_agent_enc_edge_index_a2a() {
    return  agent_enc_edge_index_a2a_;
  }

  TensorD2<float> &get_agent_enc_r_a2a() {
    return  agent_enc_r_a2a_;
  }

  TensorD2<bool> &get_agent_predict_mask() {
    return agent_predict_mask_;
  }

  TensorD1<int> &get_dec_num_t2m() {
    return dec_num_t2m_;
  }

  TensorD2<int> &get_dec_edge_index_t2m() {
    return dec_edge_index_t2m_;
  }

  TensorD2<float> &get_dec_r_t2m() {
    return dec_r_t2m_;
  }

  TensorD1<int> &get_dec_num_t2m_multi() {
    return dec_num_t2m_multi_;
  }

  TensorD2<int64_t> &get_dec_edge_index_t2m_multi() {
    return dec_edge_index_t2m_multi_;
  }

  TensorD1<int> &get_dec_num_a2m() {
    return dec_num_a2m_;
  }

  TensorD2<int> &get_dec_edge_index_a2m() {
    return dec_edge_index_a2m_;
  }

  TensorD2<float> &get_dec_r_a2m() {
    return dec_r_a2m_;
  }

  TensorD1<int> &get_dec_num_a2m_multi() {
    return dec_num_a2m_multi_;
  }

  TensorD2<int64_t> &get_dec_edge_index_a2m_multi() {
    return dec_edge_index_a2m_multi_;
  }

  TensorD1<int> &get_dec_num_m2m() {
    return dec_num_m2m_;
  }

  TensorD2<int64_t> &get_dec_edge_index_m2m() {
    return dec_edge_index_m2m_;
  }

  TensorD1<int> &get_agent_enc_num_pl2a() {
    return agent_enc_num_pl2a_;
  }

  TensorD2<int64_t> &get_agent_enc_edge_index_pl2a() {
    return agent_enc_edge_index_pl2a_;
  }

  TensorD2<float> &get_agent_enc_r_pl2a() {
    return agent_enc_r_pl2a_;
  }

  TensorD1<int> &get_dec_num_pl2m_multi() {
    return dec_num_pl2m_multi_;
  }

  TensorD2<int64_t> &get_dec_edge_index_pl2m_multi() {
    return dec_edge_index_pl2m_multi_;
  }

  TensorD2<float> &get_dec_r_pl2m() {
    return dec_r_pl2m_;
  }

  int& get_valid_num() {
    return valid_num_;
  }

 private:
  int valid_num_ = 0;
  // encoder
  TensorD2<bool> agent_valid_mask_;              // [N_a, N_h] @bool
  TensorD2<bool> agent_predict_mask_;            // [N_a, N_hp] @bool
  TensorD1<int32_t> agent_type_;                 // [N_a] @uint8_t
  TensorD3<float> agent_position_;               // [N_a, N_hp, 3] @float32
  TensorD2<float> agent_heading_;                // [N_a, N_hp] @float32
  TensorD3<float> agent_velocity_;               // [N_a, N_hp, 3] @float32
  TensorD3<float> agent_enc_x_a_;                // [N_a, N_h, 4] @float32
  TensorD1<int> agent_enc_num_t_;                // [1] @int64
  TensorD2<int64_t> agent_enc_edge_index_t_;         // [2, N_t] @int64
  TensorD2<float> agent_enc_r_t_;                // [N_t, 4] @float32
  TensorD1<int> agent_enc_num_a2a_;              // [1] @int64
  TensorD2<int64_t> agent_enc_edge_index_a2a_;       // [2, N_a2a] @int64
  TensorD2<float> agent_enc_r_a2a_;              // [N_a2a, 3] @float32
  TensorD1<int> agent_enc_num_pl2a_;             // [1] @int64
  TensorD2<int64_t> agent_enc_edge_index_pl2a_;      // [2, N_pl2a] @int64
  TensorD2<float> agent_enc_r_pl2a_;             // [N_pl2a, 3] @float32

  // decoder
  TensorD1<int> dec_num_t2m_;                    // [1] @int64
  TensorD2<int> dec_edge_index_t2m_;             // [2, N_t2m] @int64
  TensorD2<float> dec_r_t2m_;                    // [N_t2m, 4] @float32
  TensorD1<int> dec_num_t2m_multi_;              // [1] @int64
  TensorD2<int64_t> dec_edge_index_t2m_multi_;       // [2, N_t2m] @int64
  TensorD1<int> dec_num_a2m_;                    // [1] @int64
  TensorD2<int> dec_edge_index_a2m_;             // [2, N_a2m] @int64
  TensorD2<float> dec_r_a2m_;                    // [N_a2m, 3] @float32
  TensorD1<int> dec_num_a2m_multi_;              // [1] @int64
  TensorD2<int64_t> dec_edge_index_a2m_multi_;       // [2, N_a2m_multi]
  TensorD1<int> dec_num_m2m_;                    // [1] @int64
  TensorD2<int64_t> dec_edge_index_m2m_;             // [2, N_m2m] @int64
  TensorD1<int> dec_num_pl2m_multi_;              // [1] @int64
  TensorD2<int64_t> dec_edge_index_pl2m_multi_;       // [2, N_pl2m_multi] @int64
  TensorD2<float> dec_r_pl2m_;                   // [N_pl2m, 3] @float32
};

class AgentInfoManager {
 public:
  explicit AgentInfoManager(AgentDataParam &_param)
      : param_(_param), tensor_param_(_param), output_tensor_(tensor_param_) {
    agent_info_map_.clear();
    map_info_manager_ptr_ = nullptr;
  };
  ~AgentInfoManager() = default;

  void push_agent(AgentFrameInfo &info);

  void update(std::shared_ptr<MapInfoManager> &map_info_manager_ptr);

  bool data_preprocess();

  std::vector<int> find_frame_exist_id(int rel_time_step);

  std::unordered_map<int, AgentInfo> &get_agent_info_map() {
    return agent_info_map_;
  }

  // debug
  std::vector<int> mask_id;

  const AgentOutputTensor &agent_output_tensor() const {
    return output_tensor_;
  }

 private:
  std::unordered_map<int, AgentInfo> agent_info_map_;
  std::unordered_map<int, AgentFrameInfo> current_frame_agent_info_;
  std::vector<int> current_frame_agent_id_;
  AgentDataParam param_;
  AgentTensorParam tensor_param_;
  AgentOutputTensor output_tensor_;
  std::shared_ptr<MapInfoManager> map_info_manager_ptr_;
};

}