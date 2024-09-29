//
// Created by SENSETIME\fengxiaotong on 24-8-9.
//
#include "data_preprocess/agent_data_preprocess.h"
#include "cmath"
#include "iostream"

namespace DLP {

void AgentInfo::create(DLP::AgentType &agent_type, int id) {
  type_ = agent_type;
  id_ = id;
}

bool AgentInfo::update_and_check_exist(std::vector<AgentFrameInfo> &_input) {
  // 1. update latest data
  if (!_input.empty()) {
    auto latest_frame = _input.front();
    latest_frame.rel_time_step = 1;
    historical_info_.emplace_back(latest_frame);
  }
  // 2. update history info
  for (auto &info : historical_info_) {
    info.rel_time_step--;
  }

  auto remove_old_history = [&](int history_num) {
    historical_info_.erase(
        std::remove_if(historical_info_.begin(), historical_info_.end(),
                       [&](const AgentFrameInfo &info) {
                         return info.rel_time_step <= -history_num;
                       }),
        historical_info_.end());
  };

  if (param_.mode == Predict) {
    remove_old_history(param_.history_num);
  } else if (param_.mode == Train) {
    remove_old_history(param_.history_num + param_.predict_num);
  }

  // 3. check exist

  return !historical_info_.empty();
}

void AgentInfoManager::push_agent(DLP::AgentFrameInfo &info) {
  current_frame_agent_info_.insert(
      std::pair<int, AgentFrameInfo>(info.id, info));
}

std::vector<int> AgentInfoManager::find_frame_exist_id(int rel_time_step) {
  std::vector<int> id_list;
  id_list.clear();
  for (auto &agent_item : agent_info_map_) {
    auto &historical_info = agent_item.second.get_historical_info();
    for (auto &info : historical_info) {
      if (info.rel_time_step == rel_time_step) {
        id_list.emplace_back(agent_item.first);
        break;
      }
    }
  }
  return id_list;
}

void AgentInfoManager::update(std::shared_ptr<MapInfoManager> &map_info_manager_ptr) {
  map_info_manager_ptr_ = map_info_manager_ptr;
  if (current_frame_agent_info_.size() == 1 && current_frame_agent_info_.find(-1) !=
      current_frame_agent_info_.end()){
    auto& ego_info = current_frame_agent_info_.at(-1);
    double fake_distance = param_.a2a_radius - 0.1;
    AgentFrameInfo virtual_agent;
    virtual_agent.rel_time_step = 0;
    virtual_agent.position_x = ego_info.position_x - fake_distance * std::cos(ego_info.heading);
    virtual_agent.position_y = ego_info.position_y - fake_distance * std::sin(ego_info.heading);
    virtual_agent.position_z = 0.0;
    virtual_agent.heading = ego_info.heading;
    virtual_agent.velocity = 0.0;
    virtual_agent.acceleration = 0.0;
    virtual_agent.id = int(-2);
    virtual_agent.type = AgentType::VEHICLE;
    current_frame_agent_info_.insert(
      std::pair<int, AgentFrameInfo>(virtual_agent.id, virtual_agent));
  }
  // 1. update remain agent info & erase current agent
  current_frame_agent_id_.clear();
  for (auto &info : current_frame_agent_info_) {
    current_frame_agent_id_.emplace_back(info.first);
  }
  std::vector<AgentFrameInfo> tmp;

  for (auto it = agent_info_map_.begin(); it != agent_info_map_.end();) {
    auto agent_id = it->first;
    auto &agent_info = it->second;
    tmp.clear();
    // can not find refresh agent
    if (current_frame_agent_info_.find(agent_id) ==
        current_frame_agent_info_.end()) {
      bool exist = agent_info.update_and_check_exist(tmp);
      if (!exist) {
        it = agent_info_map_.erase(it);
        continue;
      }
    } else {
      // can find refresh agent
      tmp.emplace_back(current_frame_agent_info_[agent_id]);
      agent_info.update_and_check_exist(tmp);
      current_frame_agent_info_.erase(agent_id);
    }
    ++it; // only for exist agent
  }

  // 2. create agent info for remain current agent
  for (auto it = current_frame_agent_info_.begin();
       it != current_frame_agent_info_.end();
       it = current_frame_agent_info_.erase(it)) {
    auto new_agent_id = it->first;
    auto &new_agent_info = it->second;
    AgentInfo new_agent(param_);
    new_agent.create(new_agent_info.type, new_agent_id);
    agent_info_map_.insert(std::pair<int, AgentInfo>(new_agent_id, new_agent));
    tmp.clear();
    tmp.emplace_back(new_agent_info);
    auto &agent_info = agent_info_map_.at(new_agent_id);
    agent_info.update_and_check_exist(tmp);
  }
}

bool AgentInfoManager::data_preprocess() {
  // 1. find predict time info
  std::pair<int, int> history_rel_time;
  if (param_.mode == Predict) {
    history_rel_time.first = -49;
    history_rel_time.second = 0;
  }
  if (param_.mode == Train) {
    history_rel_time.first = -109;
    history_rel_time.second = -60;
  }

  std::vector<int> predict_id;
  predict_id.clear();
  if (param_.mode == Predict) {
    predict_id = current_frame_agent_id_;
  } else {
    predict_id = find_frame_exist_id(history_rel_time.second);
  }
  // 2. find ego state at predict time
  if (param_.mode == Predict &&
      (std::find(current_frame_agent_id_.begin(), current_frame_agent_id_.end(),
                 -1) == current_frame_agent_id_.end())) {
    return false;
  }
  if (agent_info_map_.find(-1) == agent_info_map_.end()) {
    return false;
  }
  auto &ego_info = agent_info_map_.at(-1);
  auto ego_predict_ptr = ego_info.rel_time_ptr(history_rel_time.second);
  if (ego_predict_ptr == nullptr) {
    return false;
  }

  // 2. update agent of interest
  std::vector<std::pair<int, double>> agent_ego_dis;
  agent_ego_dis.clear();
  double step_lon =
      std::max(ego_predict_ptr->velocity, param_.lon_dis_consider);
  for (auto &current_id : predict_id) {
    auto &agent_info = agent_info_map_.at(current_id);
    auto agent_pred_ptr = agent_info.rel_time_ptr(history_rel_time.second);
    if (agent_pred_ptr == nullptr) {
      return false;
    }
    double vec_x = agent_pred_ptr->position_x - ego_predict_ptr->position_x;
    double vec_y = agent_pred_ptr->position_y - ego_predict_ptr->position_y;
    double cos_ego = std::cos(ego_predict_ptr->heading);
    double sin_ego = std::sin(ego_predict_ptr->heading);
    double tmp_lon_dis = (cos_ego * vec_x + sin_ego * vec_y) / step_lon;
    double tmp_lat_dis =
        (sin_ego * vec_x - cos_ego * vec_y) / param_.lat_dis_consider;
//    std::cout << " id " << current_id << "pos_x " << agent_pred_ptr->position_x
//              << "pos_y " << agent_pred_ptr->position_y << "heading "
//              << agent_pred_ptr->heading << " lon " << tmp_lon_dis << " lat "
//              << tmp_lat_dis << std::endl;
    agent_ego_dis.emplace_back(current_id, tmp_lon_dis * tmp_lon_dis +
        tmp_lat_dis * tmp_lat_dis);
  }

  auto compare_by_second = [](const std::pair<int, double> &a,
                              const std::pair<int, double> &b) {
    return a.second < b.second;
  };

  std::sort(agent_ego_dis.begin(), agent_ego_dis.end(), compare_by_second);

  int mask_num = std::min(param_.agent_max_num, (int) agent_ego_dis.size());

  mask_id.clear();
  for (auto &it : agent_ego_dis) {
    mask_id.emplace_back(it.first);
    if (mask_id.size() >= mask_num) {
      break;
    }
  }
  // 3. update tensor
  auto &tensor_agent_mask = output_tensor_.get_agent_valid_mask();
  auto &tensor_agent_mask_data = tensor_agent_mask.data();
  auto &tensor_agent_type = output_tensor_.get_agent_type();
  auto &tensor_agent_type_data = tensor_agent_type.data();
  auto &tensor_agent_position = output_tensor_.get_agent_position();
  auto &tensor_agent_position_data = tensor_agent_position.data();
  auto &tensor_agent_heading = output_tensor_.get_agent_heading();
  auto &tensor_agent_heading_data = tensor_agent_heading.data();
  auto &tensor_agent_velocity = output_tensor_.get_agent_velocity();
  auto &tensor_agent_velocity_data = tensor_agent_velocity.data();

  auto &tensor_agent_x_a = output_tensor_.get_agent_enc_x_a();
  auto &tensor_agent_x_a_data = tensor_agent_x_a.data();

  auto &tensor_agent_enc_edge_index_t =
      output_tensor_.get_agent_enc_edge_index_t();
  auto &tensor_agent_enc_edge_index_t_data =
      tensor_agent_enc_edge_index_t.data();
  tensor_agent_enc_edge_index_t_data[0].resize(tensor_param_.N_t, 0);
  tensor_agent_enc_edge_index_t_data[1].resize(tensor_param_.N_t, 0);

  auto &tensor_agent_enc_num_t = output_tensor_.get_agent_enc_num_t();
  auto &tensor_agent_enc_num_t_data = tensor_agent_enc_num_t.data();
  tensor_agent_enc_num_t_data[0] = 0;

  auto &tensor_agent_enc_r_t = output_tensor_.get_agent_enc_r_t();
  auto &tensor_agent_enc_r_t_data = tensor_agent_enc_r_t.data();
  for (int i = 0; i < tensor_param_.N_t; i++) {
    tensor_agent_enc_r_t_data[i].resize(4, 0.0);
  }

  auto &tensor_agent_predict_mask = output_tensor_.get_agent_predict_mask();
  auto &tensor_agent_predict_mask_data = tensor_agent_predict_mask.data();

  auto &tensor_dec_num_t2m = output_tensor_.get_dec_num_t2m();
  auto &tensor_dec_num_t2m_data = tensor_dec_num_t2m.data();
  tensor_dec_num_t2m_data[0] = 0;

  auto &tensor_dec_num_t2m_multi = output_tensor_.get_dec_num_t2m_multi();
  auto &tensor_dec_num_t2m_multi_data = tensor_dec_num_t2m_multi.data();
  tensor_dec_num_t2m_multi_data[0] = 0;

  auto &tensor_dec_edge_index_t2m_multi =
      output_tensor_.get_dec_edge_index_t2m_multi();
  auto &tensor_dec_edge_index_t2m_multi_data =
      tensor_dec_edge_index_t2m_multi.data();
  tensor_dec_edge_index_t2m_multi_data[0].resize(
      tensor_param_.N_t2m * tensor_param_.N_m, 0);
  tensor_dec_edge_index_t2m_multi_data[0].resize(
      tensor_param_.N_t2m * tensor_param_.N_m, 0);

  auto &tensor_dec_edge_index_t2m = output_tensor_.get_dec_edge_index_t2m();
  auto &tensor_dec_edge_index_t2m_data = tensor_dec_edge_index_t2m.data();
  tensor_dec_edge_index_t2m_data[0].resize(tensor_param_.N_t2m, 0);
  tensor_dec_edge_index_t2m_data[1].resize(tensor_param_.N_t2m, 0);

  auto &tensor_dec_r_t2m = output_tensor_.get_dec_r_t2m();
  auto &tensor_dec_r_t2m_data = tensor_dec_r_t2m.data();
  for (int i = 0; i < tensor_param_.N_t2m; i++) {
    tensor_dec_r_t2m_data[i].resize(4, 0.0);
  }

  auto &tensor_dec_num_a2m = output_tensor_.get_dec_num_a2m();
  auto &tensor_dec_num_a2m_data = tensor_dec_num_a2m.data();
  tensor_dec_num_a2m_data[0] = 0;

  auto &tensor_dec_edge_index_a2m = output_tensor_.get_dec_edge_index_a2m();
  auto &tensor_dec_edge_index_a2m_data = tensor_dec_edge_index_a2m.data();
  tensor_dec_edge_index_a2m_data[0].resize(tensor_param_.N_a2m, 0);
  tensor_dec_edge_index_a2m_data[1].resize(tensor_param_.N_a2m, 0);

  auto &tensor_dec_r_a2m = output_tensor_.get_dec_r_a2m();
  auto &tensor_dec_r_a2m_data = tensor_dec_r_a2m.data();
  for (int i = 0; i < tensor_param_.N_a2m; i++) {
    tensor_dec_r_a2m_data[i].resize(3, 0.0);
  }
  auto &tensor_dec_num_a2m_multi = output_tensor_.get_dec_num_a2m_multi();
  auto &tensor_dec_num_a2m_multi_data = tensor_dec_num_a2m_multi.data();
  tensor_dec_num_a2m_multi_data[0] = 0;
  auto &tensor_dec_edge_index_a2m_multi =
      output_tensor_.get_dec_edge_index_a2m_multi();
  auto &tensor_dec_edge_index_a2m_multi_data =
      tensor_dec_edge_index_a2m_multi.data();
  tensor_dec_edge_index_a2m_multi_data[0].resize(tensor_param_.N_a2m_multi, 0);
  tensor_dec_edge_index_a2m_multi_data[1].resize(tensor_param_.N_a2m_multi, 0);

  auto &tensor_dec_num_m2m = output_tensor_.get_dec_num_m2m();
  auto &tensor_dec_num_m2m_data = tensor_dec_num_m2m.data();
  tensor_dec_num_m2m_data[0] = 0;

  auto &tensor_dec_edge_index_m2m = output_tensor_.get_dec_edge_index_m2m();
  auto &tensor_dec_edge_index_m2m_data = tensor_dec_edge_index_m2m.data();
  tensor_dec_edge_index_m2m_data[0].resize(tensor_param_.N_m2m, 0);
  tensor_dec_edge_index_m2m_data[1].resize(tensor_param_.N_m2m, 0);

  auto& valid_num = output_tensor_.get_valid_num();
  valid_num = mask_num;

  auto rel_time_num =
      param_.mode == Train ? tensor_param_.N_hp : tensor_param_.N_h;
  for (int i = 0; i < tensor_agent_mask.shape()[0]; i++) {
    // reset false
    for (int j = 0; j < tensor_param_.N_hp; j++) {
      tensor_agent_mask_data[i][j] = false;
      tensor_agent_predict_mask_data[i][j] = false;
      tensor_agent_type_data[i] = OTHERS;
      tensor_agent_position_data[i][j].resize(3, 0.0);
      std::fill(tensor_agent_position_data[i][j].begin(),tensor_agent_position_data[i][j].end(),0.0);
      tensor_agent_heading_data[i][j] = 0.0;
      tensor_agent_velocity_data[i][j].resize(3, 0.0);
      std::fill(tensor_agent_velocity_data[i][j].begin(),tensor_agent_velocity_data[i][j].end(),0.0);
    }
    for (int j = 0; j < tensor_param_.N_h; j++) {
      tensor_agent_x_a_data[i][j].resize(4, 0.0);
      std::fill(tensor_agent_x_a_data[i][j].begin(),tensor_agent_x_a_data[i][j].end(),0.0);
    }
    // mask invalid agent
    if (i >= mask_num) {
      continue;
    }

    // valid agent mask
    auto mask_id_i = mask_id[i];
    auto &mask_id_info = agent_info_map_.at(mask_id_i).get_historical_info();
    for (auto &his_info : mask_id_info) {
      int index_j = his_info.rel_time_step + rel_time_num - 1;
      if (index_j >= tensor_param_.N_h) {
        tensor_agent_predict_mask_data[i][index_j] = true;
      }
      tensor_agent_mask_data[i][index_j] = true;
      tensor_agent_position_data[i][index_j][0] =
          static_cast<float>(his_info.position_x);
      tensor_agent_position_data[i][index_j][1] =
          static_cast<float>(his_info.position_y);
      //      tensor_agent_position_data[i][index_j][2] = 0.0;
      tensor_agent_heading_data[i][index_j] =
          normalize_angle(static_cast<float>(his_info.heading));
      tensor_agent_velocity_data[i][index_j][0] =
          static_cast<float>(std::cos(his_info.heading) * his_info.velocity);
      tensor_agent_velocity_data[i][index_j][1] =
          static_cast<float>(std::sin(his_info.heading) * his_info.velocity);
      tensor_agent_velocity_data[i][index_j][2] = 0.0;
    }
    for (int j = static_cast<int>(tensor_agent_mask.shape()[1]) - 1; j > 0; j--) {
      tensor_agent_mask_data[i][j] =
          tensor_agent_mask_data[i][j] && tensor_agent_mask_data[i][j - 1];
    }
    tensor_agent_mask_data[i][0] = false;
    if (param_.mode == Predict) {
      for (int j = tensor_param_.N_h; j < tensor_param_.N_hp; j++) {
        tensor_agent_predict_mask_data[i][j] = true;
      }
    }
    // // type
    tensor_agent_type_data[i] = agent_info_map_.at(mask_id_i).agent_type();
    // encoder x_a & decoder t2m
    auto pos_m_x = tensor_agent_position_data[i][param_.history_num - 1][0];
    auto pos_m_y = tensor_agent_position_data[i][param_.history_num - 1][1];
    auto head_m = tensor_agent_heading_data[i][param_.history_num - 1];
    auto head_m_cos =
        std::cos(tensor_agent_heading_data[i][param_.history_num - 1]);
    auto head_m_sin =
        std::sin(tensor_agent_heading_data[i][param_.history_num - 1]);
    for (int j = 0; j < param_.history_num; j++) {
      // pos step length
      if (j == 0) {
        tensor_agent_x_a_data[i][j][0] = 0.0;
        tensor_agent_x_a_data[i][j][1] = 0.0;
        tensor_agent_x_a_data[i][j][2] =
            std::sqrt(tensor_agent_velocity_data[i][j][0] *
                tensor_agent_velocity_data[i][j][0] +
                tensor_agent_velocity_data[i][j][1] *
                    tensor_agent_velocity_data[i][j][1]);
        tensor_agent_x_a_data[i][j][3] = 0.0;
      } else {
        auto tmp_x = tensor_agent_position_data[i][j][0] -
            tensor_agent_position_data[i][j - 1][0];
        auto tmp_y = tensor_agent_position_data[i][j][1] -
            tensor_agent_position_data[i][j - 1][1];
        tensor_agent_x_a_data[i][j][0] =
            std::sqrt(tmp_x * tmp_x + tmp_y * tmp_y);

        auto cos_heading = std::cos(tensor_agent_heading_data[i][j]);
        auto sin_heading = std::sin(tensor_agent_heading_data[i][j]);
        tensor_agent_x_a_data[i][j][1] =
            angle_between_2d_vectors(cos_heading, sin_heading, tmp_x, tmp_y);

        tensor_agent_x_a_data[i][j][2] =
            std::sqrt(tensor_agent_velocity_data[i][j][0] *
                tensor_agent_velocity_data[i][j][0] +
                tensor_agent_velocity_data[i][j][1] *
                    tensor_agent_velocity_data[i][j][1]);
        tensor_agent_x_a_data[i][j][3] = angle_between_2d_vectors(
            cos_heading, sin_heading, tensor_agent_velocity_data[i][j][0],
            tensor_agent_velocity_data[i][j][1]);
      }
      //   // enc index
      if (tensor_agent_mask_data[i][j]) {
        int init_index = std::max(0, j - param_.time_span);
        for (int k = init_index; k < j; k++) {
          if (tensor_agent_mask_data[i][k]) {
            // agent edge index
            if (tensor_agent_enc_num_t_data[0] < tensor_param_.N_t) {
              tensor_agent_enc_edge_index_t_data
              [0][tensor_agent_enc_num_t_data[0]] =
                  i * tensor_param_.N_h + k;
              tensor_agent_enc_edge_index_t_data
              [1][tensor_agent_enc_num_t_data[0]] =
                  i * tensor_param_.N_h + j;

              //   // agent rel
              auto tmp_vec_x = tensor_agent_position_data[i][k][0] -
                  tensor_agent_position_data[i][j][0];
              auto tmp_vec_y = tensor_agent_position_data[i][k][1] -
                  tensor_agent_position_data[i][j][1];
              auto cos_heading = std::cos(tensor_agent_heading_data[i][j]);
              auto sin_heading = std::sin(tensor_agent_heading_data[i][j]);

              tensor_agent_enc_r_t_data[tensor_agent_enc_num_t_data[0]][0] =
                  std::hypot(tmp_vec_x, tmp_vec_y);
              tensor_agent_enc_r_t_data[tensor_agent_enc_num_t_data[0]][1] =
                  angle_between_2d_vectors(cos_heading, sin_heading, tmp_vec_x,
                                           tmp_vec_y);
              tensor_agent_enc_r_t_data[tensor_agent_enc_num_t_data[0]][2] =
                  normalize_angle(tensor_agent_heading_data[i][k] -
                      tensor_agent_heading_data[i][j]);
              tensor_agent_enc_r_t_data[tensor_agent_enc_num_t_data[0]][3] =
                  static_cast<float>(k - j);
              tensor_agent_enc_num_t_data[0]++;
            }
          }
        }
      } // end edge index
      //   // decoder t2m
      if (tensor_agent_mask_data[i][j] &&
          j >= tensor_param_.N_h - param_.num_t2m_steps) {
        if (tensor_dec_num_t2m_data[0] < tensor_param_.N_t2m) {
          tensor_dec_edge_index_t2m_data[0][tensor_dec_num_t2m_data[0]] =
              i * tensor_param_.N_h + j;
          tensor_dec_edge_index_t2m_data[1][tensor_dec_num_t2m_data[0]] = i;
          // decoder t2m multi
          for (int k = 0; k < tensor_param_.N_m; k++) {
            tensor_dec_edge_index_t2m_multi_data
            [0][tensor_dec_num_t2m_multi_data[0]] =
                i * tensor_param_.N_h + j;
            tensor_dec_edge_index_t2m_multi_data
            [1][tensor_dec_num_t2m_multi_data[0]] =
                i * tensor_param_.N_m + k;
            tensor_dec_num_t2m_multi_data[0]++;
          }
          auto vec_x = tensor_agent_position_data[i][j][0] - pos_m_x;
          auto vec_y = tensor_agent_position_data[i][j][1] - pos_m_y;
          tensor_dec_r_t2m_data[tensor_dec_num_t2m_data[0]][0] =
              std::hypot(vec_x, vec_y);
          tensor_dec_r_t2m_data[tensor_dec_num_t2m_data[0]][1] =
              angle_between_2d_vectors(head_m_cos, head_m_sin, vec_x, vec_y);
          tensor_dec_r_t2m_data[tensor_dec_num_t2m_data[0]][2] =
              normalize_angle(tensor_agent_heading_data[i][j] - head_m);
          tensor_dec_r_t2m_data[tensor_dec_num_t2m_data[0]][3] =
              static_cast<float>(j - tensor_param_.N_h + 1);
          tensor_dec_num_t2m_data[0]++;
        }
      }
    }
  }
  // fill encoder edge index use the front element
  if (tensor_agent_enc_num_t_data[0] > 0) {
    for (int i = tensor_agent_enc_num_t_data[0]; i < tensor_param_.N_t; i++) {
      tensor_agent_enc_edge_index_t_data[0][i] =
          tensor_agent_enc_edge_index_t_data[0][0];
      tensor_agent_enc_edge_index_t_data[1][i] =
          tensor_agent_enc_edge_index_t_data[1][0];

      tensor_agent_enc_r_t_data[i][0] = tensor_agent_enc_r_t_data[0][0];
      tensor_agent_enc_r_t_data[i][1] = tensor_agent_enc_r_t_data[0][1];
      tensor_agent_enc_r_t_data[i][2] = tensor_agent_enc_r_t_data[0][2];
      tensor_agent_enc_r_t_data[i][3] = tensor_agent_enc_r_t_data[0][3];
    }
  }
  // fill encoder edge t2m index use the front element
  if (tensor_dec_num_t2m_data[0] > 0) {
    for (int i = tensor_dec_num_t2m_data[0]; i < tensor_param_.N_t2m; i++) {
      tensor_dec_edge_index_t2m_data[0][i] =
          tensor_dec_edge_index_t2m_data[0][0];
      tensor_dec_edge_index_t2m_data[1][i] =
          tensor_dec_edge_index_t2m_data[1][0];

      tensor_dec_r_t2m_data[i][0] = tensor_dec_r_t2m_data[0][0];
      tensor_dec_r_t2m_data[i][1] = tensor_dec_r_t2m_data[0][1];
      tensor_dec_r_t2m_data[i][2] = tensor_dec_r_t2m_data[0][2];
      tensor_dec_r_t2m_data[i][3] = tensor_dec_r_t2m_data[0][3];
    }
  }
  // fill encoder edge multi t2m index use the front element
  if (tensor_dec_num_t2m_multi_data[0] > 0) {
    for (int i = tensor_dec_num_t2m_multi_data[0];
         i < tensor_param_.N_t2m_multi; i++) {
      tensor_dec_edge_index_t2m_multi_data[0][i] =
          tensor_dec_edge_index_t2m_multi_data[0][0];
      tensor_dec_edge_index_t2m_multi_data[1][i] =
          tensor_dec_edge_index_t2m_multi_data[1][0];
    }
  }

  // encoder a2a
  auto &tensor_agent_enc_num_a2a = output_tensor_.get_agent_enc_num_a2a();
  auto &tensor_agent_enc_edge_index_a2a =
      output_tensor_.get_agent_enc_edge_index_a2a();
  auto &tensor_agent_enc_r_a2a = output_tensor_.get_agent_enc_r_a2a();
  auto &tensor_agent_enc_num_a2a_data = tensor_agent_enc_num_a2a.data();
  auto &tensor_agent_enc_edge_index_a2a_data =
      tensor_agent_enc_edge_index_a2a.data();
  auto &tensor_agent_enc_r_a2a_data = tensor_agent_enc_r_a2a.data();
  tensor_agent_enc_num_a2a_data[0] = 0;
  tensor_agent_enc_edge_index_a2a_data[0].resize(tensor_param_.N_a2a, 0);
  tensor_agent_enc_edge_index_a2a_data[1].resize(tensor_param_.N_a2a, 0);
  for (int i = 0; i < tensor_param_.N_a2a; i++) {
    tensor_agent_enc_r_a2a_data[i].resize(3, 0);
  }

  auto& map_tensor_output = map_info_manager_ptr_->get_map_tensor_out();
  auto& pl_points = map_tensor_output.map_polygon_position();
  auto& pl_points_data = pl_points.value();
  auto& pl_orient = map_tensor_output.map_polygon_orientation();
  auto& pl_orient_data = pl_orient.value();
  auto& map_polygon_num = map_tensor_output.get_map_polygon_num().value();
  int valid_pl_num = map_polygon_num[0];

  for (int j = 0; j < tensor_param_.N_h; j++) {
    for (int i = 0; i < tensor_param_.N_a; i++) {
      if (tensor_agent_mask_data[i][j]) {
        for (int k = i + 1; k < tensor_param_.N_a; k++) {
          if (tensor_agent_mask_data[k][j]) {
            auto vec_x = tensor_agent_position_data[i][j][0] -
                tensor_agent_position_data[k][j][0];
            auto vec_y = tensor_agent_position_data[i][j][1] -
                tensor_agent_position_data[k][j][1];
            auto tmp_dis = std::hypot(vec_x, vec_y);
            if (tmp_dis > param_.a2a_radius) {
              continue;
            }
            tensor_agent_enc_edge_index_a2a_data
            [0][tensor_agent_enc_num_a2a_data[0]] =
                j * tensor_param_.N_a + i;
            tensor_agent_enc_edge_index_a2a_data
            [1][tensor_agent_enc_num_a2a_data[0]] =
                j * tensor_param_.N_a + k;
            auto cos_heading = std::cos(tensor_agent_heading_data[k][j]);
            auto sin_heading = std::sin(tensor_agent_heading_data[k][j]);
            tensor_agent_enc_r_a2a_data[tensor_agent_enc_num_a2a_data[0]][0] =
                tmp_dis;
            tensor_agent_enc_r_a2a_data[tensor_agent_enc_num_a2a_data[0]][1] =
                angle_between_2d_vectors(cos_heading, sin_heading, vec_x,
                                         vec_y);
            tensor_agent_enc_r_a2a_data[tensor_agent_enc_num_a2a_data[0]][2] =
                normalize_angle(tensor_agent_heading_data[i][j] -
                    tensor_agent_heading_data[k][j]);
            tensor_agent_enc_num_a2a_data[0]++;
          }
        } // end agent k search
      }
    } // end agent i search
  }
  // fill encoder edge a2a index use the front element
  if (tensor_agent_enc_num_a2a_data[0] > 0) {
    for (int i = tensor_agent_enc_num_a2a_data[0]; i < tensor_param_.N_a2a;
         i++) {
      tensor_agent_enc_edge_index_a2a_data[0][i] =
          tensor_agent_enc_edge_index_a2a_data[0][0];
      tensor_agent_enc_edge_index_a2a_data[1][i] =
          tensor_agent_enc_edge_index_a2a_data[1][0];

      tensor_agent_enc_r_a2a_data[i][0] = tensor_agent_enc_r_a2a_data[0][0];
      tensor_agent_enc_r_a2a_data[i][1] = tensor_agent_enc_r_a2a_data[0][1];
      tensor_agent_enc_r_a2a_data[i][2] = tensor_agent_enc_r_a2a_data[0][2];
    }
  }

  auto& tensor_dec_num_pl2m_multi = output_tensor_.get_dec_num_pl2m_multi();
  auto& tensor_dec_num_pl2m_multi_data = tensor_dec_num_pl2m_multi.data();
  tensor_dec_num_pl2m_multi_data[0] = 0;

  auto& tensor_dec_edge_index_pl2m_multi = output_tensor_.get_dec_edge_index_pl2m_multi();
  auto& tensor_dec_edge_index_pl2m_multi_data = tensor_dec_edge_index_pl2m_multi.data();
  tensor_dec_edge_index_pl2m_multi_data[0].resize(tensor_param_.N_pl2m_multi,0);
  tensor_dec_edge_index_pl2m_multi_data[1].resize(tensor_param_.N_pl2m_multi,0);

  auto& tensor_dec_r_pl2m= output_tensor_.get_dec_r_pl2m();
  auto& tensor_dec_r_pl2m_data = tensor_dec_r_pl2m.data();
  for (int i =0; i < tensor_param_.N_pl2m; i++){
    tensor_dec_r_pl2m_data[i].resize(3, 0.0);
  }

  // decoder agent a2m
  for (int i = 0; i < tensor_param_.N_a; i++) {
    if (tensor_agent_mask_data[i][tensor_param_.N_h - 1]) {
      // a2m
      for (int j = i + 1; j < tensor_param_.N_a; j++) {
        if (tensor_agent_mask_data[j][tensor_param_.N_h - 1]) {
          double vec_x =
              tensor_agent_position_data[i][tensor_param_.N_h - 1][0] -
                  tensor_agent_position_data[j][tensor_param_.N_h - 1][0];
          double vec_y =
              tensor_agent_position_data[i][tensor_param_.N_h - 1][1] -
                  tensor_agent_position_data[j][tensor_param_.N_h - 1][1];
          double dis_tmp = std::hypot(vec_x, vec_y);
          if (dis_tmp < param_.a2m_radius) {
            tensor_dec_edge_index_a2m_data[0][tensor_dec_num_a2m_data[0]] = i;
            tensor_dec_edge_index_a2m_data[1][tensor_dec_num_a2m_data[0]] = j;
            float cos_heading =
                std::cos(tensor_agent_heading_data[j][tensor_param_.N_h - 1]);
            float sin_heading =
                std::sin(tensor_agent_heading_data[j][tensor_param_.N_h - 1]);
            tensor_dec_r_a2m_data[tensor_dec_num_a2m_data[0]][0] =
                static_cast<float>(dis_tmp);
            tensor_dec_r_a2m_data[tensor_dec_num_a2m_data[0]][1] =
                angle_between_2d_vectors(cos_heading, sin_heading, vec_x,
                                         vec_y);
            tensor_dec_r_a2m_data[tensor_dec_num_a2m_data[0]][2] =
                normalize_angle(
                    tensor_agent_heading_data[i][tensor_param_.N_h - 1] -
                        tensor_agent_heading_data[j][tensor_param_.N_h - 1]);
            tensor_dec_num_a2m_data[0]++;
            tensor_dec_edge_index_a2m_data[0][tensor_dec_num_a2m_data[0]] = j;
            tensor_dec_edge_index_a2m_data[1][tensor_dec_num_a2m_data[0]] = i;
            float cos_heading_i =
                std::cos(tensor_agent_heading_data[i][tensor_param_.N_h - 1]);
            float sin_heading_i =
                std::sin(tensor_agent_heading_data[i][tensor_param_.N_h - 1]);
            tensor_dec_r_a2m_data[tensor_dec_num_a2m_data[0]][0] =
                static_cast<float>(dis_tmp);
            tensor_dec_r_a2m_data[tensor_dec_num_a2m_data[0]][1] =
                angle_between_2d_vectors(cos_heading_i, sin_heading_i, -vec_x,
                                         -vec_y);
            tensor_dec_r_a2m_data[tensor_dec_num_a2m_data[0]][2] =
                -normalize_angle(
                    tensor_agent_heading_data[i][tensor_param_.N_h - 1] -
                        tensor_agent_heading_data[j][tensor_param_.N_h - 1]);
            tensor_dec_num_a2m_data[0]++;
          }
        }
      }
      // pl2m
      for (int j =0; j < valid_pl_num; j++){
        auto& pos_pl_x = pl_points_data[j][0];
        auto& pos_pl_y = pl_points_data[j][1];
        float vec_x = tensor_agent_position_data[i][tensor_param_.N_h - 1][0] - pos_pl_x;
        float vec_y = tensor_agent_position_data[i][tensor_param_.N_h - 1][1] - pos_pl_y;
        float dis_tmp = std::hypot(vec_x,vec_y);
        if (dis_tmp < param_.pl2m_radius){
          // pl index
          tensor_dec_edge_index_pl2m_multi_data[0][tensor_dec_num_pl2m_multi_data[0]] = j;
          // agent index
          tensor_dec_edge_index_pl2m_multi_data[1][tensor_dec_num_pl2m_multi_data[0]] = i;
          tensor_dec_r_pl2m_data[tensor_dec_num_pl2m_multi_data[0]][0] = dis_tmp;
          float cos_heading = std::cos(tensor_agent_heading_data[i][tensor_param_.N_h - 1]);
          float sin_heading = std::sin(tensor_agent_heading_data[i][tensor_param_.N_h - 1]);
          tensor_dec_r_pl2m_data[tensor_dec_num_pl2m_multi_data[0]][1] = 
            angle_between_2d_vectors(cos_heading,sin_heading,vec_x,vec_y);
          tensor_dec_r_pl2m_data[tensor_dec_num_pl2m_multi_data[0]][2] = 
            normalize_angle(pl_orient_data[j] -tensor_agent_heading_data[i][tensor_param_.N_h - 1]);
          tensor_dec_num_pl2m_multi_data[0]++;
        }
      }
    }
  }

  const int initial_fill_count = tensor_dec_num_pl2m_multi_data[0];
  // decoder a2m multi
  tensor_dec_num_a2m_multi_data[0] =
      tensor_dec_num_a2m_data[0] * tensor_param_.N_m;
  for (int i = 0; i < tensor_param_.N_m; i++) {
    for (int j = 0; j < tensor_dec_num_a2m_data[0]; j++) {
      tensor_dec_edge_index_a2m_multi_data[0][j +
          i * tensor_dec_num_a2m_data[0]] =
          tensor_dec_edge_index_a2m_data[0][j] + i * tensor_param_.N_a;
      tensor_dec_edge_index_a2m_multi_data[1][j +
          i * tensor_dec_num_a2m_data[0]] =
          tensor_dec_edge_index_a2m_data[1][j] + i * tensor_param_.N_a;
    }
  }
  // fill decoder edge a2m multi index use the front element
  if (tensor_dec_num_a2m_multi_data[0] > 0) {
    for (int i = tensor_dec_num_a2m_multi_data[0];
         i < tensor_param_.N_a2m_multi; i++) {
      tensor_dec_edge_index_a2m_multi_data[0][i] =
          tensor_dec_edge_index_a2m_multi_data[0][0];
      tensor_dec_edge_index_a2m_multi_data[1][i] =
          tensor_dec_edge_index_a2m_multi_data[1][0];
    }
  }

  // fill decoder edge pl2m multi index use the front element
  if (tensor_dec_num_pl2m_multi_data[0] < tensor_param_.N_pl2m) {
    for (int i = tensor_dec_num_pl2m_multi_data[0];
        i < tensor_param_.N_pl2m; i++) {
        tensor_dec_edge_index_pl2m_multi_data[0][i] =
          tensor_dec_edge_index_pl2m_multi_data[0][0];
        tensor_dec_edge_index_pl2m_multi_data[1][i] =
          tensor_dec_edge_index_pl2m_multi_data[1][0];
    }
  }
  // decoder pl2m multi
  for (int i = 1; i < tensor_param_.N_m; i++){
    for(int j = 0; j < tensor_dec_num_pl2m_multi_data[0]; j++){
      // pl index
      tensor_dec_edge_index_pl2m_multi_data[0][j + i * tensor_param_.N_pl2m] = 
        tensor_dec_edge_index_pl2m_multi_data[0][j] + i * tensor_param_.N_pl;
      // agent index
      tensor_dec_edge_index_pl2m_multi_data[1][j + i * tensor_param_.N_pl2m] = 
        tensor_dec_edge_index_pl2m_multi_data[1][j] + i * tensor_param_.N_a;
    }
  }
  tensor_dec_num_pl2m_multi_data[0] = tensor_dec_num_pl2m_multi_data[0] * tensor_param_.N_m;

  // fill decoder edge pl2m multi index use the front element
  for (int i = 1; i < tensor_param_.N_m; i++) {
    for (int j = i * tensor_param_.N_pl2m + initial_fill_count; 
      j < (i + 1) * tensor_param_.N_pl2m; j++) {
        tensor_dec_edge_index_pl2m_multi_data[0][j] =
          tensor_dec_edge_index_pl2m_multi_data[0][i * tensor_param_.N_pl2m];
        tensor_dec_edge_index_pl2m_multi_data[1][j] =
          tensor_dec_edge_index_pl2m_multi_data[1][i * tensor_param_.N_pl2m];
    }
  }

  // decoder m2m
  tensor_dec_num_m2m_data[0] = mask_num * tensor_param_.N_m * tensor_param_.N_m;
  for (int i = 0; i < mask_num; i++) {
    for (int j = 0; j < tensor_param_.N_m; j++) {
      for (int k = 0; k < tensor_param_.N_m; k++) {
        tensor_dec_edge_index_m2m_data[0][i * tensor_param_.N_m *
            tensor_param_.N_m +
            j * tensor_param_.N_m + k] =
            i * tensor_param_.N_m + j;
        tensor_dec_edge_index_m2m_data[1][i * tensor_param_.N_m *
            tensor_param_.N_m +
            j * tensor_param_.N_m + k] =
            i * tensor_param_.N_m + k;
      }
    }
  }
  // fill decoder edge m2m index use the front element
  if (tensor_dec_num_m2m_data[0] > 0) {
    for (int i = tensor_dec_num_m2m_data[0]; i < tensor_param_.N_m2m; i++) {
      tensor_dec_edge_index_m2m_data[0][i] =
          tensor_dec_edge_index_m2m_data[0][0];
      tensor_dec_edge_index_m2m_data[1][i] =
          tensor_dec_edge_index_m2m_data[1][0];
    }
  }

  // fill decoder edge a2m index use the front element
  if (tensor_dec_num_a2m_data[0] > 0) {
    for (int i = tensor_dec_num_a2m_data[0]; i < tensor_param_.N_a2m; i++) {
      tensor_dec_edge_index_a2m_data[0][i] =
          tensor_dec_edge_index_a2m_data[0][0];
      tensor_dec_edge_index_a2m_data[1][i] =
          tensor_dec_edge_index_a2m_data[1][0];

      tensor_dec_r_a2m_data[i][0] = tensor_dec_r_a2m_data[0][0];
      tensor_dec_r_a2m_data[i][1] = tensor_dec_r_a2m_data[0][1];
      tensor_dec_r_a2m_data[i][2] = tensor_dec_r_a2m_data[0][2];
    }
  }

  auto& tensor_agent_enc_num_pl2a = output_tensor_.get_agent_enc_num_pl2a();
  auto& tensor_agent_enc_num_pl2a_data = tensor_agent_enc_num_pl2a.data();
  tensor_agent_enc_num_pl2a_data[0] = 0;

  auto& agent_enc_edge_index_pl2a = output_tensor_.get_agent_enc_edge_index_pl2a();
  auto& tensor_agent_enc_edge_index_pl2a_data = agent_enc_edge_index_pl2a.data();
  tensor_agent_enc_edge_index_pl2a_data[0].resize(tensor_param_.N_pl2a, 0);
  tensor_agent_enc_edge_index_pl2a_data[1].resize(tensor_param_.N_pl2a, 0);

  auto& agent_enc_r_pl2a = output_tensor_.get_agent_enc_r_pl2a();
  auto& tensor_agent_enc_r_pl2a_data = agent_enc_r_pl2a.data();
  for (int i = 0; i > tensor_param_.N_pl2a; i++){
    tensor_agent_enc_r_pl2a_data[i].resize(3, 0.0);
  }


  for(int i = 0; i < tensor_param_.N_a; i++){
    for (int j = 0; j < tensor_param_.N_h; j++){
      if (tensor_agent_mask_data[i][j]){
        if (tensor_agent_enc_num_pl2a_data[0] >= tensor_param_.N_pl2a){
          break;
        }
        for (int k = 0; k < valid_pl_num; k++){
          if (tensor_agent_enc_num_pl2a_data[0] >= tensor_param_.N_pl2a){
            break;
          }
          auto& pos_pl_x = pl_points_data[k][0];
          auto& pos_pl_y = pl_points_data[k][1];
          float vec_x = tensor_agent_position_data[i][j][0] - pos_pl_x;
          float vec_y = tensor_agent_position_data[i][j][1] - pos_pl_y;
          float dis_tmp = std::hypot(vec_x,vec_y);
          if (dis_tmp < param_.pl2a_radius){
            // pl index
            tensor_agent_enc_edge_index_pl2a_data[0][tensor_agent_enc_num_pl2a_data[0]] = j * tensor_param_.N_pl + k;
            // agent index
            tensor_agent_enc_edge_index_pl2a_data[1][tensor_agent_enc_num_pl2a_data[0]] = j * tensor_param_.N_a + i;
            float cos_heading = std::cos(tensor_agent_heading_data[i][j]);
            float sin_heading = std::sin(tensor_agent_heading_data[i][j]);
            tensor_agent_enc_r_pl2a_data[tensor_agent_enc_num_pl2a_data[0]][0] = dis_tmp;
            tensor_agent_enc_r_pl2a_data[tensor_agent_enc_num_pl2a_data[0]][1] = angle_between_2d_vectors(cos_heading, sin_heading, vec_x, vec_y);
            tensor_agent_enc_r_pl2a_data[tensor_agent_enc_num_pl2a_data[0]][2] = normalize_angle(pl_orient_data[k] -tensor_agent_heading_data[i][j]);
            tensor_agent_enc_num_pl2a_data[0] ++;
          }
        }
      }
    }
  }

  // fill encoder edge pl2a index use the front element
  if (tensor_agent_enc_num_pl2a_data[0]> 0) {
    for (int i = tensor_agent_enc_num_pl2a_data[0]; i < tensor_param_.N_pl2a; i++) {
      tensor_agent_enc_edge_index_pl2a_data[0][i] =
          tensor_agent_enc_edge_index_pl2a_data[0][0];
      tensor_agent_enc_edge_index_pl2a_data[1][i] =
          tensor_agent_enc_edge_index_pl2a_data[1][0];

      tensor_agent_enc_r_pl2a_data[i][0] = tensor_agent_enc_r_pl2a_data[0][0];
      tensor_agent_enc_r_pl2a_data[i][1] = tensor_agent_enc_r_pl2a_data[0][1];
      tensor_agent_enc_r_pl2a_data[i][2] = tensor_agent_enc_r_pl2a_data[0][2];
    }
  }

  return true;
}

} // namespace DLP