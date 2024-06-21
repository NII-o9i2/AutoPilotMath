//
// Created by SENSETIME\fengxiaotong on 24-5-17.
//
#include "iostream"
#include <algorithm>
#include "core/dcp_motion_tree.h"

namespace DCP_TREE {
MathUtils::Point2D DCPMotionTreePoint::rotate_point(const MathUtils::Point2D& point, double angle) {
  double cos_theta = std::cos(angle);
  double sin_theta = std::sin(angle);

  double x_new = point.x * cos_theta - point.y * sin_theta;
  double y_new = point.x * sin_theta + point.y * cos_theta;

  return {x_new, y_new};
}
void DCPMotionTreePoint::update_polygon() {
  polygon.clear();

  double half_length = length / 2.0;
  double half_width = width / 2.0;

  MathUtils::Point2D left_front(half_length, half_width);
  MathUtils::Point2D right_front(half_length, -half_width);
  MathUtils::Point2D right_back(-half_length, -half_width);
  MathUtils::Point2D left_back(-half_length, half_width);

  MathUtils::Point2D lf_rotated = rotate_point(left_front, theta);
  MathUtils::Point2D rf_rotated = rotate_point(right_front, theta);
  MathUtils::Point2D rb_rotated = rotate_point(right_back, theta);
  MathUtils::Point2D lb_rotated = rotate_point(left_back, theta);

  lf_rotated.x += point.x;
  lf_rotated.y += point.y;
  rf_rotated.x += point.x;
  rf_rotated.y += point.y;
  rb_rotated.x += point.x;
  rb_rotated.y += point.y;
  lb_rotated.x += point.x;
  lb_rotated.y += point.y;

  polygon.push_back(lf_rotated);
  polygon.push_back(rf_rotated);
  polygon.push_back(rb_rotated);
  polygon.push_back(lb_rotated);
}

DCPMotionTreePointPOIOutput DCPMotionTree::search_poi_from_source_hard(const std::vector<DCPMotionTreePoint> &source_pts,
                                                                       const DCP_TREE::DCPMotionTreePoint &query_point) {
  DCPMotionTreePointPOIOutput poi_out;
  if (source_pts.empty()) {
    return poi_out;
  }

  // judge source points whether is behind query point
  MathUtils::Point2D query_point_dir =
      MathUtils::Point2D(std::cos(query_point.theta), std::sin(query_point.theta));
  std::vector<std::pair<double, MathUtils::Point2D>> tmp_pois;
  for (auto& pt : source_pts) {
    MathUtils::Point2D source_point_dir(pt.point.x - query_point.point.x,
                             pt.point.y - query_point.point.y);
    if (vec_dot_product(query_point_dir, source_point_dir) < 1e-3) {
      continue;
    }
    // calculate pure pursuit control command
    double pure_suit_delta = PurePursuit::pp_control(
        pt.point, query_point.point, query_point.theta);
    tmp_pois.emplace_back(pure_suit_delta, pt.point);
  }
  if (tmp_pois.empty()) {
    return poi_out;
  }
  std::sort(
      tmp_pois.begin(), tmp_pois.end(),
      [](const std::pair<double, MathUtils::Point2D>& a,
         const std::pair<double, MathUtils::Point2D>& b) { return a.first < b.first; });

  poi_out.is_valid = true;
  poi_out.expect_delta = tmp_pois.front().first;
  poi_out.point = tmp_pois.front().second;
  return poi_out;
}

void DCPMotionTree::step_a_omega_dcp(DCPMotionTreePoint &state,
                      DCPMotionTreePoint &next_state,
                      double delta_t) {
  // state -> 0:x, 1:y, 2:v, 3:theta, 4:a, 5:omega

  double s = state.v * delta_t;

  double next_x =
      state.point.x +
          0.5 * (std::cos(state.theta) + std::cos(state.theta + state.omega * delta_t)) * s;
  double next_y =
      state.point.y +
          0.5 * (std::sin(state.theta) + std::sin(state.theta + state.omega * delta_t)) * s;
  double next_v = state.v;
  double next_theta = state.theta + state.omega * delta_t;
//  double next_a = 0.0;
  double next_omega = state.omega;

  if (next_v < 0.0) {
    next_v = 0.0;
  }

  next_state.theta = next_theta;
  next_state.omega = next_omega;
  next_state.point.x = next_x;
  next_state.point.y = next_y;
  next_state.speed_limit = state.speed_limit;
  next_state.v = next_v;
  next_state.index = state.index + 1;
  next_state.length = state.length;
  next_state.width = state.width;
}

void DCPMotionTree::process() {
  if (init_points_.empty()){
    return;
  }
  // 1.0 fill search info
  for (int i = 0; i < init_points_.size(); i++){
    init_points_[i].search_info.id = get_a_new_id();
    init_points_[i].search_info.index = i;
  }

  bool has_been_collision = false;
  for (int i = 0; i + 1 < init_points_.size(); i++){
    bool is_collision = false;
    for (auto & obj : obj_info_){
      if (check_collision(init_points_[i],init_points_[i+1],obj)){
        is_collision = true;
        break;
      }
    }
    init_points_[i].search_info.inspire_id = init_points_[i+1].search_info.id;
    init_points_[i+1].search_info.is_collision = is_collision;
    init_points_[i+1].search_info.back_id = init_points_[i].search_info.id;

    if (is_collision){
      close_ids_.emplace_back( init_points_[i+1].search_info.id);
    }

    if (!has_been_collision){
      open_ids_.emplace_back(init_points_[i].search_info.id);
      has_been_collision = is_collision || has_been_collision;
    }
  }
  SolverInfoLog::Instance()->error(" open size " +
                                   std::to_string(open_ids_.size()));

  for (auto & init_point : init_points_){
    auto ptr = std::make_shared<DCPMotionTreePoint>(init_point);
    point_map_[init_point.search_info.id] = ptr;
  }

  result_points_.clear();
  if (!has_been_collision){
    result_points_ = init_points_;
    return;
  }
  for (int i = static_cast<int>(init_points_.size()) - 1; i > 0 ; i--){
    target_source_.emplace_back(init_points_[i]);
    if (init_points_[i].search_info.is_collision || target_source_.size() >= max_target_size_){
      break;
    }
  }

  auto get_the_pop_id = [&](){
    int target_id = -1;
    double min_cost = 10000.0;
    for (auto& id : open_ids_){
      auto& point = point_map_[id]->point;
      for (auto& target : target_source_){
         double tmp_g = std::hypot(point.x -target.point.x,point.y -target.point.y);
//         double tmp_h =  point_map_[id]->search_info.h_cost;
          double tmp_h = 0.0;
         if (tmp_g + tmp_h < min_cost){
           min_cost = tmp_g + tmp_h;
           target_id = id;
         }
      }
    }

    return target_id;
  };

  auto in_close_ids = [&](int id_to_find){
    return  std::find(close_ids_.begin(), close_ids_.end(), id_to_find) != close_ids_.end();
  };

  auto check_all_collision = [&](DCPMotionTreePoint & state, DCPMotionTreePoint & next_state){
    if (state.polygon.size() == next_state.polygon.size() && dcp_motion_tree_interface_ != nullptr){
      for (int i = 0; i< state.polygon.size();i++){
        // is_out_free_space
          if (dcp_motion_tree_interface_->is_collision_in_free_space(state.polygon[i],next_state.polygon[i])){
            return true;
          }
      }
    }

    for (auto& obj : obj_info_){
      if(check_collision(state,next_state,obj)){
        return true;
      }
    }
    return false;
  };

  auto check_all_sequence_collision = [&](DCPMotionTreePoint& state, double preview_time){
    
    double t = 0;
    DCPMotionTreePoint tmp_state = state;
    DCPMotionTreePoint preview_next_state;
    while (t < preview_time){
      t += delta_t_;
      DCPMotionTree::step_a_omega_dcp(tmp_state,preview_next_state,delta_t_);
      preview_next_state.update_polygon();
      if (check_all_collision(tmp_state,preview_next_state)){
        return true;
      }
      tmp_state = preview_next_state;
    }
    return false;
  };

  auto generate_new_state = [&](DCPMotionTreePoint& fake_state,std::shared_ptr<DCPMotionTreePoint>& pop_info,double omega_dot){
    DCPMotionTreePoint next_state;
    DCPMotionTree::step_a_omega_dcp(fake_state, next_state, delta_t_);
    next_state.update_polygon();
    next_state.search_info.id = get_a_new_id();
    //        pop_info->search_info.inspire_id = next_state.search_info.id;
    next_state.search_info.back_id = pop_info->search_info.id;
    next_state.search_info.omega_dot = omega_dot;
    next_state.search_info.index = next_state.index;
    next_state.search_info.h_cost = pop_info->search_info.h_cost + fabs(next_state.omega) * 100;
    return next_state;
  };

  // 2.0 search motion trajectory

  int reach_max_index = static_cast<int>(init_points_.size());

  std::vector<std::pair<double,double>> l_expand_omega_dot_a = {
      {0.0, 0.0},
      {0.021816615, 0.0},
      {0.043633231, 0.0},
      {0.087266463, 0.0},
      {0.130899694,0.0},
      {0.174532925,0.0},
      };
  std::vector<std::pair<double,double>> r_expand_omega_dot_a = {
      {-0.021816615, 0.0},
      {-0.043633231, 0.0},
      {-0.087266463, 0.0},
      {-0.130899694,0.0},
      {-0.174532925,0.0},
  };
  std::vector<std::pair<double,double>> a_star_l_expand_omega_dot_a = {
      {0.0, 0.0},
      {0.021816615, 0.0},
      {0.043633231, 0.0},
      {0.087266463, 0.0},
      {0.130899694,0.0},
      {0.174532925,0.0},
  };
  std::vector<std::pair<double,double>> a_star_r_expand_omega_dot_a = {
      {-0.021816615, 0.0},
      {-0.043633231, 0.0},
      {-0.087266463, 0.0},
      {-0.130899694,0.0},
      {-0.174532925,0.0},
  };
  int frame_count = 0;
  if (method_option_ == 0) {
    while (!open_ids_.empty()){
      frame_count ++;
      auto pop_id = get_the_pop_id();
      auto& pop_info = point_map_[pop_id];
      std::cout << "  time "<< frame_count << " process id " << pop_id <<
      " omega " << pop_info->omega <<  std::endl;
      if (pop_info->search_info.index >= reach_max_index){
        break;
      }
      // generate inspire point
      if (pop_info->search_info.inspire_id < 0){
        auto output = search_poi_from_source_hard(target_source_,pop_info.operator*());
        if (output.is_valid){
          auto fake_state = *pop_info;
          double fake_omega = std::tan(output.expect_delta) * fake_state.v / parameter_.wheel_base;
          double omega_dot = fake_omega - fake_state.omega;
          if (omega_dot > parameter_.kMaxOmegaDot){
            fake_omega = fake_state.omega + parameter_.kMaxOmegaDot;
          }
          if (omega_dot < - parameter_.kMaxOmegaDot){
            fake_omega = fake_state.omega - parameter_.kMaxOmegaDot;
          }
          fake_state.omega = fake_omega;
          // check collision
          if (!check_all_sequence_collision(fake_state,parameter_.preview_collision_time)){
            DCPMotionTreePoint next_state;
            DCPMotionTree::step_a_omega_dcp(fake_state,next_state,delta_t_);
            next_state.update_polygon();
            next_state.search_info.id = get_a_new_id();
            pop_info->search_info.inspire_id = next_state.search_info.id;
            next_state.search_info.back_id = pop_info->search_info.id;
            next_state.search_info.omega_dot = fake_omega - fake_state.omega;
            next_state.search_info.index  = next_state.index;
            next_state.search_info.h_cost = pop_info->search_info.h_cost + fabs(next_state.omega) * 100;
            auto next_ptr = std::make_shared<DCPMotionTreePoint>(next_state);
            point_map_[next_state.search_info.id] = next_ptr;
            open_ids_.emplace_back(next_state.search_info.id);
            std::cout<< " add point i " << next_state.search_info.id << " pose "<< next_state.point.x<< " "<< next_state.point.y <<
                     " omega "<< next_state.omega<< std::endl;
  //          continue;
          }
        }
      }
      // emplace back l side open id
      for (auto& pair_cmd : a_star_l_expand_omega_dot_a){
        double omega_dot = pair_cmd.first;
        double a = pair_cmd.second;
        auto fake_state = *pop_info;
        fake_state.omega += omega_dot * delta_t_;
        fake_state.v += a * delta_t_;
        // check collision
        if (!check_all_sequence_collision(fake_state,parameter_.preview_collision_time)){
          DCPMotionTreePoint next_state;
          DCPMotionTree::step_a_omega_dcp(fake_state,next_state,delta_t_);
          next_state.update_polygon();
          next_state.search_info.id = get_a_new_id();
  //        pop_info->search_info.inspire_id = next_state.search_info.id;
          next_state.search_info.back_id = pop_info->search_info.id;
          next_state.search_info.omega_dot = omega_dot;
          next_state.search_info.index  = next_state.index;
          next_state.search_info.h_cost = pop_info->search_info.h_cost + fabs(next_state.omega) * 100;
          auto next_ptr = std::make_shared<DCPMotionTreePoint>(next_state);
          point_map_[next_state.search_info.id] = next_ptr;
          open_ids_.emplace_back(next_state.search_info.id);
          std::cout<< " add point l " << next_state.search_info.id << " pose "<< next_state.point.x<< " "<< next_state.point.y <<
          " omega "<< next_state.omega<< std::endl;
        }
      }
      // emplace back r side open id
      for (auto& pair_cmd : a_star_r_expand_omega_dot_a){
        double omega_dot = pair_cmd.first;
        double a = pair_cmd.second;
        auto fake_state = *pop_info;
        fake_state.omega += omega_dot * delta_t_;
        fake_state.v += a * delta_t_;
        // check collision
        if (!check_all_sequence_collision(fake_state,parameter_.preview_collision_time)){
          DCPMotionTreePoint next_state;
          DCPMotionTree::step_a_omega_dcp(fake_state,next_state,delta_t_);
          next_state.update_polygon();
          next_state.search_info.id = get_a_new_id();
  //        pop_info->search_info.inspire_id = next_state.search_info.id;
          next_state.search_info.back_id = pop_info->search_info.id;
          next_state.search_info.omega_dot = omega_dot;
          next_state.search_info.index  = next_state.index;
          next_state.search_info.h_cost = pop_info->search_info.h_cost + fabs(next_state.omega) * 100;
          auto next_ptr = std::make_shared<DCPMotionTreePoint>(next_state);
          point_map_[next_state.search_info.id] = next_ptr;
          open_ids_.emplace_back(next_state.search_info.id);
          std::cout<< " add point r " << next_state.search_info.id << " pose "<< next_state.point.x<< " "<< next_state.point.y <<
                   " omega "<< next_state.omega<< std::endl;
        }
      }

      // remove open id
      auto new_end = std::remove(open_ids_.begin(), open_ids_.end(), pop_id);
      open_ids_.erase(new_end, open_ids_.end());
    }
  }
  if (method_option_ == 1) {
    std::vector<int> open_ids_next;
    close_ids_.clear();
    while (!open_ids_.empty()) {
      frame_count++;
      if (frame_count > parameter_.max_iter){
        break;
      }
      open_ids_next.clear();
      for (auto &pop_id : open_ids_) {
        auto &pop_info = point_map_[pop_id];

        // if open id reach max time, remove id
        if (pop_info->search_info.index >= reach_max_index) {
          //        auto new_end = std::remove(open_ids_.begin(), open_ids_.end(), pop_id);
          //        open_ids_.erase(new_end, open_ids_.end());
          close_ids_.emplace_back(pop_id);
          continue;
        }
        // if inspire point is valid and no collision, no expand
        auto output = search_poi_from_source_hard(target_source_, pop_info.operator*());
        if (output.is_valid) {
          auto fake_state = *pop_info;
          double fake_omega = std::tan(output.expect_delta) * fake_state.v / parameter_.wheel_base;
          double omega_dot = fake_omega - fake_state.omega;
          if (omega_dot > parameter_.kMaxOmegaDot) {
            fake_omega = fake_state.omega + parameter_.kMaxOmegaDot;
          }
          if (omega_dot < -parameter_.kMaxOmegaDot) {
            fake_omega = fake_state.omega - parameter_.kMaxOmegaDot;
          }
          if (std::fabs(fake_omega * fake_state.v) > parameter_.kMaxLatAcc) {
            double max_tmp = parameter_.kMaxLatAcc / fake_state.v;
            fake_omega = fake_omega > max_tmp ? max_tmp : fake_omega;
            fake_omega = fake_omega < -max_tmp ? -max_tmp : fake_omega;
          }
          if (std::fabs(fake_omega * fake_state.v) > parameter_.kConsiderDecLatAcc) {
            fake_state.v += delta_t_ * parameter_.kMaxDecAcc;
          }
          fake_state.omega = fake_omega;
          // check collision
          if (!check_all_sequence_collision(fake_state, parameter_.preview_collision_time)) {
            DCPMotionTreePoint next_state;
            DCPMotionTree::step_a_omega_dcp(fake_state, next_state, delta_t_);
            next_state.update_polygon();
            next_state.search_info.id = get_a_new_id();
            pop_info->search_info.inspire_id = next_state.search_info.id;
            next_state.search_info.back_id = pop_info->search_info.id;
            next_state.search_info.omega_dot = fake_omega - fake_state.omega;
            next_state.search_info.index = next_state.index;
            next_state.search_info.h_cost = pop_info->search_info.h_cost + fabs(next_state.omega) * 100;
            auto next_ptr = std::make_shared<DCPMotionTreePoint>(next_state);
            point_map_[next_state.search_info.id] = next_ptr;
            open_ids_next.emplace_back(next_state.search_info.id);
//            std::cout << " add point i " << next_state.search_info.id << " pose " << next_state.point.x << " "
//                      << next_state.point.y <<
//                      " omega " << next_state.omega << std::endl;
            continue;
          }
        }
        // expand motion points l
        double min_v = std::max(0.01, pop_info->v + delta_t_ * parameter_.kMaxDecAcc);
        double max_omega = parameter_.kMaxLatAcc / min_v;
        for (auto &pair_cmd : l_expand_omega_dot_a) {
          double omega_dot = pair_cmd.first;
          double a = pair_cmd.second;
          auto fake_state = *pop_info;
          fake_state.omega += omega_dot * delta_t_;
          // if omega over lat acc constraint
          if (fake_state.omega > max_omega) {
            continue;
          }
          if (std::fabs(fake_state.omega * fake_state.v) > parameter_.kConsiderDecLatAcc) {
            fake_state.v += parameter_.kMaxDecAcc * delta_t_;
          }
          // check collision
          if (!check_all_sequence_collision(fake_state, parameter_.preview_collision_time)) {
            auto next_state = generate_new_state(fake_state,pop_info,omega_dot);
            auto next_ptr = std::make_shared<DCPMotionTreePoint>(next_state);
            point_map_[next_state.search_info.id] = next_ptr;
            open_ids_next.emplace_back(next_state.search_info.id);
            break;
          }
        }

        for (auto &pair_cmd : r_expand_omega_dot_a) {
          double omega_dot = pair_cmd.first;
          double a = pair_cmd.second;
          auto fake_state = *pop_info;
          fake_state.omega += omega_dot * delta_t_;
          // if omega over lat acc constraint
          if (fake_state.omega > max_omega) {
            continue;
          }
          if (std::fabs(fake_state.omega * fake_state.v) > parameter_.kConsiderDecLatAcc) {
            fake_state.v += parameter_.kMaxDecAcc * delta_t_;
          }
          // check collision
          if (!check_all_sequence_collision(fake_state, parameter_.preview_collision_time)) {
            auto next_state = generate_new_state(fake_state,pop_info,omega_dot);
            auto next_ptr = std::make_shared<DCPMotionTreePoint>(next_state);
            point_map_[next_state.search_info.id] = next_ptr;
            open_ids_next.emplace_back(next_state.search_info.id);
            break;
          }
        }
      }

      open_ids_.clear();
      open_ids_ = open_ids_next;
    }

    result_points_.clear();
    if (close_ids_.empty()){
      result_points_ = init_points_;
    }else{
      int best_id = -1;
      double best_cost = 1000;
      for (auto& id : close_ids_){
        auto& info = point_map_[id];
        for (auto &pt : target_source_){
          double dis = std::hypot(info->point.x -pt.point.x,info->point.y -pt.point.y);
          double theta_err = std::fabs(info->theta - pt.theta);
          if (dis < best_cost){
            best_cost = dis;
            best_id = id;
          }
//          std::cout << " dis "<< dis << " theta "<< theta_err << std::endl;
        }
      }
      int cur_id = best_id;
      for (int i = 0; i < reach_max_index - 1; i++){
        result_points_.emplace_back(*point_map_[cur_id]);
        cur_id = point_map_[cur_id]->search_info.back_id;
        if (cur_id< 0){
          break;
        }
      }
      std::reverse(result_points_.begin(),result_points_.end());
    }
  }

//  std::vector<int> last_time_id;
//  int max_index = -1;
//  for(auto &id : open_ids_){
//    if (point_map_[id]->search_info.index > max_index){
//      max_index = point_map_[id]->search_info.index;
//    }
//  }
//  for(auto &id : open_ids_){
//    if (point_map_[id]->search_info.index == max_index){
//      last_time_id.emplace_back(id);
//    }
//  }
//
//  std::cout << " finish dcp motion "<< std::endl;
}

bool DCPMotionTree::check_collision(DCPMotionTreePoint &state,
                                    DCPMotionTreePoint &next_state,
                                    DCPObjectInfo& obj_info){

  if (state.polygon.size() != next_state.polygon.size()){
    // wrong state
    return true;
  }
  double h = obj_info.center_point.x;
  double k = obj_info.center_point.y;

  double a = obj_info.ellipse_a;
  double b = obj_info.ellipse_b;

  for (int i = 0; i < state.polygon.size(); i++){
    if (is_line_segment_intersecting_tilted_ellipse(
        state.polygon[i].x, state.polygon[i].y,
        next_state.polygon[i].x, next_state.polygon[i].y,
        obj_info.center_point.x, obj_info.center_point.y,
        obj_info.ellipse_a, obj_info.ellipse_b,
        obj_info.theta
    )) {
      return true;
    }
  }

  return false;
}

bool DCPMotionTree::solve_quadratic(double a, double b, double c, double &t1, double &t2) {
  double discriminant = b * b - 4 * a * c;
  if (discriminant < 0) {
    return false; // 无实数解
  }
  t1 = (-b + std::sqrt(discriminant)) / (2 * a);
  t2 = (-b - std::sqrt(discriminant)) / (2 * a);
  return true;
}

// 判断线段是否和倾斜椭圆相交
bool DCPMotionTree::is_line_segment_intersecting_tilted_ellipse(
    double x1, double y1, double x2, double y2, // 线段端点
    double h, double k, double a, double b,     // 椭圆参数
    double theta                                // 椭圆旋转角度
) {
  // 预计算一些三角函数
  double cos_theta = std::cos(theta);
  double sin_theta = std::sin(theta);

  // 计算二次方程的系数
  double A = ((x2 - x1) * cos_theta + (y2 - y1) * sin_theta) * ((x2 - x1) * cos_theta + (y2 - y1) * sin_theta) / (a * a) +
      ((x2 - x1) * sin_theta - (y2 - y1) * cos_theta) * ((x2 - x1) * sin_theta - (y2 - y1) * cos_theta) / (b * b);

  double B = 2 * (((x2 - x1) * cos_theta + (y2 - y1) * sin_theta) * ((x1 - h) * cos_theta + (y1 - k) * sin_theta) / (a * a) +
      ((x2 - x1) * sin_theta - (y2 - y1) * cos_theta) * ((x1 - h) * sin_theta - (y1 - k) * cos_theta) / (b * b));

  double C = ((x1 - h) * cos_theta + (y1 - k) * sin_theta) * ((x1 - h) * cos_theta + (y1 - k) * sin_theta) / (a * a) +
      ((x1 - h) * sin_theta - (y1 - k) * cos_theta) * ((x1 - h) * sin_theta - (y1 - k) * cos_theta) / (b * b) - 1;

  double t1, t2;
  if (!solve_quadratic(A, B, C, t1, t2)) {
    return false; // 无实数解
  }

  // 检查解是否在 [0, 1] 范围内
  if ((t1 >= 0 && t1 <= 1) || (t2 >= 0 && t2 <= 1)) {
    return true;
  }
  return false;
}
}