/*
 * Copyright (C) 2020 by SenseTime Group Limited. All rights reserved.
 * dingjie <dingjie1@senseauto.com>
 */

#include <iostream>
#include "spline_smoothing/include/spline_reference_smoother.hpp"

namespace PolyFit {

SplineReferenceSmoother::SplineReferenceSmoother() {
  // auto& spline_smoother_config =
  //     Config::GetInstance().pp_config.spline_smoother_config();
  // spline_order_ = spline_smoother_config.spline_order();
  // spline_max_length_ = spline_smoother_config.spline_max_length();
  // anchor_point_intervel_ = spline_smoother_config.anchor_point_intervel();
  // normal_anchor_point_bound_ =
  //     spline_smoother_config.normal_anchor_point_bound();
  // relax_anchor_point_bound_ =
  // spline_smoother_config.relax_anchor_point_bound();
  // normal_reference_kernel_weight_ =
  //     spline_smoother_config.normal_reference_kernel_weight();
  // relax_reference_kernel_weight_ =
  //     spline_smoother_config.relax_reference_kernel_weight();
  // second_order_derivative_kernel_weight_ =
  //     spline_smoother_config.second_order_derivative_kernel_weight();
  // third_order_derivative_kernel_weight_ =
  //     spline_smoother_config.third_order_derivative_kernel_weight();
  // reference_point_intervel_ =
  // spline_smoother_config.reference_point_intervel();

  spline_solver_.reset(new OsqpSpline2dSolver(t_knots_, spline_order_));
}

bool SplineReferenceSmoother::SplineSmooth(
    const bool &relax_optimization,
    const std::vector<math_utils::Point2D> &raw_refline,
    std::vector<math_utils::Point2D> &spline_smooth_refline,
    std::deque<math_utils::Point2D> &smooth_first_derivative,
    std::deque<math_utils::Point2D> &smooth_second_derivative,
    std::deque<math_utils::Point2D> &smooth_third_derivative, 
    const std::vector<double> &start_point_derivatives) {
  double accumulate_dis = 0.0;
  std::deque<PathPoint> raw_reference_line;
  raw_reference_line.emplace_back(
      PathPoint(raw_refline.front().x, raw_refline.front().y, 0.0, 0.0));
  for (uint i = 1; i < raw_refline.size(); ++i) {
    float dx = raw_refline.at(i).x - raw_refline.at(i - 1).x;
    float dy = raw_refline.at(i).y - raw_refline.at(i - 1).y;
    raw_reference_line.back().theta = std::atan2(dy, dx);

    accumulate_dis += (raw_refline[i] - raw_refline[i - 1]).norm();
    raw_reference_line.emplace_back(PathPoint(
        raw_refline.at(i).x, raw_refline.at(i).y, 0.0, accumulate_dis));
  }
  auto raw_points_size = raw_reference_line.size();
  raw_reference_line.at(raw_points_size - 1).theta =
      raw_reference_line.at(raw_points_size - 2).theta;

  // auto start = std::chrono::system_clock::now();

  // std::cout << "before smooth" << std::endl;

  SetOptimizationMode(relax_optimization);
  SetRawReferenceLine(raw_reference_line, start_point_derivatives);
  if (!Smooth()) {
    // AD_LERROR(SplineRefLinePlanner) << "spline smoother failed";
    return false;
  }

  // std::cout << "after smooth";

  GetSmoothResult(spline_smooth_refline, smooth_first_derivative,
                  smooth_second_derivative, smooth_third_derivative);

  // auto end = std::chrono::system_clock::now();
  // std::chrono::duration<double> diff = end - start;
  // AD_LERROR(CostTime) << "SplineRefLineSmooth:" << diff.count() * 1000.0
  //                     << " ms.";

  // if (debug_print_) {
  //   std::string data_path = "/tmp/DM/" +
  //                           FrameTimer::GetInstance().GetFrameTimeStr() +
  //                           "SplineSmoothedRefline.json";
  //   common::utils::ConfigurationReader::WriteJSON(data_path,
  //                                                 *spline_smooth_refline);
  // }

  // for (const auto& pt : spline_smooth_refline) {
  //   std::cout << "smooth_points x " << pt.x << " y " << pt.y << std::endl;
  // }

  double lateral_error_tolerence = relax_optimization
                                       ? relax_anchor_point_bound_ + 0.25
                                       : normal_anchor_point_bound_ + 0.25;
  if (!SmoothReflineCheck(spline_smooth_refline, lateral_error_tolerence)) {
    // AD_LERROR(SplineRefLinePlanner) << "spline smoothed refline check
    // failed";
    return false;
  }

  return true;
}

bool SplineReferenceSmoother::SmoothReflineCheck(
    const std::vector<math_utils::Point2D> &smooth_refline,
    const double &lateral_error_tolerence) {
  // // auto start_spline_check = std::chrono::system_clock::now();

  // // 1. use the point which is nearest to vehicle poistion as the orgin of
  // // frenet coordinate system
  // math_utils::Point2D frenet_origin_point = ego_enu_;
  // std::vector<TrajectoryPoint> tmp_refline;
  // origin_frenet_system_->ConvertPointToTrajectory(dm_target_refline_,
  //                                                 &tmp_refline);
  // if (!origin_frenet_system_->Update(tmp_refline, frenet_origin_point)) {
  //   AD_LERROR(SplineRefLinePlanner) << "Fail to update frenet system.";
  //   return false;
  // }

  // // 2. check the length of frenet coordinate
  // constexpr double min_positive_length_frenet = 5.0;
  // if (origin_frenet_system_->GetPositiveLength() <
  // min_positive_length_frenet) {
  //   AD_LWARN(SplineRefLinePlanner)
  //       << "the positive length of frenet coord system is not enough!!!";
  //   // return false;
  // }
  // // 3. get smooth_refline in frenet system
  // std::vector<float> lateral_error;
  // std::vector<float> longi_dis;
  // uint32_t min_pos = 0, max_pos = 0;
  // auto max_lateral_error = kMin;
  // auto min_lateral_error = kMax;
  // for (uint32_t i = 0, sz = smooth_refline.size(); i < sz; ++i) {
  //   auto point = std::move(
  //       origin_frenet_system_->Cartesian2Frenet(smooth_refline.at(i)));
  //   longi_dis.emplace_back(point.x);
  //   lateral_error.emplace_back(point.y);
  //   if (point.y < min_lateral_error) {
  //     min_lateral_error = point.y;
  //     min_pos = i;
  //   }
  //   if (point.y > max_lateral_error) {
  //     max_lateral_error = point.y;
  //     max_pos = i;
  //   }
  // }
  // AD_LTRACE(SplineRefLinePlanner) << "max_lateral_error"
  //                                 << ", longi: " << longi_dis[max_pos]
  //                                 << ", lateral: " << lateral_error[max_pos]
  //                                 << ", pos: " << smooth_refline.at(max_pos);
  // AD_LTRACE(SplineRefLinePlanner) << "min_lateral_error"
  //                                 << ", longi: " << longi_dis[min_pos]
  //                                 << ", lateral: " << lateral_error[min_pos]
  //                                 << ", pos: " << smooth_refline.at(min_pos);

  // auto end_spline_check = std::chrono::system_clock::now();
  // std::chrono::duration<double> diff_spline_check =
  //     end_spline_check - start_spline_check;
  // AD_LERROR(CostTime) << "SplineRefLineCheck:"
  //                     << diff_spline_check.count() * 1000.0 << " ms.";

  // if (fabs(lateral_error[max_pos]) > lateral_error_tolerence ||
  //     fabs(lateral_error[min_pos]) > lateral_error_tolerence) {
  //   AD_LERROR(SplineRefLinePlanner)
  //       << "smooth refline lateral error check failed!!!";
  //   AD_LERROR(SplineRefLinePlanner) << "max_lateral_error"
  //                                   << ", longi: " << longi_dis[max_pos]
  //                                   << ", lateral: " <<
  //                                   lateral_error[max_pos]
  //                                   << ", pos: " <<
  //                                   smooth_refline.at(max_pos);
  //   AD_LERROR(SplineRefLinePlanner) << "min_lateral_error"
  //                                   << ", longi: " << longi_dis[min_pos]
  //                                   << ", lateral: " <<
  //                                   lateral_error[min_pos]
  //                                   << ", pos: " <<
  //                                   smooth_refline.at(min_pos);
  //   return false;
  // }
  return true;
}

void SplineReferenceSmoother::Clear() {
  t_knots_.clear();
  t_coord_.clear();
  xy_points_.clear();
  headings_.clear();
  t_ref_coord_.clear();
  xy_ref_points_.clear();
  t_interpolate_coord_.clear();
  smooth_reference_line_.clear();
  smooth_first_derivative_.clear();
  smooth_second_derivative_.clear();
  smooth_third_derivative_.clear();
}

void SplineReferenceSmoother::SetOptimizationMode(bool relax_optimization) {
  relax_optimization_ = relax_optimization;
  // AD_LINFO(SplineReferenceSmoother) << "relax_optimization: "
  //                                   << relax_optimization_;
}

void SplineReferenceSmoother::SetRawReferenceLine(
    const std::deque<PathPoint> &raw_reference_line,
    const std::vector<double> &start_point_derivatives) {
  start_point_derivatives_ = start_point_derivatives;
  raw_reference_line_ = std::move(raw_reference_line);
}

bool SplineReferenceSmoother::SampleAnchorPoints() {
  // 1. sample anchor points
  const double length =
      raw_reference_line_.back().s - raw_reference_line_.front().s;
  uint32_t num_spline =
      std::max(2u, static_cast<uint32_t>(length / spline_max_length_ + 0.5));
  for (std::uint32_t i = 0; i <= num_spline; ++i) {
    t_knots_.emplace_back(i * 1.0);
  }

  std::cout << "SplineReferenceSmoother::SampleAnchorPoints, refline length " << length 
  << " spline_max_length_ " << spline_max_length_ << " num_spline " << num_spline
  << std::endl;

  std::deque<PathPoint> anchor_points;
  anchor_points.emplace_back(raw_reference_line_.front());
  int num_of_anchors = std::max(
      2u, static_cast<uint32_t>(length / anchor_point_intervel_ + 0.5));
  std::vector<double> anchor_s;
  uniform_slice(0.0, length, num_of_anchors - 1, &anchor_s);

  uint32_t find_start_index = 0;
  uint32_t last_valid_index = raw_reference_line_.size() - 1;
  for (std::size_t i = 1; i < anchor_s.size(); ++i) {
    auto target_s = anchor_s[i];
    while (find_start_index <= last_valid_index) {
      if (raw_reference_line_.at(find_start_index).s >= target_s) {
        anchor_points.emplace_back(raw_reference_line_.at(find_start_index));
        break;
      } else {
        ++find_start_index;
      }
    }
  }

  scale_ = (anchor_points.back().s - anchor_points.front().s) /
           (t_knots_.back() - t_knots_.front());
  for (std::size_t i = 1; i < anchor_points.size(); ++i) {
    t_coord_.emplace_back(anchor_points[i].s / scale_);
    xy_points_.emplace_back(
        PolyFit::math::Vec2d(anchor_points[i].x, anchor_points[i].y));
    headings_.emplace_back(anchor_points[i].theta);
  }
  // AD_LINFO(SplineReferenceSmoother)
  //     << "raw_refline length: " << length << ", num_spline: " << num_spline
  //     << ", intervel_anchor: " << anchor_point_intervel_
  //     << ", num_anchors: " << num_of_anchors
  //     << ", anchor_point back s: " << anchor_points.back().s;

  // 2. sample reference points
  std::deque<PathPoint> ref_points;
  ref_points.emplace_back(raw_reference_line_.front());
  int num_of_refs = std::max(
      2u, static_cast<uint32_t>(length / reference_point_intervel_ + 0.5));
  std::vector<double> ref_s;
  uniform_slice(0.0, length, num_of_refs - 1, &ref_s);

  find_start_index = 0;
  for (std::size_t i = 1; i < ref_s.size(); ++i) {
    auto target_s = ref_s[i];
    while (find_start_index <= last_valid_index) {
      if (raw_reference_line_.at(find_start_index).s >= target_s) {
        ref_points.emplace_back(raw_reference_line_.at(find_start_index));
        break;
      } else {
        ++find_start_index;
      }
    }
  }
  double scale = (ref_points.back().s - ref_points.front().s) /
                 (t_knots_.back() - t_knots_.front());
  for (std::size_t i = 1; i < ref_points.size(); ++i) {
    t_ref_coord_.emplace_back(ref_points[i].s / scale);
    xy_ref_points_.emplace_back(
        PolyFit::math::Vec2d(ref_points[i].x, ref_points[i].y));
  }

  // 3. interpolate
  s_interpolate_coord_.clear();
  const double interpolate_intervel = 1.0;
  int num_of_interpolate =
      std::max(2u, static_cast<uint32_t>(length / interpolate_intervel + 0.5));
  uniform_slice(0.0, length, num_of_interpolate - 1, &s_interpolate_coord_);
  std::for_each(s_interpolate_coord_.begin(), s_interpolate_coord_.end(),
                [this](double &s_interpolate) {
                  t_interpolate_coord_.emplace_back(s_interpolate / scale_);
                });
  return true;
}

bool SplineReferenceSmoother::Smooth() {
  // auto start_spline_osqp_definition = std::chrono::system_clock::now();
  Clear();
  SampleAnchorPoints();

  // std::cout << "after SampleAnchorPoints" << std::endl;

  spline_solver_->Reset(t_knots_, spline_order_);

  if (!AddConstraint()) {
    // AD_LERROR(SplineReferenceSmoother)
    //     << "Add constraint for spline smoother failed";
    return false;
  }

  // std::cout << "after AddConstraint" << std::endl;

  if (!AddKernel()) {
    // AD_LERROR(SplineReferenceSmoother)
    //     << "Add kernel for spline smoother failed.";
    return false;
  }
  // auto end_spline_osqp_definition = std::chrono::system_clock::now();
  // std::chrono::duration<double> diff_spline_osqp_definition =
      // end_spline_osqp_definition - start_spline_osqp_definition;
  // AD_LERROR(CostTime) << "SplineReflineOsqpDefinition:"
                      // << diff_spline_osqp_definition.count() * 1000.0 << "
                      // ms.";

  // std::cout << "after AddKernel" << std::endl;

  // auto start_spline_osqp_solve = std::chrono::system_clock::now();
  if (!Solve()) {
    // AD_LERROR(SplineReferenceSmoother)
    //     << "Solve spline smoother problem failed";
    std::cout << "solve failed" << std::endl;
    return false;
  }
  // auto end_spline_osqp_solve = std::chrono::system_clock::now();
  // std::chrono::duration<double> diff_spline_osqp_solve =
      // end_spline_osqp_solve - start_spline_osqp_solve;
  // AD_LERROR(CostTime) << "SplineReflineOsqpSolve:"
  //                     << diff_spline_osqp_solve.count() * 1000.0 << " ms.";

  // std::cout << "after Solve" << std::endl;

  if (debug_print_) {
    // WriteSolverMessIntoProto();
  }

  std::cout << "SplineReferenceSmoother::Smooth, t_interpolate_coord_ size " << t_interpolate_coord_.size() << std::endl;

  const auto &spline = spline_solver_->spline();
  for (const auto &t : t_interpolate_coord_) {
    auto xy = spline(t);
    const double heading =
        std::atan2(spline.DerivativeY(t), spline.DerivativeX(t));
    smooth_reference_line_.emplace_back(
        PathPoint(xy.first, xy.second, heading, t * scale_));
    smooth_first_derivative_.emplace_back(spline.DerivativeX(t),
                                          spline.DerivativeY(t));
    smooth_second_derivative_.emplace_back(spline.SecondDerivativeX(t),
                                           spline.SecondDerivativeY(t));
    smooth_third_derivative_.emplace_back(spline.ThirdDerivativeX(t),
                                          spline.ThirdDerivativeY(t));

    // const double kappa = CurveMath::ComputeCurvature(
    //     spline.DerivativeX(t), spline.SecondDerivativeX(t),
    //     spline.DerivativeY(t), spline.SecondDerivativeY(t));
    // const double dkappa = CurveMath::ComputeCurvatureDerivative(
    //     spline.DerivativeX(t), spline.SecondDerivativeX(t),
    //     spline.ThirdDerivativeX(t), spline.DerivativeY(t),
    //     spline.SecondDerivativeY(t), spline.ThirdDerivativeY(t));

    // std::cout << "xy x " << xy.first << " y " << xy.second << std::endl;
  }
  return true;
}

bool SplineReferenceSmoother::AddKernel() {
  auto *kernel = spline_solver_->mutable_kernel();

  kernel->AddReferenceLineKernelMatrix(t_ref_coord_, xy_ref_points_,
                                       relax_optimization_
                                           ? relax_reference_kernel_weight_
                                           : normal_reference_kernel_weight_);
  kernel->AddSecondOrderDerivativeMatrix(
      second_order_derivative_kernel_weight_);
  kernel->AddThirdOrderDerivativeMatrix(third_order_derivative_kernel_weight_);
  kernel->AddRegularization(0.1);
  return true;
}

bool SplineReferenceSmoother::AddConstraint() {
  auto *spline_constraint = spline_solver_->mutable_constraint();

  // auto t_coord = {t_coord_.front(), t_coord_.back()};
  // auto headings = {headings_.front(), headings_.back()};
  // auto xy_points = {xy_points_.front(), xy_points_.back()};

  // // all points (x, y) should not deviate anchor points by a bounding box
  // std::vector<double> longitudinal_bound(t_coord.size(),
  //                                        normal_anchor_point_bound_);
  // std::vector<double> lateral_bound(
  //     t_coord.size(), relax_optimization_ ? relax_anchor_point_bound_
  //                                          : normal_anchor_point_bound_);
  // if (!spline_constraint->Add2dBoundary(t_coord, headings, xy_points,
  //                                       longitudinal_bound, lateral_bound))
  //                                       {
  //     AD_LERROR(SplineReferenceSmoother)
  //         << "Add 2d boundary constraint failed.";
  //     return false;
  // }

  // all points (x, y) should not deviate anchor points by a bounding box
  std::vector<double> longitudinal_bound(t_coord_.size(),
                                         normal_anchor_point_bound_);
  std::vector<double> lateral_bound(
      t_coord_.size(), relax_optimization_ ? relax_anchor_point_bound_
                                           : normal_anchor_point_bound_);
  if (!spline_constraint->Add2dBoundary(t_coord_, headings_, xy_points_,
                                        longitudinal_bound, lateral_bound)) {
    // AD_LERROR(SplineReferenceSmoother) << "Add 2d boundary constraint
    // failed.";
    return false;
  }

  // all spline should be connected smoothly to the second order derivative.
  if (!spline_constraint->AddSecondDerivativeSmoothConstraint()) {
    // AD_LERROR(SplineReferenceSmoother) << "Add jointness constraint failed.";
    return false;
  }

  if (!start_point_derivatives_.empty()) {
    if (!spline_constraint->AddPointConstraint(0.0, start_point_derivatives_[0],
                                               start_point_derivatives_[1])) {
      // AD_LERROR(SplineReferenceSmoother) << "Add jointness constraint
      // failed.";
      return false;
    }
    if (!spline_constraint->AddPointAngleConstraint(
            0.0, start_point_derivatives_[2])) {
      // AD_LERROR(SplineReferenceSmoother) << "Add jointness constraint
      // failed.";
      return false;
    }
    // if (!spline_constraint->AddPointSecondDerivativeConstraint(
    //         0.0, start_point_derivatives_[3],
    //         start_point_derivatives_[4])) {
    //     AD_LERROR(SplineReferenceSmoother)
    //         << "Add jointness constraint failed.";
    //     return false;
    // }
    // if (!spline_constraint->AddPointThirdDerivativeConstraint(
    //         0.0, start_point_derivatives_[5],
    //         start_point_derivatives_[6])) {
    //     AD_LERROR(SplineReferenceSmoother)
    //         << "Add jointness constraint failed.";
    //     return false;
    // }
  }
  return true;
}

bool SplineReferenceSmoother::Solve() { return spline_solver_->Solve(); }

void SplineReferenceSmoother::GetSmoothResult(
    std::vector<math_utils::Point2D> &smooth_reference_points,
    std::deque<math_utils::Point2D> &smooth_first_derivative,
    std::deque<math_utils::Point2D> &smooth_second_derivative,
    std::deque<math_utils::Point2D> &smooth_third_derivative) {
  std::for_each(smooth_reference_line_.begin(), smooth_reference_line_.end(),
                [&](const PathPoint &pt) {
                  smooth_reference_points.emplace_back(pt.x, pt.y);
                });
  smooth_first_derivative = std::move(smooth_first_derivative_);
  smooth_second_derivative = std::move(smooth_second_derivative_);
  smooth_third_derivative = std::move(smooth_third_derivative_);
}

void SplineReferenceSmoother::WriteSolverMessIntoProto() {
  // SplineSmoother smoother;
  // smoother.mutable_raw_refline_size()->CopyFrom(GetRawReflineSizeForProto());
  // smoother.mutable_online_datas()->CopyFrom(GetOnlineDatasForProto());
  // if (!start_point_derivatives_.empty()) {
  //   smoother.mutable_initial_state()->CopyFrom(GetInitialStateForProto());
  // }

  // std::string data_path;
  // if (unittest_flag_) {
  //   data_path = "/tmp/spline/" + std::to_string(unittest_data_count_) +
  //               "unittest_refline_spline_smoother.pb";
  //   unittest_data_count_++;
  // } else {
  //   data_path = "/tmp/spline/" + FrameTimer::GetInstance().GetFrameTimeStr()
  //   +
  //               "refline_spline_smoother.pb";
  // }
  // proto_io::SetProtoToASCIIFile(smoother, data_path);
}

// SplineSmootherRawReflineSize
// SplineReferenceSmoother::GetRawReflineSizeForProto() {
//   SplineSmootherRawReflineSize raw_refline_size;
//   raw_refline_size.set_size(raw_reference_line_.size());
//   return raw_refline_size;
// }

// SplineSmootherOnlineDatas SplineReferenceSmoother::GetOnlineDatasForProto() {
//   SplineSmootherOnlineDatas online_datas;
//   SplineSmootherOnlineData online_data;
//   for (std::size_t i = 0; i < raw_reference_line_.size(); ++i) {
//     online_data.set_x(raw_reference_line_.at(i).x);
//     online_data.set_y(raw_reference_line_.at(i).y);
//     online_data.set_theta(raw_reference_line_.at(i).theta);
//     online_data.set_s(raw_reference_line_.at(i).s);

//     online_datas.add_online_data()->CopyFrom(online_data);
//   }
//   return online_datas;
// }

// SplineSmootherInitialState SplineReferenceSmoother::GetInitialStateForProto()
// {
//   SplineSmootherInitialState initial_state;

//   initial_state.set_x(start_point_derivatives_[0]);
//   initial_state.set_y(start_point_derivatives_[1]);
//   initial_state.set_theta(start_point_derivatives_[2]);
//   initial_state.set_ddx(start_point_derivatives_[3]);
//   initial_state.set_ddy(start_point_derivatives_[4]);
//   initial_state.set_dddx(start_point_derivatives_[5]);
//   initial_state.set_dddy(start_point_derivatives_[6]);

//   return initial_state;
// }

} // namespace PolyFit
