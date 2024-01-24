#pragma once

#include <deque>
#include <memory>
#include <vector>

#include "common/utils.h"

#include "spline_smoothing/include/common/discrete_path2d.hpp"
#include "spline_smoothing/include/spline/osqp_spline_2d_solver.hpp"
// #include "utils/frame_timer/frame_timer.hpp"

namespace PolyFit {

/**
 * uniformly slice a segment [start, end] to num + 1 pieces
 * the result sliced will contain the n + 1 points that slices the provided
 * segment. `start` and `end` will be the first and last element in `sliced`.
 */
template <typename T>
void uniform_slice(const T start, const T end, uint32_t num,
                   std::vector<T> *sliced) {
  if (!sliced || num == 0) {
    return;
  }
  const T delta = (end - start) / num;
  sliced->resize(num + 1);
  T s = start;
  for (uint32_t i = 0; i < num; ++i, s += delta) {
    sliced->at(i) = s;
  }
  sliced->at(num) = end;
}

class SplineReferenceSmoother {
public:
  SplineReferenceSmoother();
  ~SplineReferenceSmoother() = default;

  bool SplineSmooth(const bool &relax_optimization,
                    const std::vector<math_utils::Point2D> &raw_refline,
                    std::vector<math_utils::Point2D> &spline_smooth_refline,
                    std::deque<math_utils::Point2D> &smooth_first_derivative,
                    std::deque<math_utils::Point2D> &smooth_second_derivative,
                    std::deque<math_utils::Point2D> &smooth_third_derivative,
                    const std::vector<double> &start_point_derivatives = {});

private:
  void SetOptimizationMode(bool relax_optimization);
  void SetRawReferenceLine(const std::deque<PathPoint> &raw_reference_line,
                           const std::vector<double> &start_point_derivatives);
  bool Smooth();

  void
  GetSmoothResult(std::vector<math_utils::Point2D> &smooth_reference_points,
                  std::deque<math_utils::Point2D> &smooth_first_derivative,
                  std::deque<math_utils::Point2D> &smooth_second_derivative,
                  std::deque<math_utils::Point2D> &smooth_third_derivative);

  bool
  SmoothReflineCheck(const std::vector<math_utils::Point2D> &smooth_refline,
                     const double &lateral_error_tolerence);

  void Clear();
  bool SampleAnchorPoints();

  bool AddConstraint();
  bool AddKernel();
  bool Solve();

private:
  uint32_t unittest_data_count_ = 0;
  bool unittest_flag_ = true;
  bool debug_print_ = false;

  void WriteSolverMessIntoProto();

  // SplineSmootherOnlineDatas GetOnlineDatasForProto();

  // SplineSmootherInitialState GetInitialStateForProto();

  // SplineSmootherRawReflineSize GetRawReflineSizeForProto();

private:
  uint32_t spline_order_ = 5;
  double spline_max_length_ = 25.0;
  double anchor_point_intervel_ = 10.0;
  double normal_anchor_point_bound_ = 0.10;
  double relax_anchor_point_bound_ = 0.20;
  double normal_reference_kernel_weight_ = 0.01;
  double relax_reference_kernel_weight_ = 0.01;
  double second_order_derivative_kernel_weight_ = 1.0;
  double third_order_derivative_kernel_weight_ = 1.0;
  double reference_point_intervel_ = 2.0;
  std::unique_ptr<OsqpSpline2dSolver> spline_solver_;

  double scale_{1.0};
  std::vector<double> t_coord_;
  std::vector<double> headings_;
  std::vector<PolyFit::math::Vec2d> xy_points_;
  std::vector<double> t_ref_coord_;
  std::vector<PolyFit::math::Vec2d> xy_ref_points_;

  std::vector<double> t_knots_{0.0, 10.0, 20.0, 30.0, 40.0, 50.0};
  std::deque<PathPoint> raw_reference_line_;

  std::vector<double> s_interpolate_coord_;
  std::vector<double> t_interpolate_coord_;
  std::deque<PathPoint> smooth_reference_line_;
  std::deque<math_utils::Point2D> smooth_first_derivative_;
  std::deque<math_utils::Point2D> smooth_second_derivative_;
  std::deque<math_utils::Point2D> smooth_third_derivative_;

  // {x, y, theta, ddx, ddy, dddx, dddy}
  std::vector<double> start_point_derivatives_ = {};

  bool relax_optimization_{false};
};

} // namespace PolyFit
