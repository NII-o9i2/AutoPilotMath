//
// Created by 冯晓彤 on 2023/4/28.
//
#include "Eigen/Dense"
#include "iostream"
#include "random"

#include "polyfit.h"
#include "spline_new/spline_reference_smoother.hpp"
#include "spline_smoothing/include/spline_reference_smoother.hpp"

namespace PolyFit {
std::vector<math_utils::Point2D>
Gen_random_curv(const double &noise_std, const double &A0, const double &A1,
                const double &A2, const double &A3, const double &B0,
                const double &B1, const double &B2, const double &B3,
                const int &point_size, const double& point_resolution) {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::normal_distribution<double> noise(
      0.0, noise_std); // 0均值，0.1标准差的高斯噪声

  auto line_size = static_cast<size_t>(std::max(0, point_size));
  std::vector<math_utils::Point2D> res;
  res.reserve(line_size);

  if (line_size == 0) {
    return res;
  }

  // 生成point
  //    constexpr int MaxParamT = 100;
  int i = 0;
  for (int t = 0; i < point_size; t+=point_resolution, ++i) {
    double x = static_cast<double>(A0 + A1 * t + A2 * t * t + A3 * t * t * t) +
               noise(gen);
    double y = static_cast<double>(B0 + B1 * t + B2 * t * t + B3 * t * t * t) +
               noise(gen);
    res.emplace_back(x, y);
  }

  return res;
}

bool PolyLineFit(const std::vector<math_utils::Point2D> &points,
                 const size_t order, Eigen::VectorXd &line,
                 bool x_independence) {
  if (points.size() < 2) {
    std::cout << "Wrong parameter with points: " << points.size() << std::endl;
    return false;
  }
  if (order > points.size() - 1) {
    std::cout << "points number-" << points.size() << " is too less to fit "
              << order << "-order line" << std::endl;
    return false;
  }

  Eigen::MatrixXd A(points.size(), order + 1);
  for (size_t i = 0; i < points.size(); ++i) {
    A(i, order) = 1.0;
  }

  Eigen::VectorXd b(points.size());
  for (size_t i = 0; i < points.size(); ++i) {
    auto x = points[i].x;
    auto y = points[i].y;
    if (!x_independence) {
      std::swap(x, y);
    }
    for (int j = order - 1; j >= 0; --j) {
      A(i, j) = A(i, j + 1) * x;
    }
    b(i) = y;
  }
  line = A.householderQr().solve(b);
  return true;
}

bool PolyFitSplineSmooth(const std::vector<math_utils::Point2D> &raw_points,
                         std::vector<math_utils::Point2D> &smooth_points) {
  SplineReferenceSmoother smoother;

  bool relax_optimization = false;
  std::deque<math_utils::Point2D> smooth_first_derivative;
  std::deque<math_utils::Point2D> smooth_second_derivative;
  std::deque<math_utils::Point2D> smooth_third_derivative;
  return smoother.SplineSmooth(
      relax_optimization, raw_points, smooth_points, smooth_first_derivative,
      smooth_second_derivative, smooth_third_derivative);
}

bool PolyFitSplineSmoothNew(const std::vector<math_utils::Point2D> &raw_points,
                            std::vector<math_utils::Point2D> &smooth_points,std::vector<double>& curva_list) {
  SplineReferenceSmootherNew smoother;
  return smoother.SplineSmooth(raw_points, smooth_points, curva_list);
}

bool PolyFitSplineSmoothNewPiece(const std::vector<math_utils::Point2D> &raw_points,
                                std::vector<std::vector<math_utils::Point2D>> &smooth_points,std::vector<double>& curva_list) {
        SplineReferenceSmootherNew smoother;
        return smoother.SplineSmoothPiece(raw_points, smooth_points, curva_list);
}

}