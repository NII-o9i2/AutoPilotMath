//
// Created by 冯晓彤 on 2023/4/28.
//

#ifndef BENCH_TEST_POLYFIT_H
#define BENCH_TEST_POLYFIT_H

#include "Eigen/Dense"
#include "common/utils.h"
#include "vector"
namespace PolyFit {
bool PolyLineFit(const std::vector<math_utils::Point2D> &points,
                 const size_t order, Eigen::VectorXd &line,
                 bool x_independence = true);

bool PolyFitSplineSmooth(const std::vector<math_utils::Point2D> &raw_points,
                         std::vector<math_utils::Point2D> &smooth_points);

bool PolyFitSplineSmoothNew(const std::vector<math_utils::Point2D> &raw_points,
                            std::vector<math_utils::Point2D> &smooth_points, std::vector<double> &curva_list);

bool PolyFitSplineSmoothNewPiece(const std::vector<math_utils::Point2D> &raw_points,
                                 std::vector<std::vector<math_utils::Point2D>> &smooth_points,std::vector<double>& curva_list);

std::vector<math_utils::Point2D>
Gen_random_curv(const double &noise_std, const double &A0, const double &A1,
                const double &A2, const double &A3, const double &B0,
                const double &B1, const double &B2, const double &B3,
                const int &point_size, const double& point_resolution);
}
#endif // BENCH_TEST_POLYFIT_H
