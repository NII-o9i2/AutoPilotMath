//
// Created by SENSETIME\fengxiaotong on 23-12-8.
//

#ifndef ALGORITHM_ALGORITHM_MATH_COMMON_SPLINE_H_
#define ALGORITHM_ALGORITHM_MATH_COMMON_SPLINE_H_
#include "utils.h"
#include "vector"
void interpolate_points(const std::vector<MathUtils::Point2D> &input_points,
                        std::vector<MathUtils::Point2D> &output_points,
                        const int &output_points_num);

MathUtils::Point2D get_point_via_s(
    const std::vector<MathUtils::Point2D> &ref_points, double s);
#endif  // ALGORITHM_ALGORITHM_MATH_COMMON_SPLINE_H_
