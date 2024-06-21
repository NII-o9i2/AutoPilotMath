//
// Created by 冯晓彤 on 2023/4/28.
//
#include "utils.h"
#include <iostream>

namespace MathUtils {

double CalculateRadius(const Point2D& first_point,
                       const Point2D& second_point,
                       const Point2D& third_point) {
  double radius = 10000000;
  double A1, A2, B1, B2, C1, C2, denominator;
  A1 = first_point.x - second_point.x;
  B1 = first_point.y - second_point.y;
  C1 = (std::pow(first_point.x, 2) - std::pow(second_point.x, 2) +
        std::pow(first_point.y, 2) - std::pow(second_point.y, 2)) /
       2.0;
  A2 = third_point.x - second_point.x;
  B2 = third_point.y - second_point.y;
  C2 = (std::pow(third_point.x, 2) - std::pow(second_point.x, 2) +
        std::pow(third_point.y, 2) - std::pow(second_point.y, 2)) /
       2.0;
  denominator = A1 * B2 - A2 * B1;
  Point2D center;
  if (std::fabs(denominator) < 1.0e-4) {
    radius = 10000000;
  } else {
    center.x = (C1 * B2 - C2 * B1) / denominator;
    center.y = (A1 * C2 - A2 * C1) / denominator;
    radius = std::sqrt((center.x - first_point.x) * (center.x - first_point.x) +
                       (center.y - first_point.y) * (center.y - first_point.y));
  }
  radius =
      std::atan2(first_point.y - center.y, first_point.x - center.x) <
              std::atan2(third_point.y - center.y, third_point.x - center.x)
          ? radius
          : -radius;
  //   std::cout << "radius " << radius << " center "
  //             << center << " point_1 " << first_point
  //             << " point_2 " << second_point << " point_3 " << third_point
  //             << std::endl;
  return radius;
}

bool is_float_equal(double a, double b) {
  constexpr double kEps = 1e-4;
  if (std::abs(a - b) < kEps) {
    return true;
  }
  return false;
}

double normalize_angle(const double angle) {
  double normalized_angle = angle;
  if (normalized_angle > M_PI) {
    normalized_angle = normalized_angle -
                       std::round(normalized_angle / (2.0 * M_PI)) * 2.0 * M_PI;
  } else if (normalized_angle < -1.0 * M_PI) {
    normalized_angle =
        normalized_angle +
        std::round(normalized_angle / (-2.0 * M_PI)) * 2.0 * M_PI;
  }
  return normalized_angle;
}
}