//
// Created by 冯晓彤 on 2023/4/28.
//

#pragma once

#include <cmath>
#include <sstream>
#include <string>
#include "vector"

namespace MathUtils {
typedef enum { FuncFailed = 0, FuncSucceed = 1 } FuncStatus;
struct Point2D {
  double x = 0.0;
  double y = 0.0;

  Point2D() = default;
  Point2D(double a, double b) : x(a), y(b){};

  double norm() const { return std::hypot(x, y); }

  Point2D operator+(const Point2D& b) const {
    Point2D c;
    c.x = this->x + b.x;
    c.y = this->y + b.y;
    return c;
  }

  Point2D operator-(const Point2D& b) const {
    Point2D c;
    c.x = this->x - b.x;
    c.y = this->y - b.y;
    return c;
  }

  friend std::ostream& operator<<(std::ostream& out, const Point2D& point) {
    out << "(" << point.x << ", " << point.y << ")";
    return out;
  }
};

struct Circle {
  Point2D center;
  double radius;

  Circle(const Point2D& center_, const double& r_)
      : center(center_), radius(r_) {}
};

double CalculateRadius(const Point2D& first_point,
                       const Point2D& second_point,
                       const Point2D& third_point);

template <typename X, typename Y>
bool interpolate(X x, Y& res, std::vector<X> x_std, std::vector<Y> y_std) {
  if (x_std.empty() || y_std.empty() || x_std.size() != y_std.size()) {
    return false;
  }
  if (x_std.size() < 2) {
    res = y_std.front();
    return true;
  }
  if (x < x_std.front()) {
    res = y_std.front();
    return true;
  }
  if (x > x_std.back()) {
    res = y_std.back();
    return true;
  }

  size_t left = 0;
  size_t right = 0;

  for (right = 1; right < y_std.size(); right++) {
    left = right - 1;
    if (x_std[right] > x) {
      break;
    }
  }
  double interval = x_std[right] - x_std[left];
  if (interval < 1e-3) {
    res = y_std[right];
    return true;
  }
  double rate = (x - x_std[left]) / interval;
  res = rate * (y_std[left] - y_std[right]) + y_std[left];
  return true;
}
}
