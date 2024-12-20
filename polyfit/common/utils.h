//
// Created by 冯晓彤 on 2023/4/28.
//

#ifndef BENCH_TEST_UTILS_H
#define BENCH_TEST_UTILS_H

#include <cmath>
#include <sstream>
#include <string>

namespace math_utils {
struct Point2D {
  double x = 0.0;
  double y = 0.0;

  Point2D() = default;
  Point2D(double a, double b) : x(a), y(b){};

  double norm() const { return std::hypot(x, y); }

  Point2D operator+(const Point2D &b) const {
    Point2D c;
    c.x = this->x + b.x;
    c.y = this->y + b.y;
    return c;
  }

  Point2D operator-(const Point2D &b) const {
    Point2D c;
    c.x = this->x - b.x;
    c.y = this->y - b.y;
    return c;
  }

  friend std::ostream& operator<<(std::ostream& out,
                                  const Point2D& point) {
    out << "(" << point.x << ", " << point.y << ")";
    return out;
  }
};

struct Circle {
  Point2D center;
  double radius;

  Circle(const Point2D &center_, const double &r_)
      : center(center_), radius(r_) {}
};
}
#endif // BENCH_TEST_UTILS_H
