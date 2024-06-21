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

  double dot(const Point2D& p) const { return p.x * x + p.y * y; }

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

  template <typename N, std::enable_if_t<std::is_arithmetic<N>::value, int> = 0>
  Point2D operator*(const N scalar) const {
    Point2D q;
    q.x = this->x * scalar;
    q.y = this->y * scalar;
    return q;
  }

  template <typename N, std::enable_if_t<std::is_arithmetic<N>::value, int> = 0>
  friend Point2D operator*(const N scalar, const Point2D& rhs) {
    Point2D q;
    q.x = rhs.x * scalar;
    q.y = rhs.y * scalar;
    return q;
  }

  template <typename N, std::enable_if_t<std::is_arithmetic<N>::value, int> = 0>
  Point2D operator/(const N scalar) {
    Point2D c;
    constexpr double kEps = 1e-8;
    if (std::abs(scalar) < kEps) {
      return c;
    }
    c.x = this->x / scalar;
    c.y = this->y / scalar;
    return c;
  }

  friend std::ostream& operator<<(std::ostream& out, const Point2D& point) {
    out << "(" << point.x << ", " << point.y << ")";
    return out;
  }
};

struct FrenetPoint2D {
  double s = 0;
  double l = 0;

  FrenetPoint2D() = default;
  explicit FrenetPoint2D(double s_, double l_) : s(s_), l(l_) {}
  explicit FrenetPoint2D(const Point2D& pt) : s(pt.x), l(pt.y) {}

  double norm() const { return std::hypot(s, l); }

  FrenetPoint2D norm_direction() const {
    FrenetPoint2D p;
    if (norm() > 0.0001) {
      p.s = this->s / this->norm();
      p.l = this->l / this->norm();
    } else {
      p.s = this->s;
      p.l = this->l;
    }
    return p;
  }

  double squared_norm() const { return (s * s + l * l); }

  double dot(const FrenetPoint2D& p) const { return p.s * s + p.l * l; }

  double cross(const FrenetPoint2D& p) const {
    return this->s * p.l - this->l * p.s;
  }

  friend std::ostream& operator<<(std::ostream& out,  // NOLINT
                                  const FrenetPoint2D& point) {
    out << "(" << point.s << ", " << point.l << ")";
    return out;
  }

  void operator=(const FrenetPoint2D& b) {
    this->s = b.s;
    this->l = b.l;
  }

  FrenetPoint2D operator-() const {
    FrenetPoint2D p;
    p.s = -this->s;
    p.l = -this->l;
    return p;
  }

  FrenetPoint2D operator+(const FrenetPoint2D& b) const {
    FrenetPoint2D c;
    c.s = this->s + b.s;
    c.l = this->l + b.l;
    return c;
  }

  FrenetPoint2D& operator+=(const FrenetPoint2D& b) {
    this->s += b.s;
    this->l += b.l;
    return *this;
  }

  FrenetPoint2D operator-(const FrenetPoint2D& b) const {
    FrenetPoint2D c;
    c.s = this->s - b.s;
    c.l = this->l - b.l;
    return c;
  }

  template <typename N, std::enable_if_t<std::is_arithmetic<N>::value, int> = 0>
  FrenetPoint2D operator*(const N scalar) const {
    FrenetPoint2D q;
    q.s = this->s * scalar;
    q.l = this->l * scalar;
    return q;
  }

  template <typename N, std::enable_if_t<std::is_arithmetic<N>::value, int> = 0>
  friend FrenetPoint2D operator*(const N scalar, const FrenetPoint2D& rhs) {
    FrenetPoint2D q;
    q.s = rhs.s * scalar;
    q.l = rhs.l * scalar;
    return q;
  }

  template <typename N, std::enable_if_t<std::is_arithmetic<N>::value, int> = 0>
  FrenetPoint2D operator/(const N scalar) {
    FrenetPoint2D c;
    constexpr double kEps = 1e-8;
    if (std::abs(scalar) < kEps) {
      // AD_LERROR(FrenetPoint2D) << "scalar close to 0";
      return c;
    }
    c.s = this->s / scalar;
    c.l = this->l / scalar;
    return c;
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
  res = rate * (y_std[right] - y_std[left]) + y_std[left];
  return true;
}

bool is_float_equal(double a, double b);

/**
 * @brief normalize_angle normalize angle to [-pi, pi]
 *
 * @param angle is a angle with arbitrary range [unit, rad]
 *
 * @return normailized angle within [-pi, pi]
 */
double normalize_angle(const double angle);
}
