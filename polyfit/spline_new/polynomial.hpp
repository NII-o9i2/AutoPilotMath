#pragma once

#include "array"
#include "string"
#include "common/utils.h"

namespace PolyFit {

constexpr int N_DEG = 5;
constexpr int N_DIM = 2;
constexpr int N_CONTINUOUS = 4; // continuity up to jerk should be enough
constexpr int REGULATOR_TOP_DEGREE = 3;
constexpr std::array<int, 9> FACTORIALS = {1,   1,   2,    6,    24,
                                           120, 720, 5040, 40320};

template <int N> class Polynomial {
  /**
   * @brief Polynomial class, coeffs are stored in reversed order
   * The parameterization is given by
   * f(s) = coeffs_(n) + coeffs_(n-1)/1!^s + ... + coeffs_(0)/(n!)*s^n
   * f(s) = coeffs_normal_order_(0) + coeffs_normal_orders_(1)^s + ... +
   * coeffs_normal_order_(n)*s^n
   * coeffs are scaled to have straightforward physical
   * meaning, also improve the efficiency when evaluating the derivatives.
   */
public:
  using Coeffs = std::array<double, N + 1>;

  Polynomial() {
    coeffs_.fill(0.0);
    coeffs_normal_order_.fill(0.0);
  }

  explicit Polynomial(const Coeffs &coeffs) : coeffs_(coeffs) {
    for (int i = 0; i < coeffs_.size(); i++) {
      coeffs_normal_order_[i] = coeffs_[N - i] / FACTORIALS[i];
    }
  }

  inline double evaluate(const double s, const int degree) const {
    double ret = coeffs_[0] / FACTORIALS[N - degree];
    for (int i = 1; i + degree <= N; i++) {
      ret = ret * s + coeffs_[i] / FACTORIALS[N - degree - i];
    }
    return ret;
  }

  inline double evaluate(const double s) const {
    double ret = coeffs_normal_order_[N];
    for (int i = 1; i <= N; i++) {
      ret = ret * s + coeffs_normal_order_[N - i];
    }
    return ret;
  }

  std::string debug_string() const {
    std::stringstream ss;
    ss << "Polynomial_" << N;
    ss << " coeffs: ";
    for (const auto &coeff : coeffs_) {
      ss << coeff << ", ";
    }
    ss << " coeffs_normal_order: ";
    for (const auto &coeff : coeffs_normal_order_) {
      ss << coeff << ", ";
    }
    return ss.str();
  }

private:
  Coeffs coeffs_;
  Coeffs coeffs_normal_order_;
};

template <int N> class Polynomial2D {
public:
  using Coeffs = std::array<double, N + 1>;

  Polynomial2D() = default;

  Polynomial2D(const Coeffs &coeffs_x, const Coeffs &coeffs_y)
      : polys_({Polynomial<N>(coeffs_x), Polynomial<N>(coeffs_y)}) {}

  inline math_utils::Point2D evaluate(const double s, const int degree) const {
    return math_utils::Point2D(polys_[0].evaluate(s, degree),
                               polys_[1].evaluate(s, degree));
  }

  inline math_utils::Point2D evaluate(const double s) const {
    return math_utils::Point2D(polys_[0].evaluate(s), polys_[1].evaluate(s));
  }

  std::string debug_string() const {
    std::stringstream ss;
    ss << "Polynomial2D_" << N;
    ss << " x: ";
    ss << polys_[0].debug_string();
    ss << " y: ";
    ss << polys_[1].debug_string();
    return ss.str();
  }

  Polynomial<N> &operator[](const unsigned char j) {
    assert(j < 2);
    return polys_[j];
  }

private:
  std::array<Polynomial<N>, 2> polys_;
};

} // namespace PolyFit
