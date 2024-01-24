#pragma once

#include "polynomial.hpp"
#include "common/utils.h"
#include <iostream>
#include <utility>

namespace PolyFit {

template <int N> class Spline2D {
public:
  enum class EvaluateStatus {
    OK = 0,
    OUT_OF_RANGE = 1,
  };

  using QuinticPoly2D = Polynomial2D<N>;

  Spline2D() = default;

  bool set_domains_and_polys(const std::vector<double> &domains,
                             const std::vector<QuinticPoly2D> &polys) {
    if (polys.empty()) {
      return false;
    }
    if (domains.size() != polys.size() + 1) {
      return false;
    }
    if (!std::is_sorted(domains.begin(), domains.end())) {
      return false;
    }

    accumulate_domains_ = domains;
    polys_ = polys;

    circles_.clear();
    circles_.reserve(polys_.size());
    for (int i = 0; i < polys_.size(); ++i) {
      double l = accumulate_domains_[i + 1] - accumulate_domains_[i];
      const auto &seg = polys_[i];
      auto pt_center = seg.evaluate(0.5 * l);
      const double radius = l / 2 + 5.0;
      circles_.emplace_back(pt_center, radius);
    }
    return true;
  }

  int get_num_segments() const { return static_cast<int>(polys_.size()); }

  const std::vector<double> &get_accumulate_domains() const {
    return accumulate_domains_;
  }

  double get_start_domain() const { return accumulate_domains_.front(); }

  double get_end_domain() const { return accumulate_domains_.back(); }

  const std::vector<QuinticPoly2D> &get_polys() const { return polys_; }

  std::pair<EvaluateStatus, math_utils::Point2D>
  evaluate(const double s, const int degree) const {
    if (s < accumulate_domains_.front()) {
      return {EvaluateStatus::OUT_OF_RANGE, polys_.front().evaluate(s, degree)};
    }
    if (s >= accumulate_domains_.back()) {
      return {
          EvaluateStatus::OUT_OF_RANGE,
          polys_.back().evaluate(
              s - accumulate_domains_[accumulate_domains_.size() - 2], degree)};
    }

    auto iter = std::upper_bound(accumulate_domains_.begin(),
                                 accumulate_domains_.end(), s);
    int idx = iter - 1 - accumulate_domains_.begin();
    auto start_s = accumulate_domains_[idx];

    // std::cout << "s " << s << " idx " << idx << std::endl;

    return {EvaluateStatus::OK, polys_[idx].evaluate(s - start_s, degree)};
  }

  std::pair<EvaluateStatus, math_utils::Point2D>
  evaluate(const double s) const {
    if (s < accumulate_domains_.front()) {
      return {EvaluateStatus::OUT_OF_RANGE, polys_.front().evaluate(0.0)};
    }
    if (s >= accumulate_domains_.back()) {
      return {EvaluateStatus::OUT_OF_RANGE,
              polys_.back().evaluate(
                  s - accumulate_domains_[accumulate_domains_.size() - 2])};
    }

    // auto idx = fplus::find_last_idx_by(
    //     [s](const double &domain) { return s >= domain; },
    //     accumulate_domains_);
    // return {EvaluateStatus::OK,
    //         polys_[idx.unsafe_get_just()].Evaluate(
    //             s - accumulate_domains_[idx.unsafe_get_just()])};

    auto iter = std::upper_bound(accumulate_domains_.begin(),
                                 accumulate_domains_.end(), s);
    int idx = iter - 1 - accumulate_domains_.begin();
    auto start_s = accumulate_domains_[idx];

    // std::cout << "s " << s << " idx " << idx << " domain_start_s " << start_s << std::endl;

    return {EvaluateStatus::OK, polys_[idx].evaluate(s - start_s)};
  }

  Polynomial<N> &operator()(const uint8_t idx, uint8_t dim) {
    assert(idx < polys_.size());
    return polys_[idx](dim);
  }

  const QuinticPoly2D &get_poly_at(const uint8_t idx) const {
    assert(idx < polys_.size());
    return polys_[idx];
  }

  const math_utils::Circle &get_circle_at(const uint8_t idx) const {
    assert(idx < polys_.size());
    return circles_[idx];
  }

  std::string debug_string() const {
    std::stringstream ss;
    ss << "Spline2D_" << N;
    ss << " accumulate_domains: ";
    for (const auto &num : accumulate_domains_) {
      ss << num << ", ";
    }
    ss << "polys: ";
    for (const auto &poly : polys_) {
      ss << poly.debug_string();
    }
    return ss.str();
  }

private:
  std::vector<QuinticPoly2D> polys_;
  std::vector<double> accumulate_domains_;
  std::vector<math_utils::Circle> circles_;
};

} // namespace PolyFit