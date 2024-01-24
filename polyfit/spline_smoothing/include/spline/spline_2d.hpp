/*
 * Copyright (C) 2020 by SenseTime Group Limited. All rights reserved.
 * dingjie <dingjie1@senseauto.com>
 */

/**
 * @file : spline_2d.h
 * @brief: piecewise smoothing spline 2d class
 **/

#pragma once

#include <utility>
#include <vector>

#include "Eigen/Core"

#include "spline_smoothing/include/math/polynomial_xd.hpp"
#include "spline_smoothing/include/spline/spline_2d_seg.hpp"

namespace PolyFit {

class Spline2d {
public:
  Spline2d(const std::vector<double> &t_knots, const uint32_t order);
  std::pair<double, double> operator()(const double t) const;
  double x(const double t) const;
  double y(const double t) const;
  double DerivativeX(const double t) const;
  double DerivativeY(const double t) const;
  double SecondDerivativeX(const double t) const;
  double SecondDerivativeY(const double t) const;
  double ThirdDerivativeX(const double t) const;
  double ThirdDerivativeY(const double t) const;
  bool set_splines(const Eigen::MatrixXd &params, const uint32_t order);
  const Spline2dSeg &smoothing_spline(const uint32_t index) const;
  const std::vector<double> &t_knots() const;
  uint32_t spline_order() const;

private:
  uint32_t find_index(const double x) const;

private:
  std::vector<Spline2dSeg> splines_;
  std::vector<double> t_knots_;
  uint32_t spline_order_;
};

} // namespace PolyFit
