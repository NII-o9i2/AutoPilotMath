/*
 * Copyright (C) 2020 by SenseTime Group Limited. All rights reserved.
 * dingjie <dingjie1@senseauto.com>
 */

/**
 * @file : affine_constraint.h
 **/

#pragma once

#include "Eigen/Core"
#include "spline_smoothing/include/math/polynomial_xd.hpp"

namespace PolyFit {

class AffineConstraint {
public:
  AffineConstraint() = default;
  explicit AffineConstraint(const bool is_equality);
  AffineConstraint(const Eigen::MatrixXd &constraint_matrix,
                   const Eigen::MatrixXd &constraint_boundary,
                   const bool is_equality);

  void SetIsEquality(const double is_equality);

  const Eigen::MatrixXd &constraint_matrix() const;
  const Eigen::MatrixXd &constraint_boundary() const;
  bool AddConstraint(const Eigen::MatrixXd &constraint_matrix,
                     const Eigen::MatrixXd &constraint_boundary);

private:
  Eigen::MatrixXd constraint_matrix_;
  Eigen::MatrixXd constraint_boundary_;
  bool is_equality_ = true;
};

} // namespace PolyFit
