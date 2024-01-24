/*
 * Copyright (C) 2020 by SenseTime Group Limited. All rights reserved.
 * dingjie <dingjie1@senseauto.com>
 */

/**
 * @file
 **/

#pragma once

#include <vector>
#include "chrono"
#include "osqp.h"

#include "spline_smoothing/include/spline/spline_2d.hpp"
#include "spline_smoothing/include/spline/spline_2d_solver.hpp"

namespace PolyFit {

using Eigen::MatrixXd;

class OsqpSpline2dSolver final : public Spline2dSolver {
public:
  OsqpSpline2dSolver(const std::vector<double> &t_knots, const uint32_t order);

  void Reset(const std::vector<double> &t_knots, const uint32_t order) override;

  // customize setup
  Spline2dConstraint *mutable_constraint() override;
  Spline2dKernel *mutable_kernel() override;
  Spline2d *mutable_spline() override;

  void CalculateKernel(const MatrixXd &P, std::vector<c_float> *P_data,
                       std::vector<c_int> *P_indices,
                       std::vector<c_int> *P_indptr);

  void CalculateAffineConstraint(const MatrixXd &A,
                                 std::vector<c_float> *A_data,
                                 std::vector<c_int> *A_indices,
                                 std::vector<c_int> *A_indptr,
                                 std::vector<c_float> *lower_bounds,
                                 std::vector<c_float> *upper_bounds);

  // solve
  bool Solve() override;

  // extract
  const Spline2d &spline() const override;

private:
  OSQPSettings *osqp_settings_ = nullptr;
  OSQPWorkspace *work_ = nullptr; // Workspace
  OSQPData *data_ = nullptr;      // OSQPData

  int last_num_constraint_ = 0;
  int last_num_param_ = 0;
  bool last_problem_success_ = false;

  std::vector<double> t_knots_;
  uint32_t spline_order_;
  std::size_t total_params_;
  std::vector<double> t_coord_;
};

} // namespace PolyFit
