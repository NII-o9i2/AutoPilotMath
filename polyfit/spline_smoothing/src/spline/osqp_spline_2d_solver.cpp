/*
 * Copyright (C) 2020 by SenseTime Group Limited. All rights reserved.
 * dingjie <dingjie1@senseauto.com>
 */

/**
 * @file
 **/
// #include "ad_log/ad_log.hpp"
#include <iostream>

#include "spline_smoothing/include/spline/osqp_spline_2d_solver.hpp"
#include "spline_smoothing/include/math/matrix_operations.hpp"

namespace PolyFit {
namespace {
constexpr double kRoadBound = 1e10;
}

using PolyFit::math::DenseToCSCMatrix;

OsqpSpline2dSolver::OsqpSpline2dSolver(const std::vector<double> &t_knots,
                                       const uint32_t order)
    : Spline2dSolver(t_knots, order) {}

void OsqpSpline2dSolver::Reset(const std::vector<double> &t_knots,
                               const uint32_t order) {
  t_knots_ = t_knots;
  spline_order_ = order;
  total_params_ =
      (t_knots_.size() > 1 ? 2 * (t_knots_.size() - 1) * (1 + spline_order_)
                           : 0);
  spline_ = Spline2d(t_knots, order);
  kernel_ = Spline2dKernel(t_knots, order);
  constraint_ = Spline2dConstraint(t_knots, order);
}

// customize setup
Spline2dConstraint *OsqpSpline2dSolver::mutable_constraint() {
  return &constraint_;
}

Spline2dKernel *OsqpSpline2dSolver::mutable_kernel() { return &kernel_; }

Spline2d *OsqpSpline2dSolver::mutable_spline() { return &spline_; }

void OsqpSpline2dSolver::CalculateKernel(const MatrixXd &P,
                                         std::vector<c_float> *P_data,
                                         std::vector<c_int> *P_indices,
                                         std::vector<c_int> *P_indptr) {
  // Only upper triangle needs to be filled
  std::vector<std::vector<std::pair<c_int, c_float>>> columns;
  columns.resize(total_params_);

  int spline_num = t_knots_.size() - 1;
  for (uint32_t spline_index = 0; spline_index < spline_num; ++spline_index) {
    const uint32_t index_offset = 2 * spline_index * (spline_order_ + 1);
    for (uint32_t col = 0; col < spline_order_ + 1; ++col) {
      uint32_t col_offset = index_offset + col;
      for (uint32_t row = 0; row <= col; ++row) {
        uint32_t row_offset = index_offset + row;
        columns[col_offset].emplace_back(row_offset, P(row_offset, col_offset));
      }
    }

    for (uint32_t col = spline_order_ + 1; col < 2 * (spline_order_ + 1);
         ++col) {
      uint32_t col_offset = index_offset + col;
      for (uint32_t row = spline_order_ + 1; row <= col; ++row) {
        uint32_t row_offset = index_offset + row;
        columns[col_offset].emplace_back(row_offset, P(row_offset, col_offset));
      }
    }
  }

  int ind_p = 0;
  for (int i = 0; i < total_params_; ++i) {
    P_indptr->push_back(ind_p);
    for (const auto &row_data_pair : columns[i]) {
      // Rescale by 2.0 as the quadratic term in osqp default qp problem
      // setup is set as (1/2) * x' * P * x
      P_data->push_back(row_data_pair.second);
      P_indices->push_back(row_data_pair.first);
      ++ind_p;
    }
  }
  P_indptr->push_back(ind_p);
}

bool OsqpSpline2dSolver::Solve() {
  // std::cout << "enter Solve" << std::endl;

  // Namings here are following osqp convention.
  // For details, visit: https://osqp.org/docs/examples/demo.html

  auto start_spline_osqp_setup = std::chrono::system_clock::now();

  // change P to csc format
  const MatrixXd &P = kernel_.kernel_matrix();
  // AD_LINFO(OsqpSpline2dSolver) << "P: " << P.rows() << ", " << P.cols();
  if (P.rows() == 0) {
    return false;
  }

  // std::cout << "before CalculateKernel" << std::endl;

  std::vector<c_float> P_data;
  std::vector<c_int> P_indices;
  std::vector<c_int> P_indptr;
  // DenseToCSCMatrix(P, &P_data, &P_indices, &P_indptr);
  CalculateKernel(P, &P_data, &P_indices, &P_indptr);

  // std::cout << "after CalculateKernel" << std::endl;

  // change A to csc format
  const MatrixXd &inequality_constraint_matrix =
      constraint_.inequality_constraint().constraint_matrix();
  const MatrixXd &equality_constraint_matrix =
      constraint_.equality_constraint().constraint_matrix();
  // AD_LINFO(OsqpSpline2dSolver)
  //     << "inequality_constraint_matrix"
  //     << "  rows: " << inequality_constraint_matrix.rows()
  //     << "  cols: " << inequality_constraint_matrix.cols();
  // AD_LINFO(OsqpSpline2dSolver)
  //     << "equality_constraint_matrix"
  //     << "  rows: " << equality_constraint_matrix.rows()
  //     << "  cols: " << equality_constraint_matrix.cols();
  MatrixXd A(inequality_constraint_matrix.rows() +
                 equality_constraint_matrix.rows(),
             inequality_constraint_matrix.cols());
  if (inequality_constraint_matrix.rows() == 0) {
    A = equality_constraint_matrix;
  } else {
    A << inequality_constraint_matrix, equality_constraint_matrix;
  }
  // AD_LINFO(OsqpSpline2dSolver) << "A: " << A.rows() << ", " << A.cols();
  if (A.rows() == 0) {
    // AD_LDEBUG(OsqpSpline2dSolver) << "A.rows() == 0";
    return false;
  }

  // set q, l, u: l < A < u
  const MatrixXd &q_eigen = kernel_.offset();
  c_float q[q_eigen.rows()]; // NOLINT
  for (int i = 0; i < q_eigen.size(); ++i) {
    q[i] = q_eigen(i);
  }

  // Calculate affine constraints
  std::vector<c_float> A_data;
  std::vector<c_int> A_indices;
  std::vector<c_int> A_indptr;
  DenseToCSCMatrix(A, &A_data, &A_indices, &A_indptr);

  const MatrixXd &inequality_constraint_boundary =
      constraint_.inequality_constraint().constraint_boundary();
  const MatrixXd &equality_constraint_boundary =
      constraint_.equality_constraint().constraint_boundary();

  auto constraint_num = inequality_constraint_boundary.rows() +
                        equality_constraint_boundary.rows();

  static constexpr float kEpsilon = 1e-9f;
  static constexpr float kUpperLimit = 1e9f;
  c_float l[constraint_num]; // NOLINT
  c_float u[constraint_num]; // NOLINT
  for (int i = 0; i < constraint_num; ++i) {
    if (i < inequality_constraint_boundary.rows()) {
      l[i] = inequality_constraint_boundary(i, 0);
      u[i] = kUpperLimit;
    } else {
      const auto idx = i - inequality_constraint_boundary.rows();
      l[i] = equality_constraint_boundary(idx, 0) - kEpsilon;
      u[i] = equality_constraint_boundary(idx, 0) + kEpsilon;
    }
  }

  // Problem settings
  OSQPSettings *settings =
      reinterpret_cast<OSQPSettings *>(c_malloc(sizeof(OSQPSettings)));

  // Populate data
  OSQPData *data = reinterpret_cast<OSQPData *>(c_malloc(sizeof(OSQPData)));
  data->n = P.rows();
  data->m = constraint_num;
  data->P = csc_matrix(data->n, data->n, P_data.size(), P_data.data(),
                       P_indices.data(), P_indptr.data());
  data->q = q;
  data->A = csc_matrix(data->m, data->n, A_data.size(), A_data.data(),
                       A_indices.data(), A_indptr.data());
  data->l = l;
  data->u = u;

  // Define Solver settings as default
  osqp_set_default_settings(settings);
  settings->alpha = 1.0; // Change alpha parameter
  settings->eps_abs = 1.0e-05;
  settings->eps_rel = 1.0e-05;
  settings->eps_prim_inf = 1.0e-3;
  settings->eps_dual_inf = 1.0e-3;
  settings->max_iter = 1500;
  settings->polish = true;
  settings->verbose = false; // FLAGS_enable_osqp_debug;

  // // Setup workspace
  // OSQPWorkspace* work = osqp_setup(data, settings);
  OSQPWorkspace *work;
  osqp_setup(&work, data, settings);
  auto end_spline_osqp_setup = std::chrono::system_clock::now();
  std::chrono::duration<double> diff_spline_osqp_setup =
      end_spline_osqp_setup - start_spline_osqp_setup;
  // AD_LERROR(CostTime) << "SplineOsqpCSCMatrixTransAndOsqpSetup:"
  // << diff_spline_osqp_setup.count() * 1000.0 << " ms.";

  // Solve Problem
  auto start_spline_osqp_solve = std::chrono::system_clock::now();
  osqp_solve(work);
  auto end_spline_osqp_solve = std::chrono::system_clock::now();
  std::chrono::duration<double> diff_spline_osqp_solve =
      end_spline_osqp_solve - start_spline_osqp_solve;
  // AD_LERROR(CostTime) << "SplineOsqpSolve:"
  // << diff_spline_osqp_solve.count() * 1000.0 << " ms.";

  bool solved = true;
  auto status = work->info->status_val;
  // TODO(dingjie)
  // https://osqp.org/docs/interfaces/status_values.html
  //  1 means OSQP_SOLVED, 2 means OSQP_SOLVED_INACCURATE
  // -2 means OSQP_MAX_ITER_REACHED, QP problem may not be solved
  if (status != 1 && status != 2) {
    // AD_LERROR(OsqpSpline2dSolver) << "failed optimization status: "
    // << work->info->status;
    solved = false;
  }
  if (work->solution == nullptr) {
    // AD_LERROR(OsqpSpline2dSolver) << "solution nullptr ";
    solved = false;
  }
  // AD_LERROR(OsqpSpline2dSolver)
  // << "optimization status: " << status
  // << ",  number of iterations: " << work->info->iter
  // << ",  run_time(ms): " << work->info->run_time * 1000;

  std::cout << "OsqpSpline2dSolver::CalculateKernel, optimization status: " << status
            << ",  number of iterations: " << work->info->iter
            << ",  run_time(ms): " << work->info->run_time * 1000
            << std::endl;

  if (solved) {
    // if (status == 1) {
    //     if (work->info->status_polish == 1) {
    //         AD_LINFO(OsqpSpline2dSolver) << "solution polish:
    //         successful";
    //     } else if (work->info->status_polish < 0) {
    //         AD_LINFO(OsqpSpline2dSolver) << "solution polish:
    //         unsuccessful";
    //     }
    // }
    // if (status == 1 || status == 2) {
    //     AD_LINFO(OsqpSpline2dSolver)
    //         << "optimal objective: " << work->info->obj_val;
    // }

    MatrixXd solved_params = MatrixXd::Zero(P.rows(), 1);
    for (int i = 0; i < P.rows(); ++i) {
      solved_params(i, 0) = work->solution->x[i];
    }
    solved = spline_.set_splines(solved_params, spline_.spline_order());
  }

  // Cleanup
  osqp_cleanup(work);
  c_free(data->A);
  c_free(data->P);
  c_free(data);
  c_free(settings);

  return solved;
}

// extract
const Spline2d &OsqpSpline2dSolver::spline() const { return spline_; }

} // namespace PolyFit
