//
// Created by mm on 22-11-18.
//

#pragma once

#include "osqp.h"
#include <vector>
#include <memory>
#include <string>
#include <sstream>
#include <array>
#include <iostream>

namespace PolyFit {

struct Triplet {
  int x;      ///< row index
  int y;      ///< column index
  double val; ///< numerical value
  Triplet() : x(0), y(0), val(0.0) {}
  Triplet(int x_in, int y_in, double val_in) : x(x_in), y(y_in), val(val_in) {}
};

struct CSC {
  int nzmax; ///< maximum number of entries
  int m;     ///< number of rows
  int n;     ///< number of columns
  ///< column pointers (size n+1); col indices (size nzmax) start from 0 when
  ///< using triplet format (direct KKT matrix formation)
  std::vector<long long> p;
  std::vector<long long> i; ///< row indices, size nzmax starting from 0
  std::vector<double> x;    ///< numerical values, size nzmax
  int nz;                   ///< number of entries in triplet matrix, -1 for csc
  CSC() : nzmax(0), m(0), n(0), nz(-1) {}
  CSC(int nzmax, int m, int n) : nzmax(nzmax), m(m), n(n), nz(-1) {
    p.resize(n + 1);
    i.resize(nzmax);
    x.resize(nzmax);
  }
  void Reset() {
    nzmax = 0;
    m = 0;
    n = 0;
    nz = -1;
    p.clear();
    i.clear();
    x.clear();
  }
  void Reserve(int nzmax_in, int m_in, int n_in) {
    this->nzmax = nzmax_in;
    this->m = m_in;
    this->n = n_in;
    p.resize(n_in + 1);
    i.resize(nzmax_in);
    x.resize(nzmax_in);
  }
  void SetFromTriplets(const std::vector<Triplet> &triplets, int m_in,
                       int n_in) {
    int nzmax_in = static_cast<int>(triplets.size());
    this->Reserve(nzmax_in, m_in, n_in);
    p[n_in] = nzmax_in;
    int last_col = -1;
    int A_p_idx = 0;
    for (int idx = 0; idx < nzmax_in; idx++) {
      const auto &t = triplets[idx];
      if (t.y != last_col) {
        for (int j = last_col + 1; j <= t.y; j++) {
          p[A_p_idx++] = idx;
        }
        last_col = t.y;
      }
      i[idx] = t.x;
      x[idx] = t.val;
    }
  }
};

namespace OSQP {

struct OsqpInstance {
  long long num_variables() const {
    return static_cast<long long>(objective_matrix.n);
  }
  long long num_constraints() const {
    return static_cast<long long>(constraint_matrix.m);
  }
  CSC objective_matrix;
  std::vector<double> objective_vector;
  CSC constraint_matrix;
  std::vector<double> lower_bounds;
  std::vector<double> upper_bounds;

  OsqpInstance() = default;
};

using c_int = long long;
struct OsqpSettings {
  OsqpSettings(); // Sets default values.

  double rho;
  double sigma;
  c_int scaling;
  bool adaptive_rho;
  c_int adaptive_rho_interval;
  double adaptive_rho_tolerance;
  double adaptive_rho_fraction;
  c_int max_iter;
  double eps_abs;
  double eps_rel;
  double eps_prim_inf;
  double eps_dual_inf;
  double alpha;
  // linsys_solver is omitted. We don't change this.
  double delta;
  bool polish;
  c_int polish_refine_iter;
  bool verbose;
  bool scaled_termination;
  c_int check_termination;
  bool warm_start;
  double time_limit;
};

enum class OsqpExitCode {
  kOptimal,           // Optimal solution found.
  kPrimalInfeasible,  // Certificate of primal infeasibility found.
  kDualInfeasible,    // Certificate of dual infeasibility found.
  kOptimalInaccurate, // Optimal solution found subject to reduced tolerances
  kPrimalInfeasibleInaccurate, // Certificate of primal infeasibility found
                               // subject to reduced tolerances.
  kDualInfeasibleInaccurate,   // Certificate of dual infeasibility found
                               // subject to reduced tolerances.
  kMaxIterations,              // Maximum number of iterations reached.
  kInterrupted,                // Interrupted by signal or CTRL-C.
  kTimeLimitReached,           // Ran out of time.
  kNonConvex,                  // The problem was found to be non-convex.
  kUnknown,                    // Unknown problem in solver.
};

std::string ToString(OsqpExitCode exitcode);

struct OSQPWorkspaceHelper;

class OsqpSolver {
public:
  OsqpSolver() = default;
  // Move-only.
  OsqpSolver(OsqpSolver &&rhs) = default;
  OsqpSolver &operator=(OsqpSolver &&rhs) = default;
  OsqpSolver(const OsqpSolver &) = delete;
  OsqpSolver &operator=(const OsqpSolver &) = delete;
  using Status = std::pair<bool, std::string>;
  Status Init(OsqpInstance &instance, const OsqpSettings &settings);
  // Returns true if Init() has been called successfully.
  bool IsInitialized() const { return workspace_ != nullptr; }

  OsqpExitCode Solve();

  // The number of iterations taken. CHECK-fails if IsInitialized() is false.
  long long iterations() const;

  // The objective value of the primal solution. CHECK-fails if IsInitialized()
  // is false.
  double objective_value() const;

  // The primal solution, i.e., x. The Map is valid only for the lifetime of
  // the OSQP workspace. It will be invalidated by a call to Init() or if the
  // OsqpSolver is deleted. CHECK-fails if IsInitialized() is false.
  // Implementation details (do not depend on these): The underlying memory is
  // overwritten by SetPrimalWarmStart(). Modification of the problem data does
  // not destroy the solution.
  std::vector<double> primal_solution() const;
  Status GetPrimalSolutionAtIndex(int index, double *value) const;

  // The vector of lagrange multipliers on the linear constraints. The Map is
  // valid only for the lifetime of the OSQP workspace. It will be invalidated
  // by a call to Init() or if the OsqpSolver is deleted. CHECK-fails if
  // IsInitialized() is false. Implementation details (do not depend on these):
  // The underlying memory is overwritten by SetDualWarmStart(). Modification of
  // the problem data does not destroy the solution.
  std::vector<double> dual_solution() const;

private:
  struct OsqpDeleter {
    void operator()(OSQPWorkspaceHelper *workspace) const;
  };

  std::unique_ptr<OSQPWorkspaceHelper, OsqpDeleter> workspace_;
};

} // namespace OSQP

} // namespace PolyFit