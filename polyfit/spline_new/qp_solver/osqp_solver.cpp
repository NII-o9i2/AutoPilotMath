//
// Created by mm on 22-11-18.
//
#include "osqp_solver.h"

#include <vector>
#include <cassert>
#include <string>

namespace PolyFit {

namespace OSQP {
namespace {
void CopyFromInternalSettings(const ::OSQPSettings &osqp_settings,
                              OsqpSettings *settings) {
  settings->rho = osqp_settings.rho;
  settings->sigma = osqp_settings.sigma;
  settings->scaling = osqp_settings.scaling;
  settings->adaptive_rho = osqp_settings.adaptive_rho;
  settings->adaptive_rho_interval = osqp_settings.adaptive_rho_interval;
  settings->adaptive_rho_tolerance = osqp_settings.adaptive_rho_tolerance;
  settings->adaptive_rho_fraction = osqp_settings.adaptive_rho_fraction;
  settings->max_iter = osqp_settings.max_iter;
  settings->eps_abs = osqp_settings.eps_abs;
  settings->eps_rel = osqp_settings.eps_rel;
  settings->eps_prim_inf = osqp_settings.eps_prim_inf;
  settings->eps_dual_inf = osqp_settings.eps_dual_inf;
  settings->alpha = osqp_settings.alpha;
  settings->delta = osqp_settings.delta;
  settings->polish = osqp_settings.polish;
  settings->polish_refine_iter = osqp_settings.polish_refine_iter;
  settings->verbose = osqp_settings.verbose;
  settings->scaled_termination = osqp_settings.scaled_termination;
  settings->check_termination = osqp_settings.check_termination;
  settings->warm_start = osqp_settings.warm_start;
  settings->time_limit = osqp_settings.time_limit;
}

::OSQPSettings ToInternalSettings(const OsqpSettings &settings) {
  OSQPSettings osqp_settings;
  osqp_settings.rho = settings.rho;
  osqp_settings.sigma = settings.sigma;
  osqp_settings.scaling = settings.scaling;
  osqp_settings.adaptive_rho = settings.adaptive_rho;
  osqp_settings.adaptive_rho_interval = settings.adaptive_rho_interval;
  osqp_settings.adaptive_rho_tolerance = settings.adaptive_rho_tolerance;
  osqp_settings.adaptive_rho_fraction = settings.adaptive_rho_fraction;
  osqp_settings.max_iter = settings.max_iter;
  osqp_settings.eps_abs = settings.eps_abs;
  osqp_settings.eps_rel = settings.eps_rel;
  osqp_settings.eps_prim_inf = settings.eps_prim_inf;
  osqp_settings.eps_dual_inf = settings.eps_dual_inf;
  osqp_settings.alpha = settings.alpha;
  osqp_settings.delta = settings.delta;
  osqp_settings.polish = settings.polish;
  osqp_settings.polish_refine_iter = settings.polish_refine_iter;
  osqp_settings.verbose = settings.verbose;
  osqp_settings.scaled_termination = settings.scaled_termination;
  osqp_settings.check_termination = settings.check_termination;
  osqp_settings.warm_start = settings.warm_start;
  osqp_settings.time_limit = settings.time_limit;
  osqp_settings.linsys_solver = ::QDLDL_SOLVER;
  return osqp_settings;
}
} // namespace

OsqpSettings::OsqpSettings() {
  ::OSQPSettings osqp_settings;
  osqp_set_default_settings(&osqp_settings);
  CopyFromInternalSettings(osqp_settings, this);
}

struct OSQPWorkspaceHelper : public ::OSQPWorkspace {};

void OsqpSolver::OsqpDeleter::operator()(OSQPWorkspaceHelper *workspace) const {
  osqp_cleanup(workspace);
}

// OSQP_HANDLE_EXITCODE(x) expands to 'case x: return "x"'. Using this macro
// prevents typos in the strings.
#define OSQP_HANDLE_EXITCODE(x)                                                \
  case x:                                                                      \
    return #x

std::string ToString(OsqpExitCode exitcode) {
  switch (exitcode) {
    OSQP_HANDLE_EXITCODE(OsqpExitCode::kOptimal);
    OSQP_HANDLE_EXITCODE(OsqpExitCode::kPrimalInfeasible);
    OSQP_HANDLE_EXITCODE(OsqpExitCode::kDualInfeasible);
    OSQP_HANDLE_EXITCODE(OsqpExitCode::kOptimalInaccurate);
    OSQP_HANDLE_EXITCODE(OsqpExitCode::kPrimalInfeasibleInaccurate);
    OSQP_HANDLE_EXITCODE(OsqpExitCode::kDualInfeasibleInaccurate);
    OSQP_HANDLE_EXITCODE(OsqpExitCode::kMaxIterations);
    OSQP_HANDLE_EXITCODE(OsqpExitCode::kInterrupted);
    OSQP_HANDLE_EXITCODE(OsqpExitCode::kTimeLimitReached);
    OSQP_HANDLE_EXITCODE(OsqpExitCode::kNonConvex);
    OSQP_HANDLE_EXITCODE(OsqpExitCode::kUnknown);
  }
  return "Unknown exit code";
}

OsqpSolver::Status CheckDimensions(const int left_value, const int right_value,
                                   const std::string &left_name,
                                   const std::string &right_name) {
  if (left_value != right_value) {
    std::array<std::string, 9> strs{"Dimension mismatch: ",
                                    left_name,
                                    " (= ",
                                    std::to_string(left_value),
                                    ") must equal ",
                                    right_name,
                                    " (= ",
                                    std::to_string(right_value),
                                    ")."};
    std::stringstream ss;
    for (const auto &tmp_str : strs) {
      ss << tmp_str;
    }
    return {false, ss.str()};
  } else {
    return {true, ""};
  }
}

#define OSQP_CHECK_DIMENSIONS(left_value, right_value)                         \
  CheckDimensions((left_value), (right_value), #left_value, #right_value)

#define OSQP_RETURN_IF_ERROR(expr)                                             \
  {                                                                            \
    const OsqpSolver::Status result = expr;                                    \
    if (!result.first)                                                         \
      return result;                                                           \
  }

#define OSQP_CHECK(expr) assert(expr)

#undef OSQP_HANDLE_EXITCODE
OsqpSolver::Status OsqpSolver::Init(OsqpInstance &instance,
                                    const OsqpSettings &settings) {
  const c_int num_variables = instance.num_variables();
  const c_int num_constraints = instance.num_constraints();

  OSQP_RETURN_IF_ERROR(
      OSQP_CHECK_DIMENSIONS(instance.objective_matrix.n, num_variables))
  OSQP_RETURN_IF_ERROR(
      OSQP_CHECK_DIMENSIONS(instance.objective_matrix.m, num_variables))
  OSQP_RETURN_IF_ERROR(
      OSQP_CHECK_DIMENSIONS(instance.objective_vector.size(), num_variables))
  OSQP_RETURN_IF_ERROR(
      OSQP_CHECK_DIMENSIONS(instance.lower_bounds.size(), num_constraints))
  OSQP_RETURN_IF_ERROR(
      OSQP_CHECK_DIMENSIONS(instance.upper_bounds.size(), num_constraints))

  ::csc objective_matrix = {instance.objective_matrix.nzmax,
                            instance.objective_matrix.m,
                            instance.objective_matrix.n,
                            instance.objective_matrix.p.data(),
                            instance.objective_matrix.i.data(),
                            instance.objective_matrix.x.data(),
                            -1};

  ::csc constraint_matrix = {instance.constraint_matrix.nzmax,
                             instance.constraint_matrix.m,
                             instance.constraint_matrix.n,
                             instance.constraint_matrix.p.data(),
                             instance.constraint_matrix.i.data(),
                             instance.constraint_matrix.x.data(),
                             -1};
  OSQPData data;
  data.n = num_variables;
  data.m = num_constraints;

  data.P = &objective_matrix;
  data.A = &constraint_matrix;

  data.q = const_cast<double *>(instance.objective_vector.data());
  data.l = const_cast<double *>(instance.lower_bounds.data());
  data.u = const_cast<double *>(instance.upper_bounds.data());

  ::OSQPSettings osqp_settings = ToInternalSettings(settings);

  OSQPWorkspace *workspace = nullptr;
  const auto return_code = osqp_setup(&workspace, &data, &osqp_settings);
  workspace_.reset(static_cast<OSQPWorkspaceHelper *>(workspace));
  if (return_code == 0) {
    return {true, ""};
  }
  switch (static_cast<osqp_error_type>(return_code)) {
  case OSQP_DATA_VALIDATION_ERROR:
    return {false, "Unable to initialize OSQP: data validation error."};
  case OSQP_SETTINGS_VALIDATION_ERROR:
    return {false, "Unable to initialize OSQP: invalid settings."};
  case OSQP_LINSYS_SOLVER_LOAD_ERROR:
    // This should never happen because qdldl is statically linked in.
    return {false, "Unable to initialize OSQP: unable to load linear solver."};
  case OSQP_LINSYS_SOLVER_INIT_ERROR:
    return {false,
            "Unable to initialize OSQP: unable to initialize linear solver."};
  case OSQP_NONCVX_ERROR:
    return {false,
            "Unable to initialize OSQP: the problem appears non-convex."};
  case OSQP_MEM_ALLOC_ERROR:
    return {false, "Unable to initialize OSQP: memory allocation error."};
  case OSQP_WORKSPACE_NOT_INIT_ERROR:
    return {false, "Unable to initialize OSQP: workspace not initialized."};
  }
  return {false, "Unable to initialize OSQP: unrecognized error code."};
}

namespace {
OsqpExitCode StatusToExitCode(const c_int status_val) {
  switch (status_val) {
  case OSQP_SOLVED:
    return OsqpExitCode::kOptimal;
  case OSQP_SOLVED_INACCURATE:
    return OsqpExitCode::kOptimalInaccurate;
  case OSQP_PRIMAL_INFEASIBLE:
    return OsqpExitCode::kPrimalInfeasible;
  case OSQP_PRIMAL_INFEASIBLE_INACCURATE:
    return OsqpExitCode::kPrimalInfeasibleInaccurate;
  case OSQP_DUAL_INFEASIBLE:
    return OsqpExitCode::kDualInfeasible;
  case OSQP_DUAL_INFEASIBLE_INACCURATE:
    return OsqpExitCode::kDualInfeasibleInaccurate;
  case OSQP_MAX_ITER_REACHED:
    return OsqpExitCode::kMaxIterations;
  case OSQP_SIGINT:
    return OsqpExitCode::kInterrupted;
  case OSQP_TIME_LIMIT_REACHED:
    return OsqpExitCode::kTimeLimitReached;
  case OSQP_NON_CVX:
    return OsqpExitCode::kNonConvex;
  default:
    return OsqpExitCode::kUnknown;
  }
}
} // namespace

OsqpExitCode OsqpSolver::Solve() {
  OSQP_CHECK(IsInitialized());
  if (osqp_solve(workspace_.get()) != 0) {
    // From looking at the code, this can happen if the solve process is
    // interrupted with ctrl-c or if updating "rho" fails.
    //    if (osqp_is_interrupted()) {
    //      return OsqpExitCode::kInterrupted;
    //    }
    return OsqpExitCode::kUnknown;
  }

  std::cout << "OsqpSolver::Solve, optimization status: " << workspace_->info->status_val
            << ",  number of iterations: " << workspace_->info->iter
            << ",  run_time(ms): " << workspace_->info->run_time * 1000
            << std::endl;

  return StatusToExitCode(workspace_->info->status_val);
}

long long OsqpSolver::iterations() const {
  OSQP_CHECK(IsInitialized());
  return workspace_->info->iter;
}

double OsqpSolver::objective_value() const {
  OSQP_CHECK(IsInitialized());
  return workspace_->info->obj_val;
}

std::vector<double> OsqpSolver::primal_solution() const {
  OSQP_CHECK(IsInitialized());
  return {workspace_->solution->x,
          workspace_->solution->x + workspace_->data->n};
}

OsqpSolver::Status OsqpSolver::GetPrimalSolutionAtIndex(int index,
                                                        double *value) const {
  OSQP_CHECK(IsInitialized());
  if (index < 0 || index >= workspace_->data->n) {
    std::string str{"Index "};
    str.append(std::to_string(index));
    str.append(" is out of bounds.");
    return {false, str};
  }
  *value = workspace_->solution->x[index];
  return {true, ""};
}

std::vector<double> OsqpSolver::dual_solution() const {
  OSQP_CHECK(IsInitialized());
  return {workspace_->solution->y,
          workspace_->solution->y + workspace_->data->m};
}

} // namespace OSQP

} // namespace PolyFit