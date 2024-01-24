/*
 * Copyright (C) 2023 by SenseTime Group Limited. All rights reserved.
 * Shengfa Zhu <zhushengfa@senseauto.com>
 */

#pragma once

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <cstddef>
#include <algorithm>
#include <vector>
#include <utility>
#include <string>
#include <memory>
#include <iostream>
#include "dynamic.hpp"
#include "constrain.hpp"
#include "constrain_value.hpp"
#include "obj_expansion.hpp"
#include "al_cost.hpp"
#include "cost_func.hpp"

namespace senseAD {
namespace planning {
namespace cilqr {

struct CilqrSolverOpt {
    bool verbose = false;
    int max_line_search_ite = 0;
    double line_search_init_step = 1.0;
    double line_search_step_decay = 0.0;
    double rho = 0.0;
    double max_reg = 1.0e8;
    double min_reg = 1.0e-8;
    double reg_scale = 1.5;
    int max_inner_loop_ite = 5;
    int max_outer_loop_ite = 10;
    double intermediate_obj_tol = 0.1;
    double violation_tol = 1e-2;
    double penalty = 1.0;
    double penalty_scaling = 6.0;

    bool CheckRegularization() {
        if (std::fabs(rho - max_reg) < 1e-2) {
            std::cerr << "rho reach max regularization" << std::endl;
            return false;
        }

        return true;
    }

    void IncreaseRegularization() {
        rho = std::min(max_reg, std::max(rho * reg_scale, min_reg));
        rho = std::max(rho, reg_scale);
        if (verbose) {
            std::cout << "Increase rho to " << rho << std::endl;
        }
    }

    void DecreaseRegularization() {
        rho = std::min(max_reg, std::max(rho / reg_scale, min_reg));
        if (rho < reg_scale) {
            rho = 0.0;
        }
        if (verbose) {
            std::cout << "Decrease rho to " << rho << std::endl;
        }
    }
};

enum class CilqrSolveStatus {
    SUCCESS = 0,
    // maybe convergent at local point, but meet all constrains
    LOCAL_OPTIMUM = -1,
    INVALID_PARAM = -2,
    REACH_INNER_MAX_ITERATION = -3,
    REACH_OUTER_MAX_ITERATION = -4,
    ILQR_FAILED = -5,
    REACH_MAX_REGULARIZATION = -6
};

/**
 * @brief Constrainted Cilqr Problem
 */
template <int n, int m>
class CilqrProblem {
 public:
    using Ptr = std::shared_ptr<CilqrProblem<n, m>>;
    using VectorNd = Eigen::Matrix<double, n, 1>;
    using MatrixNd = Eigen::Matrix<double, n, n>;
    using VectorMd = Eigen::Matrix<double, m, 1>;
    using MatrixMd = Eigen::Matrix<double, m, m>;
    using DynamicPtr = typename Dynamic<n, m>::Ptr;
    template <class ConType>
    using ConstrainSized = Constrain<n, m, ConType>;
    template <class ConType>
    using ConstrainSet = std::vector<std::shared_ptr<ConstrainSized<ConType>>>;
    using ObjectivePtr = typename ObjectiveExpansion<n, m>::Ptr;

    /**
     * @brief the detailed statisticss between iteration
     * the IterationStatistic will be recorded when verbose is true
     */
    struct IterationStatistic {
        // the iteration of outer loop
        int outer_loop_ite = 0;
        // the iteration of inner loop
        int inner_loop_ite = 0;
        // the trajectory
        std::vector<VectorNd> traj;
        // the control sequence
        std::vector<VectorMd> ctrls;
        // the cost of state
        double cost = 0.0;
        // the max constrain violation
        double max_constrain_violation = 0.0;
        // the constrain voliation
        std::vector<std::pair<std::string, double>> constrains;
    };

    explicit CilqrProblem(const CilqrSolverOpt& opt, const int num_step)
        : opt_(opt),
          num_step_(num_step),
          eq_(num_step_ + 1),
          ineq_(num_step_ + 1),
          al_costs_(num_step_ + 1),
          cost_funcs_(num_step_ + 1) {}

    void Clear() {
        for (std::size_t step = 0; step < num_step_ + 1; ++step) {
            eq_[step].clear();
            ineq_[step].clear();
            al_costs_[step].clear();
            cost_funcs_[step].clear();
        }
    }

    void Update(const VectorNd& init_state,
                const std::vector<VectorNd>& init_traj,
                const std::vector<VectorMd>& init_ctrls,
                const DynamicPtr& dynamic) {
        init_state_ = init_state;
        init_traj_ = init_traj;
        init_ctrls_ = init_ctrls;
        dynamic_ = dynamic;
        statistics_.clear();
    }

    void Assemble() {
        // put cost and constrain into al_cost
        for (std::size_t step = 0; step < num_step_ + 1; ++step) {
            al_costs_[step] = std::make_shared<ALCost<n, m>>(
                opt_.penalty, opt_.penalty_scaling);
            al_costs_[step]->SetCostFunc(cost_funcs_[step].begin(),
                                         cost_funcs_[step].end());
            al_costs_[step]->SetEqualityConstrain(eq_[step].begin(),
                                                  eq_[step].end());
            al_costs_[step]->SetInequalityConstrain(ineq_[step].begin(),
                                                    ineq_[step].end());
        }
    }

    void SetCostFunc(CostFuncPtr<n, m> cost_func, const int step) {
        if (step < num_step_ + 1 && cost_func != nullptr) {
            cost_funcs_[step].emplace_back(cost_func);
        }
    }

    void SetInequalityConstrain(
        const std::shared_ptr<ConstrainSized<Inequality>>& con, const int k) {
        if (con != nullptr && k < ineq_.size()) {
            ineq_[k].emplace_back(con);
        }
    }

    void SetEqualityConstrain(
        const std::shared_ptr<ConstrainSized<Equality>>& con, const int k) {
        if (con != nullptr && k < eq_.size()) {
            eq_[k].emplace_back(con);
        }
    }

    CilqrSolveStatus Solve(std::vector<VectorNd>* solution,
                           std::vector<VectorMd>* ctrls) {
        if (solution == nullptr || ctrls == nullptr) {
            std::cerr << "solution or ctrls are nullptr";
            return CilqrSolveStatus::INVALID_PARAM;
        }

        // TODO(Shengfa) check status of Assemble
        Assemble();
        std::vector<VectorNd> curr_X = init_traj_;
        std::vector<VectorMd> curr_U = init_ctrls_;
        // record iteration statistics
        if (opt_.verbose) {
            IterationStatistic statistic;
            statistic.traj.reserve(curr_X.size());
            for (const auto& state : curr_X) {
                statistic.traj.emplace_back(state);
            }
            statistic.ctrls.reserve(curr_U.size());
            for (const auto& ctrl : curr_U) {
                statistic.ctrls.emplace_back(ctrl);
            }
            // TODO(Shengfa) constrain violation
            statistics_.emplace_back(statistic);
        }
        Refresh(curr_X, curr_U);
        for (int i = 0; i < opt_.max_outer_loop_ite; ++i) {
            std::cout << "outer loop ite = " << i << std::endl;
            std::vector<VectorNd> new_X;
            std::vector<VectorMd> new_U;
            auto ilqr_status = ILQR(i, curr_X, curr_U, &new_X, &new_U);
            if (ilqr_status != CilqrSolveStatus::ILQR_FAILED &&
                ilqr_status != CilqrSolveStatus::REACH_MAX_REGULARIZATION) {
                curr_X = new_X;
                curr_U = new_U;
                Refresh(curr_X, curr_U);
                UpdateConstrains(curr_X, curr_U);
            }
            if (ilqr_status == CilqrSolveStatus::SUCCESS) {
                if (CheckConstrainsMeet(curr_X)) {
                    *solution = curr_X;
                    *ctrls = curr_U;
                    std::cerr << "ILQR success" << std::endl;
                    return CilqrSolveStatus::SUCCESS;
                }
            } else if (ilqr_status ==
                       CilqrSolveStatus::REACH_INNER_MAX_ITERATION) {
                std::cerr << "reach inner max interation";
                if (CheckConstrainsMeet(curr_X)) {
                    *solution = curr_X;
                    *ctrls = curr_U;
                    std::cout << "maybe convergent at local optimum"
                              << std::endl;
                    return CilqrSolveStatus::LOCAL_OPTIMUM;
                }
            } else if (ilqr_status ==
                       CilqrSolveStatus::REACH_MAX_REGULARIZATION) {
                std::cerr << "reach max regularization" << std::endl;
                return ilqr_status;
            } else {
                if (CheckConstrainsMeet(curr_X)) {
                    // although the cost doesn't decrease, but all the
                    // constraints are already satisfied. The initial solution
                    // is very close to the optimal solution
                    *solution = curr_X;
                    *ctrls = curr_U;
                    return CilqrSolveStatus::SUCCESS;
                }
                std::cerr << "ilqr_status = " << static_cast<int>(ilqr_status)
                          << std::endl;
            }
            // FIXME(Shengfa) confirm the cost will be calculated right
            LagrangianUpdate();
        }

        std::cerr << "reach max iteration" << std::endl;

        return CilqrSolveStatus::REACH_OUTER_MAX_ITERATION;
    }

    /**
     * @brief get statistics when verbose is true
     *
     * @param statistics[out] are iteration statistics
     */
    void GetStatistics(std::vector<IterationStatistic>* statistics) {
        *statistics = statistics_;
    }

 private:
    CilqrSolveStatus ILQR(const int outer_loop_ite,
                          const std::vector<VectorNd>& curr_traj,
                          const std::vector<VectorMd>& curr_ctrls,
                          std::vector<VectorNd>* new_traj,
                          std::vector<VectorMd>* new_ctrls) {
        std::vector<VectorNd> curr_X = curr_traj;
        std::vector<VectorMd> curr_U = curr_ctrls;
        double J_old = GetTotalCost(curr_X, curr_U);
        bool cost_deceased = false;
        for (int i = 0; i < opt_.max_inner_loop_ite; ++i) {
            IterationStatistic statistic;
            statistic.outer_loop_ite = outer_loop_ite;
            statistic.inner_loop_ite = i;
            std::vector<Eigen::Matrix<double, m, n>> K;
            std::vector<VectorMd> k;
            double delta_V1 = 0.0, delta_V2 = 0.0;
            if (!opt_.CheckRegularization()) {
                return CilqrSolveStatus::REACH_MAX_REGULARIZATION;
            }
            CalcExpansion(curr_X, curr_U);
            BackwardPass(curr_X, curr_U, &K, &k, &delta_V1, &delta_V2);
            std::vector<VectorNd> new_X;
            std::vector<VectorMd> new_U;
            double J_new = 0.0;
            if (ForwardPass(curr_X, curr_U, K, k, delta_V1, delta_V2, &new_X,
                            &new_U, &J_new)) {
                // if ForwardPass failed to make progress, the solver will
                // increase regularization and loop again
                if (opt_.verbose) {
                    // record iteration statistics
                    statistic.traj.reserve(curr_X.size());
                    for (const auto& state : curr_X) {
                        statistic.traj.emplace_back(state);
                    }
                    statistic.ctrls.reserve(curr_U.size());
                    for (const auto& ctrl : curr_U) {
                        statistic.ctrls.emplace_back(ctrl);
                    }
                    statistic.cost = GetTotalCost(curr_X, curr_U);
                    statistics_.emplace_back(statistic);
                }

                if (J_new < J_old) {
                    // logging solution at each iteration
                    // TODO(Shengfa) mayebe decrase some duplicate copy ?
                    curr_X = new_X;
                    curr_U = new_U;
                    *new_traj = curr_X;
                    *new_ctrls = curr_U;
                    if (std::fabs(J_new - J_old) < opt_.intermediate_obj_tol) {
                        std::cout
                            << "early break because of no grate imrprovement"
                            << std::endl;
                        return CilqrSolveStatus::SUCCESS;
                    }
                    J_old = J_new;
                    cost_deceased = true;
                }
            }
        }

        if (cost_deceased) {
            std::cout << "the cost could decrease with more iterations";
            return CilqrSolveStatus::REACH_INNER_MAX_ITERATION;
        } else {
            return CilqrSolveStatus::ILQR_FAILED;
        }
    }

    void BackwardPass(const std::vector<VectorNd>& normal_traj,
                      const std::vector<VectorMd>& normal_ctrls,
                      std::vector<Eigen::Matrix<double, m, n>>* K,
                      std::vector<VectorMd>* k,
                      double* delta_v1,
                      double* delta_v2) {
        // TODO(Shengfa) check size
        // TODO(Shengfa) try to allocate then set zero
        std::vector<MatrixNd> f_x =
            std::vector<MatrixNd>(num_step_, MatrixNd::Zero());
        std::vector<Eigen::Matrix<double, n, m>> f_u =
            std::vector<Eigen::Matrix<double, n, m>>(
                num_step_, Eigen::Matrix<double, n, m>::Zero());
        std::vector<VectorNd> l_x =
            std::vector<VectorNd>(num_step_ + 1, VectorNd::Zero());
        std::vector<VectorMd> l_u =
            std::vector<VectorMd>(num_step_, VectorMd::Zero());
        std::vector<MatrixNd> l_xx =
            std::vector<MatrixNd>(num_step_ + 1, MatrixNd::Zero());
        std::vector<MatrixMd> l_uu =
            std::vector<MatrixMd>(num_step_, MatrixMd::Zero());
        K->assign(num_step_, Eigen::Matrix<double, m, n>::Zero());
        k->assign(num_step_, VectorMd::Zero());

        // calcuate K and k
        for (int step = 0; step < num_step_; ++step) {
            const VectorNd& state = normal_traj[step];
            const VectorMd& ctrl = normal_ctrls[step];
            dynamic_->GetDerivates(state, ctrl, &f_x[step], &f_u[step]);
            al_costs_[step]->Gradient(state, ctrl, l_x[step], l_u[step]);
            // TODO(Shengfa) how about l_xu
            Eigen::Matrix<double, n, m> l_xu_temp;
            al_costs_[step]->Hessian(state, ctrl, l_xx[step], l_xu_temp,
                                     l_uu[step]);
        }
        Eigen::MatrixXd V_x = l_x.at(num_step_);
        Eigen::MatrixXd V_xx = l_xx.at(num_step_);
        int step = num_step_ - 1;
        while (step >= 0) {
            Eigen::MatrixXd Q_x = l_x[step] + f_x[step].transpose() * V_x;
            Eigen::MatrixXd Q_u = l_u[step] + f_u[step].transpose() * V_x;
            Eigen::MatrixXd Q_xx =
                l_xx[step] + f_x[step].transpose() * V_xx * f_x[step];
            Eigen::MatrixXd Q_uu =
                l_uu[step] + f_u[step].transpose() * V_xx * f_u[step];

            // Regularization
            Q_uu = Q_uu + opt_.rho * Eigen::MatrixXd::Identity(m, m);
            if (opt_.CheckRegularization() &&
                (!IsMatrixPositiveDefinite(Q_uu))) {
                opt_.IncreaseRegularization();

                // reset and backward from the end
                step = num_step_ - 1;
                *delta_v1 = 0.0, *delta_v2 = 0.0;
                V_x = l_x[num_step_];
                V_xx = l_xx[num_step_];
                continue;
            }

            Eigen::MatrixXd Q_ux = f_u[step].transpose() * V_xx * f_x[step];
            Eigen::MatrixXd Q_uu_inv = Q_uu.inverse();
            k->at(step) = -Q_uu_inv * Q_u;
            K->at(step) = -Q_uu_inv * Q_ux;
            // V_x = Q_x - K->at(step).transpose() * Q_uu * k->at(step);
            // V_xx = Q_xx - K->at(step).transpose() * Q_uu * K->at(step);
            V_x = Q_x + K->at(step).transpose() * Q_uu * k->at(step) +
                  K->at(step).transpose() * Q_u +
                  Q_ux.transpose() * k->at(step);
            V_xx = Q_xx + K->at(step).transpose() * Q_uu * K->at(step) +
                   K->at(step).transpose() * Q_ux +
                   Q_ux.transpose() * K->at(step);

            *delta_v1 += (k->at(step).transpose() * Q_u)(0, 0);
            *delta_v2 += 0.5 * k->at(step).transpose() * Q_uu * k->at(step);
            step--;
        }
        if (opt_.CheckRegularization()) {
            opt_.DecreaseRegularization();
        }
    }

    bool Rollout(const double alpha,
                 const std::vector<VectorNd>& normal_traj,
                 const std::vector<VectorMd>& normal_ctrls,
                 const std::vector<Eigen::Matrix<double, m, n>>& K,
                 const std::vector<VectorMd>& k,
                 std::vector<VectorNd>* new_traj,
                 std::vector<VectorMd>* new_ctrls) {
        if (new_traj == nullptr || new_ctrls == nullptr) {
            return false;
        }
        if (new_traj->size() != num_step_ + 1 ||
            new_ctrls->size() != num_step_) {
            return false;
        }
        new_traj->at(0) = init_state_;
        for (int step = 0; step < num_step_; step++) {
            new_ctrls->at(step) =
                normal_ctrls[step] +
                K[step] * (new_traj->at(step) - normal_traj[step]) +
                alpha * k[step];

            VectorNd updated_state;
            if (!dynamic_->Forward(new_traj->at(step), new_ctrls->at(step),
                                   &new_traj->at(step + 1))) {
                std::cout << "violate bound";
                return false;
            }
        }
        return true;
    }

    bool ForwardPass(const std::vector<VectorNd>& normal_traj,
                     const std::vector<VectorMd>& normal_ctrls,
                     const std::vector<Eigen::Matrix<double, m, n>>& K,
                     const std::vector<VectorMd>& k,
                     const double delta_v1,
                     const double delta_v2,
                     std::vector<VectorNd>* new_traj,
                     std::vector<VectorMd>* new_ctrls,
                     double* new_cost) {
        bool valid_descent_found = false;
        double J_old = GetTotalCost(normal_traj, normal_ctrls);
        double alpha = opt_.line_search_init_step;
        new_traj->clear(), new_ctrls->clear();
        std::vector<VectorNd> updated_traj(num_step_ + 1, VectorNd::Zero());
        std::vector<VectorMd> updated_ctrls(num_step_, VectorMd::Zero());
        for (int ite = 0; ite < opt_.max_line_search_ite; ++ite) {
            // expected decrease
            double expected =
                -1.0 * alpha * delta_v1 - alpha * alpha * delta_v2;

            if (Rollout(alpha, normal_traj, normal_ctrls, K, k, &updated_traj,
                        &updated_ctrls)) {
                double J_new = GetTotalCost(updated_traj, updated_ctrls);
                std::cout << "J_old = " << J_old << ", J_new = " << J_new
                          << std::endl;

                if (J_new < J_old) {
                    valid_descent_found = true;
                    *new_traj = updated_traj;
                    *new_ctrls = updated_ctrls;
                    *new_cost = J_new;
                    double z = (J_old - J_new) / std::max(expected, 1e-5);
                    std::cerr << "ratio of actual decrease = " << z
                              << std::endl;
                    break;
                }
            }
            alpha = alpha * opt_.line_search_step_decay;
        }

        if (valid_descent_found) {
            if (opt_.verbose) {
                std::cout << "valid descent found, alpha = " << alpha
                          << std::endl;
            }
            return true;
        } else {
            if (opt_.verbose) {
                std::cout << "valid descent not found, alpha = " << alpha
                          << ", try to increase regularization" << std::endl;
            }
            opt_.IncreaseRegularization();
            return false;
        }
    }

    double GetTotalCost(const std::vector<VectorNd>& traj,
                        const std::vector<VectorMd>& ctrls) const {
        double cost = 0.0;
        for (std::size_t step = 0; step < ctrls.size(); ++step) {
            cost += al_costs_[step]->Evaluate(traj[step], ctrls[step]);
        }

        VectorMd final_ctrl = VectorMd::Zero();
        cost += al_costs_[num_step_]->Evaluate(traj[num_step_], final_ctrl);
        return cost;
    }

    bool CheckConstrainsMeet(const std::vector<VectorNd>& traj) const {
        double max_vio = 0.0;
        std::size_t max_vio_step = 0;
        for (std::size_t step = 0; step < num_step_ + 1; ++step) {
            double vio = al_costs_[step]->MaxViolation();
            if (vio > max_vio) {
                max_vio_step = step;
                max_vio = vio;
            }
        }

        if (opt_.verbose) {
            std::shared_ptr<ConstrainValue<n, m, Inequality>> ineq;
            al_costs_[max_vio_step]->GetMaxViolatedIneq(&ineq);
            std::shared_ptr<const ConstrainValue<n, m, Equality>> eq;
            al_costs_[max_vio_step]->GetMaxViolatedEq(&eq);
            if (eq != nullptr) {
                std::cout << "max_vio = " << max_vio << ", max_vio of "
                          << eq->Name() << " = "
                          << eq->template MaxViolation<Eigen::Infinity>()
                          << std::endl;
            }
            if (ineq != nullptr) {
                std::cout << "max_vio = " << max_vio << ", max_vio of "
                          << ineq->Name() << " = "
                          << ineq->template MaxViolation<Eigen::Infinity>()
                          << std::endl;
            }
        }
        return max_vio < opt_.violation_tol;
    }

    bool IsMatrixPositiveDefinite(const Eigen::MatrixXd& A) {
        int row = A.rows();
        Eigen::MatrixXd sub_matrix(row, row);
        for (int k = 1; k <= row; k++) {
            sub_matrix = A.block(0, 0, k, k);
            if (sub_matrix.determinant() <= 0) {
                return false;
            }
        }

        return true;
    }

    void LagrangianUpdate() {
        for (int step = 0; step < num_step_ + 1; ++step) {
            al_costs_[step]->UpdateDuals();
            al_costs_[step]->UpdatePenalties();
        }
    }

    void CalcExpansion(const std::vector<VectorNd>& traj,
                       const std::vector<VectorMd>& ctrls) const {
        for (int step = 0; step < num_step_; ++step) {
            al_costs_[step]->CalcExpansion(traj[step], ctrls[step]);
        }
        VectorMd final_ctrl = VectorMd::Zero();
        al_costs_[num_step_]->CalcExpansion(traj[num_step_], final_ctrl);
    }

    void UpdateConstrains(const std::vector<VectorNd>& traj,
                          const std::vector<VectorMd>& ctrls) const {
        for (int step = 0; step < num_step_; ++step) {
            al_costs_[step]->UpdateConstrainValue(traj[step], ctrls[step]);
        }
    }

    void Refresh(const std::vector<VectorNd>& traj,
                 const std::vector<VectorMd>& ctrls) {
        for (std::size_t step = 0; step < num_step_; ++step) {
            al_costs_[step]->Refresh(traj[step], ctrls[step]);
        }
        VectorMd final_ctrl = VectorMd::Zero();
        al_costs_[num_step_]->Refresh(traj[num_step_], final_ctrl);
    }

 private:
    int num_step_;
    CilqrSolverOpt opt_;
    VectorNd init_state_;
    std::vector<VectorNd> init_traj_;
    std::vector<VectorMd> init_ctrls_;
    DynamicPtr dynamic_;
    std::vector<IterationStatistic> statistics_;

    // new implement
    // augument lagrangian cost including cost and constrian at each step
    std::vector<ConstrainSet<Equality>> eq_;
    std::vector<ConstrainSet<Inequality>> ineq_;
    std::vector<ALCostPtr<n, m>> al_costs_;
    std::vector<CostFuncVec<n, m>> cost_funcs_;
};

}  // namespace cilqr
}  // namespace planning
}  // namespace senseAD
