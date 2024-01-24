/*
 * Copyright (C) 2023 by SenseTime Group Limited. All rights reserved.
 * Shengfa Zhu <zhushengfa@senseauto.com>
 */

#pragma once

#include <eigen3/Eigen/Core>
#include <cstddef>
#include <memory>
#include <vector>
#include "constrain_value.hpp"
#include "cost_func.hpp"

namespace senseAD {
namespace planning {
namespace cilqr {

/*
 * Augumented Lagrangian Cost at one step with num_state = n, num_control = m
 * */
template <int n, int m>
class ALCost {
    using VectorNd = Eigen::Matrix<double, n, 1>;
    using VectorNdRef = Eigen::Ref<const VectorNd>;
    using VectorMd = Eigen::Matrix<double, m, 1>;
    using VectorMdRef = Eigen::Ref<const VectorMd>;
    using MatrixNd = Eigen::Matrix<double, n, n>;
    using MatrixMd = Eigen::Matrix<double, m, m>;
    template <class ConType>
    using ConstrainValueVec =
        std::vector<std::shared_ptr<ConstrainValue<n, m, ConType>>>;

 public:
    ALCost(const double penalty, const double penalty_scaling)
        : penalty_(penalty), penalty_scaling_(penalty_scaling) {}

    bool IsWellDefined() const {
        // if and only if size of cost_funcs more than zeor, the alcost is
        // well-defined
        return cost_funcs_.size() > 0;
    }

    void Reset() {
        cost_funcs_.clear();
        eqs_.clear();
        ineqs_.clear();
    }

    template <class Iterator>
    void SetCostFunc(const Iterator& begin, const Iterator& end) {
        for (auto it = begin; it != end; ++it) {
            if (*it != nullptr) {
                cost_funcs_.emplace_back(*it);
            }
        }
    }

    template <class Iterator>
    void SetEqualityConstrain(const Iterator& begin, const Iterator& end) {
        eqs_.clear();
        CopyToConstrainValues<Iterator, Equality>(begin, end, &eqs_);
    }

    template <class Iterator>
    void SetInequalityConstrain(const Iterator& begin, const Iterator& end) {
        ineqs_.clear();
        CopyToConstrainValues<Iterator, Inequality>(begin, end, &ineqs_);
    }

    double Evaluate(const VectorNdRef& x, const VectorMdRef& u) const {
        double J = 0.0;
        for (auto& func : cost_funcs_) {
            if (func == nullptr) {
                return 0.0;
            }
            J += func->Evaluate(x, u);
        }

        for (size_t i = 0; i < eqs_.size(); ++i) {
            J += eqs_[i]->AugLag(x, u);
        }
        for (size_t i = 0; i < ineqs_.size(); ++i) {
            J += ineqs_[i]->AugLag(x, u);
        }
        return J;
    }

    void UpdateConstrainValue(const VectorNdRef& x, const VectorMdRef& u) {
        for (size_t i = 0; i < eqs_.size(); ++i) {
            eqs_[i]->UpdateConstrainValue(x, u);
        }
        for (size_t i = 0; i < ineqs_.size(); ++i) {
            ineqs_[i]->UpdateConstrainValue(x, u);
        }
    }

    void Gradient(const VectorNdRef& x,
                  const VectorMdRef& u,
                  Eigen::Ref<VectorNd> dx,
                  Eigen::Ref<VectorMd> du) {
        dx.setZero();
        du.setZero();
        for (auto& func : cost_funcs_) {
            Eigen::Matrix<double, n + m, 1> grad;
            func->Gradient(x, u, grad);
            dx += grad.block(0, 0, n, 1);
            du += grad.block(n, 0, m, 1);
        }
        for (size_t i = 0; i < eqs_.size(); ++i) {
            VectorNd dx_tmp;
            VectorMd du_tmp;
            eqs_[i]->AugLagGradient(dx_tmp, du_tmp);
            dx += dx_tmp;
            du += du_tmp;
        }
        for (size_t i = 0; i < ineqs_.size(); ++i) {
            VectorNd dx_tmp;
            VectorMd du_tmp;
            ineqs_[i]->AugLagGradient(dx_tmp, du_tmp);
            dx += dx_tmp;
            du += du_tmp;
        }
    }

    void Hessian(const VectorNdRef& x,
                 const VectorMdRef& u,
                 Eigen::Ref<MatrixNd> dxdx,
                 Eigen::Ref<Eigen::Matrix<double, n, m>> dxdu,
                 Eigen::Ref<MatrixMd> dudu) {
        dxdu.setZero();
        dxdx.setZero();
        dudu.setZero();
        for (auto& func : cost_funcs_) {
            Eigen::Matrix<double, n + m, n + m> hess;
            func->Hessian(x, u, hess);
            dxdx += hess.block(0, 0, n, n);
            dxdu += hess.block(0, n, n, m);
            dudu += hess.block(n, n, m, m);
        }
        for (size_t i = 0; i < eqs_.size(); ++i) {
            MatrixNd dxdx_tmp;
            MatrixMd dudu_tmp;
            Eigen::Matrix<double, n, m> dxdu_tmp;

            eqs_[i]->AugLagHessian(dxdx_tmp, dxdu_tmp, dudu_tmp);
            dxdx += dxdx_tmp;
            dxdu += dxdu_tmp;
            dudu += dudu_tmp;
        }
        for (size_t i = 0; i < ineqs_.size(); ++i) {
            MatrixNd dxdx_tmp;
            MatrixMd dudu_tmp;
            Eigen::Matrix<double, n, m> dxdu_tmp;
            ineqs_[i]->AugLagHessian(dxdx_tmp, dxdu_tmp, dudu_tmp);
            dxdx += dxdx_tmp;
            dxdu += dxdu_tmp;
            dudu += dudu_tmp;
        }
    }

    /**
     * @brief Apply the dual update to all of the constraints
     *
     */
    void UpdateDuals() {
        for (size_t i = 0; i < eqs_.size(); ++i) {
            eqs_[i]->UpdateDual();
        }
        for (size_t i = 0; i < ineqs_.size(); ++i) {
            ineqs_[i]->UpdateDual();
        }
    }

    /**
     * @brief Apply the penalty update to all of the constraints
     *
     */
    void UpdatePenalties() {
        for (size_t i = 0; i < eqs_.size(); ++i) {
            eqs_[i]->UpdatePenalty();
        }
        for (size_t i = 0; i < ineqs_.size(); ++i) {
            ineqs_[i]->UpdatePenalty();
        }
    }

    void CalcExpansion(const VectorNdRef& x, const VectorMdRef& u) {
        for (size_t i = 0; i < eqs_.size(); ++i) {
            eqs_[i]->CalcExpansion(x, u);
        }
        for (size_t i = 0; i < ineqs_.size(); ++i) {
            ineqs_[i]->CalcExpansion(x, u);
        }
    }

    /**
     * @brief Find the maximum constraint violation for the current knot
     * point
     *
     * @tparam p Norm to use when calculating the violation (default is
     * Infinity)
     * @return Maximum constraint violation
     */
    template <int p = Eigen::Infinity>
    double MaxViolation() const {
        Eigen::Matrix<double, Eigen::Dynamic, 1> eqs_vio(eqs_.size());
        for (size_t i = 0; i < eqs_.size(); ++i) {
            eqs_vio(i) = eqs_[i]->template MaxViolation<p>();
        }
        Eigen::Matrix<double, Eigen::Dynamic, 1> ineqs_vio(ineqs_.size());
        for (size_t i = 0; i < ineqs_.size(); ++i) {
            ineqs_vio(i) = ineqs_[i]->template MaxViolation<p>();
        }
        Eigen::Vector2d tmp(eqs_vio.template lpNorm<p>(),
                            ineqs_vio.template lpNorm<p>());
        return tmp.template lpNorm<p>();
    }

    template <int p = Eigen::Infinity>
    void GetMaxViolatedEq(
        std::shared_ptr<const ConstrainValue<n, m, Equality>>* eq) const {
        if (eq == nullptr) {
            return;
        }
        size_t idx = 0;
        double max_vio = 0.0;
        for (size_t i = 0; i < eqs_.size(); ++i) {
            double vio = eqs_[i]->template MaxViolation<p>();
            if (vio > max_vio) {
                max_vio = vio;
                idx = i;
            }
        }
        if (idx < eqs_.size()) {
            *eq = eqs_[idx];
        } else {
            *eq = nullptr;
        }
    }

    template <int p = Eigen::Infinity>
    void GetMaxViolatedIneq(
        std::shared_ptr<ConstrainValue<n, m, Inequality>>* ineq) const {
        if (ineq == nullptr) {
            return;
        }
        size_t idx = 0;
        double max_vio = 0.0;
        for (size_t i = 0; i < ineqs_.size(); ++i) {
            double vio = ineqs_[i]->template MaxViolation<p>();
            if (vio > max_vio) {
                max_vio = vio;
                idx = i;
            }
        }
        if (idx < ineqs_.size()) {
            *ineq = ineqs_[idx];
        } else {
            *ineq = nullptr;
        }
    }

    template <class Iterator, class ConType>
    void CopyToConstrainValues(const Iterator& begin,
                               const Iterator& end,
                               ConstrainValueVec<ConType>* con_vals) const {
        if (con_vals == nullptr) {
            return;
        }
        // FIXME(Shengfa) it will cause dead loop when begin > end
        for (Iterator it = begin; it != end; ++it) {
            ConstrainPtr<n, m, ConType> con = *it;
            con_vals->emplace_back(
                std::make_shared<ConstrainValue<n, m, ConType>>(
                    *it, penalty_, penalty_scaling_));
        }
    }

    void Refresh(const VectorNdRef& x, const VectorMdRef& u) {
        // refresh cost function
        for (auto& func : cost_funcs_) {
            func->Refresh(x, u);
        }

        for (auto& con : ineqs_) {
            con->Refresh(x, u);
        }

        for (auto& con : eqs_) {
            con->Refresh(x, u);
        }
    }

 private:
    const double penalty_;
    const double penalty_scaling_;
    ConstrainValueVec<Equality> eqs_;
    ConstrainValueVec<Inequality> ineqs_;
    std::vector<CostFuncPtr<n, m>> cost_funcs_;
};

template <int n, int m>
using ALCostPtr = std::shared_ptr<ALCost<n, m>>;

}  // namespace cilqr
}  // namespace planning
}  // namespace senseAD
