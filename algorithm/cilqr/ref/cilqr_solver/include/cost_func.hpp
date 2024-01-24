/*
 * Copyright (C) 2023 by SenseTime Group Limited. All rights reserved.
 * Shengfa Zhu <zhushengfa@senseauto.com>
 */

#pragma once

#include <eigen3/Eigen/Core>
#include <vector>
#include <memory>
#include <random>
#include <iostream>
#include "derivative_checker.hpp"

namespace senseAD {
namespace planning {
namespace cilqr {

/*
 * Cost Func at one step with num_state = n, num_control = m
 * */
template <int n, int m>
class CostFunc {
 public:
    using VectorNd = Eigen::Matrix<double, n, 1>;
    using VectorNdRef = Eigen::Ref<const VectorNd>;
    using MatrixNd = Eigen::Matrix<double, n, n>;
    using MatrixMd = Eigen::Matrix<double, m, m>;
    using VectorMd = Eigen::Matrix<double, m, 1>;
    using VectorMdRef = Eigen::Ref<const VectorMd>;

    CostFunc() = default;

    virtual ~CostFunc() = default;

    /**
     * @brief Refresh by trajectory after every iteration
     *
     * @param traj is the current solution
     */

    // virtual void Refresh(const VectorNdRef& x, const VectorMdRef& u) = 0;

    virtual double Evaluate(const VectorNdRef& x, const VectorMdRef& u) = 0;

    virtual void Gradient(const VectorNdRef& x,
                          const VectorMdRef& u,
                          Eigen::Ref<Eigen::Matrix<double, n + m, 1>> grad) = 0;

    virtual void Hessian(
        const VectorNdRef& x,
        const VectorMdRef& u,
        Eigen::Ref<Eigen::Matrix<double, n + m, n + m>> hess) = 0;

    bool Check(const int num = 1000,
               const double eps = 1e-6,
               const bool verbose = false) {
        Eigen::VectorXd grad = Eigen::VectorXd::Zero(n + m);     // NOLINT
        Eigen::VectorXd fd_grad = Eigen::VectorXd::Zero(n + m);  // NOLINT

        auto fz = [&](auto z) -> double {
            return this->Evaluate(z.head(n), z.tail(m));
        };

        for (int i = 0; i < num; ++i) {
            VectorNd state = Eigen::VectorXd::Random(n);
            VectorMd ctrl = Eigen::VectorXd::Random(m);
            Gradient(state, ctrl, grad);
            Eigen::VectorXd z(n + m);
            z << state, ctrl;
            fd_grad = FiniteDiffGradient<-1>(fz, z);
            if (!MatrixComparison(fd_grad, grad, eps, verbose)) return false;
        }
        if (verbose) std::cout << "OK!" << std::endl;
        return true;
    }

    virtual void Refresh(const VectorNdRef& x, const VectorMdRef& u) {}
};

template <int n, int m>
using CostFuncPtr = std::shared_ptr<CostFunc<n, m>>;

template <int n, int m>
using CostFuncVec = std::vector<CostFuncPtr<n, m>>;

}  // namespace cilqr
}  // namespace planning
}  // namespace senseAD
